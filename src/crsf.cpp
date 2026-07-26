// crsf.cpp — hand-rolled CRSF TX + telemetry parse, half-duplex echo rejection (module #5).
#include "crsf.h"

#include <Arduino.h>

#include "gconfig.h"
#include "gimbals.h"
#include "pins.h"
#include "switches.h"

namespace {

constexpr uint32_t CRSF_BAUD = 400000;
// 250 Hz, not 500: our vTaskDelayUntil runs on the 1 ms tick, so at a 2 ms (500 Hz)
// period the ±1 ms jitter collides with the module's telemetry and drops the link
// intermittently. 250 Hz (4 ms) has enough margin to be rock-solid. The module
// still RF-transmits at its own rate, reusing the latest channels.
constexpr int TX_RATE_HZ = 250;

// CRSF addressing / frame types.
constexpr uint8_t ADDR_MODULE = 0xEE;  // CRSF transmitter (our destination)
constexpr uint8_t ADDR_RADIO = 0xEA;   // radio transmitter (telemetry addressed to us)
constexpr uint8_t ADDR_FC = 0xC8;
constexpr uint8_t ADDR_RX = 0xEC;
constexpr uint8_t TYPE_RC_CHANNELS = 0x16;      // our TX (echo -> ignore on RX)
constexpr uint8_t TYPE_LINK_STATISTICS = 0x14;  // module telemetry we care about
constexpr uint8_t TYPE_BATTERY_SENSOR = 0x08;   // FC pack V/I/mAh relayed up the link
constexpr uint8_t TYPE_RADIO_ID = 0x3A;         // OpenTX sync (frequent — skip in probe log)
constexpr uint8_t TYPE_DEVICE_PING = 0x28;      // probe: ask "who's on the bus?"
constexpr uint8_t TYPE_DEVICE_INFO = 0x29;      // reply to ping: module name + param count
constexpr uint8_t TYPE_PARAM_WRITE = 0x2D;      // Lua parameter write (e.g. set TX power)
constexpr uint8_t ADDR_BROADCAST = 0x00;

// "Max Power" Lua field ID (ELRS_POWER_COMMAND). Verified on the Nano TX V2. ELRS assigns
// field IDs by registration order, so a different firmware/module may need another value.
constexpr uint8_t POWER_FIELD_ID = 6;

// CRSF 11-bit channel range: 172=min, 992=center, 1811=max.
constexpr uint16_t CRSF_MIN = 172;
constexpr uint16_t CRSF_MID = 992;
constexpr uint16_t CRSF_MAX = 1811;

HardwareSerial& uart = Serial2;
TaskHandle_t g_task = nullptr;

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
CrsfLink g_link = {};
uint16_t g_channels[16] = {0};  // last sent channel values, guarded by g_mux
int g_arm_state = -1;  // -1 = no ARM toggle mapped, 0 = disarmed, 1 = armed (g_mux)
uint32_t g_linkstats_ms = 0;  // millis() of the last link-stats frame (g_mux)
int g_power_idx = 0;      // pending TX-power value to write (g_mux)
int g_power_repeat = 0;   // remaining times to resend the power write (g_mux)
// Diagnostic: ping the module + tally/log RX frame types to serial. Left off — testing
// showed the module streams sync (0x3A) but never answers device pings (0x29) on this
// half-duplex link, so the CRSF param/command path (incl. WiFi trigger) isn't available;
// use the no-packets timeout instead. Flip to true to re-run the probe.
bool g_probe = false;

uint8_t crc8_dvb_s2(const uint8_t* data, int len) {
  uint8_t crc = 0;
  for (int i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

// Pack 16 x 11-bit channels little-endian into 22 bytes (standard CRSF layout).
void packChannels(const uint16_t ch[16], uint8_t out[22]) {
  uint32_t bits = 0;
  int nbits = 0, o = 0;
  for (int i = 0; i < 16; ++i) {
    bits |= (uint32_t)(ch[i] & 0x07FF) << nbits;
    nbits += 11;
    while (nbits >= 8) {
      out[o++] = (uint8_t)(bits & 0xFF);
      bits >>= 8;
      nbits -= 8;
    }
  }
}

// Toggle -> MCP23017 bit map (A0-A7 = bits 0-7, B0-B7 = bits 8-15).
// Index i = Toggle(i+1) and drives channel CH(5+i). 2-position for now.
constexpr uint8_t TOGGLE_BIT[10] = {
    11,  // CH5  = Toggle1  = B3
    13,  // CH6  = Toggle2  = B5
    12,  // CH7  = Toggle3  = B4
    8,   // CH8  = Toggle4  = B0
    9,   // CH9  = Toggle5  = B1
    10,  // CH10 = Toggle6  = B2
    0,   // CH11 = Toggle7  = A0
    15,  // CH12 = Toggle8  = B7
    14,  // CH13 = Toggle9  = B6
    7,   // CH14 = Toggle10 = A7
};

void buildChannels(uint16_t ch[16]) {
  GimbalState g;
  gimbals_get(g);
  SwitchState sw;
  switches_get(sw);

  // AETR: CH1=Roll, CH2=Pitch, CH3=Throttle, CH4=Yaw. Calibrated (min/center/max) +
  // flip-aware map, matching the UI's stick %.
  ch[0] = gconfig_map_crsf(3, g.roll);
  ch[1] = gconfig_map_crsf(2, g.pitch);
  ch[2] = gconfig_map_crsf(0, g.throttle);
  ch[3] = gconfig_map_crsf(1, g.yaw);

  // CH5..CH16 default to center, then each toggle drives its assigned channel
  // (configurable in the Toggle Mapping menu; Toggle6 is reserved for menu mode).
  for (int c = 4; c < 16; ++c) ch[c] = CRSF_MID;
  for (int i = 0; i < 10; ++i) {
    uint8_t chan = gconfig_toggle_chan(i);  // 5..16, 0 = unassigned (incl. Toggle6)
    if (chan < 5 || chan > 16) continue;
    ch[chan - 1] = (sw.bits & (1u << TOGGLE_BIT[i])) ? CRSF_MAX : CRSF_MIN;
  }

  // Arm state for the UI banner + flight timer only. The throttle is NOT gated: the real
  // stick value is always sent (built above). Motor safety is the flight controller's job
  // via the ARM channel (the ARM toggle's mapped channel) — the FC won't spin motors while
  // disarmed regardless of throttle. If no toggle has the ARM function, state = unconfigured.
  int armT = gconfig_toggle_with_func(GFUNC_ARM);
  bool armed = (armT < 0) ? true : (sw.bits & (1u << TOGGLE_BIT[armT])) != 0;

  portENTER_CRITICAL(&g_mux);
  g_arm_state = (armT < 0) ? -1 : (armed ? 1 : 0);
  portEXIT_CRITICAL(&g_mux);
}

void sendRcFrame() {
  uint16_t ch[16];
  buildChannels(ch);

  portENTER_CRITICAL(&g_mux);
  memcpy(g_channels, ch, sizeof(g_channels));
  portEXIT_CRITICAL(&g_mux);

  uint8_t frame[26];
  frame[0] = ADDR_MODULE;
  frame[1] = 24;  // len = type(1) + payload(22) + crc(1)
  frame[2] = TYPE_RC_CHANNELS;
  packChannels(ch, &frame[3]);
  frame[25] = crc8_dvb_s2(&frame[2], 23);  // over type + payload

  uart.write(frame, sizeof(frame));
  uart.flush();  // wait for TX complete so the echo lands in the RX buffer
}

// Diagnostic: broadcast a CRSF device ping. A live ELRS module answers with a
// device-info (0x29) frame — which is what the probe is checking for.
void sendPing() {
  uint8_t f[6];
  f[0] = ADDR_MODULE;        // sync/addr, same as our RC frames
  f[1] = 4;                  // len = type(1) + dest(1) + origin(1) + crc(1)
  f[2] = TYPE_DEVICE_PING;
  f[3] = ADDR_MODULE;        // dest: the TX module directly (0xEE)
  f[4] = ADDR_RADIO;         // origin: us (the handset)
  f[5] = crc8_dvb_s2(&f[2], 3);
  uart.write(f, sizeof(f));
  uart.flush();
}

// Probe frame-type tally (single-writer in the CRSF task; diagnostics only).
uint16_t g_pc_sync = 0, g_pc_link = 0, g_pc_echo = 0, g_pc_info = 0, g_pc_other = 0;

// Send a CRSF parameter write to the module's "Max Power" field. value = power index
// (0=10mW,1=25,2=50,3=100,4=250,5=500,6=1000,7=2000), assuming the module's min power is
// 10 mW. No device discovery needed — the module acts on the write immediately.
void sendPowerWrite(uint8_t value) {
  while (uart.available()) uart.read();  // drain RX (echo/telemetry) before the write
  uint8_t f[8];
  f[0] = ADDR_MODULE;       // sync/dest
  f[1] = 6;                 // frame_size = type+dest+orig+field+value+crc
  f[2] = TYPE_PARAM_WRITE;  // 0x2D
  f[3] = ADDR_MODULE;       // dest ext = the TX module (0xEE)
  f[4] = ADDR_RADIO;        // origin ext = radio handset (0xEA) — module validates this
  f[5] = POWER_FIELD_ID;    // "Max Power" field
  f[6] = value;             // power index (into the module's filtered table, min-power = 0)
  f[7] = crc8_dvb_s2(&f[2], 5);
  uart.write(f, sizeof(f));
  uart.flush();
}

// --- Telemetry parser (fed all RX bytes; validates CRC; ignores our own echo).
uint8_t rxbuf[64];
int rxlen = 0;

bool isKnownAddr(uint8_t a) {
  return a == ADDR_MODULE || a == ADDR_RADIO || a == ADDR_FC || a == ADDR_RX;
}

void handleFrame(uint8_t type, const uint8_t* payload, int plen) {
  if (type == TYPE_RC_CHANNELS) return;  // echo of our own TX on the shared wire

  portENTER_CRITICAL(&g_mux);
  g_link.rx_frames++;
  if (type == TYPE_LINK_STATISTICS && plen >= 10) {
    // CRSF LinkStatistics: [0]up RSSI ant1 [1]ant2 [2]up LQ [3]up SNR [4]active ant
    // [5]RF mode [6]up TX power idx [7]dn RSSI [8]dn LQ [9]dn SNR.
    g_link.up_rssi = -(int16_t)payload[0];  // uplink RSSI (dBm)
    g_link.up_lq = payload[2];              // uplink link quality (%)
    g_link.up_snr = (int8_t)payload[3];
    g_link.ant = payload[4];
    g_link.rf_mode = payload[5];
    g_link.tx_power = payload[6];
    g_link.dn_rssi = -(int16_t)payload[7];
    g_link.dn_lq = payload[8];
    g_link.dn_snr = (int8_t)payload[9];
    g_link.valid = true;
    g_linkstats_ms = millis();
  } else if (type == TYPE_BATTERY_SENSOR && plen >= 8) {
    // Big-endian: [0-1] voltage (0.1V), [2-3] current (0.1A), [4-6] used mAh, [7] %.
    g_link.batt_dv = (uint16_t)((payload[0] << 8) | payload[1]);
    g_link.batt_da = (uint16_t)((payload[2] << 8) | payload[3]);
    g_link.batt_mah = ((uint32_t)payload[4] << 16) | ((uint32_t)payload[5] << 8) | payload[6];
    g_link.batt_pct = payload[7];
    g_link.batt_valid = true;
  }
  portEXIT_CRITICAL(&g_mux);

  // Probe: tally frame types (printed once/sec by the task) + dump interesting ones.
  if (g_probe) {
    if (type == TYPE_RADIO_ID) g_pc_sync++;
    else if (type == TYPE_LINK_STATISTICS) g_pc_link++;
    else if (type == TYPE_DEVICE_PING || type == TYPE_RC_CHANNELS) g_pc_echo++;
    else if (type == TYPE_DEVICE_INFO) g_pc_info++;
    else g_pc_other++;

    if (type != TYPE_LINK_STATISTICS && type != TYPE_RADIO_ID &&
        type != TYPE_DEVICE_PING && type != TYPE_RC_CHANNELS) {
      Serial.printf("[probe] rx type=0x%02X len=%d:", type, plen);
      for (int i = 0; i < plen && i < 24; ++i) Serial.printf(" %02X", payload[i]);
      Serial.println();
      if (type == TYPE_DEVICE_INFO && plen > 2)
        Serial.printf("[probe] >>> DEVICE INFO — module name: '%s'\n", (const char*)&payload[2]);
    }
  }
}

void feed(uint8_t b) {
  if (rxlen == 0) {
    if (!isKnownAddr(b)) return;  // resync on a valid address byte
    rxbuf[rxlen++] = b;
    return;
  }
  if (rxlen == 1) {
    if (b < 2 || b > 62) {  // implausible length -> resync
      rxlen = 0;
      return;
    }
    rxbuf[rxlen++] = b;
    return;
  }
  rxbuf[rxlen++] = b;
  int total = rxbuf[1] + 2;  // addr + len + (len bytes)
  if (rxlen >= total) {
    uint8_t type = rxbuf[2];
    uint8_t crc = rxbuf[total - 1];
    if (crc8_dvb_s2(&rxbuf[2], rxbuf[1] - 1) == crc) {
      handleFrame(type, &rxbuf[3], rxbuf[1] - 2);
    }
    rxlen = 0;
  }
  if (rxlen >= (int)sizeof(rxbuf)) rxlen = 0;  // overflow guard
}

void crsfTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(1000 / TX_RATE_HZ);  // 2 ms @ 500 Hz
  TickType_t last = xTaskGetTickCount();
  uint32_t frameCnt = 0;
  for (;;) {
    // Pending TX-power write? Hold the RC stream for this tick and send the parameter
    // write on a quiet wire instead — interleaving it with RC channel frames corrupts
    // the module's parameter parsing on the shared single wire (ELRS param-traffic rule).
    portENTER_CRITICAL(&g_mux);
    int pwr = (g_power_repeat > 0) ? g_power_idx : -1;
    if (g_power_repeat > 0) g_power_repeat--;
    portEXIT_CRITICAL(&g_mux);

    if (pwr >= 0) {
      sendPowerWrite((uint8_t)pwr);
    } else {
      sendRcFrame();
      portENTER_CRITICAL(&g_mux);
      g_link.tx_frames++;
      portEXIT_CRITICAL(&g_mux);

      // Probe: once a second, ping the module and report the RX frame-type tally.
      if (g_probe && (++frameCnt % TX_RATE_HZ == 0)) {
        sendPing();
        Serial.printf("[probe] 1s tally  sync=%u link=%u echo=%u INFO=%u other=%u\n", g_pc_sync,
                      g_pc_link, g_pc_echo, g_pc_info, g_pc_other);
        g_pc_sync = g_pc_link = g_pc_echo = g_pc_info = g_pc_other = 0;
      }
    }

    // Drain everything the module (and our echo) put on the wire.
    while (uart.available()) feed((uint8_t)uart.read());

    // Fixed cadence (independent of TX/parse time) for a steady frame rate.
    vTaskDelayUntil(&last, period);
  }
}

}  // namespace

bool crsf_start() {
  uart.begin(CRSF_BAUD, SERIAL_8N1, PIN_CRSF_RX, PIN_CRSF_TX);
  uart.setTimeout(0);

  BaseType_t ok = xTaskCreatePinnedToCore(crsfTask, "crsf", 4096, nullptr,
                                          /*priority=*/3, &g_task, /*core=*/0);
  if (ok != pdPASS) {
    Serial.println("[crsf] task create FAILED");
    return false;
  }
  Serial.println("[crsf] UART2 up @ 400000 (AETR, 250 Hz, half-duplex)");
  return true;
}

bool crsf_get(CrsfLink& out) {
  portENTER_CRITICAL(&g_mux);
  out = g_link;
  portEXIT_CRITICAL(&g_mux);
  return out.valid;
}

void crsf_get_channels(uint16_t out[16]) {
  portENTER_CRITICAL(&g_mux);
  memcpy(out, g_channels, sizeof(g_channels));
  portEXIT_CRITICAL(&g_mux);
}

int crsf_arm_state() {
  portENTER_CRITICAL(&g_mux);
  int s = g_arm_state;
  portEXIT_CRITICAL(&g_mux);
  return s;
}

bool crsf_telem_online() {
  portENTER_CRITICAL(&g_mux);
  bool v = g_link.valid;
  uint32_t t = g_linkstats_ms;
  portEXIT_CRITICAL(&g_mux);
  if (!v) return false;                 // never received link stats
  return (millis() - t) < 1000;         // stale after 1 s without a link-stats frame
}

void crsf_set_power(int level_index) {
  if (level_index < 0 || level_index > 7) return;
  portENTER_CRITICAL(&g_mux);
  g_power_idx = level_index;
  g_power_repeat = 6;  // hold RC for ~6 ticks (~24 ms) and send the write on the quiet wire
  portEXIT_CRITICAL(&g_mux);
}

uint16_t crsf_tx_power_mw(uint8_t idx) {
  // Standard CRSF power-index table.
  static const uint16_t P[] = {0, 10, 25, 100, 500, 1000, 2000, 250, 50};
  return idx < (sizeof(P) / sizeof(P[0])) ? P[idx] : 0;
}


void crsf_pause() {
  if (!g_task) return;             // task never started (e.g. module-WiFi mode)
  vTaskSuspend(g_task);
  uart.flush();                    // let in-flight TX finish
  vTaskDelay(pdMS_TO_TICKS(10));   // and the echo/telemetry burst drain
}

void crsf_resume() {
  if (g_task) vTaskResume(g_task);
}
