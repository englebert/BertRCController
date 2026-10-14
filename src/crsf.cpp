// crsf.cpp — hand-rolled CRSF TX + telemetry parse, half-duplex echo rejection (module #5).
#include "crsf.h"

#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_timer.h>

#include "gconfig.h"
#include "gimbals.h"
#include "pins.h"
#include "switches.h"

void queueParamWrite(int field, int value);     // fwd decls (defined below, file scope)
void queueParamWriteRaw(int field, int value);
void queueSettingWrite(ElrsSetting s, int value);

namespace {

constexpr uint32_t CRSF_BAUD_DEFAULT = 921600;  // unlocks 500Hz/F1000 (400k caps at 333Hz)
// Frame pacing. The module streams OpenTX-sync frames (0x3A) carrying its RF packet
// interval and how early our frames land before each RF packet; like EdgeTX's mixer
// scheduler we follow them (period = interval + lag, lag consumed once) so every RF
// packet carries fresh channels. A µs esp_timer drives the task (the 1 ms FreeRTOS tick
// was too coarse: ±1 ms jitter at 2 ms collided with the module's telemetry window).
// Without valid sync (none yet, or > 2 s old) we fall back to a fixed 250 Hz.
constexpr uint32_t FALLBACK_PERIOD_US = 4000;  // 250 Hz
constexpr uint32_t MIN_PERIOD_US = 850;        // EdgeTX MIN_REFRESH_RATE
constexpr uint32_t MAX_PERIOD_US = 50000;      // EdgeTX MAX_REFRESH_RATE
constexpr uint32_t SYNC_VALID_MS = 2000;
// Module considered reset when it has been silent this long and then talks again; the
// model ID and stored settings are then re-sent (EdgeTX MODULE_ALIVE_TIMEOUT).
constexpr uint32_t MODULE_ALIVE_TIMEOUT_MS = 500;

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
constexpr uint8_t TYPE_PARAM_ENTRY = 0x2B;      // reply to a param read (chunked)
constexpr uint8_t TYPE_PARAM_READ = 0x2C;       // request one param entry (by field + chunk)
constexpr uint8_t TYPE_PARAM_WRITE = 0x2D;      // Lua parameter write (e.g. set TX power)
constexpr uint8_t TYPE_COMMAND = 0x32;          // CRSF command frame (inner CRC, poly 0xBA)
constexpr uint8_t CMD_CRSF = 0x10;              // command class: CRSF
constexpr uint8_t CMD_CRSF_BIND = 0x01;         // CRSF sub-command: enter bind mode
constexpr uint8_t CMD_CRSF_MODEL_ID = 0x05;     // CRSF sub-command: select model ID (1 arg)
constexpr uint8_t MODEL_ID = 0;  // the model we announce; ELRS keeps its settings per model ID
constexpr uint8_t ADDR_BROADCAST = 0x00;

// "Max Power" Lua field ID (ELRS_POWER_COMMAND). Verified on the Nano TX V2. ELRS assigns
// field IDs by registration order, so a different firmware/module may need another value.
constexpr uint8_t POWER_FIELD_ID = CRSF_FIELD_MAX_POWER;

// CRSF 11-bit channel range: 172=min, 992=center, 1811=max.
constexpr uint16_t CRSF_MIN = 172;
constexpr uint16_t CRSF_MID = 992;
constexpr uint16_t CRSF_MAX = 1811;

HardwareSerial& uart = Serial2;
TaskHandle_t g_task = nullptr;
uint32_t g_crsf_baud = CRSF_BAUD_DEFAULT;  // active UART baud (module auto-detects it)
volatile bool g_tx_enabled = true;  // false = stop transmitting RC (module fails safe)

// Boot-time restore of the module settings (crsf_set_restore). Single-writer: the task.
uint8_t g_restore[ELRS_SETTING_N] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
int g_restore_next = 0;      // position in RESTORE_ORDER
uint32_t g_restore_due = 0;  // millis() the next write may go out; 0 = not started yet
uint32_t g_start_ms = 0;     // millis() at crsf_start()
// Task-local (single writer: the CRSF task).
uint32_t g_sync_rate_us = 0;  // module RF packet interval from 0x3A, 0 = none yet
int32_t g_sync_lag_us = 0;    // remaining lag to absorb into the next period
uint32_t g_sync_ms = 0;       // millis() of the last 0x3A timing frame
uint32_t g_last_rx_ms = 0;    // millis() of the last valid frame from the module
bool g_module_alive = false;
uint32_t g_ping_due = 0;      // millis() to send one device ping, 0 = none
// Module Lua field ids, looked up by name at boot (fieldScan*), because they shift between
// ELRS versions/builds (3.5 inserts "Link Mode": Max Power 6->7, Fan 8->9). 0 = not found.
enum ElrsField {
  EF_PKT_RATE, EF_SWITCH_MODE, EF_MAX_POWER, EF_FAN, EF_BIND, EF_TLM_RATIO, EF_MODEL_MATCH, EF_N
};
constexpr const char* FIELD_NAME[EF_N] = {"Packet Rate", "Switch Mode", "Max Power", "Fan Thresh",
                                          "Bind", "Telem Ratio", "Model Match"};
// Ids used before the scan finishes (ELRS 3.4.x order); 0 = no default, wait for the scan.
constexpr int FIELD_DEFAULT[EF_N] = {CRSF_FIELD_PKT_RATE, CRSF_FIELD_SWITCH_MODE,
                                     CRSF_FIELD_MAX_POWER, CRSF_FIELD_FAN, 0, 2, 0};
// Module field behind each stored setting (index = ElrsSetting).
constexpr ElrsField SETTING_FIELD[ELRS_SETTING_N] = {EF_PKT_RATE, EF_SWITCH_MODE, EF_MAX_POWER,
                                                     EF_FAN, EF_TLM_RATIO, EF_MODEL_MATCH};
int g_field[EF_N] = {0};
bool g_scan_active = false, g_scan_done = false;
int g_scan_next = 0;      // next field id to read
int g_scan_pass = 0;      // 0 = first pass; later passes re-read fields that didn't answer
uint64_t g_scan_seen = 0; // bit n = field n answered
uint32_t g_scan_due = 0;  // millis() of the next read (or of finishing, once past the end)
int g_param_count = 0;  // module's Lua field count (from DEVICE_INFO), 0 = unknown
int g_read_field = -1;  // queued param read (field id), -2 = device ping, -1 = none
uint8_t g_read_dest = 0xEE;  // queued read's destination (module, or receiver over the air)
uint8_t g_read_chunk = 0;    // queued read's chunk number
int g_rx_param_count = 0;    // receiver's Lua field count (its DEVICE_INFO), 0 = unknown

// Module setting read-back (as the EdgeTX Lua shows them): full entries (all chunks) of the
// selection fields below, re-read after the boot restore and after each write, so the menus
// show the module's real value and its option list (switch-mode options follow the rate).
constexpr uint8_t READBACK_FIELDS = 1 << EF_SWITCH_MODE | 1 << EF_TLM_RATIO | 1 << EF_MODEL_MATCH;
struct ModSel {
  bool valid;
  uint8_t value;
  char options[CRSF_OPTS_LEN];
};
ModSel g_msel[EF_N] = {};    // guarded by g_mux (the UI copies it)
uint8_t g_mr_pending = 0;    // bit n = ElrsField n to read
int g_mr_ef = -1;            // field being read, -1 = none
uint8_t g_mr_chunk = 0, g_mr_tries = 0, g_mr_remain = 0;
uint8_t g_mr_buf[160];
int g_mr_len = 0;
volatile bool g_mr_got = false;
uint32_t g_mr_due = 0, g_mr_sent = 0;
bool g_restore_done = false;

// Receiver menu reader (crsf_rx_read). Requests go to the RX address (0xEC); the module
// relays them over the air and the RX's PARAM_ENTRY replies come back as telemetry —
// slowly (telemetry slots), and chunked for long entries, so each chunk gets a generous
// timeout and a couple of retries. Single writer: the CRSF task.
constexpr int RX_MAX_PARAMS = 24;
CrsfRxParam g_rx_params[RX_MAX_PARAMS];
volatile int g_rx_n = 0;              // entries filled
volatile int g_rx_state = RXR_IDLE;
volatile int g_rx_field = 0;          // field being read (progress)
uint8_t g_rx_chunk = 0, g_rx_tries = 0;
uint8_t g_rx_buf[256];
int g_rx_len = 0;
bool g_rx_got = false;                // a chunk for g_rx_field arrived
uint8_t g_rx_remain = 0;              // chunks remaining after the one that arrived
uint32_t g_rx_due = 0;
esp_timer_handle_t g_tick_timer = nullptr;
// Packet rate first: switch-mode option indices are only meaningful for the rate in force.
constexpr ElrsSetting RESTORE_ORDER[] = {ELRS_PKT_RATE,   ELRS_SWITCH_MODE, ELRS_TLM_RATIO,
                                         ELRS_MODEL_MATCH, ELRS_TX_POWER,   ELRS_FAN};
constexpr const char* RESTORE_NAME[ELRS_SETTING_N] = {"packet rate", "switch mode", "TX power",
                                                      "fan", "telem ratio", "model match"};

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
CrsfLink g_link = {};
uint16_t g_channels[16] = {0};  // last sent channel values, guarded by g_mux
int g_arm_state = -1;  // -1 = no ARM toggle mapped, 0 = disarmed, 1 = armed (g_mux)
uint32_t g_linkstats_ms = 0;  // millis() of the last link-stats frame (g_mux)
int g_power_idx = 0;      // pending param write value (g_mux)
int g_power_repeat = 0;   // remaining times to resend the queued write (g_mux)
int g_write_field = 6;    // param field id for the queued write (6 = Max Power)
int g_cmd_repeat = 0;     // remaining sends of the queued command (g_mux)
uint8_t g_cmd_dest = 0;   // queued command's destination address (g_mux)
uint8_t g_cmd_sub = 0;    // queued command's CRSF sub-command (g_mux)
int g_cmd_arg = -1;       // queued command's argument byte, -1 = none (g_mux)
// Diagnostic: ping the module + tally/log RX frame types to serial. Left off — testing
// showed the module streams sync (0x3A) but never answers device pings (0x29) on this
// half-duplex link, so the CRSF param/command path (incl. WiFi trigger) isn't available;
// use the no-packets timeout instead. Flip to true to re-run the probe.
bool g_probe = false;

uint8_t crc8_poly(const uint8_t* data, int len, uint8_t poly) {
  uint8_t crc = 0;
  for (int i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ poly) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

// Frame CRC (DVB-S2, poly 0xD5).
uint8_t crc8_dvb_s2(const uint8_t* data, int len) { return crc8_poly(data, len, 0xD5); }

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
  // (pin + channel are both configurable in the Toggle Mapping menu; in the standard
  // build Toggle6 is reserved for menu mode and has no channel).
  for (int c = 4; c < 16; ++c) ch[c] = CRSF_MID;
  for (int i = 0; i < 10; ++i) {
    uint8_t chan = gconfig_toggle_chan(i);  // 5..16, 0 = unassigned (incl. Toggle6)
    if (chan < 5 || chan > 16) continue;
    ch[chan - 1] = gconfig_toggle_on(i, sw.bits) ? CRSF_MAX : CRSF_MIN;
  }

  // Arm state for the UI banner + flight timer only. The throttle is NOT gated: the real
  // stick value is always sent (built above). Motor safety is the flight controller's job
  // via the ARM channel (the ARM toggle's mapped channel) — the FC won't spin motors while
  // disarmed regardless of throttle. If no toggle has the ARM function, state = unconfigured.
  int armT = gconfig_toggle_with_func(GFUNC_ARM);
  bool armed = (armT < 0) ? true : gconfig_toggle_on(armT, sw.bits);

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
// CRSF command to the TX module: bind (crsf_bind) or model-ID select (sent at boot, as
// EdgeTX does). arg < 0 = no argument byte. Command frames carry an inner CRC (poly 0xBA)
// over type..payload ahead of the normal frame CRC.
void sendCommand(uint8_t dest, uint8_t sub, int arg) {
  while (uart.available()) uart.read();  // drain RX (echo/telemetry) first
  uint8_t f[10];
  int n = 0;
  f[n++] = ADDR_MODULE;
  f[n++] = 0;  // length, filled below
  f[n++] = TYPE_COMMAND;
  f[n++] = dest;        // the TX module, or the receiver (relayed over the air)
  f[n++] = ADDR_RADIO;  // origin: us
  f[n++] = CMD_CRSF;
  f[n++] = sub;
  if (arg >= 0) f[n++] = (uint8_t)arg;
  f[n] = crc8_poly(&f[2], n - 2, 0xBA);
  n++;
  f[n] = crc8_dvb_s2(&f[2], n - 2);
  n++;
  f[1] = n - 2;  // type .. frame crc
  uart.write(f, n);
  uart.flush();
}

void queueCommand(uint8_t dest, uint8_t sub, int arg, int repeat) {
  portENTER_CRITICAL(&g_mux);
  g_cmd_dest = dest;
  g_cmd_sub = sub;
  g_cmd_arg = arg;
  g_cmd_repeat = repeat;
  portEXIT_CRITICAL(&g_mux);
}

void sendPowerWrite(uint8_t value) {
  while (uart.available()) uart.read();  // drain RX (echo/telemetry) before the write
  uint8_t f[8];
  f[0] = ADDR_MODULE;       // sync/dest
  f[1] = 6;                 // frame_size = type+dest+orig+field+value+crc
  f[2] = TYPE_PARAM_WRITE;  // 0x2D
  f[3] = ADDR_MODULE;       // dest ext = the TX module (0xEE)
  f[4] = ADDR_RADIO;        // origin ext = radio handset (0xEA) — module validates this
  f[5] = (uint8_t)g_write_field;  // param field id (6 = Max Power, 8 = Fan Thresh, ...)
  f[6] = value;             // option index
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

// Link-stats RSSI byte -> dBm. The CRSF spec sends "dBm * -1" (positive), but ELRS puts
// the negative dBm in the byte as an int8 (e.g. 0xD6 = -42 dBm). Accept both.
int16_t rssiDbm(uint8_t b) {
  int8_t v = (int8_t)b;
  return v < 0 ? v : -v;
}

void handleFrame(uint8_t type, const uint8_t* payload, int plen) {
  if (type == TYPE_RC_CHANNELS) return;  // echo of our own TX on the shared wire
  g_last_rx_ms = millis();

  // DEVICE_INFO (ping reply): [dest][origin][name\0][serial4][hw4][sw4][field count][ver].
  if (type == TYPE_DEVICE_INFO && plen > 2) {
    int n = strnlen((const char*)&payload[2], plen - 2);
    int o = 2 + n + 1;
    if (o + 14 <= plen) {
      int cnt = payload[o + 12];
      bool mod = payload[1] == ADDR_MODULE;  // the receiver answers the broadcast ping too
      if (!mod || cnt != g_param_count)
        Serial.printf("[crsf] %s: '%.*s' ELRS %u.%u.%u, %d params\n", mod ? "module" : "receiver", n,
                      (const char*)&payload[2], payload[o + 9], payload[o + 10], payload[o + 11], cnt);
      if (mod) g_param_count = cnt;
      else if (payload[1] == ADDR_RX) g_rx_param_count = cnt;
    }
  }
  // Receiver menu entry chunk: [dest][origin 0xEC][field][chunks left][data...]; chunk 0's
  // data starts with [parent][type][name\0], later chunks continue it. ELRS RXs address
  // these to the TX module (0xEE), which just passes them on — accept either destination.
  if (type == TYPE_PARAM_ENTRY && plen > 4 && payload[1] == ADDR_RX &&
      (payload[0] == ADDR_RADIO || payload[0] == ADDR_MODULE) &&
      g_rx_state == RXR_READING && payload[2] == g_rx_field && !g_rx_got) {
    int n = plen - 4;
    if (g_rx_len + n > (int)sizeof(g_rx_buf)) n = sizeof(g_rx_buf) - g_rx_len;
    memcpy(g_rx_buf + g_rx_len, &payload[4], n);
    g_rx_len += n;
    g_rx_remain = payload[3];
    g_rx_got = true;
  }
  // Module setting read-back chunk: [dest][origin 0xEE][field][chunks left][data...].
  if (type == TYPE_PARAM_ENTRY && plen > 4 && payload[0] == ADDR_RADIO && payload[1] == ADDR_MODULE &&
      g_mr_ef >= 0 && payload[2] == g_field[g_mr_ef] && !g_mr_got) {
    int n = plen - 4;
    if (g_mr_len + n > (int)sizeof(g_mr_buf)) n = sizeof(g_mr_buf) - g_mr_len;
    memcpy(g_mr_buf + g_mr_len, &payload[4], n);
    g_mr_len += n;
    g_mr_remain = payload[3];
    g_mr_got = true;
  }
  // PARAM_ENTRY chunk 0: [dest][origin][field][chunks left][parent][type][name\0]...
  if (type == TYPE_PARAM_ENTRY && plen > 7 && payload[0] == ADDR_RADIO && payload[1] == ADDR_MODULE &&
      g_scan_active) {
    if (payload[2] < 64) g_scan_seen |= 1ULL << payload[2];
    const char* name = (const char*)&payload[6];
    int nlen = strnlen(name, plen - 6);
    for (int i = 0; i < EF_N; ++i) {
      if (nlen == (int)strlen(FIELD_NAME[i]) && !strncmp(name, FIELD_NAME[i], nlen) &&
          (i != EF_BIND || (payload[5] & 0x3F) == 0x0D))  // Bind must be a COMMAND
        g_field[i] = payload[2];
    }
  }

  // OpenTX sync: [dest 0xEA][origin][0x10][rate u32 BE][offset s32 BE], both in 0.1 us.
  if (type == TYPE_RADIO_ID && plen >= 11 && payload[0] == ADDR_RADIO && payload[2] == 0x10) {
    uint32_t rate = ((uint32_t)payload[3] << 24 | (uint32_t)payload[4] << 16 |
                     (uint32_t)payload[5] << 8 | payload[6]) / 10;
    int32_t lag = (int32_t)((uint32_t)payload[7] << 24 | (uint32_t)payload[8] << 16 |
                            (uint32_t)payload[9] << 8 | payload[10]) / 10;
    if (rate > 0) {
      if (rate < MIN_PERIOD_US) rate = rate * (MIN_PERIOD_US / (rate + 1));  // as EdgeTX
      if (rate > MAX_PERIOD_US) rate = MAX_PERIOD_US;
      if (rate != g_sync_rate_us) Serial.printf("[crsf] module sync: %lu us/frame\n", (unsigned long)rate);
      g_sync_rate_us = rate;
      g_sync_lag_us = lag;
      g_sync_ms = millis();
    }
  }

  portENTER_CRITICAL(&g_mux);
  g_link.rx_frames++;
  if (type == TYPE_LINK_STATISTICS && plen >= 10) {
    // CRSF LinkStatistics: [0]up RSSI ant1 [1]ant2 [2]up LQ [3]up SNR [4]active ant
    // [5]RF mode [6]up TX power idx [7]dn RSSI [8]dn LQ [9]dn SNR.
    g_link.up_rssi = rssiDbm(payload[0]);   // uplink RSSI (dBm)
    g_link.up_lq = payload[2];              // uplink link quality (%)
    g_link.up_snr = (int8_t)payload[3];
    g_link.ant = payload[4];
    g_link.rf_mode = payload[5];
    g_link.tx_power = payload[6];
    g_link.dn_rssi = rssiDbm(payload[7]);
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

  // Link log, once a second while link stats arrive.
  static uint32_t s_link_log_ms = 0;
  if (type == TYPE_LINK_STATISTICS && plen >= 10 && millis() - s_link_log_ms >= 1000) {
    s_link_log_ms = millis();
    Serial.printf("[link] LQ %u/%u RSSI %d/%d SNR %d/%d rf=%u ant=%u pw=%u\n",
                  payload[2], payload[8], rssiDbm(payload[0]), rssiDbm(payload[7]),
                  (int8_t)payload[3], (int8_t)payload[9], payload[5], payload[4], payload[6]);
  }

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
      for (int i = 0; i < plen && i < 64; ++i) Serial.printf(" %02X", payload[i]);
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

// ---- ELRS parameter discovery/write (blocking; used with the CRSF task suspended) ----

// Send an extended-header request: [sync][len][type][dest][origin][args...][crc].
void paramSendExt(uint8_t type, uint8_t dest, const uint8_t* args, int nargs) {
  uint8_t f[16];
  f[0] = ADDR_MODULE;
  f[1] = (uint8_t)(3 + nargs + 1);  // type + dest + origin + args + crc
  f[2] = type;
  f[3] = dest;
  f[4] = ADDR_RADIO;  // origin = radio handset (module validates this)
  for (int i = 0; i < nargs; ++i) f[5 + i] = args[i];
  f[5 + nargs] = crc8_dvb_s2(&f[2], (int)f[1] - 1);
  uart.write(f, 6 + nargs);
  uart.flush();
}

// Block until a CRSF frame of `wantType` arrives (skipping sync/echo/other). Returns the
// payload length (bytes after type, minus CRC) into payload[], or -1 on timeout.
int paramRecv(uint8_t wantType, uint8_t* payload, int maxlen, uint32_t timeout_ms) {
  uint32_t start = millis();
  uint8_t buf[64];
  int len = 0, total = -1;
  while ((millis() - start) < timeout_ms) {
    if (!uart.available()) {
      vTaskDelay(1);
      continue;
    }
    uint8_t b = (uint8_t)uart.read();
    if (len == 0) {
      if (b != ADDR_RADIO && b != ADDR_FC && b != ADDR_MODULE) continue;  // resync on addr
      buf[len++] = b;
    } else if (len == 1) {
      if (b < 2 || b > 62) {
        len = 0;
        continue;
      }
      buf[len++] = b;
      total = buf[1] + 2;
    } else {
      buf[len++] = b;
      if (len >= total) {
        if (crc8_dvb_s2(&buf[2], buf[1] - 1) == buf[total - 1] && buf[2] == wantType) {
          int plen = buf[1] - 2;
          if (plen > maxlen) plen = maxlen;
          memcpy(payload, &buf[3], plen);
          return plen;
        }
        len = 0;
        total = -1;
      }
      if (len >= (int)sizeof(buf)) {
        len = 0;
        total = -1;
      }
    }
  }
  return -1;
}

// Ping the module and read the parameter count from its DEVICE_INFO reply.
bool paramDeviceInfo(uint8_t* count) {
  for (int attempt = 0; attempt < 4; ++attempt) {
    while (uart.available()) uart.read();
    paramSendExt(TYPE_DEVICE_PING, ADDR_BROADCAST, nullptr, 0);
    uint8_t p[64];
    int plen = paramRecv(TYPE_DEVICE_INFO, p, sizeof(p), 400);
    if (plen >= 4) {
      // payload: [dest][origin][name\0][serial4][hw4][sw4][param_count][param_ver]
      *count = p[plen - 2];  // second-to-last byte
      return true;
    }
  }
  return false;
}

// Read and reassemble one parameter entry's data (from [parent] onward) into data[].
// Returns the reassembled length, or -1 on error/timeout.
int paramReadEntry(uint8_t field, uint8_t* data, int maxlen) {
  int total = 0;
  for (int chunk = 0; chunk < 16; ++chunk) {
    while (uart.available()) uart.read();
    uint8_t args[2] = {field, (uint8_t)chunk};
    paramSendExt(TYPE_PARAM_READ, ADDR_MODULE, args, 2);
    uint8_t p[64];
    int plen = paramRecv(TYPE_PARAM_ENTRY, p, sizeof(p), 400);
    if (plen < 4 || p[2] != field) return -1;  // p = [dest][origin][field][remaining][data..]
    uint8_t remaining = p[3];
    int dlen = plen - 4;
    if (dlen < 0) dlen = 0;
    if (total + dlen > maxlen) dlen = maxlen - total;
    memcpy(data + total, &p[4], dlen);
    total += dlen;
    if (remaining == 0) return total;
  }
  return -1;
}

// Module menu scan: read chunk 0 of every Lua field (one per 30 ms, interleaved with RC —
// replies only come back in the window after a frame we send) and match names in
// handleFrame. Field count comes from the boot ping; 48 if it never answered.
void fieldScanStart() {
  for (int i = 0; i < EF_N; ++i) g_field[i] = 0;
  g_scan_active = true;
  g_scan_done = false;
  g_scan_next = 1;
  g_scan_pass = 0;
  g_scan_seen = 0;
  g_scan_due = millis();
}

// Returns true while scanning (it owns the command slot).
bool fieldScanStep() {
  if (!g_scan_active) return false;
  uint32_t now = millis();
  if ((int32_t)(now - g_scan_due) < 0) return true;
  int last = g_param_count > 0 ? g_param_count : 48;
  // Skip fields that already answered (on retry passes).
  while (g_scan_next <= last && g_scan_next < 64 && (g_scan_seen >> g_scan_next & 1)) g_scan_next++;
  if (g_scan_next <= last) {
    g_read_field = g_scan_next++;
    g_scan_due = now + 30;
    return true;
  }
  // End of a pass: wait for late replies, then retry the gaps (up to 3 passes), if the
  // module answered at all (no ping reply = unknown count = don't retry 48 blanks).
  if ((int32_t)(now - g_scan_due) < 300) return true;
  bool gaps = false;
  for (int f = 1; f <= last && f < 64; ++f)
    if (!(g_scan_seen >> f & 1)) gaps = true;
  if (gaps && g_param_count > 0 && ++g_scan_pass < 3) {
    g_scan_next = 1;
    return true;
  }
  g_scan_active = false;
  g_scan_done = true;
  Serial.printf("[crsf] module fields: rate=%d switch=%d power=%d fan=%d bind=%d tlm=%d mm=%d\n",
                g_field[EF_PKT_RATE], g_field[EF_SWITCH_MODE], g_field[EF_MAX_POWER],
                g_field[EF_FAN], g_field[EF_BIND], g_field[EF_TLM_RATIO], g_field[EF_MODEL_MATCH]);
  return false;
}

// Called every task tick. Once the module first answers (it has locked our baud) or after
// 3 s, whichever comes first: announce our model ID (ELRS holds RF until a handset sends
// one, and keeps its settings per model), then queue the next stored setting each time the
// previous write has gone out and its spacing has elapsed.
void restoreStep() {
  if (!g_tx_enabled) return;
  uint32_t now = millis();

  // Module reset (power blip, its own reboot, WiFi exit): silent > 500 ms, then back.
  // It comes back on model 0 with RF held for a model ID — re-announce and re-restore.
  bool heard = g_last_rx_ms != 0 && now - g_last_rx_ms <= MODULE_ALIVE_TIMEOUT_MS;
  if (g_module_alive && !heard) {
    g_module_alive = false;
  } else if (!g_module_alive && heard) {
    g_module_alive = true;
    if (g_restore_due != 0) {  // not the first contact after boot
      Serial.println("[crsf] module came back — re-sending model ID + settings");
      g_restore_next = 0;
      g_restore_due = 0;
      g_scan_done = false;  // it may have been reflashed: look the fields up again
      g_param_count = 0;
      g_restore_done = false;
    }
  }
  portENTER_CRITICAL(&g_mux);
  uint32_t rx = g_link.rx_frames;
  bool busy = g_power_repeat > 0 || g_cmd_repeat > 0;
  portEXIT_CRITICAL(&g_mux);
  if (g_restore_due == 0) {
    if (rx == 0 && now - g_start_ms < 3000) return;
    queueCommand(ADDR_MODULE, CMD_CRSF_MODEL_ID, MODEL_ID, 3);
    g_ping_due = now + 150;  // then ask who it is (logs name + ELRS version + field count)
    Serial.printf("[crsf] model ID %u sent\n", MODEL_ID);
    g_restore_due = now + 300;
    return;
  }
  if (!g_scan_done) {  // settings go to fields found by name, so scan the menu first
    if (!g_scan_active && (int32_t)(now - g_restore_due) >= 0) fieldScanStart();
    return;
  }

  constexpr int N = sizeof(RESTORE_ORDER) / sizeof(RESTORE_ORDER[0]);
  while (g_restore_next < N && g_restore[RESTORE_ORDER[g_restore_next]] == 0xFF)
    g_restore_next++;  // skip settings never chosen
  if (g_restore_next >= N) {
    if (!g_restore_done) {  // all written: read back what the module actually took
      g_restore_done = true;
      g_mr_pending |= READBACK_FIELDS;
      g_mr_due = now + 500;
    }
    return;
  }
  if (busy || (int32_t)(now - g_restore_due) < 0) return;

  ElrsSetting s = RESTORE_ORDER[g_restore_next++];
  queueSettingWrite(s, g_restore[s]);
  Serial.printf("[crsf] restore %s = %u\n", RESTORE_NAME[s], g_restore[s]);
  // A packet-rate change re-inits the radio; give it longer before the next write.
  g_restore_due = now + (s == ELRS_PKT_RATE ? 1000 : 400);
}

// Software bind (crsf_bind). Two module-side paths: ELRS >= 3.4 accepts the CRSF bind
// command; ELRS 3.0-3.3 ignore it, but every version has a "Bind" command in its Lua menu,
// "clicked" by a param write of 1 (lcsClick). Its field id depends on the build, so the
// first bind reads the module's fields (PARAM_READ, interleaved with RC — replies only come
// back in the window after each frame we send) and finds the one named "Bind".
// The rest, the EdgeTX way. ELRS's module-side bind command only makes the
// module broadcast its UID for ~0.3 s (6 bind packets, then it exits) — the receiver must
// already be listening in bind mode. So: if a receiver is linked, first send the bind
// command addressed to the RECEIVER (0xEC); the module relays it over the air and the RX
// drops into bind mode (ELRS lua.cpp). Once the link drops, put the module into bind a
// few times, spaced out, until the link comes back.
volatile int g_bind_state = BIND_IDLE;
uint32_t g_bind_t0 = 0, g_bind_next = 0;
int g_bind_tries = 0;

bool linkUp(uint32_t now, uint32_t maxAgeMs) {
  portENTER_CRITICAL(&g_mux);
  bool up = g_link.valid && now - g_linkstats_ms < maxAgeMs;
  portEXIT_CRITICAL(&g_mux);
  return up;
}

// Returns true while a bind is in progress (it owns the command slot).
bool bindStep() {
  int st = g_bind_state;
  if (st != BIND_SCAN && st != BIND_RX && st != BIND_TX) return false;
  uint32_t now = millis();
  if (st == BIND_SCAN) {  // wait for the module menu scan (started at boot)
    if (!g_scan_active && !g_scan_done) fieldScanStart();
    if (g_scan_done) {
      if (!g_field[EF_BIND]) Serial.println("[crsf] bind: no Bind field, using CRSF bind command");
      bool linked = linkUp(now, 1000);
      g_bind_state = linked ? BIND_RX : BIND_TX;  // a linked RX is told to bind first
      g_bind_t0 = now;
      g_bind_next = now;
      g_bind_tries = 0;
      Serial.printf("[crsf] bind: %s\n", linked ? "RX linked, RX first" : "module only");
    }
    return true;
  }
  if (st == BIND_RX) {
    if (!linkUp(now, 1000) || now - g_bind_t0 > 5000) {  // RX left the link (or ignored us)
      Serial.printf("[crsf] bind: %s -> module bind\n",
                    linkUp(now, 1000) ? "RX still linked" : "RX in bind mode");
      g_bind_state = BIND_TX;
      g_bind_t0 = now;
      g_bind_next = now + 300;
      g_bind_tries = 0;
    } else if ((int32_t)(now - g_bind_next) >= 0) {
      queueCommand(ADDR_RX, CMD_CRSF_BIND, -1, 1);
      g_bind_next = now + 300;
    }
    return true;
  }
  // BIND_TX
  if (g_bind_tries > 0 && linkUp(now, 500) && g_linkstats_ms - g_bind_t0 > 500) {
    Serial.println("[crsf] bind: receiver linked");
    g_bind_state = BIND_DONE;
    return false;
  }
  if ((int32_t)(now - g_bind_next) >= 0) {
    if (g_bind_tries >= 4) {
      Serial.println("[crsf] bind: no receiver answered");
      g_bind_state = BIND_FAILED;
      return false;
    }
    if (g_field[EF_BIND])
      queueParamWriteRaw(g_field[EF_BIND], 1);  // "click" the module's Lua Bind command
    else
      queueCommand(ADDR_MODULE, CMD_CRSF_BIND, -1, 3);
    g_bind_tries++;
    Serial.printf("[crsf] bind: module bind #%d\n", g_bind_tries);
    g_bind_next = now + 2500;
  }
  return true;
}

// Decode one reassembled Lua entry ([parent][type][name\0][type data]) into name + value text.
void rxDecodeEntry(int field, const uint8_t* b, int len) {
  if (len < 3 || g_rx_n >= RX_MAX_PARAMS) return;
  CrsfRxParam& e = g_rx_params[g_rx_n];
  e.field = field;
  e.type = b[1] & 0x3F;
  e.hidden = b[1] & 0x80;
  const char* name = (const char*)&b[2];
  int nlen = strnlen(name, len - 2);
  snprintf(e.name, sizeof(e.name), "%.*s", nlen, name);
  const uint8_t* d = b + 2 + nlen + 1;  // type-specific data
  int dl = len - (2 + nlen + 1);
  e.value[0] = 0;
  switch (e.type) {
    case 9: {  // TEXT_SELECTION: options\0 value min max default units\0
      int olen = dl > 0 ? strnlen((const char*)d, dl) : 0;
      if (olen + 1 < dl) {
        int v = d[olen + 1], i = 0;
        const char* o = (const char*)d;
        const char* end = o + olen;
        while (i < v && o < end) {  // walk to option v
          const char* semi = (const char*)memchr(o, ';', end - o);
          if (!semi) { o = end; break; }
          o = semi + 1;
          i++;
        }
        const char* semi = (const char*)memchr(o, ';', end - o);
        int ol = (semi ? semi : end) - o;
        snprintf(e.value, sizeof(e.value), "%.*s", ol, o);
        if (!e.value[0]) snprintf(e.value, sizeof(e.value), "#%d", v);
      }
      break;
    }
    case 0: case 1:  // UINT8 / INT8: value min max default units
      if (dl >= 1) snprintf(e.value, sizeof(e.value), "%d", e.type ? (int)(int8_t)d[0] : (int)d[0]);
      break;
    case 2: case 3:  // UINT16 / INT16, big-endian
      if (dl >= 2) {
        int v = (d[0] << 8) | d[1];
        snprintf(e.value, sizeof(e.value), "%d", e.type == 3 ? (int)(int16_t)v : v);
      }
      break;
    case 10: case 12:  // STRING / INFO
      if (dl > 0) snprintf(e.value, sizeof(e.value), "%.*s", (int)strnlen((const char*)d, dl), d);
      break;
    case 11: snprintf(e.value, sizeof(e.value), "[folder]"); break;
    case 13: snprintf(e.value, sizeof(e.value), "[command]"); break;
    default: snprintf(e.value, sizeof(e.value), "type %d", e.type); break;
  }
  Serial.printf("[crsf] rx %2d %-18s = %s%s\n", field, e.name, e.value, e.hidden ? " (hidden)" : "");
  g_rx_n = g_rx_n + 1;
}

// Returns true when it queued a request this tick (it owns the read slot).
bool rxReadStep() {
  if (g_rx_state != RXR_READING) return false;
  uint32_t now = millis();
  if (g_rx_got) {  // a chunk arrived: ask for the next one, or decode the finished entry
    g_rx_got = false;
    g_rx_tries = 0;
    if (g_rx_remain > 0) {
      g_rx_chunk++;
    } else {
      rxDecodeEntry(g_rx_field, g_rx_buf, g_rx_len);
      g_rx_field = g_rx_field + 1;
      g_rx_chunk = 0;
      g_rx_len = 0;
    }
    g_rx_due = now;
  }
  if (g_rx_param_count && g_rx_field > g_rx_param_count) {
    g_rx_state = RXR_DONE;
    Serial.printf("[crsf] rx menu read done (%d entries)\n", (int)g_rx_n);
    return false;
  }
  if ((int32_t)(now - g_rx_due) < 0) return false;
  if (!g_rx_param_count) {  // don't know the RX yet: ping it (broadcast) first
    if (g_rx_tries >= 4) {
      g_rx_state = RXR_NO_REPLY;
      Serial.println("[crsf] rx menu read: receiver doesn't answer (not linked?)");
      return false;
    }
    g_rx_tries++;
    g_read_field = -2;
    g_rx_due = now + 1500;
    return true;
  }
  if (g_rx_tries >= 3) {  // this chunk never came: skip the field
    Serial.printf("[crsf] rx %2d (no reply)\n", (int)g_rx_field);
    g_rx_field = g_rx_field + 1;
    g_rx_chunk = 0;
    g_rx_len = 0;
    g_rx_tries = 0;
  }
  g_rx_tries++;
  g_read_field = g_rx_field;
  g_read_chunk = g_rx_chunk;
  g_read_dest = ADDR_RX;
  g_rx_due = now + 2000;  // over the air + telemetry slots: be patient
  return true;
}

// Decode a reassembled TEXT_SELECTION entry ([parent][type][name\0][options\0][value]...).
void modDecode(int ef, const uint8_t* b, int len) {
  if (len < 3 || (b[1] & 0x3F) != 9) return;
  int nlen = strnlen((const char*)&b[2], len - 2);
  const uint8_t* d = b + 2 + nlen + 1;
  int dl = len - (2 + nlen + 1);
  int olen = dl > 0 ? strnlen((const char*)d, dl) : 0;
  if (olen + 1 >= dl) return;
  portENTER_CRITICAL(&g_mux);
  ModSel& m = g_msel[ef];
  snprintf(m.options, sizeof(m.options), "%.*s", olen, (const char*)d);
  m.value = d[olen + 1];
  m.valid = true;
  portEXIT_CRITICAL(&g_mux);
  Serial.printf("[crsf] module %s = %u (%.*s)\n", FIELD_NAME[ef], d[olen + 1], olen, (const char*)d);
}

// Returns true when it queued a read this tick (it owns the read slot).
bool modReadStep() {
  if (!g_scan_done || g_scan_active) return false;
  uint32_t now = millis();
  if (g_mr_ef < 0) {
    if (!g_mr_pending || (int32_t)(now - g_mr_due) < 0) return false;
    int ef = __builtin_ctz(g_mr_pending);
    g_mr_pending &= ~(1 << ef);
    if (!g_field[ef]) return false;  // module doesn't have it
    g_mr_ef = ef;
    g_mr_chunk = 0;
    g_mr_len = 0;
    g_mr_tries = 0;
    g_mr_got = false;
  } else if (g_mr_got) {  // a chunk arrived: next chunk, or decode the finished entry
    g_mr_got = false;
    g_mr_tries = 0;
    if (g_mr_remain == 0) {
      modDecode(g_mr_ef, g_mr_buf, g_mr_len);
      g_mr_ef = -1;
      return false;
    }
    g_mr_chunk++;
  } else if (now - g_mr_sent < 250) {
    return false;  // waiting for the reply
  } else if (g_mr_tries >= 4) {
    Serial.printf("[crsf] module %s: no reply\n", FIELD_NAME[g_mr_ef]);
    g_mr_ef = -1;
    return false;
  }
  g_mr_tries++;
  g_mr_sent = now;
  g_read_field = g_field[g_mr_ef];
  g_read_chunk = g_mr_chunk;
  g_read_dest = ADDR_MODULE;
  return true;
}

void tickCb(void*) { xTaskNotifyGive(g_task); }

// Next frame period: the module's interval plus the outstanding lag (EdgeTX
// ModuleSyncStatus::getAdjustedRefreshRate), or the fixed fallback.
uint32_t nextPeriodUs() {
  if (g_sync_rate_us == 0 || millis() - g_sync_ms > SYNC_VALID_MS) return FALLBACK_PERIOD_US;
  int32_t p = (int32_t)g_sync_rate_us + g_sync_lag_us;
  if (p < (int32_t)MIN_PERIOD_US) p = MIN_PERIOD_US;
  if (p > (int32_t)MAX_PERIOD_US) p = MAX_PERIOD_US;
  g_sync_lag_us -= p - (int32_t)g_sync_rate_us;
  return (uint32_t)p;
}

void crsfTask(void*) {
  int64_t next = esp_timer_get_time();
  uint32_t probeMs = millis();
  for (;;) {
    // Pending TX-power write? Hold the RC stream for this tick and send the parameter
    // write on a quiet wire instead — interleaving it with RC channel frames corrupts
    // the module's parameter parsing on the shared single wire (ELRS param-traffic rule).
    portENTER_CRITICAL(&g_mux);
    int pwr = (g_power_repeat > 0) ? g_power_idx : -1;
    if (g_power_repeat > 0) g_power_repeat--;
    int cmd = -1;  // a queued command (bind / model ID) goes out after any pending param write
    int cmdArg = g_cmd_arg;
    uint8_t cmdDest = g_cmd_dest;
    if (pwr < 0 && g_cmd_repeat > 0) {
      cmd = g_cmd_sub;
      g_cmd_repeat--;
    }
    portEXIT_CRITICAL(&g_mux);
    if (pwr < 0 && cmd < 0 && g_ping_due && (int32_t)(millis() - g_ping_due) >= 0 &&
        g_cmd_repeat == 0) {
      g_ping_due = 0;
      g_read_field = -2;  // sentinel: device ping in the read slot
    }
    // Menu scan, then bind, then model ID + settings.
    if (pwr < 0 && cmd < 0 && !fieldScanStep() && !bindStep() && !rxReadStep() &&
        !modReadStep())
      restoreStep();
    int rd = -1;
    if (pwr < 0 && cmd < 0) {
      rd = g_read_field;
      g_read_field = -1;
    }
    bool ping = rd == -2;

    bool special = pwr >= 0 || cmd >= 0 || rd >= 0 || ping;  // param write / command / read slot
    if (!g_tx_enabled) {
      // Output disabled from the menu: transmit nothing (module fails safe). Still drain
      // RX so telemetry/state stays current if the module is still sending.
    } else if (pwr >= 0) {
      sendPowerWrite((uint8_t)pwr);
    } else if (cmd >= 0) {
      sendCommand(cmdDest, (uint8_t)cmd, cmdArg);
    } else if (ping) {
      paramSendExt(TYPE_DEVICE_PING, ADDR_BROADCAST, nullptr, 0);
    } else if (rd >= 0) {
      uint8_t args[2] = {(uint8_t)rd, g_read_chunk};  // field, chunk (chunk 0 has the name)
      paramSendExt(TYPE_PARAM_READ, g_read_dest, args, 2);
      g_read_dest = ADDR_MODULE;
      g_read_chunk = 0;
    } else {
      sendRcFrame();
      portENTER_CRITICAL(&g_mux);
      g_link.tx_frames++;
      portEXIT_CRITICAL(&g_mux);

      // Probe: once a second, ping the module and report the RX frame-type tally.
      if (g_probe && millis() - probeMs >= 1000) {
        probeMs = millis();
        sendPing();
        Serial.printf("[probe] 1s tally  sync=%u link=%u echo=%u INFO=%u other=%u\n", g_pc_sync,
                      g_pc_link, g_pc_echo, g_pc_info, g_pc_other);
        g_pc_sync = g_pc_link = g_pc_echo = g_pc_info = g_pc_other = 0;
      }
    }

    // Drain everything the module (and our echo) put on the wire.
    while (uart.available()) feed((uint8_t)uart.read());

    // Absolute schedule (independent of TX/parse time). Param-write/command slots keep the
    // proven fixed 4 ms spacing (the module needs ~24 ms of quiet wire for 6 copies).
    next += special ? FALLBACK_PERIOD_US : nextPeriodUs();
    int64_t wait = next - esp_timer_get_time();
    if (wait < 50) {  // fell behind (or resumed after a pause): restart the schedule
      wait = 50;
      next = esp_timer_get_time() + wait;
    }
    esp_timer_stop(g_tick_timer);  // no-op unless a previous wait timed out
    esp_timer_start_once(g_tick_timer, (uint64_t)wait);
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
  }
}

}  // namespace

void crsf_set_restore(const uint8_t values[ELRS_SETTING_N]) {
  for (int i = 0; i < ELRS_SETTING_N; ++i) g_restore[i] = values[i];
  g_restore_next = 0;
  g_restore_due = 0;
}

bool crsf_start() {
  g_start_ms = millis();
  uart.begin(g_crsf_baud, SERIAL_8N1, PIN_CRSF_RX, PIN_CRSF_TX);
  uart.setTimeout(0);

  if (!g_tick_timer) {
    esp_timer_create_args_t args = {};
    args.callback = tickCb;
    args.dispatch_method = ESP_TIMER_TASK;
    args.name = "crsf_tick";
    esp_timer_create(&args, &g_tick_timer);
  }
  BaseType_t ok = xTaskCreatePinnedToCore(crsfTask, "crsf", 4096, nullptr,
                                          /*priority=*/3, &g_task, /*core=*/0);
  if (ok != pdPASS) {
    Serial.println("[crsf] task create FAILED");
    return false;
  }
  Serial.printf("[crsf] UART2 up @ %lu (AETR, synced to module, half-duplex)\n",
                (unsigned long)g_crsf_baud);
  return true;
}

uint32_t crsf_get_baud() { return g_crsf_baud; }

void crsf_set_output(bool on) { g_tx_enabled = on; }
bool crsf_output_enabled() { return g_tx_enabled; }

uint16_t crsf_toggle_mask() {
  SwitchState sw;
  switches_get(sw);
  uint16_t m = 0;
  for (int i = 0; i < 10; ++i)
    if (gconfig_toggle_on(i, sw.bits)) m |= (uint16_t)(1u << i);
  return m;
}

void crsf_set_baud(uint32_t baud) {
  g_crsf_baud = baud;
  // If the link is already up, change the UART live — the module's autobaud re-locks
  // within ~1-2 s (link drops briefly). Before start, this just seeds crsf_start().
  if (g_task) uart.updateBaudRate(baud);
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

// Queue a parameter write the CRSF task sends by holding RC for ~6 ticks (the proven path).
// field = one of the CRSF_FIELD_* defaults (crsf.h); translated to the id the module
// actually uses, found by name at boot. Before the scan finishes the default is used; after
// it, a setting the module doesn't have is skipped rather than written to a wrong field.
void queueParamWrite(int field, int value) {
  for (int i = 0; i < EF_N; ++i) {
    if (FIELD_DEFAULT[i] != field) continue;
    if (g_scan_done) {
      if (!g_field[i]) {
        Serial.printf("[crsf] module has no '%s' — write skipped\n", FIELD_NAME[i]);
        return;
      }
      field = g_field[i];
    }
    break;
  }
  queueParamWriteRaw(field, value);
}

// A write to an exact module field id (no translation).
void queueParamWriteRaw(int field, int value) {
  portENTER_CRITICAL(&g_mux);
  g_write_field = field;
  g_power_idx = value;
  g_power_repeat = 6;
  portEXIT_CRITICAL(&g_mux);
}

// Write a stored setting to the module field found by name (skipped if the module has no
// such field, or the scan hasn't found it yet and there's no safe default id).
void queueSettingWrite(ElrsSetting s, int value) {
  int ef = SETTING_FIELD[s];
  int field = g_scan_done ? g_field[ef] : FIELD_DEFAULT[ef];
  if (!field) {
    Serial.printf("[crsf] module has no '%s' — write skipped\n", FIELD_NAME[ef]);
    return;
  }
  queueParamWriteRaw(field, value);
  if (READBACK_FIELDS >> ef & 1) {  // read back what the module made of it
    g_mr_pending |= 1 << ef;
    g_mr_due = millis() + 600;
  }
}

int crsf_bind_state() { return g_bind_state; }

void crsf_rx_read() {
  g_rx_n = 0;
  g_rx_field = 1;
  g_rx_chunk = 0;
  g_rx_len = 0;
  g_rx_tries = 0;
  g_rx_got = false;
  g_rx_due = millis();
  g_rx_state = RXR_READING;
  Serial.println("[crsf] rx menu read started");
}
int crsf_rx_read_state() { return g_rx_state; }
int crsf_rx_progress(int* total) {
  if (total) *total = g_rx_param_count;
  return g_rx_field;
}
int crsf_rx_params(const CrsfRxParam** out) {
  if (out) *out = g_rx_params;
  return g_rx_n;
}

void crsf_bind() {
  uint32_t now = millis();
  g_bind_t0 = now;
  g_bind_next = now;
  g_bind_tries = 0;
  bool linked = linkUp(now, 1000);
  if (!g_scan_done) {  // the module menu scan hasn't finished yet: wait for it
    g_bind_state = BIND_SCAN;
    Serial.println("[crsf] bind started: waiting for module menu scan");
    return;
  }
  g_bind_state = linked ? BIND_RX : BIND_TX;  // a linked RX is told to bind first
  Serial.printf("[crsf] bind started (%s)\n", linked ? "RX linked: RX first" : "module only");
}

void crsf_set_setting(ElrsSetting s, int value) {
  if (s >= ELRS_SETTING_N || value < 0 || value > 255) return;
  queueSettingWrite(s, value);
}

void crsf_refresh_setting(ElrsSetting s) {
  if (s >= ELRS_SETTING_N || !(READBACK_FIELDS >> SETTING_FIELD[s] & 1)) return;
  g_mr_pending |= 1 << SETTING_FIELD[s];
  g_mr_due = millis();
}

bool crsf_get_setting(ElrsSetting s, int* value, char* options, int optlen) {
  if (s >= ELRS_SETTING_N) return false;
  portENTER_CRITICAL(&g_mux);
  const ModSel& m = g_msel[SETTING_FIELD[s]];
  bool ok = m.valid;
  if (ok) {
    if (value) *value = m.value;
    if (options && optlen > 0) snprintf(options, optlen, "%s", m.options);
  }
  portEXIT_CRITICAL(&g_mux);
  return ok;
}

void crsf_set_power(int level_index) {
  if (level_index < 0 || level_index > 7) return;
  queueParamWrite(POWER_FIELD_ID, level_index);
}

void crsf_param_set(int field, int value) {
  if (field < 1 || field > 255 || value < 0 || value > 255) return;
  queueParamWrite(field, value);
}

bool crsf_param_scan(const char* name_substr, CrsfParam& out) {
  out.field = -1;
  out.value = 0;
  out.max = 0;
  out.options[0] = 0;
  out.dbg_count = 0;
  out.dbg_read = 0;

  // Suspend RC so the whole ping/read sequence runs on a quiet wire.
  bool suspended = false;
  if (g_task) {
    vTaskSuspend(g_task);
    suspended = true;
    vTaskDelay(pdMS_TO_TICKS(5));  // let any in-flight TX/echo drain
  }
  uart.flush();

  uint8_t count = 0;
  if (paramDeviceInfo(&count) && count > 0 && count < 64) {
    out.dbg_count = count;
    uint8_t data[256];
    for (uint8_t field = 1; field <= count && out.field < 0; ++field) {
      int dlen = paramReadEntry(field, data, sizeof(data) - 1);
      if (dlen < 3) continue;
      out.dbg_read++;
      data[dlen] = 0;  // ensure strings are terminated
      // Reassembled entry: [0]parent [1]type [2..]name\0 [then type-specific data].
      const char* pname = (const char*)&data[2];
      if (!strstr(pname, name_substr)) continue;
      out.field = field;
      // Text-selection payload after the name: options\0 value min max default unit\0.
      int o = 2 + (int)strlen(pname) + 1;
      if (o < dlen) {
        strncpy(out.options, (const char*)&data[o], sizeof(out.options) - 1);
        out.options[sizeof(out.options) - 1] = 0;
        int vpos = o + (int)strlen((const char*)&data[o]) + 1;
        if (vpos + 2 < dlen) {
          out.value = data[vpos];
          out.max = data[vpos + 2];
        }
      }
    }
  }

  if (suspended) vTaskResume(g_task);
  return out.field >= 0;
}

bool crsf_param_write(int field, uint8_t value) {
  if (field < 1 || field > 255) return false;
  bool suspended = false;
  if (g_task) {
    vTaskSuspend(g_task);
    suspended = true;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  uart.flush();
  while (uart.available()) uart.read();
  uint8_t args[2] = {(uint8_t)field, value};
  for (int i = 0; i < 4; ++i) {  // send a few times on the quiet wire
    paramSendExt(TYPE_PARAM_WRITE, ADDR_MODULE, args, 2);
    vTaskDelay(pdMS_TO_TICKS(4));
  }
  if (suspended) vTaskResume(g_task);
  return true;
}

uint16_t crsf_tx_power_mw(uint8_t idx) {
  // Standard CRSF power-index table.
  static const uint16_t P[] = {0, 10, 25, 100, 500, 1000, 2000, 250, 50};
  return idx < (sizeof(P) / sizeof(P[0])) ? P[idx] : 0;
}


namespace {
// One pass of the three probes (see crsf.h). Pins must already be plain GPIOs.
void wireProbe(CrsfWireTest& r) {
  const gpio_num_t rx = (gpio_num_t)PIN_CRSF_RX, tx = (gpio_num_t)PIN_CRSF_TX;
  constexpr int SETTLE_US = 300;  // >> RC of the 45k pulls into the pad + wiring capacitance

  // idle: TX hi-Z, RX pulled each way.
  gpio_set_direction(tx, GPIO_MODE_INPUT);
  gpio_set_pull_mode(tx, GPIO_FLOATING);
  gpio_set_direction(rx, GPIO_MODE_INPUT);
  gpio_set_pull_mode(rx, GPIO_PULLUP_ONLY);
  delayMicroseconds(SETTLE_US);
  r.idle_pu = gpio_get_level(rx);
  gpio_set_pull_mode(rx, GPIO_PULLDOWN_ONLY);
  delayMicroseconds(SETTLE_US);
  r.idle_pd = gpio_get_level(rx);

  // loopback: TX drives, RX pulled the opposite way so a floating RX can't pass by luck.
  gpio_set_level(tx, 1);
  gpio_set_direction(tx, GPIO_MODE_OUTPUT);
  delayMicroseconds(SETTLE_US);  // RX still pulled down
  r.loop_hi = gpio_get_level(rx);
  gpio_set_pull_mode(rx, GPIO_PULLUP_ONLY);
  gpio_set_level(tx, 0);
  delayMicroseconds(SETTLE_US);
  r.loop_lo = gpio_get_level(rx);

  // contend: only when a path exists, at the weakest drive so a real short stays well
  // inside the pin ratings for the few hundred microseconds it lasts.
  r.hold_a = r.hold_b = false;
  if (r.loop_hi && !r.loop_lo) {
    gpio_set_pull_mode(rx, GPIO_FLOATING);
    gpio_set_drive_capability(rx, GPIO_DRIVE_CAP_0);
    gpio_set_drive_capability(tx, GPIO_DRIVE_CAP_0);
    gpio_set_direction(tx, GPIO_MODE_INPUT_OUTPUT);  // input stage on: read back own level
    gpio_set_level(rx, 1);
    gpio_set_direction(rx, GPIO_MODE_INPUT_OUTPUT);  // TX is still LOW
    delayMicroseconds(SETTLE_US);
    r.hold_a = gpio_get_level(rx) == 1 && gpio_get_level(tx) == 0;
    gpio_set_level(rx, 0);
    gpio_set_level(tx, 1);
    delayMicroseconds(SETTLE_US);
    r.hold_b = gpio_get_level(rx) == 0 && gpio_get_level(tx) == 1;
  }

  // Release both pins.
  gpio_set_direction(rx, GPIO_MODE_INPUT);
  gpio_set_direction(tx, GPIO_MODE_INPUT);
  gpio_set_pull_mode(rx, GPIO_FLOATING);
  gpio_set_drive_capability(rx, GPIO_DRIVE_CAP_DEFAULT);
  gpio_set_drive_capability(tx, GPIO_DRIVE_CAP_DEFAULT);
}
}  // namespace

CrsfWireTest crsf_wire_test() {
  // Take the pins away from UART2 (the task is parked first so nothing is mid-frame).
  const bool was_up = (g_task != nullptr);
  if (was_up) {
    crsf_pause();
    uart.end();
  }
  gpio_reset_pin((gpio_num_t)PIN_CRSF_RX);  // back to plain GPIO function
  gpio_reset_pin((gpio_num_t)PIN_CRSF_TX);

  // Several passes: a module talking on the wire (or noise) shows up as disagreement.
  constexpr int PASSES = 5;
  CrsfWireTest r = {};
  bool stable = true;
  for (int i = 0; i < PASSES; ++i) {
    CrsfWireTest p = {};
    wireProbe(p);
    if (i == 0) {
      r = p;
    } else if (p.idle_pu != r.idle_pu || p.idle_pd != r.idle_pd || p.loop_hi != r.loop_hi ||
               p.loop_lo != r.loop_lo || p.hold_a != r.hold_a || p.hold_b != r.hold_b) {
      stable = false;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  if (!stable) r.verdict = CrsfWire::Busy;
  else if (!r.idle_pu) r.verdict = CrsfWire::HeldLow;  // nothing we drive can be trusted
  else if (!(r.loop_hi && !r.loop_lo)) r.verdict = CrsfWire::Open;
  else if (!(r.hold_a && r.hold_b)) r.verdict = CrsfWire::Short;
  else r.verdict = CrsfWire::Ok;

  if (was_up) {  // hand the pins back to UART2 exactly as crsf_start() set them up
    uart.begin(g_crsf_baud, SERIAL_8N1, PIN_CRSF_RX, PIN_CRSF_TX);
    uart.setTimeout(0);
    crsf_resume();
  }
  return r;
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
