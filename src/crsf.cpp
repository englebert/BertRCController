// crsf.cpp — hand-rolled CRSF TX + telemetry parse, half-duplex echo rejection (module #5).
#include "crsf.h"

#include <Arduino.h>

#include "gconfig.h"
#include "gimbals.h"
#include "pins.h"
#include "switches.h"

namespace {

constexpr uint32_t CRSF_BAUD = 400000;
constexpr int TX_RATE_HZ = 500;  // matches the module's F500 packet rate

// CRSF addressing / frame types.
constexpr uint8_t ADDR_MODULE = 0xEE;  // CRSF transmitter (our destination)
constexpr uint8_t ADDR_RADIO = 0xEA;   // radio transmitter (telemetry addressed to us)
constexpr uint8_t ADDR_FC = 0xC8;
constexpr uint8_t ADDR_RX = 0xEC;
constexpr uint8_t TYPE_RC_CHANNELS = 0x16;      // our TX (echo -> ignore on RX)
constexpr uint8_t TYPE_LINK_STATISTICS = 0x14;  // module telemetry we care about

// CRSF 11-bit channel range: 172=min, 992=center, 1811=max.
constexpr uint16_t CRSF_MIN = 172;
constexpr uint16_t CRSF_MID = 992;
constexpr uint16_t CRSF_MAX = 1811;

HardwareSerial& uart = Serial2;
TaskHandle_t g_task = nullptr;

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
CrsfLink g_link = {false, 0, 0, 0, 0, 0};
uint16_t g_channels[16] = {0};  // last sent channel values, guarded by g_mux

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

// idx: 0=T,1=Y,2=P,3=R (for the per-channel reverse/flip flag).
uint16_t adcToCrsf(int16_t adc, int idx) {
  int v = constrain((int)adc, 0, 4095);
  uint16_t out = (uint16_t)(CRSF_MIN + (long)v * (CRSF_MAX - CRSF_MIN) / 4095);
  if (gconfig_flip(idx)) out = (uint16_t)(CRSF_MIN + CRSF_MAX - out);  // reverse
  return out;
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

  // AETR: CH1=Roll, CH2=Pitch, CH3=Throttle, CH4=Yaw.
  ch[0] = adcToCrsf(g.roll, 3);
  ch[1] = adcToCrsf(g.pitch, 2);
  ch[2] = adcToCrsf(g.throttle, 0);
  ch[3] = adcToCrsf(g.yaw, 1);

  // CH5..CH16 default to center, then each toggle drives its assigned channel
  // (configurable in the Toggle Mapping menu; Toggle6 is reserved for menu mode).
  for (int c = 4; c < 16; ++c) ch[c] = CRSF_MID;
  for (int i = 0; i < 10; ++i) {
    uint8_t chan = gconfig_toggle_chan(i);  // 5..16, 0 = unassigned (incl. Toggle6)
    if (chan < 5 || chan > 16) continue;
    ch[chan - 1] = (sw.bits & (1u << TOGGLE_BIT[i])) ? CRSF_MAX : CRSF_MIN;
  }
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
    g_link.up_rssi = -(int16_t)payload[0];  // uplink RSSI (dBm)
    g_link.up_lq = payload[2];              // uplink link quality (%)
    g_link.rf_mode = payload[5];
    g_link.valid = true;
  }
  portEXIT_CRITICAL(&g_mux);
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
  for (;;) {
    sendRcFrame();
    portENTER_CRITICAL(&g_mux);
    g_link.tx_frames++;
    portEXIT_CRITICAL(&g_mux);

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
  Serial.println("[crsf] UART2 up @ 400000 (AETR, 500 Hz, half-duplex)");
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


void crsf_pause() {
  if (!g_task) return;             // task never started (e.g. module-WiFi mode)
  vTaskSuspend(g_task);
  uart.flush();                    // let in-flight TX finish
  vTaskDelay(pdMS_TO_TICKS(10));   // and the echo/telemetry burst drain
}

void crsf_resume() {
  if (g_task) vTaskResume(g_task);
}
