// switches.cpp — MCP23017 switch reader (module #3).
#include "switches.h"

#include <Adafruit_MCP23X17.h>
#include <Arduino.h>
#include <Wire.h>

#include "pins.h"

namespace {

constexpr uint8_t MCP23017_ADDR = 0x20;

// DEBUG: expose ALL 16 MCP23017 channels (GPA0-7 = bits 0-7, GPB0-7 = bits 8-15) so
// the UI can show every pin's state. The 10 real switches are pins 0..9; the rest are
// spare. readGPIOAB() returns port A in the low byte and port B in the high byte.
constexpr uint16_t SWITCH_MASK = 0xFFFF;  // all 16 channels
constexpr int CHANNEL_COUNT = 16;

// Debounce: commit a raw sample only after it holds steady for this many polls.
constexpr int DEBOUNCE_POLLS = 3;  // 3 * 20 ms = 60 ms

Adafruit_MCP23X17 mcp;
bool g_present = false;

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
SwitchState g_state = {0, false};

void publish(uint16_t bits, bool valid) {
  portENTER_CRITICAL(&g_mux);
  g_state.bits = bits;
  g_state.valid = valid;
  portEXIT_CRITICAL(&g_mux);
}

void switchesTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(20);  // 50 Hz poll
  uint16_t last_raw = 0xFFFF;
  int stable = 0;
  uint16_t committed_raw = 0xFFFF;  // active-low: all released
  bool have_commit = false;

  for (;;) {
    if (g_present) {
      uint16_t raw = mcp.readGPIOAB();  // active-low on the wire
      if (raw == last_raw) {
        if (stable < DEBOUNCE_POLLS) stable++;
      } else {
        last_raw = raw;
        stable = 0;
      }
      if (stable >= DEBOUNCE_POLLS) {
        committed_raw = raw;
        have_commit = true;
      }
      // Invert to active-high (1 = closed) and keep only the 10 switch bits.
      uint16_t bits = (~committed_raw) & SWITCH_MASK;
      publish(bits, have_commit);
    } else {
      publish(0, false);
    }
    vTaskDelay(period);
  }
}

}  // namespace

bool switches_start() {
  // I2C0 is shared (also INA226). Ensure the bus is up regardless of module order.
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  if (!mcp.begin_I2C(MCP23017_ADDR, &Wire)) {
    Serial.println("[switches] MCP23017 not found at 0x20");
    g_present = false;
  } else {
    for (int p = 0; p < CHANNEL_COUNT; ++p) {
      mcp.pinMode(p, INPUT_PULLUP);  // active-low; pull-ups keep spare pins OFF
    }
    g_present = true;
    Serial.println("[switches] MCP23017 ready (16 channels, active-low)");
  }

  BaseType_t ok = xTaskCreatePinnedToCore(switchesTask, "switches", 3072, nullptr,
                                          /*priority=*/1, nullptr, /*core=*/0);
  return ok == pdPASS;
}

bool switches_get(SwitchState& out) {
  portENTER_CRITICAL(&g_mux);
  out = g_state;
  portEXIT_CRITICAL(&g_mux);
  return out.valid;
}
