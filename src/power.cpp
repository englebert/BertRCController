// power.cpp — INA226 pack V/I/P monitoring (module #2).
#include "power.h"

#include <Arduino.h>
#include <INA226.h>
#include <Wire.h>

#include "pins.h"

namespace {

constexpr uint8_t INA226_ADDR = 0x44;

// Shunt design (see CLAUDE.md): 0.025 ohm — two 0.05 ohm resistors in PARALLEL to double
// the current range. The theoretical FS is 3.2768 A (81.92 mV), but the library caps
// shunt*maxCurrent at 81.9 mV, so we use 3.274 A (81.85 mV). LSB = 3.274/32768 = 99.9 uA.
constexpr float SHUNT_OHMS = 0.025f;
constexpr float MAX_CURRENT_A = 3.274f;

// Bus-voltage calibration: single-point gain trim so the reading matches a meter.
// Per board: pocket (2S + step-down) meter 7.87 V vs INA226 raw 8.268 V; standard handset
// meter 6.70 V vs 6.21 V. NOTE: this corrects a constant gain error; if the discrepancy is
// actually IR drop it grows with current, so re-check against the meter at your normal
// operating load and retune if needed.
#ifdef POCKETVERSION
constexpr float VBUS_SCALE = 7.87f / 8.268f;  // = 0.9519
#else
constexpr float VBUS_SCALE = 6.70f / 6.21f;  // = 1.0789
#endif
constexpr float FULL_SCALE_mA = MAX_CURRENT_A * 1000.0f;
constexpr float NEAR_FULL_mA = FULL_SCALE_mA * 0.90f;  // flag within ~10% of ceiling

INA226 ina(INA226_ADDR);

// Shared reading, guarded by a spinlock (copies are tiny; no I2C inside the lock).
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
PowerReading g_reading = {0, 0, 0, false, false};

void publish(const PowerReading& r) {
  portENTER_CRITICAL(&g_mux);
  g_reading = r;
  portEXIT_CRITICAL(&g_mux);
}

void powerTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(200);  // ~5 Hz
  for (;;) {
    PowerReading r;
    if (ina.isConnected()) {
      r.voltage_V = ina.getBusVoltage() * VBUS_SCALE;
      r.current_mA = ina.getCurrent_mA();
      r.power_mW = r.voltage_V * r.current_mA;  // P=V*I, consistent with calibrated V
      r.near_full = fabsf(r.current_mA) >= NEAR_FULL_mA;
      r.valid = true;
    } else {
      r = {0, 0, 0, false, false};
    }
    publish(r);
    vTaskDelay(period);
  }
}

}  // namespace

bool power_start() {
  // I2C0 is shared (INA226 + later MCP23017). First module to come up inits the bus.
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  if (!ina.begin()) {
    Serial.println("[power] INA226 not found at 0x44");
    // Still start the task so the UI shows a clear "no sensor" state if it appears later.
  } else {
    // normalize=false keeps a clean ~100 uA current LSB (81.85 mV @ 3.274 A).
    int rc = ina.setMaxCurrentShunt(MAX_CURRENT_A, SHUNT_OHMS, false);
    if (rc != INA226_ERR_NONE) {
      Serial.printf("[power] setMaxCurrentShunt rc=0x%04X\n", rc);
    }
    ina.setAverage(INA226_16_SAMPLES);  // ~35 ms conversion, smooths handset bursts
    Serial.println("[power] INA226 configured (0.025 ohm, FS 3.274 A)");
  }

  BaseType_t ok = xTaskCreatePinnedToCore(powerTask, "power", 3072, nullptr,
                                          /*priority=*/1, nullptr, /*core=*/0);
  return ok == pdPASS;
}

bool power_get(PowerReading& out) {
  portENTER_CRITICAL(&g_mux);
  out = g_reading;
  portEXIT_CRITICAL(&g_mux);
  return out.valid;
}
