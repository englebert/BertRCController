// main.cpp — task creation and core pinning.
//
// The firmware is built as FreeRTOS tasks (see the RTOS architecture table in
// CLAUDE.md). setup() wires up peripherals and spawns each module's task, then the
// Arduino loop() is left idle — all real work lives in tasks.
//
// Modules brought up so far:
//   [x] Display / UI     (core 0, low priority)
//   [x] Power / INA226   (core 0, low priority)
//   [x] Switches/MCP23017 (core 0, low priority)
//   [x] Gimbal sampling  (core 1, highest)
//   [x] CRSF / ELRS      (core 0, medium)
#include <Arduino.h>

#include "crsf.h"
#include "gimbals.h"
#include "power.h"
#include "switches.h"
#include "ui.h"

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[boot] BertRCController");

  if (!power_start()) {
    Serial.println("[boot] FAILED to start power task");
  }
  if (!switches_start()) {
    Serial.println("[boot] FAILED to start switches task");
  }
  if (!gimbals_start()) {
    Serial.println("[boot] FAILED to start gimbals task");
  }
  if (!crsf_start()) {
    Serial.println("[boot] FAILED to start crsf task");
  }
  if (!ui_start()) {
    Serial.println("[boot] FAILED to start UI task");
  }
}

void loop() {
  // Nothing here on purpose — work happens in tasks.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
