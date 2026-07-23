// main.cpp — task creation and core pinning.
//
// The firmware is built as FreeRTOS tasks (see the RTOS architecture table in
// CLAUDE.md). setup() starts only the app/UI controller task; that task runs the
// startup sequence (boot screen -> module init -> throttle gate -> mode select) and
// then owns the active screen. The per-module tasks are spawned by the app task as it
// steps through the init sequence.
#include <Arduino.h>

#include "ui.h"

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[boot] BertRCController");

  if (!ui_start()) {
    Serial.println("[boot] FAILED to start app task");
  }
}

void loop() {
  // Nothing here on purpose — work happens in tasks.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
