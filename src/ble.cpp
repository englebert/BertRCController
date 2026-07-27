// ble.cpp — BLE HID gamepad: 4 gimbal axes + 10 toggle buttons, for FPV simulators.
#include "ble.h"

#include <Arduino.h>
#include <BleGamepad.h>

#include "crsf.h"  // crsf_toggle_mask()
#include "gconfig.h"
#include "gimbals.h"

namespace {

BleGamepad bleGamepad("BertRC", "NaMiQuad", 100);  // device name, manufacturer, battery %
volatile bool g_on = false;
bool g_started = false;
TaskHandle_t g_task = nullptr;

constexpr int AXIS_MAX = 32767;

// Calibrated + flip-aware gimbal value (via the CRSF map, 172..1811) scaled to 0..AXIS_MAX.
int16_t axisFromGimbal(int idx, int raw) {
  uint16_t c = gconfig_map_crsf(idx, raw);  // 172..1811
  long a = (long)(c - 172) * AXIS_MAX / (1811 - 172);
  if (a < 0) a = 0;
  if (a > AXIS_MAX) a = AXIS_MAX;
  return (int16_t)a;
}

void bleTask(void*) {
  BleGamepadConfiguration cfg;
  cfg.setAutoReport(false);
  cfg.setControllerType(CONTROLLER_TYPE_GAMEPAD);
  cfg.setButtonCount(10);
  cfg.setWhichAxes(true, true, true, true, false, false, false, false);  // X Y Z RX
  cfg.setWhichSimulationControls(false, false, false, false, false);
  cfg.setHatSwitchCount(0);
  cfg.setAxesMin(0);
  cfg.setAxesMax(AXIS_MAX);
  bleGamepad.begin(&cfg);

  const TickType_t period = pdMS_TO_TICKS(20);  // ~50 Hz report rate
  for (;;) {
    if (g_on && bleGamepad.isConnected()) {
      GimbalState g;
      gimbals_get(g);
      bleGamepad.setX(axisFromGimbal(3, g.roll));      // roll  -> X
      bleGamepad.setY(axisFromGimbal(2, g.pitch));     // pitch -> Y
      bleGamepad.setZ(axisFromGimbal(0, g.throttle));  // throttle -> Z
      bleGamepad.setRX(axisFromGimbal(1, g.yaw));      // yaw -> RX
      uint16_t tm = crsf_toggle_mask();
      for (int i = 0; i < 10; ++i) {
        if (tm & (1u << i)) bleGamepad.press(BUTTON_1 + i);
        else bleGamepad.release(BUTTON_1 + i);
      }
      bleGamepad.sendReport();
    }
    vTaskDelay(period);
  }
}

}  // namespace

void ble_set(bool on) {
  g_on = on;
  if (on && !g_started) {
    g_started = true;
    xTaskCreatePinnedToCore(bleTask, "ble", 8192, nullptr, /*priority=*/2, &g_task, /*core=*/0);
  }
}

bool ble_on() { return g_on; }

bool ble_connected() { return g_started && bleGamepad.isConnected(); }
