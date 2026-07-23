// ui.cpp — app controller task (Phase 1b).
//
// Startup flow: BOOTING UP (3 s) -> module init list -> gimbal calibration wizard
// (first boot only) -> throttle-safety gate (>15% warns until lowered) -> live mode
// loop: Toggle6 ON = configuration mode, OFF = radio-control screen.
// Black background throughout (TASK01.md).
#include "ui.h"

#include <Arduino.h>
#include <TFT_eSPI.h>

#include "crsf.h"
#include "display_cfg.h"
#include "gimbals.h"
#include "pins.h"
#include "power.h"
#include "storage.h"
#include "switches.h"

namespace {

TFT_eSPI tft;  // pins come from build_flags (platformio.ini)

// Battery % mapping (2S): 6.0 V = 0%, 8.4 V = 100%.
constexpr float VBAT_MIN = 6.0f;
constexpr float VBAT_MAX = 8.4f;

// Startup safety: block until throttle is at/below this percent.
constexpr float THROTTLE_GATE_PCT = 15.0f;

// Toggle6 = mode switch (B2 = MCP bit 10). ON (closed) = configuration mode.
constexpr int TOGGLE6_BIT = 10;
// Toggle5 (B1 = MCP bit 9) ON at power-up = force the gimbal calibration wizard.
constexpr int TOGGLE5_BIT = 9;

// Menu-key thresholds (TASK01.md), on the RC microsecond scale 1000..2000 (center
// 1500) derived from calibrated stick percent. Pitch: >1800 = UP, <1200 = DOWN.
// Roll: >1800 = OK, <1200 = CANCEL.
constexpr int KEY_HI_US = 1800;
constexpr int KEY_LO_US = 1200;

GimbalCal g_cal = {};

// NVS write with the interrupt-heavy peripherals quiesced (see crash note in appTask).
bool safeSaveCal(const GimbalCal& cal) {
  crsf_pause();
  gimbals_pause();
  bool ok = storage_save_cal(cal);
  gimbals_resume();
  crsf_resume();
  return ok;
}

void backlightOn() {
  ledcAttach(PIN_TFT_BL, BL_PWM_FREQ, BL_PWM_BITS);  // arduino-esp32 3.x API
  ledcWrite(PIN_TFT_BL, BL_FULL);
}

// ---- Calibrated stick helpers (index: 0=T, 1=Y, 2=P, 3=R) ----
// Piecewise around the stored center: min..mid -> 0..50%, mid..max -> 50..100%.
// Keeps a resting stick at exactly 50% (1500 us) even with asymmetric pot travel.
float calPct(int idx, int raw) {
  if (!g_cal.valid || g_cal.max[idx] <= g_cal.min[idx]) {
    return constrain(raw, 0, 4095) * 100.0f / 4095.0f;  // uncalibrated fallback
  }
  int lo = g_cal.min[idx], hi = g_cal.max[idx], mid = g_cal.mid[idx];
  if (mid <= lo || mid >= hi) mid = (lo + hi) / 2;  // sane fallback
  float p;
  if (raw <= mid) p = (raw - lo) * 50.0f / (mid - lo);
  else p = 50.0f + (raw - mid) * 50.0f / (hi - mid);
  return constrain(p, 0.0f, 100.0f);
}

int calUs(int idx, int raw) { return 1000 + (int)(calPct(idx, raw) * 10.0f + 0.5f); }

// ---- Boot screen ----
void bootScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("BOOTING UP", LCD_W / 2, LCD_H / 2, 4);
}

// ---- Module init sequence ----
const char* const MOD_NAME[4] = {"INA226", "MCP23017", "Gimbals", "CRSF"};
int rowY(int i) { return 9 + i * 17; }  // 4 rows in 76 px

void drawInitList() {
  tft.fillScreen(TFT_BLACK);
  for (int i = 0; i < 4; ++i) {
    tft.setTextPadding(0);
    tft.setTextDatum(ML_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(MOD_NAME[i], 4, rowY(i), 2);
    tft.setTextDatum(MR_DATUM);
    tft.setTextPadding(tft.textWidth("FAIL", 2));
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.drawString("...", LCD_W - 4, rowY(i), 2);
  }
}

void setStatus(int i, bool ok) {
  tft.setTextDatum(MR_DATUM);
  tft.setTextPadding(tft.textWidth("FAIL", 2));
  tft.setTextColor(ok ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.drawString(ok ? "OK" : "FAIL", LCD_W - 4, rowY(i), 2);
}

void runInit() {
  drawInitList();
  vTaskDelay(pdMS_TO_TICKS(250));
  setStatus(0, power_start());
  vTaskDelay(pdMS_TO_TICKS(350));
  setStatus(1, switches_start());
  vTaskDelay(pdMS_TO_TICKS(350));
  setStatus(2, gimbals_start());
  vTaskDelay(pdMS_TO_TICKS(350));
  setStatus(3, crsf_start());
  vTaskDelay(pdMS_TO_TICKS(700));
}

// ---- Gimbal calibration wizard (first boot / no stored cal) ----
// No timer: keeps capturing until every stick has covered MIN_SPAN. Live per-stick
// cells show captured span (red -> green), so a dead/under-ranging pot is obvious.
void calibrationWizard() {
  constexpr int TICK_MS = 25;
  // Must beat electrical noise (seen up to ~400 p-p on a marginal harness); a real
  // full-stick sweep spans ~2000+.
  constexpr int MIN_SPAN = 800;
  const char* const NAME[4] = {"THR", "YAW", "PIT", "ROL"};

  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("CALIBRATE: circle sticks around corners", LCD_W / 2, 8, 1);

  uint16_t mn[4] = {4095, 4095, 4095, 4095};
  uint16_t mx[4] = {0, 0, 0, 0};

  // 4 cells across: label + live span, green border once that stick passes.
  const int cw = 66, gap = 4;
  const int x0 = (LCD_W - 4 * cw - 3 * gap) / 2;
  const int cy = 20, chh = 44;
  bool done_cell[4] = {false, false, false, false};
  for (int i = 0; i < 4; ++i) {
    int x = x0 + i * (cw + gap);
    tft.drawRoundRect(x, cy, cw, chh, 4, TFT_RED);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(NAME[i], x + cw / 2, cy + 8, 1);
  }

  uint32_t tick = 0;
  for (;;) {
    GimbalState g;
    gimbals_get(g);
    const int raw[4] = {g.throttle, g.yaw, g.pitch, g.roll};
    for (int i = 0; i < 4; ++i) {
      uint16_t v = (uint16_t)constrain(raw[i], 0, 4095);
      if (v < mn[i]) mn[i] = v;
      if (v > mx[i]) mx[i] = v;
    }

    // Update cells ~5 Hz: live raw value (top) + captured span (bottom).
    if ((tick++ % 8) == 0) {
      bool all = true;
      char b[10];
      for (int i = 0; i < 4; ++i) {
        int span = (mx[i] >= mn[i]) ? mx[i] - mn[i] : 0;
        bool pass = span >= MIN_SPAN;
        if (!pass) all = false;
        int x = x0 + i * (cw + gap);
        tft.setTextDatum(MC_DATUM);
        tft.setTextPadding(cw - 8);
        // Live raw: proves whether the pot tracks the stick at all.
        snprintf(b, sizeof(b), "%d", raw[i]);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawString(b, x + cw / 2, cy + 20, 1);
        // Captured span so far.
        snprintf(b, sizeof(b), "%d", span);
        tft.setTextColor(pass ? TFT_GREEN : TFT_ORANGE, TFT_BLACK);
        tft.drawString(b, x + cw / 2, cy + 33, 1);
        if (pass && !done_cell[i]) {
          done_cell[i] = true;
          tft.drawRoundRect(x, cy, cw, chh, 4, TFT_GREEN);
        }
      }
      if (all) break;
    }
    vTaskDelay(pdMS_TO_TICKS(TICK_MS));
  }

  for (int i = 0; i < 4; ++i) {
    g_cal.min[i] = mn[i];
    g_cal.max[i] = mx[i];
    g_cal.mid[i] = (uint16_t)((mn[i] + mx[i]) / 2);  // center from the endpoints
  }
  g_cal.valid = true;
  bool saved = safeSaveCal(g_cal);
  Serial.printf("[cal] %s  T=%u..%u/%u  Y=%u..%u/%u  P=%u..%u/%u  R=%u..%u/%u\n",
                saved ? "saved" : "SAVE FAILED",
                mn[0], mx[0], g_cal.mid[0], mn[1], mx[1], g_cal.mid[1], mn[2], mx[2],
                g_cal.mid[2], mn[3], mx[3], g_cal.mid[3]);
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("SAVED", LCD_W / 2, LCD_H / 2, 4);
  vTaskDelay(pdMS_TO_TICKS(1200));
}

// ---- Throttle-safety gate ----
void drawThrottleWarning() {
  tft.fillScreen(TFT_BLACK);
  // Warning triangle, left side.
  const int cx = 34, cy = LCD_H / 2;
  tft.fillTriangle(cx, cy - 24, cx - 26, cy + 22, cx + 26, cy + 22, TFT_YELLOW);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_BLACK, TFT_YELLOW);
  tft.drawString("!", cx, cy + 4, 4);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.drawString("THROTTLE HIGH", (LCD_W + 68) / 2, cy - 12, 2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Lower to continue", (LCD_W + 68) / 2, cy + 8, 2);
}

void throttleGate() {
  bool warning_shown = false;
  char b[12];
  for (;;) {
    GimbalState g;
    gimbals_get(g);
    float pct = calPct(0, g.throttle);
    if (pct <= THROTTLE_GATE_PCT) return;

    if (!warning_shown) {
      drawThrottleWarning();
      warning_shown = true;
    }
    snprintf(b, sizeof(b), "%3.0f%%", pct);
    tft.setTextDatum(MC_DATUM);
    tft.setTextPadding(tft.textWidth("100%", 2));
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawString(b, (LCD_W + 68) / 2, LCD_H / 2 + 26, 2);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ---- Radio-control screen (interim Phase 2 seed: battery %) ----
int batteryPct(float v) {
  float p = (v - VBAT_MIN) / (VBAT_MAX - VBAT_MIN) * 100.0f;
  return (int)constrain(p, 0.0f, 100.0f);
}

void drawRcStatic() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(ML_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString("RC", 4, 8, 2);
}

void renderRc() {
  PowerReading pr;
  bool ok = power_get(pr);
  char b[16];

  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(120);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  if (ok) snprintf(b, sizeof(b), "%d%%", batteryPct(pr.voltage_V));
  else snprintf(b, sizeof(b), "--%%");
  tft.drawString(b, LCD_W / 2, 30, 4);

  tft.setTextPadding(100);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  if (ok) snprintf(b, sizeof(b), "%.2f V", pr.voltage_V);
  else snprintf(b, sizeof(b), "-- V");
  tft.drawString(b, LCD_W / 2, 58, 2);
}

// ---- Configuration screen (menu framework seed: gimbal keys) ----
// Pitch = UP/DOWN, Roll = OK/CANCEL per TASK01.md thresholds.
struct Keys {
  bool up, down, ok, cancel;
};

Keys readKeys() {
  GimbalState g;
  gimbals_get(g);
  int pitch_us = calUs(2, g.pitch);
  int roll_us = calUs(3, g.roll);
  Keys k;
  k.up = pitch_us > KEY_HI_US;
  k.down = pitch_us < KEY_LO_US;
  k.ok = roll_us > KEY_HI_US;
  k.cancel = roll_us < KEY_LO_US;
  return k;
}

void drawConfigStatic() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("CONFIG MODE", LCD_W / 2, 10, 2);
}

void drawKeyBox(int i, const char* label, bool on) {
  const int bw = 64, bh = 34, gap = 6;
  const int x0 = (LCD_W - 4 * bw - 3 * gap) / 2 + i * (bw + gap);
  const int y0 = 26;
  uint16_t bg = on ? TFT_GREEN : TFT_BLACK;
  tft.fillRoundRect(x0, y0, bw, bh, 4, bg);
  tft.drawRoundRect(x0, y0, bw, bh, 4, on ? TFT_GREEN : TFT_DARKGREY);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(on ? TFT_BLACK : TFT_WHITE, bg);
  tft.drawString(label, x0 + bw / 2, y0 + bh / 2, 2);
}

void renderConfig() {
  Keys k = readKeys();
  drawKeyBox(0, "UP", k.up);
  drawKeyBox(1, "DOWN", k.down);
  drawKeyBox(2, "OK", k.ok);
  drawKeyBox(3, "CXL", k.cancel);
}

// ---- Gimbal wiring debug screen (bars + raw per channel). Not in the normal flow;
// kept for hardware debugging — call from appTask in place of the startup sequence.
[[maybe_unused]] void gimbalDebugLoop() {
  const char* const NAME[4] = {"THR", "YAW", "PIT", "ROL"};
  constexpr int ROWH = LCD_H / 4;                  // 19
  constexpr int BX = 34;                           // after label
  constexpr int VW = 34;                           // raw value width
  constexpr int BW = LCD_W - BX - VW;              // bar width
  tft.fillScreen(TFT_BLACK);

  char b[8];
  for (;;) {
    GimbalState g;
    gimbals_get(g);
    const int raw[4] = {g.throttle, g.yaw, g.pitch, g.roll};
    for (int i = 0; i < 4; ++i) {
      int v = constrain(raw[i], 0, 4095);
      int y = i * ROWH;
      int fw = (int)((long)v * BW / 4095);

      tft.setTextDatum(ML_DATUM);
      tft.setTextPadding(0);
      tft.setTextColor(TFT_CYAN, TFT_BLACK);
      tft.drawString(NAME[i], 2, y + ROWH / 2, 2);

      tft.fillRect(BX, y + 3, fw, ROWH - 5, TFT_GREEN);
      tft.fillRect(BX + fw, y + 3, BW - fw, ROWH - 5, TFT_DARKGREY);

      snprintf(b, sizeof(b), "%4d", v);
      tft.setTextDatum(MR_DATUM);
      tft.setTextPadding(VW);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.drawString(b, LCD_W - 2, y + ROWH / 2, 2);
    }
    vTaskDelay(pdMS_TO_TICKS(33));  // ~30 Hz
  }
}

// ---- Toggle helpers ----
bool toggleOn(int bit) {
  SwitchState sw;
  if (!switches_get(sw)) return false;  // sensor missing -> treat as OFF
  return sw.bits & (1u << bit);
}

void modeLoop() {
  enum class Screen { None, Rc, Config };
  Screen cur = Screen::None;
  const TickType_t period = pdMS_TO_TICKS(50);  // ~20 Hz

  for (;;) {
    Screen want = toggleOn(TOGGLE6_BIT) ? Screen::Config : Screen::Rc;
    if (want != cur) {
      cur = want;
      if (cur == Screen::Config) drawConfigStatic();
      else drawRcStatic();
    }
    if (cur == Screen::Config) renderConfig();
    else renderRc();
    vTaskDelay(period);
  }
}

void appTask(void*) {
  // NVS first, while no peripheral tasks/ISRs exist yet — flash init may write, and a
  // busy ISR during a flash-cache-disabled window panics the chip.
  storage_init();

  tft.init();
  tft.setRotation(PANEL_ROTATION);
  if (PANEL_INVERT) tft.invertDisplay(true);
  tft.fillScreen(TFT_BLACK);
  tft.setViewport(VP_X, VP_Y, LCD_W, LCD_H);
  backlightOn();

  Serial.printf("[ui] app up: %dx%d @ viewport(%d,%d) rot=%d\n", LCD_W, LCD_H, VP_X, VP_Y,
                PANEL_ROTATION);

  bootScreen();
  vTaskDelay(pdMS_TO_TICKS(3000));  // "BOOTING UP" for 3 s

  runInit();  // INA226 -> MCP23017 -> Gimbals -> CRSF, with per-module status

  // Toggle5 ON at power-up forces a fresh calibration; otherwise use the stored one.
  // First boot (nothing stored) always calibrates.
  bool have_cal = storage_load_cal(g_cal);
  if (toggleOn(TOGGLE5_BIT) || !have_cal) {
    calibrationWizard();
  } else {
    Serial.println("[cal] loaded from NVS");
  }

  throttleGate();  // block while throttle > 15%

  modeLoop();  // live Toggle6: Config vs RC (does not return)
}

}  // namespace

bool ui_start() {
  BaseType_t ok = xTaskCreatePinnedToCore(appTask, "app", 6144, nullptr,
                                          /*priority=*/1, nullptr, /*core=*/0);
  return ok == pdPASS;
}
