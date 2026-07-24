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
#include "gconfig.h"
#include "gimbals.h"
#include "pins.h"
#include "power.h"
#include "storage.h"
#include "switches.h"

namespace {

TFT_eSPI tft;  // pins come from build_flags (platformio.ini)

// ---- Theme (TASK01.md: black background throughout) ----
// Single source of truth for the screen background. All screen clears and text
// backgrounds use this so the theme can be changed in one place.
constexpr uint16_t COL_BG = TFT_BLACK;

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

bool safeSaveGconfig() {  // persists flip + toggle map
  crsf_pause();
  gimbals_pause();
  bool ok = gconfig_save();
  gimbals_resume();
  crsf_resume();
  return ok;
}

void backlightOn() {
  ledcAttach(PIN_TFT_BL, BL_PWM_FREQ, BL_PWM_BITS);  // arduino-esp32 3.x API
  ledcWrite(PIN_TFT_BL, BL_FULL);
}

// ---- Calibrated stick helpers (index: 0=T, 1=Y, 2=P, 3=R) ----
// Piecewise around the stored center: (min-M)..mid -> 0..50%, mid..(max+M) -> 50..100%.
// A small margin M past the captured endpoints (min-5 = 0%, max+5 = 100%) gives a bit
// of headroom so the physical extremes sit just inside the rails. Center is unchanged
// (still (min+max)/2, which is also the midpoint of the padded range).
constexpr int CAL_MARGIN = 5;
float calPct(int idx, int raw) {
  if (!g_cal.valid || g_cal.max[idx] <= g_cal.min[idx]) {
    return constrain(raw, 0, 4095) * 100.0f / 4095.0f;  // uncalibrated fallback
  }
  int lo = (int)g_cal.min[idx] - CAL_MARGIN;
  int hi = (int)g_cal.max[idx] + CAL_MARGIN;
  int mid = g_cal.mid[idx];
  if (mid <= lo || mid >= hi) mid = (lo + hi) / 2;  // sane fallback
  float p;
  if (raw <= mid) p = (raw - lo) * 50.0f / (mid - lo);
  else p = 50.0f + (raw - mid) * 50.0f / (hi - mid);
  return constrain(p, 0.0f, 100.0f);
}

// Flip-aware stick position: applies the channel reverse flag so display, menu keys,
// and CRSF output all agree on stick direction.
float stickPct(int idx, int raw) {
  float p = calPct(idx, raw);
  return gconfig_flip(idx) ? (100.0f - p) : p;
}
int stickUs(int idx, int raw) { return 1000 + (int)(stickPct(idx, raw) * 10.0f + 0.5f); }

// ---- Boot screen ----
void bootScreen() {
  tft.fillScreen(COL_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("BOOTING UP", LCD_W / 2, LCD_H / 2, 4);
}

// ---- Module init sequence ----
const char* const MOD_NAME[4] = {"INA226", "MCP23017", "Gimbals", "CRSF"};
int rowY(int i) { return 9 + i * 17; }  // 4 rows in 76 px

void drawInitList() {
  tft.fillScreen(COL_BG);
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

void runInit(bool start_crsf) {
  drawInitList();
  vTaskDelay(pdMS_TO_TICKS(250));
  setStatus(0, power_start());
  vTaskDelay(pdMS_TO_TICKS(350));
  setStatus(1, switches_start());
  vTaskDelay(pdMS_TO_TICKS(350));
  setStatus(2, gimbals_start());
  vTaskDelay(pdMS_TO_TICKS(350));
  // In module-WiFi-setup mode, deliberately do NOT start CRSF so the module (seeing
  // no handset) drops into its WiFi config mode.
  if (start_crsf) {
    setStatus(3, crsf_start());
  } else {
    tft.setTextDatum(MR_DATUM);
    tft.setTextPadding(tft.textWidth("FAIL", 2));
    tft.setTextColor(TFT_CYAN, COL_BG);
    tft.drawString("OFF", LCD_W - 4, rowY(3), 2);
  }
  vTaskDelay(pdMS_TO_TICKS(700));
}

// ---- Gimbal calibration wizard (first boot / Toggle5 at power-up) ----
// A stick is confirmed only after it has been swept fully in each direction the
// required number of times: low->high (records the MAX) x3, and high->low (records
// the MIN) x3. Direction is detected with hysteresis (crossing the 20%/80% band of
// the span seen so far). Rejects single accidental extremes and noise.
void calibrationWizard() {
  constexpr int TICK_MS = 20;
  constexpr int NEED = 3;          // sweeps required each direction
  constexpr int COUNT_SPAN = 1000;  // span must exceed this before counting starts
  const char* const NAME[4] = {"THR", "YAW", "PIT", "ROL"};
  enum Zone : uint8_t { UNK, LO, HI };

  uint16_t center[4] = {2048, 2048, 2048, 2048};

  // ---- Phase 1: sweep for MIN/MAX ----
  tft.fillScreen(COL_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("CALIBRATE: sweep each stick min<->max x3", LCD_W / 2, 8, 1);

  uint16_t mn[4] = {4095, 4095, 4095, 4095};
  uint16_t mx[4] = {0, 0, 0, 0};
  uint8_t up[4] = {0, 0, 0, 0};    // low->high sweeps (confirm MAX)
  uint8_t dn[4] = {0, 0, 0, 0};    // high->low sweeps (confirm MIN)
  Zone zone[4] = {UNK, UNK, UNK, UNK};

  const int cw = 66, gap = 4;
  const int x0 = (LCD_W - 4 * cw - 3 * gap) / 2;
  const int cy = 20, chh = 44;
  bool done_cell[4] = {false, false, false, false};
  for (int i = 0; i < 4; ++i) {
    int x = x0 + i * (cw + gap);
    tft.drawRoundRect(x, cy, cw, chh, 4, TFT_RED);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(NAME[i], x + cw / 2, cy + 7, 1);
  }

  uint32_t tick = 0;
  for (;;) {
    GimbalState g;
    gimbals_get(g);
    const int raw[4] = {g.throttle, g.yaw, g.pitch, g.roll};
    for (int i = 0; i < 4; ++i) {
      int v = constrain(raw[i], 0, 4095);
      if (v < mn[i]) mn[i] = v;
      if (v > mx[i]) mx[i] = v;
      int span = mx[i] - mn[i];
      if (span < COUNT_SPAN) continue;  // range not established yet
      int hi = mn[i] + span * 4 / 5;    // 80%
      int lo = mn[i] + span / 5;        // 20%
      if (v >= hi) {
        if (zone[i] == LO && up[i] < NEED) up[i]++;  // completed a low->high sweep
        zone[i] = HI;
      } else if (v <= lo) {
        if (zone[i] == HI && dn[i] < NEED) dn[i]++;  // completed a high->low sweep
        zone[i] = LO;
      }
    }

    // Update cells ~5 Hz: up-sweep count (records MAX) + down-sweep count (records MIN).
    if ((tick++ % 8) == 0) {
      bool all = true;
      char b[10];
      for (int i = 0; i < 4; ++i) {
        bool cell_done = (up[i] >= NEED) && (dn[i] >= NEED);
        if (!cell_done) all = false;
        int x = x0 + i * (cw + gap);
        tft.setTextDatum(MC_DATUM);
        tft.setTextPadding(cw - 8);
        snprintf(b, sizeof(b), "MX %d/%d", up[i], NEED);
        tft.setTextColor(up[i] >= NEED ? TFT_GREEN : TFT_ORANGE, TFT_BLACK);
        tft.drawString(b, x + cw / 2, cy + 22, 1);
        snprintf(b, sizeof(b), "MN %d/%d", dn[i], NEED);
        tft.setTextColor(dn[i] >= NEED ? TFT_GREEN : TFT_ORANGE, TFT_BLACK);
        tft.drawString(b, x + cw / 2, cy + 34, 1);
        if (cell_done && !done_cell[i]) {
          done_cell[i] = true;
          tft.drawRoundRect(x, cy, cw, chh, 4, TFT_GREEN);
        }
      }
      if (all) break;
    }
    vTaskDelay(pdMS_TO_TICKS(TICK_MS));
  }

  // ---- Phase 2: capture the resting CENTER (after sweeping, sticks settle to rest) ----
  {
    tft.fillScreen(COL_BG);
    tft.setTextDatum(MC_DATUM);
    tft.setTextPadding(0);
    tft.setTextColor(TFT_YELLOW, COL_BG);
    tft.drawString("CENTER THE STICKS", LCD_W / 2, 12, 2);
    tft.setTextColor(TFT_WHITE, COL_BG);
    tft.drawString("leave at rest...", LCD_W / 2, 28, 1);
    const int bx = 8, bw = LCD_W - 16, by = 58, bh = 12;
    tft.drawRect(bx, by, bw, bh, TFT_DARKGREY);
    const int HOLD_MS = 5000;
    int last_sec = -1;
    for (int t = 0; t < HOLD_MS; t += TICK_MS) {
      GimbalState g;
      gimbals_get(g);
      center[0] = constrain(g.throttle, 0, 4095);
      center[1] = constrain(g.yaw, 0, 4095);
      center[2] = constrain(g.pitch, 0, 4095);
      center[3] = constrain(g.roll, 0, 4095);
      int fw = (bw - 2) * (HOLD_MS - t) / HOLD_MS;  // countdown
      tft.fillRect(bx + 1, by + 1, fw, bh - 2, TFT_GREEN);
      tft.fillRect(bx + 1 + fw, by + 1, bw - 2 - fw, bh - 2, COL_BG);
      int sec = (HOLD_MS - t + 999) / 1000;  // seconds remaining
      if (sec != last_sec) {
        last_sec = sec;
        char b[8];
        snprintf(b, sizeof(b), "%ds", sec);
        tft.setTextDatum(MC_DATUM);
        tft.setTextPadding(tft.textWidth("00s", 4));
        tft.setTextColor(TFT_GREEN, COL_BG);
        tft.drawString(b, LCD_W / 2, 44, 4);
      }
      vTaskDelay(pdMS_TO_TICKS(TICK_MS));
    }
  }

  for (int i = 0; i < 4; ++i) {
    g_cal.min[i] = mn[i];
    g_cal.max[i] = mx[i];
    // Captured resting center, clamped just inside the swept range for a valid piecewise.
    g_cal.mid[i] = (uint16_t)constrain((int)center[i], mn[i] + 1, mx[i] - 1);
  }
  g_cal.valid = true;
  bool saved = safeSaveCal(g_cal);  // quiesces gimbal + CRSF tasks around the NVS write
  Serial.printf("[cal] %s  T=%u..%u/%u  Y=%u..%u/%u  P=%u..%u/%u  R=%u..%u/%u\n",
                saved ? "saved" : "SAVE FAILED",
                mn[0], mx[0], g_cal.mid[0], mn[1], mx[1], g_cal.mid[1], mn[2], mx[2],
                g_cal.mid[2], mn[3], mx[3], g_cal.mid[3]);
  tft.fillScreen(COL_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("SAVED", LCD_W / 2, LCD_H / 2, 4);
  vTaskDelay(pdMS_TO_TICKS(1200));
}

// ---- Throttle-safety gate ----
void drawThrottleWarning() {
  tft.fillScreen(COL_BG);
  // Warning triangle, left side.
  const int cx = 34, cy = LCD_H / 2;
  tft.fillTriangle(cx, cy - 24, cx - 26, cy + 22, cx + 26, cy + 22, TFT_RED);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_WHITE, TFT_RED);
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
  tft.fillScreen(COL_BG);
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

// ---- Config-mode menu system ----
// Navigation via gimbal keys (TASK01.md): pitch >1800us = UP, <1200 = DOWN;
// roll >1800 = OK (enter), <1200 = CANCEL (back).
bool toggleOn(int bit);  // fwd decl (defined below)

const char* const MENU_ITEMS[] = {"GIMBALS & TRIM", "TOGGLE MAPPING", "BATTERY INFO",
                                   "CRSF MODULE", "REBOOT"};
constexpr int MENU_N = 5;

struct RawKeys {
  bool up, down, ok, cancel;
};

RawKeys readRawKeys() {
  GimbalState g;
  gimbals_get(g);
  int p = stickUs(2, g.pitch);  // flip-aware: nav follows the pitch reverse flag
  int r = stickUs(3, g.roll);   // flip-aware: OK/CANCEL follow the roll reverse flag
  return {p > KEY_HI_US, p < KEY_LO_US, r > KEY_HI_US, r < KEY_LO_US};
}

// Edge detector with auto-repeat (fires once on press, then repeats while held).
struct EdgeKey {
  bool prev = false;
  int ctr = 0;
  static constexpr int REPEAT_DELAY = 10;   // ~500 ms at 50 ms/tick before repeat
  static constexpr int REPEAT_PERIOD = 4;   // ~200 ms between repeats
  bool poll(bool pressed) {
    bool fire = false;
    if (pressed) {
      if (!prev) {
        fire = true;
        ctr = 0;
      } else if (++ctr >= REPEAT_DELAY) {
        fire = true;
        ctr = REPEAT_DELAY - REPEAT_PERIOD;
      }
    } else {
      ctr = 0;
    }
    prev = pressed;
    return fire;
  }
};

// Draw a single menu row (clears just its own area — no full-screen blank).
void drawMenuItem(int i, bool sel) {
  int y = 16 + i * 12;
  uint16_t bg = sel ? TFT_NAVY : COL_BG;
  tft.fillRect(0, y - 6, LCD_W, 12, bg);  // clear only this row
  tft.setTextDatum(ML_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(sel ? TFT_WHITE : TFT_DARKGREY, bg);
  tft.drawString(MENU_ITEMS[i], 8, y, 1);
}

void drawMenuListFull(int sel) {
  tft.fillScreen(COL_BG);
  tft.setTextDatum(ML_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_YELLOW, COL_BG);
  tft.drawString("CONFIG", 4, 5, 1);
  for (int i = 0; i < MENU_N; ++i) drawMenuItem(i, i == sel);
}

// --- Menu item 1: Gimbals position + channel reverse (flip).
// Toggle5 ON = edit mode: pitch up/down moves the cursor, roll-right (OK) flips the
// selected channel (saved to NVS), roll-left (CANCEL) backs out. Toggle5 OFF = view only.
void drawGimbalPosStatic() {
  tft.fillScreen(COL_BG);
  tft.setTextPadding(0);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_YELLOW, COL_BG);
  tft.drawString("GIMBALS", 4, 6, 1);
  tft.setTextDatum(MR_DATUM);
  tft.setTextColor(TFT_DARKGREY, COL_BG);
  tft.drawString("T5:edit/back  OK:flip", LCD_W - 4, 6, 1);
}

void drawGimbalPage(int cursor, bool edit) {
  static const char* const NAME[4] = {"THR", "YAW", "PIT", "ROL"};
  GimbalState g;
  gimbals_get(g);
  const int raw[4] = {g.throttle, g.yaw, g.pitch, g.roll};
  const int top = 16, rowh = 15, bx = 30, bw = 150;
  char b[10];
  for (int i = 0; i < 4; ++i) {
    bool sel = edit && (i == cursor);
    bool fl = gconfig_flip(i);
    float pct = stickPct(i, raw[i]);  // flip-aware position
    int y = top + i * rowh, cy = y + rowh / 2;
    int fw = constrain((int)(pct * bw / 100.0f), 0, bw);

    // Cursor (only in edit mode).
    tft.setTextDatum(ML_DATUM);
    tft.setTextPadding(7);
    tft.setTextColor(sel ? TFT_YELLOW : COL_BG, COL_BG);
    tft.drawString(">", 1, cy, 1);
    // Name.
    tft.setTextPadding(0);
    tft.setTextColor(sel ? TFT_YELLOW : TFT_CYAN, COL_BG);
    tft.drawString(NAME[i], 9, cy, 1);
    // Bar.
    tft.fillRect(bx, y + 2, fw, rowh - 4, TFT_GREEN);
    tft.fillRect(bx + fw, y + 2, bw - fw, rowh - 4, TFT_DARKGREY);
    // Percent.
    snprintf(b, sizeof(b), "%3d%%", (int)(pct + 0.5f));
    tft.setTextDatum(MR_DATUM);
    tft.setTextPadding(30);
    tft.setTextColor(TFT_WHITE, COL_BG);
    tft.drawString(b, bx + bw + 34, cy, 1);
    // Reverse flag.
    tft.setTextPadding(28);
    tft.setTextColor(fl ? TFT_RED : TFT_DARKGREY, COL_BG);
    tft.drawString(fl ? "REV" : "NOR", LCD_W - 2, cy, 1);
  }
}

void drawRebootConfirm() {
  tft.fillScreen(COL_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_RED, COL_BG);
  tft.drawString("REBOOT?", LCD_W / 2, 24, 4);
  tft.setTextColor(TFT_WHITE, COL_BG);
  tft.drawString("OK = yes     CXL = no", LCD_W / 2, 56, 1);
}

void doReboot() {
  tft.fillScreen(COL_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_YELLOW, COL_BG);
  tft.drawString("REBOOTING", LCD_W / 2, LCD_H / 2, 4);
  vTaskDelay(pdMS_TO_TICKS(500));
  ESP.restart();
}

// --- Menu item 3: Battery info (INA226) with a fill-level battery icon.
uint16_t batteryColor(int pct) {
  if (pct > 50) return TFT_GREEN;
  if (pct > 20) return TFT_ORANGE;
  return TFT_RED;
}

void drawBatteryIcon(int x, int y, int w, int h, int pct, uint16_t fill) {
  tft.drawRoundRect(x, y, w, h, 3, TFT_WHITE);          // body
  int nubh = h / 2;
  tft.fillRect(x + w, y + (h - nubh) / 2, 5, nubh, TFT_WHITE);  // terminal
  int ix = x + 3, iy = y + 3, iw = w - 6, ih = h - 6;  // inner
  int fw = iw * constrain(pct, 0, 100) / 100;
  tft.fillRect(ix, iy, fw, ih, fill);
  tft.fillRect(ix + fw, iy, iw - fw, ih, COL_BG);
}

void drawBatteryStatic() {
  tft.fillScreen(COL_BG);
  tft.setTextPadding(0);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_YELLOW, COL_BG);
  tft.drawString("BATTERY", 4, 6, 1);
  tft.setTextDatum(MR_DATUM);
  tft.setTextColor(TFT_DARKGREY, COL_BG);
  tft.drawString("CXL=back", LCD_W - 4, 6, 1);
}

// Redraw only what changed (force = redraw all, e.g. on page entry) to avoid the
// erase/redraw flash — especially the % which sits transparently over the icon fill.
void drawBatteryDynamic(bool force) {
  static int last_pct = -999, last_v = -1, last_i = -2000000, last_p = -1;
  PowerReading pr;
  bool ok = power_get(pr);
  int pct = ok ? batteryPct(pr.voltage_V) : -1;

  const int bx = 8, by = 24, bw = 96, bh = 40;
  const int rx = LCD_W - 4;
  char b[16];

  if (force || pct != last_pct) {  // icon fill + % only on change
    last_pct = pct;
    int p = pct < 0 ? 0 : pct;
    drawBatteryIcon(bx, by, bw, bh, p, batteryColor(p));
    snprintf(b, sizeof(b), ok ? "%d%%" : "--", p);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE);  // transparent over the freshly-drawn fill
    tft.drawString(b, bx + bw / 2, by + bh / 2, 4);
  }

  int vi = ok ? (int)(pr.voltage_V * 100 + 0.5f) : -1;
  int ii = ok ? (int)pr.current_mA : -2000000;
  int pi = ok ? (int)(pr.power_mW / 10.0f + 0.5f) : -1;  // 0.01 W units

  tft.setTextDatum(MR_DATUM);
  tft.setTextPadding(96);
  if (force || vi != last_v) {
    last_v = vi;
    tft.setTextColor(TFT_WHITE, COL_BG);
    if (ok) snprintf(b, sizeof(b), "%.2f V", pr.voltage_V);
    else snprintf(b, sizeof(b), "-- V");
    tft.drawString(b, rx, 24, 2);
  }
  if (force || ii != last_i) {
    last_i = ii;
    tft.setTextColor(ok && pr.near_full ? TFT_RED : TFT_CYAN, COL_BG);
    if (!ok) snprintf(b, sizeof(b), "-- A");
    else if (fabsf(pr.current_mA) >= 1000.0f) snprintf(b, sizeof(b), "%.2f A", pr.current_mA / 1000.0f);
    else snprintf(b, sizeof(b), "%.0f mA", pr.current_mA);
    tft.drawString(b, rx, 44, 2);
  }
  if (force || pi != last_p) {
    last_p = pi;
    tft.setTextColor(TFT_GREENYELLOW, COL_BG);
    if (ok) snprintf(b, sizeof(b), "%.2f W", pr.power_mW / 1000.0f);
    else snprintf(b, sizeof(b), "-- W");
    tft.drawString(b, rx, 64, 2);
  }
}

// --- Menu item 2: Toggle -> CRSF channel mapping (Toggle6 reserved for menu mode).
const int EDIT_TOGGLE[9] = {0, 1, 2, 3, 4, 6, 7, 8, 9};  // toggle indices, skip Toggle6
constexpr int EDIT_TOGGLE_N = 9;

void drawToggleHeader(bool editing) {
  tft.fillRect(0, 0, LCD_W, 12, COL_BG);  // clear header row only
  tft.setTextPadding(0);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_YELLOW, COL_BG);
  tft.drawString("TOGGLES", 4, 5, 1);
  tft.setTextDatum(MR_DATUM);
  tft.setTextColor(TFT_DARKGREY, COL_BG);
  tft.drawString(editing ? "UP/DN:chan  OK:set" : "OK:edit  CXL:back", LCD_W - 4, 5, 1);
}

// Redraw a single toggle cell (clears just its own area — no full-screen blank).
void drawToggleCell(int t, bool sel, bool editing) {
  int col = t < 5 ? 0 : 1;
  int row = t < 5 ? t : t - 5;
  int x = col ? 152 : 8;
  int y = 17 + row * 12;
  uint16_t bg = sel ? TFT_NAVY : COL_BG;
  tft.fillRect(col ? 148 : 2, y - 6, 134, 12, bg);  // clear only this cell
  char b[16];
  if (t == 5) {
    snprintf(b, sizeof(b), "T6  MENU");
  } else {
    uint8_t c = gconfig_toggle_chan(t);
    if (c) snprintf(b, sizeof(b), "T%-2d CH%u", t + 1, c);
    else snprintf(b, sizeof(b), "T%-2d --", t + 1);
  }
  uint16_t fg = (t == 5) ? TFT_DARKGREY : (sel ? (editing ? TFT_GREEN : TFT_YELLOW) : TFT_WHITE);
  tft.setTextDatum(ML_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(fg, bg);
  tft.drawString(b, x, y, 1);
}

void drawToggleMapFull(int cursor, bool editing) {
  tft.fillScreen(COL_BG);
  drawToggleHeader(editing);
  int cur_toggle = EDIT_TOGGLE[cursor];
  for (int t = 0; t < 10; ++t) drawToggleCell(t, t == cur_toggle, editing);
}

// --- Menu item 4: CRSF module status (BetaFPV Nano TX V2).
void drawCrsfStatic() {
  tft.fillScreen(COL_BG);
  tft.setTextPadding(0);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_YELLOW, COL_BG);
  tft.drawString("CRSF MODULE", 4, 6, 1);
  tft.setTextDatum(MR_DATUM);
  tft.setTextColor(TFT_DARKGREY, COL_BG);
  tft.drawString("OK:WiFi  CXL:back", LCD_W - 4, 6, 1);
  // Static labels.
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_DARKGREY, COL_BG);
  tft.drawString("MODULE", 4, 20, 1);
  tft.drawString("LINK", 4, 32, 1);
  tft.drawString("LQ", 4, 44, 1);
  tft.drawString("RSSI", 4, 56, 1);
  tft.drawString("RATE", 4, 68, 1);
  // Static value: our fixed link config.
  tft.setTextColor(TFT_WHITE, COL_BG);
  tft.drawString("250 Hz  400k", 70, 68, 1);
}

void drawCrsfVal(int y, const char* s, uint16_t col) {
  tft.fillRect(66, y - 6, LCD_W - 66, 12, COL_BG);  // clear value area only
  tft.setTextDatum(ML_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(col, COL_BG);
  tft.drawString(s, 70, y, 1);
}

void drawCrsfDynamic(bool force) {
  static uint32_t prev_rx = 0;
  static int last_online = -1, last_link = -1, last_lq = -999, last_rssi = -999;
  CrsfLink l;
  crsf_get(l);
  char b[20];

  // MODULE: is the TX module sending anything back (sync/telemetry frames)?
  int online = (l.rx_frames != prev_rx) ? 1 : 0;
  prev_rx = l.rx_frames;
  if (force || online != last_online) {
    last_online = online;
    drawCrsfVal(20, online ? "ONLINE" : "OFFLINE", online ? TFT_GREEN : TFT_RED);
  }
  // LINK: RF link to a bound receiver (link-stats present with LQ > 0).
  int link = (l.valid && l.up_lq > 0) ? 1 : 0;
  if (force || link != last_link) {
    last_link = link;
    drawCrsfVal(32, link ? "UP" : "DOWN", link ? TFT_GREEN : TFT_DARKGREY);
  }
  int lq = l.valid ? l.up_lq : -1;
  if (force || lq != last_lq) {
    last_lq = lq;
    if (lq < 0) snprintf(b, sizeof(b), "--");
    else snprintf(b, sizeof(b), "%d %%", lq);
    drawCrsfVal(44, b, TFT_WHITE);
  }
  int rssi = l.valid ? l.up_rssi : -999;
  if (force || rssi != last_rssi) {
    last_rssi = rssi;
    if (rssi == -999) snprintf(b, sizeof(b), "--");
    else snprintf(b, sizeof(b), "%d dBm", rssi);
    drawCrsfVal(56, b, TFT_WHITE);
  }
}

// NVS write for the WiFi-boot flag with interrupt-heavy peripherals quiesced.
bool safeSetWifiBoot(bool v) {
  crsf_pause();
  gimbals_pause();
  switches_pause();
  storage_set_wifi_boot(v);
  switches_resume();
  gimbals_resume();
  crsf_resume();
  return true;
}

void drawWifiConfirm() {
  tft.fillScreen(COL_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_YELLOW, COL_BG);
  tft.drawString("MODULE WIFI SETUP?", LCD_W / 2, 16, 2);
  tft.setTextColor(TFT_WHITE, COL_BG);
  tft.drawString("reboots with CRSF off", LCD_W / 2, 38, 1);
  tft.drawString("OK = yes     CXL = no", LCD_W / 2, 58, 1);
}

// Shown at boot while the WiFi-boot flag is set (CRSF never started).
void drawWifiSetup() {
  tft.fillScreen(COL_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_CYAN, COL_BG);
  tft.drawString("MODULE WIFI SETUP", LCD_W / 2, 8, 1);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_WHITE, COL_BG);
  tft.drawString("1. Power-cycle ELRS module", 6, 20, 1);
  tft.drawString("2. WiFi: ExpressLRS TX", 6, 32, 1);
  tft.drawString("   pw: expresslrs", 6, 44, 1);
  tft.drawString("3. http://10.0.0.1", 6, 56, 1);
  tft.setTextColor(TFT_YELLOW, COL_BG);
  tft.drawString("Toggle6 OFF = exit & fly", 6, 68, 1);
}

// Config menu loop — runs while Toggle6 is ON; returns when it goes OFF.
void configLoop() {
  enum class Page : uint8_t { List, Gimbals, Toggles, Battery, Crsf, Reboot };
  Page page = Page::List;
  int sel = 0;
  int gcursor = 0;   // selected channel on the Gimbals page
  int tmcursor = 0;  // selected toggle on the Toggle Mapping page
  bool tm_editing = false;
  uint8_t tm_prevchan = 0;  // channel before entering edit (for CANCEL revert)
  bool crsf_confirm = false;  // CRSF page: WiFi-setup confirm dialog
  bool redraw = true;
  EdgeKey ku, kd, kok, kcx;
  const TickType_t period = pdMS_TO_TICKS(50);

  while (toggleOn(TOGGLE6_BIT)) {
    RawKeys r = readRawKeys();
    bool up = ku.poll(r.up), dn = kd.poll(r.down), ok = kok.poll(r.ok),
         cx = kcx.poll(r.cancel);

    switch (page) {
      case Page::List:
        if (redraw) {
          drawMenuListFull(sel);
          redraw = false;
        }
        if (up && sel > 0) {  // redraw only old + new rows
          drawMenuItem(sel, false);
          drawMenuItem(--sel, true);
        }
        if (dn && sel < MENU_N - 1) {
          drawMenuItem(sel, false);
          drawMenuItem(++sel, true);
        }
        if (ok) {
          page = (Page)(1 + sel);
          redraw = true;
        }
        break;
      case Page::Gimbals: {
        if (redraw) {
          drawGimbalPosStatic();
          redraw = false;
        }
        bool edit = toggleOn(TOGGLE5_BIT);  // Toggle5 gates editing AND back
        if (edit) {
          if (up && gcursor > 0) gcursor--;
          if (dn && gcursor < 3) gcursor++;
          if (ok) {  // flip the selected channel and persist
            gconfig_set_flip(gcursor, !gconfig_flip(gcursor));
            safeSaveGconfig();
          }
          if (cx) {
            page = Page::List;
            redraw = true;
            break;
          }
        }
        drawGimbalPage(gcursor, edit);
        break;
      }
      case Page::Toggles: {
        if (redraw) {
          drawToggleMapFull(tmcursor, tm_editing);
          redraw = false;
        }
        int t = EDIT_TOGGLE[tmcursor];
        if (!tm_editing) {
          // Selection mode: move cursor (redraw only old+new cells).
          if (up && tmcursor > 0) {
            tmcursor--;
            drawToggleCell(t, false, false);
            drawToggleCell(EDIT_TOGGLE[tmcursor], true, false);
          }
          if (dn && tmcursor < EDIT_TOGGLE_N - 1) {
            tmcursor++;
            drawToggleCell(t, false, false);
            drawToggleCell(EDIT_TOGGLE[tmcursor], true, false);
          }
          if (ok) {
            tm_editing = true;
            tm_prevchan = gconfig_toggle_chan(t);
            drawToggleHeader(true);
            drawToggleCell(t, true, true);
          }
          if (cx) {
            page = Page::List;
            redraw = true;
          }
        } else {
          // Edit mode: up/down cycle channel (redraw only this cell).
          uint8_t c = gconfig_toggle_chan(t);
          if (up) {  // none -> 5..16 -> none
            gconfig_set_toggle_chan(t, (c == 0) ? 5 : (c >= 16 ? 0 : c + 1));
            drawToggleCell(t, true, true);
          }
          if (dn) {  // none -> 16..5 -> none
            gconfig_set_toggle_chan(t, (c == 0) ? 16 : (c <= 5 ? 0 : c - 1));
            drawToggleCell(t, true, true);
          }
          if (ok) {
            safeSaveGconfig();
            tm_editing = false;
            drawToggleHeader(false);
            drawToggleCell(t, true, false);
          }
          if (cx) {
            gconfig_set_toggle_chan(t, tm_prevchan);  // revert
            tm_editing = false;
            drawToggleHeader(false);
            drawToggleCell(t, true, false);
          }
        }
        break;
      }
      case Page::Battery: {
        bool first = redraw;  // full redraw on page entry
        if (redraw) {
          drawBatteryStatic();
          redraw = false;
        }
        drawBatteryDynamic(first);
        if (cx) {
          page = Page::List;
          redraw = true;
        }
        break;
      }
      case Page::Reboot:
        if (redraw) {
          drawRebootConfirm();
          redraw = false;
        }
        if (ok) doReboot();  // does not return
        if (cx) {
          page = Page::List;
          redraw = true;
        }
        break;
      case Page::Crsf: {
        bool first = redraw;
        if (redraw) {
          if (crsf_confirm) drawWifiConfirm();
          else drawCrsfStatic();
          redraw = false;
        }
        if (crsf_confirm) {
          if (ok) {  // enter module WiFi setup: set flag, reboot (CRSF stays off)
            safeSetWifiBoot(true);
            doReboot();  // does not return
          }
          if (cx) {
            crsf_confirm = false;
            redraw = true;
          }
        } else {
          drawCrsfDynamic(first);
          if (ok) {  // OK -> confirm module WiFi setup
            crsf_confirm = true;
            redraw = true;
          }
          if (cx) {
            page = Page::List;
            redraw = true;
          }
        }
        break;
      }
      default:  // unreachable — all pages handled
        if (cx) {
          page = Page::List;
          redraw = true;
        }
        break;
    }
    vTaskDelay(period);
  }
}

// ---- Gimbal wiring debug screen (bars + raw per channel). Not in the normal flow;
// kept for hardware debugging — call from appTask in place of the startup sequence.
[[maybe_unused]] void gimbalDebugLoop() {
  const char* const NAME[4] = {"THR", "YAW", "PIT", "ROL"};
  constexpr int ROWH = LCD_H / 4;                  // 19
  constexpr int BX = 34;                           // after label
  constexpr int VW = 34;                           // raw value width
  constexpr int BW = LCD_W - BX - VW;              // bar width
  tft.fillScreen(COL_BG);

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

// RC screen loop — runs while Toggle6 is OFF; returns when it goes ON.
void rcLoop() {
  drawRcStatic();
  const TickType_t period = pdMS_TO_TICKS(200);  // ~5 Hz
  while (!toggleOn(TOGGLE6_BIT)) {
    renderRc();
    vTaskDelay(period);
  }
}

// Live Toggle6 mode switch: OFF = RC screen, ON = configuration menu.
void modeLoop() {
  for (;;) {
    if (toggleOn(TOGGLE6_BIT)) configLoop();
    else rcLoop();
  }
}

void appTask(void*) {
  // NVS first, while no peripheral tasks/ISRs exist yet — flash init may write, and a
  // busy ISR during a flash-cache-disabled window panics the chip.
  storage_init();
  gconfig_init();  // load per-channel flip flags (default none) before CRSF uses them

  tft.init();
  tft.setRotation(PANEL_ROTATION);
  tft.invertDisplay(PANEL_INVERT);  // explicit: false actively disables inversion
  tft.fillScreen(COL_BG);
  tft.setViewport(VP_X, VP_Y, LCD_W, LCD_H);
  backlightOn();

  Serial.printf("[ui] app up: %dx%d @ viewport(%d,%d) rot=%d\n", LCD_W, LCD_H, VP_X, VP_Y,
                PANEL_ROTATION);

  // Module-WiFi-setup mode: init peripherals but WITHOUT CRSF, then hold on the WiFi
  // info screen until Toggle6 is flipped to exit (clears the flag and reboots to fly).
  bool wifi_setup = storage_get_wifi_boot();

  bootScreen();
  vTaskDelay(pdMS_TO_TICKS(wifi_setup ? 1000 : 3000));  // "BOOTING UP"

  runInit(!wifi_setup);  // start CRSF only when NOT in WiFi setup

  if (wifi_setup) {
    Serial.println("[ui] MODULE WIFI SETUP mode (CRSF off)");
    drawWifiSetup();
    // We entered from config mode (Toggle6 ON). Exit when Toggle6 goes OFF (= fly),
    // once the switch reading is valid so we don't exit before the MCP is read.
    for (;;) {
      SwitchState sw;
      if (switches_get(sw) && !(sw.bits & (1u << TOGGLE6_BIT))) {
        safeSetWifiBoot(false);  // clear flag, reboot to normal
        doReboot();              // does not return
      }
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }

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
