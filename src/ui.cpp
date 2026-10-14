// ui.cpp — app controller task (Phase 1b).
//
// Startup flow: BOOTING UP (3 s) -> module init list -> gimbal calibration wizard
// (first boot only) -> throttle-safety gate (>15% warns until lowered) -> live mode
// loop: Toggle6 ON = configuration mode, OFF = radio-control screen.
// Black background throughout (TASK01.md).
#include "ui.h"

#include <Arduino.h>
#include <TFT_eSPI.h>

#include "ble.h"
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

// ---- Orientation ----
// The standard build is landscape (284x76); POCKETVERSION mounts the same panel portrait
// (76x284, see display_cfg.h). Screens branch on PORTRAIT for their geometry. The
// landscape numbers are the original hardware-tuned layout — keep them as they are.
constexpr bool PORTRAIT = !PANEL_LANDSCAPE;

// Orientation pick for a label/hint: the landscape string vs. a short portrait one
// (76 px fits 12 chars of font 1, roughly 9 of font 2).
constexpr const char* lp(const char* land, const char* port) { return PORTRAIT ? port : land; }

// Largest of fonts 4/2/1 (not above maxFont) whose text fits `avail` px. Landscape always
// keeps maxFont — its layout was sized for it.
int fitFont(const char* s, int maxFont, int avail) {
  if (!PORTRAIT) return maxFont;
  if (maxFont >= 4 && tft.textWidth(s, 4) <= avail) return 4;
  if (maxFont >= 2 && tft.textWidth(s, 2) <= avail) return 2;
  return 1;
}

// Battery % mapping, 3.0 V/cell = 0%, 4.2 V/cell = 100%. The standard handset runs a 2S
// pack (6.0..8.4 V); the POCKETVERSION is 1S only (3.0..4.2 V).
#ifdef POCKETVERSION
constexpr int VBAT_CELLS = 1;
#else
constexpr int VBAT_CELLS = 2;
#endif
constexpr float VBAT_MIN = 3.0f * VBAT_CELLS;
constexpr float VBAT_MAX = 4.2f * VBAT_CELLS;

// Startup safety: block until throttle is at/below this percent.
constexpr float THROTTLE_GATE_PCT = 15.0f;

// Special toggles, as toggle indices (0 = Toggle1). Which MCP23017 pin each one reads
// comes from the gconfig pin map (TOGGLE MAP menu), never a hard-coded bit.
// Toggle6 = mode switch: ON (closed) = configuration mode (standard build).
constexpr int TOGGLE6 = 5;
// Toggle5 ON at power-up = force the gimbal calibration wizard.
constexpr int TOGGLE5 = 4;
// Toggle3 + Toggle4 (+ Toggle5) ON at power-up = MCP23017 debug screen.
constexpr int TOGGLE3 = 2;
constexpr int TOGGLE4 = 3;

// ---- Menu-mode source ----
// Standard build: Toggle6's level (ON = configuration mode). POCKETVERSION: each press
// of the BOOT button (GPIO0, active-low) flips between the RC screen and configuration
// mode; it always powers up on the RC screen. The press is latched in a GPIO interrupt
// because the RC loop only wakes every 200 ms and would miss a short tap.
bool toggleOn(int toggle);  // fwd decl (defined below)
#ifdef POCKETVERSION
constexpr bool MENU_ON_BOOT_BTN = true;
constexpr TickType_t MENU_BTN_DEBOUNCE = pdMS_TO_TICKS(250);
volatile bool g_menuMode = false;
volatile TickType_t g_menuBtnTick = 0;

void IRAM_ATTR menuBtnIsr() {
  TickType_t now = xTaskGetTickCountFromISR();
  if ((TickType_t)(now - g_menuBtnTick) < MENU_BTN_DEBOUNCE) return;  // contact bounce
  g_menuBtnTick = now;
  g_menuMode = !g_menuMode;
}

void menuModeInit() {
  pinMode(PIN_BOOT_BTN, INPUT_PULLUP);
  attachInterrupt(PIN_BOOT_BTN, menuBtnIsr, FALLING);
}
bool menuModeOn() { return g_menuMode; }
#else
constexpr bool MENU_ON_BOOT_BTN = false;
void menuModeInit() {}
bool menuModeOn() { return toggleOn(TOGGLE6); }
#endif

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

bool safeSaveTelemSel(uint16_t sel) {  // persist main-screen telemetry selection
  crsf_pause();
  gimbals_pause();
  switches_pause();
  storage_set_telem_sel(sel);
  switches_resume();
  gimbals_resume();
  crsf_resume();
  return true;
}

// Module settings last chosen in the CRSF MODULE menu (index = ElrsSetting, 0xFF = never).
// Loaded once at boot (before the peripheral tasks run), re-sent to the module by the CRSF
// task, and kept in sync with NVS by safeSaveElrs.
uint8_t g_elrs[ELRS_SETTING_N] = {0xFF, 0xFF, 0xFF, 0xFF};

bool safeSaveElrs(ElrsSetting s, uint8_t v) {  // remember a module setting we wrote
  g_elrs[s] = v;
  crsf_pause();
  gimbals_pause();
  switches_pause();
  storage_set_elrs(s, v);
  switches_resume();
  gimbals_resume();
  crsf_resume();
  return true;
}

bool safeAddFlight(uint32_t secs) {  // accumulate armed time into NVS total
  crsf_pause();
  gimbals_pause();
  switches_pause();
  storage_add_flight_secs(secs);
  switches_resume();
  gimbals_resume();
  crsf_resume();
  return true;
}

// ---- Flight timer ----
// Session runs while the ARM-mapped toggle is ON (crsf_arm_state()==1). State lives at
// this scope so it survives RC<->config mode switches; flightTimerTick() is polled from
// both loops. On disarm the completed session is banked into the NVS total.
bool g_fltArmed = false;
uint32_t g_fltStartMs = 0;
uint32_t g_fltSessS = 0;  // current session seconds (0 when disarmed)

void flightTimerTick() {
  bool armed = (crsf_arm_state() == 1);
  uint32_t now = millis();
  if (armed && !g_fltArmed) {  // just armed
    g_fltStartMs = now;
    g_fltSessS = 0;
  } else if (armed) {  // still armed
    g_fltSessS = (now - g_fltStartMs) / 1000;
  } else if (g_fltArmed) {  // just disarmed: bank the session
    uint32_t secs = (now - g_fltStartMs) / 1000;
    if (secs > 0) safeAddFlight(secs);
    g_fltSessS = 0;
  }
  g_fltArmed = armed;
}

uint32_t flightTimerSession() { return g_fltSessS; }

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
  if (PORTRAIT) {
    tft.drawString("BOOTING", LCD_W / 2, LCD_H / 2 - 10, 2);
    tft.drawString("UP", LCD_W / 2, LCD_H / 2 + 10, 2);
  } else {
    tft.drawString("BOOTING UP", LCD_W / 2, LCD_H / 2, 4);
  }
}

// ---- Module init sequence ----
const char* const MOD_NAME[4] = {"INA226", "MCP23017", "Gimbals", "CRSF"};
// Landscape: 4 rows in 76 px, status on the name's line. Portrait: status under the name.
int rowY(int i) { return PORTRAIT ? 40 + i * 56 : 9 + i * 17; }
int statY(int i) { return rowY(i) + (PORTRAIT ? 18 : 0); }

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
    tft.drawString("...", LCD_W - 4, statY(i), 2);
  }
}

void setStatus(int i, bool ok) {
  tft.setTextDatum(MR_DATUM);
  tft.setTextPadding(tft.textWidth("FAIL", 2));
  tft.setTextColor(ok ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.drawString(ok ? "OK" : "FAIL", LCD_W - 4, statY(i), 2);
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
    tft.drawString("OFF", LCD_W - 4, statY(3), 2);
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
  if (PORTRAIT) {
    tft.drawString("CALIBRATE", LCD_W / 2, 6, 1);
    tft.drawString("sweep sticks", LCD_W / 2, 18, 1);
    tft.drawString("min<->max x3", LCD_W / 2, 30, 1);
  } else {
    tft.drawString("CALIBRATE: sweep each stick min<->max x3", LCD_W / 2, 8, 1);
  }

  uint16_t mn[4] = {4095, 4095, 4095, 4095};
  uint16_t mx[4] = {0, 0, 0, 0};
  uint8_t up[4] = {0, 0, 0, 0};    // low->high sweeps (confirm MAX)
  uint8_t dn[4] = {0, 0, 0, 0};    // high->low sweeps (confirm MIN)
  Zone zone[4] = {UNK, UNK, UNK, UNK};

  // Landscape: 4 cells in a row. Portrait: the same cells stacked.
  const int cw = 66, gap = 4, chh = 44;
  const int x0 = PORTRAIT ? (LCD_W - cw) / 2 : (LCD_W - 4 * cw - 3 * gap) / 2;
  const int y0 = PORTRAIT ? 40 : 20;
  auto cellX = [&](int i) { return PORTRAIT ? x0 : x0 + i * (cw + gap); };
  auto cellY = [&](int i) { return PORTRAIT ? y0 + i * (chh + gap) : y0; };
  bool done_cell[4] = {false, false, false, false};
  for (int i = 0; i < 4; ++i) {
    int x = cellX(i), cy = cellY(i);
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
        int x = cellX(i), cy = cellY(i);
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
    if (PORTRAIT) {  // wrapped, mid-screen
      tft.drawString("CENTER", LCD_W / 2, 90, 2);
      tft.drawString("STICKS", LCD_W / 2, 108, 2);
      tft.setTextColor(TFT_WHITE, COL_BG);
      tft.drawString("leave them", LCD_W / 2, 128, 1);
      tft.drawString("at rest...", LCD_W / 2, 140, 1);
    } else {
      tft.drawString("CENTER THE STICKS", LCD_W / 2, 12, 2);
      tft.setTextColor(TFT_WHITE, COL_BG);
      tft.drawString("leave at rest...", LCD_W / 2, 28, 1);
    }
    const int bx = 8, bw = LCD_W - 16, by = PORTRAIT ? 200 : 58, bh = 12;
    const int cnty = PORTRAIT ? 172 : 44;  // countdown seconds, above the bar
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
        tft.drawString(b, LCD_W / 2, cnty, 4);
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
  tft.drawString("SAVED", LCD_W / 2, LCD_H / 2, fitFont("SAVED", 4, LCD_W - 4));
  vTaskDelay(pdMS_TO_TICKS(1200));
}

// ---- Throttle-safety gate ----
void drawThrottleWarning() {
  tft.fillScreen(COL_BG);
  // Warning triangle: left side (landscape) / top (portrait).
  const int cx = PORTRAIT ? LCD_W / 2 : 34, cy = PORTRAIT ? 60 : LCD_H / 2;
  tft.fillTriangle(cx, cy - 24, cx - 26, cy + 22, cx + 26, cy + 22, TFT_RED);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_WHITE, TFT_RED);
  tft.drawString("!", cx, cy + 4, 4);
  if (PORTRAIT) {  // text wrapped under the triangle
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("THROTTLE", LCD_W / 2, 112, 2);
    tft.drawString("HIGH", LCD_W / 2, 130, 2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("Lower to", LCD_W / 2, 162, 2);
    tft.drawString("continue", LCD_W / 2, 180, 2);
    return;
  }
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
    if (PORTRAIT) tft.drawString(b, LCD_W / 2, 212, 2);
    else tft.drawString(b, (LCD_W + 68) / 2, LCD_H / 2 + 26, 2);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ---- Radio-control screen (interim Phase 2 seed: battery %) ----
int batteryPct(float v) {
  float p = (v - VBAT_MIN) / (VBAT_MAX - VBAT_MIN) * 100.0f;
  return (int)constrain(p, 0.0f, 100.0f);
}

// ---- Telemetry items (CRSF link statistics) ----
struct TeleItem {
  const char* name;
};
const TeleItem TELE_ITEMS[] = {
    {"UPLINK LQ"},  {"UPLINK RSSI"}, {"UPLINK SNR"}, {"DOWN LQ"},
    {"DOWN RSSI"},  {"DOWN SNR"},    {"TX POWER"},   {"RF MODE"},
    {"BATT VOLT"},  {"BATT CURR"},   {"BATT USED"},  {"BATT LEVEL"},
};
constexpr int TELE_N = 12;

uint16_t g_telemSel = 0;  // bitmask of items shown on the main RC screen (bit i = item i)

// Full value string for item i, e.g. "100%", "-45 dBm", "25 mW"; "--" when unavailable.
// Link-stats items need `ok` (link telemetry seen); battery items need l.batt_valid.
void teleValue(int i, const CrsfLink& l, bool ok, char* b, size_t n) {
  bool avail = (i < 8) ? ok : l.batt_valid;
  if (!avail) {
    snprintf(b, n, "--");
    return;
  }
  switch (i) {
    case 0: snprintf(b, n, "%u%%", l.up_lq); break;
    case 1: snprintf(b, n, "%d dBm", l.up_rssi); break;
    case 2: snprintf(b, n, "%d dB", l.up_snr); break;
    case 3: snprintf(b, n, "%u%%", l.dn_lq); break;
    case 4: snprintf(b, n, "%d dBm", l.dn_rssi); break;
    case 5: snprintf(b, n, "%d dB", l.dn_snr); break;
    case 6: snprintf(b, n, "%u mW", crsf_tx_power_mw(l.tx_power)); break;
    case 7: snprintf(b, n, "%u", l.rf_mode); break;
    case 8: snprintf(b, n, "%.1f V", l.batt_dv / 10.0f); break;
    case 9: snprintf(b, n, "%.1f A", l.batt_da / 10.0f); break;
    case 10: snprintf(b, n, "%lu mAh", (unsigned long)l.batt_mah); break;
    case 11: snprintf(b, n, "%u%%", l.batt_pct); break;
    default: snprintf(b, n, "--"); break;
  }
}

// Compact form for the main-screen strip, e.g. "LQ100", "R-45", "25mW".
void teleShort(int i, const CrsfLink& l, char* b, size_t n) {
  switch (i) {
    case 0: snprintf(b, n, "LQ%u", l.up_lq); break;
    case 1: snprintf(b, n, "R%d", l.up_rssi); break;
    case 2: snprintf(b, n, "SNR%d", l.up_snr); break;
    case 3: snprintf(b, n, "dLQ%u", l.dn_lq); break;
    case 4: snprintf(b, n, "dR%d", l.dn_rssi); break;
    case 5: snprintf(b, n, "dSNR%d", l.dn_snr); break;
    case 6: snprintf(b, n, "%umW", crsf_tx_power_mw(l.tx_power)); break;
    case 7: snprintf(b, n, "M%u", l.rf_mode); break;
    case 8: snprintf(b, n, l.batt_valid ? "%.1fV" : "V--", l.batt_dv / 10.0f); break;
    case 9: snprintf(b, n, l.batt_valid ? "%.1fA" : "A--", l.batt_da / 10.0f); break;
    case 10: snprintf(b, n, l.batt_valid ? "%lumAh" : "mAh--", (unsigned long)l.batt_mah); break;
    case 11: snprintf(b, n, l.batt_valid ? "b%u%%" : "b--", l.batt_pct); break;
    default: b[0] = 0; break;
  }
}

// Space-joined strip of the selected items (in item order) into out.
void buildTeleStrip(char* out, size_t n, const CrsfLink& l) {
  out[0] = 0;
  size_t len = 0;
  for (int i = 0; i < TELE_N && len < n - 1; ++i) {
    if (!(g_telemSel & (1u << i))) continue;
    char part[16];
    teleShort(i, l, part, sizeof(part));
    int w = snprintf(out + len, n - len, "%s%s", len ? " " : "", part);
    if (w < 0) break;
    len += (size_t)w;
  }
}

uint16_t batteryColor(int pct);  // fwd decls (defined with the battery page, below)
void drawBatteryIcon(int x, int y, int w, int h, int pct, uint16_t fill);

// Bottom strip: the 10 toggle states as cells (ON = green, OFF = dark), sprite-blitted.
// One row of 10 in landscape; two rows of 5 in portrait.
void drawToggleStates(uint16_t mask) {
  constexpr int COLS = PORTRAIT ? 5 : 10, ROWS = 10 / COLS;
  constexpr int CW = LCD_W / COLS;  // 28 px per cell (15 in portrait)
  TFT_eSprite spr(&tft);
  spr.createSprite(LCD_W, 16 * ROWS);
  spr.setTextFont(1);
  spr.setTextSize(1);
  spr.fillSprite(TFT_BLACK);
  for (int i = 0; i < 10; ++i) {
    int x = (i % COLS) * CW + 1, y = (i / COLS) * 16;
    bool on = mask & (1u << i);
    uint16_t bg = on ? TFT_GREEN : 0x2965;  // dark grey when off
    spr.fillRoundRect(x, y + 1, CW - 2, 14, 2, bg);
    char n[4];
    snprintf(n, sizeof(n), "%d", i + 1);
    spr.setTextDatum(MC_DATUM);
    spr.setTextColor(on ? TFT_BLACK : TFT_DARKGREY, bg);
    spr.drawString(n, x + (CW - 2) / 2, y + 8, 1);
  }
  spr.pushSprite(0, LCD_H - 16 * ROWS);
  spr.deleteSprite();
}

// Build the active flight-mode label from the toggle FUNCTION labels + live state.
// FUNC_NAMES indices: 2=ANGLE 3=HORIZ 4=ACRO 5=AIR 6=TURTLE 7=RTH. RTH and TURTLE are
// special states that override the display; otherwise the stabilization mode (ANGLE >
// HORIZON > ACRO default) with " AIR" appended when airmode is on. Returns false (out
// empty) if no toggle is assigned any flight-mode function; sets the display colour.
bool flightModeStr(char* out, size_t n, uint16_t* colOut) {
  uint16_t tm = crsf_toggle_mask();
  bool have = false, angle = false, horizon = false, air = false, turtle = false, rth = false;
  for (int i = 0; i < 10; ++i) {
    uint8_t f = gconfig_toggle_func(i);
    if (f >= 2 && f <= 7) have = true;
    if (tm & (1u << i)) {
      switch (f) {
        case 2: angle = true; break;
        case 3: horizon = true; break;
        case 5: air = true; break;
        case 6: turtle = true; break;
        case 7: rth = true; break;
      }
    }
  }
  if (!have) {
    out[0] = 0;
    return false;
  }
  if (rth) {
    snprintf(out, n, "RTH");
    *colOut = TFT_RED;
  } else if (turtle) {
    snprintf(out, n, "TURTLE");
    *colOut = TFT_ORANGE;
  } else {
    const char* stab = angle ? "ANGLE" : (horizon ? "HORIZ" : "ACRO");
    snprintf(out, n, air ? "%s AIR" : "%s", stab);
    *colOut = TFT_YELLOW;
  }
  return true;
}

void drawRcStatic() { tft.fillScreen(COL_BG); }

// Portrait telemetry block: the selected items stacked one per line (landscape packs them
// into a single strip). Font 2 while they fit the 9-line area, font 1 (12 lines) beyond.
// Every line is sprite-blitted each frame, blanks included, so stale rows clear themselves.
void drawTeleLines(const CrsfLink& l, bool ever, bool online) {
  constexpr int Y0 = 104, AREA_H = 144;
  int count = 0;
  for (int i = 0; i < TELE_N; ++i)
    if (g_telemSel & (1u << i)) ++count;
  const int pitch = (count > AREA_H / 16) ? 12 : 16;
  TFT_eSprite spr(&tft);
  spr.createSprite(LCD_W, pitch);
  spr.setTextFont(1);
  spr.setTextSize(1);
  spr.setTextDatum(ML_DATUM);
  spr.setTextPadding(0);
  spr.setTextColor(online ? TFT_GREENYELLOW : TFT_RED, TFT_BLACK);
  int item = 0;
  for (int s = 0; s < AREA_H / pitch; ++s) {
    char part[16] = "";
    if (g_telemSel == 0) {
      // nothing selected — leave the block blank
    } else if (!online && !ever) {
      if (s == 0) snprintf(part, sizeof(part), "NO TELEMETRY");
    } else {
      while (item < TELE_N && !(g_telemSel & (1u << item))) ++item;
      if (item < TELE_N) teleShort(item++, l, part, sizeof(part));
    }
    spr.fillSprite(TFT_BLACK);
    if (part[0]) {
      bool big = (pitch == 16) && tft.textWidth(part, 2) <= LCD_W - 3;
      spr.drawString(part, 2, pitch / 2, big ? 2 : 1);
    }
    spr.pushSprite(0, Y0 + s * pitch);
  }
  spr.deleteSprite();
}

void renderRc(bool force) {
  PowerReading pr;
  bool ok = power_get(pr);
  int pct = ok ? batteryPct(pr.voltage_V) : -1;
  char b[16];

  // Arm banner (top-right; its own line under the battery row in portrait). "TX OFF"
  // (output disabled) overrides the arm state; otherwise red ARMED / green DISARMED /
  // grey NO ARM. Drawn every frame (padding clears it).
  int as = crsf_arm_state();
  const char* at;
  uint16_t ac;
  if (!crsf_output_enabled()) {
    at = "TX OFF";
    ac = TFT_RED;
  } else {
    at = (as < 0) ? "NO ARM" : (as == 0 ? "DISARMED" : "ARMED");
    ac = (as < 0) ? TFT_DARKGREY : (as == 0 ? TFT_GREEN : TFT_RED);
  }
  tft.setTextColor(ac, TFT_BLACK);
  if (PORTRAIT) {
    tft.setTextDatum(MC_DATUM);
    tft.setTextPadding(LCD_W);
    tft.drawString(at, LCD_W / 2, 34, 2);
  } else {
    tft.setTextDatum(MR_DATUM);
    tft.setTextPadding(96);
    tft.drawString(at, LCD_W - 4, 10, 2);
  }

  // Small battery icon with % inside (top-left), change-driven.
  static int last_pct = -999;
  if (force || pct != last_pct) {
    last_pct = pct;
    int p = pct < 0 ? 0 : pct;
    drawBatteryIcon(2, 2, 28, 18, p, batteryColor(p));
    snprintf(b, sizeof(b), ok ? "%d%%" : "--", p);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE);  // transparent over the icon fill
    tft.drawString(b, 16, 10, 1);
  }

  // Voltage (right of the icon), change-driven.
  static int last_v = -2;
  int vi = ok ? (int)(pr.voltage_V * 100 + 0.5f) : -1;
  if (force || vi != last_v) {
    last_v = vi;
    if (!ok) snprintf(b, sizeof(b), "--V");
    else if (PORTRAIT && pr.voltage_V >= 9.995f)  // only 5 glyphs fit beside the icon
      snprintf(b, sizeof(b), "%.1fV", pr.voltage_V);
    else snprintf(b, sizeof(b), "%.2fV", pr.voltage_V);
    tft.setTextDatum(ML_DATUM);
    tft.setTextPadding(PORTRAIT ? 40 : 72);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString(b, PORTRAIT ? 36 : 42, 10, 2);
  }

  // Flight mode (top-center; third line in portrait) from the assigned mode toggles,
  // change-driven.
  static char last_mode[16] = "?";
  static uint16_t last_mcol = 0;
  char mode[16];
  uint16_t mcol = TFT_YELLOW;
  flightModeStr(mode, sizeof(mode), &mcol);
  if (force || strcmp(mode, last_mode) != 0 || mcol != last_mcol) {
    strcpy(last_mode, mode);
    last_mcol = mcol;
    tft.setTextDatum(MC_DATUM);
    tft.setTextPadding(PORTRAIT ? LCD_W : 80);
    tft.setTextColor(mcol, TFT_BLACK);
    tft.drawString(mode, LCD_W / 2, PORTRAIT ? 54 : 10, 2);
  }

  // Telemetry details (middle), sprite-blitted each frame (values update live). Green when
  // fresh; RED (stale values, or "NO TELEMETRY") when offline; blank if none selected.
  CrsfLink l;
  bool ever = crsf_get(l);
  bool online = crsf_telem_online();
  if (PORTRAIT) {
    drawTeleLines(l, ever, online);
  } else {
    char strip[64];
    buildTeleStrip(strip, sizeof(strip), l);
    TFT_eSprite spr(&tft);
    spr.createSprite(LCD_W, 16);
    spr.fillSprite(TFT_BLACK);
    spr.setTextFont(2);
    spr.setTextSize(1);
    spr.setTextDatum(ML_DATUM);
    spr.setTextPadding(0);
    if (g_telemSel == 0) {
      // nothing selected — leave the strip blank
    } else if (online) {
      spr.setTextColor(TFT_GREENYELLOW, TFT_BLACK);
      spr.drawString(strip, 2, 8, 2);
    } else {
      spr.setTextColor(TFT_RED, TFT_BLACK);
      spr.drawString(ever ? strip : "NO TELEMETRY", 2, 8, 2);
    }
    spr.pushSprite(0, 30);
    spr.deleteSprite();
  }

  // Flight timer, change-driven: small above the toggle row (landscape) / large and
  // centered above the telemetry block (portrait).
  static uint32_t last_s = 0xFFFFFFFFu;
  static int last_armed = -1;
  uint32_t s = flightTimerSession();
  int armed = g_fltArmed ? 1 : 0;
  if (force || s != last_s || armed != last_armed) {
    last_s = s;
    last_armed = armed;
    snprintf(b, sizeof(b), "%02lu:%02lu", (unsigned long)(s / 60), (unsigned long)(s % 60));
    tft.setTextColor(g_fltArmed ? TFT_WHITE : TFT_DARKGREY, TFT_BLACK);
    if (PORTRAIT) {
      tft.setTextDatum(MC_DATUM);
      tft.setTextPadding(70);
      tft.drawString(b, LCD_W / 2, 82, 4);
    } else {
      tft.setTextDatum(ML_DATUM);
      tft.setTextPadding(48);
      tft.drawString(b, 4, 52, 1);
    }
  }

  // 10 toggle states (bottom), change-driven sprite.
  static int last_tm = -1;
  uint16_t tm = crsf_toggle_mask();
  if (force || (int)tm != last_tm) {
    last_tm = tm;
    drawToggleStates(tm);
  }
}

// ---- Config-mode menu system ----
// Navigation via gimbal keys (TASK01.md): pitch >1800us = UP, <1200 = DOWN;
// roll >1800 = OK (enter), <1200 = CANCEL (back).
const char* const MENU_ITEMS[] = {
    lp("GIMBALS & TRIM", "GIMBALS"), lp("TOGGLE MAPPING", "TOGGLES"),
    lp("BATTERY INFO", "BATTERY"),   "TELEMETRY",
    lp("CRSF MODULE", "CRSF"),       "REBOOT"};
constexpr int MENU_N = 6;

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

// Shared scrolling-list geometry: font 2 (16px) rows under the header. Landscape shows
// 4 rows on the 76px-high panel under a one-line header; portrait shows 16 under a
// two-line header (title, then the hint).
constexpr int HDR_H = PORTRAIT ? 24 : 12;
constexpr int LIST_ROW_H = 16;
constexpr int LIST_VISIBLE = (LCD_H - HDR_H) / LIST_ROW_H;  // 4 / 16
constexpr int LIST_ROW_Y0 = HDR_H + LIST_ROW_H / 2;  // center-y of the first visible row
constexpr int LIST_TEXT_X = PORTRAIT ? 2 : 8;        // plain-label rows
constexpr int BUL_X = PORTRAIT ? 6 : 9;              // bullet rows: bullet center-x ...
constexpr int BUL_TEXT_X = PORTRAIT ? 14 : 22;       // ... and the label after it

// Header strip: clears it and draws the yellow title (y only applies in landscape).
void drawHeaderTitle(const char* title, int y = 5) {
  tft.fillRect(0, 0, LCD_W, HDR_H, COL_BG);
  // Portrait rows stop a few px short of the glass; clear that leftover strip here so
  // the list pages still fully repaint without a (flashing) fillScreen.
  constexpr int TAIL = (LCD_H - HDR_H) % LIST_ROW_H;
  if (TAIL) tft.fillRect(0, LCD_H - TAIL, LCD_W, TAIL, COL_BG);
  tft.setTextPadding(0);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_YELLOW, COL_BG);
  tft.drawString(title, PORTRAIT ? 2 : 4, PORTRAIT ? 5 : y, 1);
}

// Header hint/status: right of the title in landscape, on its own second line in portrait
// (keep portrait strings <= 12 chars). pad != 0 = overwrite-in-place for live status.
void drawHeaderHint(const char* hint, uint16_t col, int pad = 0, int y = 5) {
  tft.setTextColor(col, COL_BG);
  if (PORTRAIT) {
    tft.setTextDatum(ML_DATUM);
    tft.setTextPadding(pad ? LCD_W - 2 : 0);
    tft.drawString(hint, 2, 16, 1);
  } else {
    tft.setTextDatum(MR_DATUM);
    tft.setTextPadding(pad);
    tft.drawString(hint, LCD_W - 4, y, 1);
  }
  tft.setTextPadding(0);
}

// "OK = yes / CXL = no" prompt for the confirm dialogs (caller sets datum MC + colour).
void drawYesNo(int y) {
  if (PORTRAIT) {
    tft.drawString("OK = yes", LCD_W / 2, y, 2);
    tft.drawString("CXL = no", LCD_W / 2, y + 18, 2);
  } else {
    tft.drawString("OK = yes     CXL = no", LCD_W / 2, y, 1);
  }
}

void drawMenuHeader() {
  drawHeaderTitle("CONFIG");
  drawHeaderHint(lp("OK:enter  CXL:exit", "OK:enter"), TFT_DARKGREY);
}

// Draw one visible menu row into an off-screen sprite and blit it once (no flash).
// slot = visual position 0..LIST_VISIBLE-1; idx = menu item index (or out of range = blank).
void drawMenuRow(int slot, int idx, bool sel) {
  int y = LIST_ROW_Y0 + slot * LIST_ROW_H;
  uint16_t bg = sel ? TFT_NAVY : COL_BG;
  TFT_eSprite spr(&tft);
  spr.createSprite(LCD_W, LIST_ROW_H);
  spr.setTextFont(1);  // init inherited text state (gfxFont/size are garbage in a new sprite)
  spr.setTextSize(1);
  spr.fillSprite(bg);
  if (idx >= 0 && idx < MENU_N) {
    spr.setTextDatum(ML_DATUM);
    spr.setTextPadding(0);
    spr.setTextColor(sel ? TFT_WHITE : TFT_DARKGREY, bg);
    spr.drawString(MENU_ITEMS[idx], LIST_TEXT_X, LIST_ROW_H / 2,
                   fitFont(MENU_ITEMS[idx], 2, LCD_W - LIST_TEXT_X - 1));
  }
  spr.pushSprite(0, y - LIST_ROW_H / 2);
  spr.deleteSprite();
}

void drawMenuListFull(int scroll, int sel) {
  // Header + sprite rows cover the panel, so no fillScreen (which would flash).
  drawMenuHeader();
  for (int s = 0; s < LIST_VISIBLE; ++s) {
    int idx = scroll + s;
    drawMenuRow(s, idx, idx == sel);
  }
}

// --- Menu item: Telemetry — live CRSF link stats; OK toggles "show on main screen". ---
void drawTeleHeader() {
  drawHeaderTitle("TELEMETRY");
  drawHeaderHint(lp("OK:main  CXL:back", "OK:show/hide"), TFT_DARKGREY);
}

// Telemetry rows: one list row in landscape (name left, value right); in portrait name
// and value can't share 76 px, so each item takes a double-height row (name over value).
constexpr int TE_ROW_H = PORTRAIT ? 2 * LIST_ROW_H : LIST_ROW_H;
constexpr int TE_VISIBLE = (LCD_H - HDR_H) / TE_ROW_H;  // 4 / 8
constexpr int TE_ROW_Y0 = HDR_H + TE_ROW_H / 2;

// One telemetry row: selection bullet + name + live value. Sprite blit.
void drawTeleRow(int slot, int item, bool cursor, const CrsfLink& l, bool lok) {
  int y = TE_ROW_Y0 + slot * TE_ROW_H;
  uint16_t bg = cursor ? TFT_NAVY : COL_BG;
  TFT_eSprite spr(&tft);
  spr.createSprite(LCD_W, TE_ROW_H);
  spr.setTextFont(1);  // init inherited text state (gfxFont/size are garbage in a new sprite)
  spr.setTextSize(1);
  spr.fillSprite(bg);
  if (item >= 0 && item < TELE_N) {
    bool on = g_telemSel & (1u << item);
    char v[16];
    teleValue(item, l, lok, v, sizeof(v));
    spr.setTextPadding(0);
    // selection bullet: filled yellow = shown on main; hollow grey = not.
    if (PORTRAIT) {
      if (on) spr.fillCircle(4, 8, 3, TFT_YELLOW);
      else spr.drawCircle(4, 8, 3, TFT_DARKGREY);
      spr.setTextDatum(ML_DATUM);
      spr.setTextColor(TFT_WHITE, bg);
      spr.drawString(TELE_ITEMS[item].name, 10, 8, 1);
      spr.setTextDatum(MR_DATUM);
      spr.setTextColor(lok ? TFT_CYAN : TFT_DARKGREY, bg);
      spr.drawString(v, LCD_W - 2, 23, 2);
    } else {
      int cy = TE_ROW_H / 2;
      if (on) spr.fillCircle(9, cy, 4, TFT_YELLOW);
      else spr.drawCircle(9, cy, 4, TFT_DARKGREY);
      spr.setTextDatum(ML_DATUM);
      spr.setTextColor(TFT_WHITE, bg);
      spr.drawString(TELE_ITEMS[item].name, 22, cy, 2);
      spr.setTextDatum(MR_DATUM);
      spr.setTextColor(lok ? TFT_CYAN : TFT_DARKGREY, bg);
      spr.drawString(v, LCD_W - 6, cy, 2);
    }
  }
  spr.pushSprite(0, y - TE_ROW_H / 2);
  spr.deleteSprite();
}

void drawTeleVisible(int scroll, int cursor, const CrsfLink& l, bool lok) {
  for (int s = 0; s < TE_VISIBLE; ++s) {
    int item = scroll + s;
    drawTeleRow(s, item, item == cursor, l, lok);
  }
}

void drawTeleFull(int scroll, int cursor, const CrsfLink& l, bool lok) {
  drawTeleHeader();
  drawTeleVisible(scroll, cursor, l, lok);
}

// --- Menu item: TX Power. OK sends a CRSF param write; module clamps to its range and
// reports the actual power back via telemetry (green bullet = module's current power). ---
// Index == the value byte sent = index into the module's filtered power table, which
// starts at its MIN power. The Nano TX V2's min is 25 mW (there is no 10 mW), so the list
// starts at 25 mW; otherwise every level lands one step high.
const uint16_t POWER_MW[] = {25, 50, 100, 250, 500, 1000};
constexpr int POWER_N = 6;

// force = full redraw (clears strip + draws title); otherwise only the NOW/SET status is
// refreshed, and only when it changes, via text padding (no fillRect) so it never flickers.
void drawPowerHeader(uint16_t now_mw, int sent, bool force) {
  static int last_now = -99999, last_sent = -99;
  if (!force && (int)now_mw == last_now && sent == last_sent) return;
  last_now = now_mw;
  last_sent = sent;
  if (force) drawHeaderTitle("TX POWER");
  char nows[8];
  if (now_mw) snprintf(nows, sizeof(nows), "%u", now_mw);
  else snprintf(nows, sizeof(nows), "--");
  char b[28];
  // Portrait has room for NOW only; the level we SET is the cyan row in the list.
  if (PORTRAIT) snprintf(b, sizeof(b), "NOW:%s mW", nows);
  else if (sent >= 0) snprintf(b, sizeof(b), "NOW:%s SET:%u", nows, POWER_MW[sent]);
  else snprintf(b, sizeof(b), "NOW:%s  OK:set", nows);
  drawHeaderHint(b, sent >= 0 ? TFT_GREEN : TFT_DARKGREY, 150);
}

// green bullet = module's current power (telemetry); cyan text = the level we last sent.
void drawPowerRow(int slot, int idx, bool cursor, uint16_t now_mw, int sent) {
  int y = LIST_ROW_Y0 + slot * LIST_ROW_H;
  uint16_t bg = cursor ? TFT_NAVY : COL_BG;
  TFT_eSprite spr(&tft);
  spr.createSprite(LCD_W, LIST_ROW_H);
  spr.setTextFont(1);
  spr.setTextSize(1);
  spr.fillSprite(bg);
  if (idx >= 0 && idx < POWER_N) {
    int cy = LIST_ROW_H / 2;
    if (now_mw == POWER_MW[idx]) spr.fillCircle(BUL_X, cy, 4, TFT_GREEN);
    else spr.drawCircle(BUL_X, cy, 4, TFT_DARKGREY);
    char b[12];
    snprintf(b, sizeof(b), "%u mW", POWER_MW[idx]);
    spr.setTextDatum(ML_DATUM);
    spr.setTextColor(idx == sent ? TFT_CYAN : TFT_WHITE, bg);
    spr.drawString(b, BUL_TEXT_X, cy, fitFont(b, 2, LCD_W - BUL_TEXT_X - 1));
  }
  spr.pushSprite(0, y - LIST_ROW_H / 2);
  spr.deleteSprite();
}

void drawPowerVisible(int scroll, int cursor, uint16_t now_mw, int sent) {
  for (int s = 0; s < LIST_VISIBLE; ++s) {
    int idx = scroll + s;
    drawPowerRow(s, idx, idx == cursor, now_mw, sent);
  }
}

void drawPowerFull(int scroll, int cursor, uint16_t now_mw, int sent) {
  drawPowerHeader(now_mw, sent, true);
  drawPowerVisible(scroll, cursor, now_mw, sent);
}

// --- Menu item: FAN threshold — the TX power at/above which the module's fan runs.
// Blind CRSF parameter write to "Fan Thresh" (ELRS 3.4.x field 8; value = raw option
// index, no min-shift). No telemetry read-back for the fan, so no live "current" marker.
const char* const FAN_OPTS[] = {"10mW",  "25mW",  "50mW",   "100mW", "250mW",
                                "500mW", "1000mW", "2000mW", "NEVER"};
constexpr int FAN_N = 9;

void drawSettingHeader(const char* title, int sent) {
  drawHeaderTitle(title);
  drawHeaderHint(sent >= 0 ? "SAVED" : lp("OK:set  CXL:back", "OK:set"),
                 sent >= 0 ? TFT_GREEN : TFT_DARKGREY);
}

// green bullet = module's current threshold; cyan text = the option we just wrote.
void drawFanRow(int slot, const char* label, bool cursor, bool is_current, bool is_sent,
                int font) {
  int y = LIST_ROW_Y0 + slot * LIST_ROW_H;
  uint16_t bg = cursor ? TFT_NAVY : COL_BG;
  TFT_eSprite spr(&tft);
  spr.createSprite(LCD_W, LIST_ROW_H);
  spr.setTextFont(1);
  spr.setTextSize(1);
  spr.fillSprite(bg);
  if (label) {
    int cy = LIST_ROW_H / 2;
    if (is_current) spr.fillCircle(BUL_X, cy, 4, TFT_GREEN);
    else spr.drawCircle(BUL_X, cy, 4, TFT_DARKGREY);
    spr.setTextDatum(ML_DATUM);
    spr.setTextColor(is_sent ? TFT_CYAN : TFT_WHITE, bg);
    spr.drawString(label, BUL_TEXT_X, cy, font);
  }
  spr.pushSprite(0, y - LIST_ROW_H / 2);
  spr.deleteSprite();
}

// Rows only (sprite blits, no header) — use for navigation/telemetry refresh so the header
// fillRect doesn't flash on every cursor move.
void drawSettingRows(int scroll, int cursor, const char* const* labels, int n, int current,
                     int sent) {
  // One font for the whole list: font 2 unless some label would overflow (portrait).
  int font = 2;
  for (int i = 0; i < n; ++i)
    if (fitFont(labels[i], 2, LCD_W - BUL_TEXT_X - 1) < font) font = 1;
  for (int s = 0; s < LIST_VISIBLE; ++s) {
    int idx = scroll + s;
    drawFanRow(s, idx < n ? labels[idx] : nullptr, idx == cursor, idx == current, idx == sent,
               font);
  }
}

void drawSettingList(const char* title, int scroll, int cursor, const char* const* labels,
                     int n, int current, int sent) {
  drawSettingHeader(title, sent);  // full redraw (entry / OK) — header fillRect
  drawSettingRows(scroll, cursor, labels, n, current, sent);
}

// --- Menu item: PACKET RATE (ELRS field 1). value = list index; verified via telemetry
// rf_Mode. NOTE: at 400000 baud on this half-duplex module, rates above 333Hz are capped by
// the module — the green dot (actual rf_Mode) reveals it. Use BAUD 921600 to unlock them. ---
const char* const PKT_OPTS[] = {
    "50Hz",  lp("100Hz FULL", "100 FULL"), "150Hz", "250Hz", lp("333Hz FULL", "333 FULL"),
    "500Hz", "D250",                       "D500",  "F500",  "F1000"};
constexpr int PKT_N = 10;
// rf_Mode (LinkStatistics enum_rate) the module reports for each option index above.
const uint8_t PKT_RFMODE[PKT_N] = {2, 4, 5, 7, 8, 9, 10, 11, 12, 13};

// Index of the option whose rf_Mode matches the module's reported rate (-1 if unknown).
int pktCurrentIndex(uint8_t rf_mode, bool link_ok) {
  if (!link_ok) return -1;
  for (int i = 0; i < PKT_N; ++i)
    if (PKT_RFMODE[i] == rf_mode) return i;
  return -1;
}

// --- Menu item: SWITCH MODE (ELRS field 3 on 3.4.x) — how many channels go over the air.
// Blind write, value = option index, and the module's option list depends on the packet
// rate: normal rates offer {Wide, Hybrid} — both carry CH1-12 only, so toggles mapped to
// CH13-16 never reach the receiver. The FULL rates (100Hz FULL / 333Hz FULL) offer
// {8ch, 16ch Rate/2, 12ch Mixed}; "16ch Rate/2" is the one that carries all 16 channels.
// No telemetry read-back, so there is no "current" marker. The module only accepts a
// switch-mode change while no receiver is connected. ---
const char* const SWMODE_OPTS[] = {lp("0: Wide  | FULL: 8ch", "Wide/8ch"),
                                   lp("1: Hybrid| FULL: 16ch Rate/2", "Hyb/16ch"),
                                   lp("2:  -    | FULL: 12ch Mixed", "-/12ch Mix")};
constexpr int SWMODE_N = 3;

// --- Menu item: BAUD RATE (handset UART; module auto-detects). ---
const char* const BAUD_OPTS[] = {"400000", "921600"};
const uint32_t BAUD_VALS[] = {400000, 921600};
constexpr int BAUD_N = 2;

// --- Menu item 1: Gimbals position + channel reverse (flip).
// Toggle5 ON = edit mode: pitch up/down moves the cursor, roll-right (OK) flips the
// selected channel (saved to NVS), roll-left (CANCEL) backs out. Toggle5 OFF = view only.
void drawGimbalPosStatic() {
  tft.fillScreen(COL_BG);
  drawHeaderTitle("GIMBALS", 6);
  drawHeaderHint(lp("T5:edit/back  OK:flip", "T5+OK:flip"), TFT_DARKGREY, 0, 6);
}

void drawGimbalPage(int cursor, bool edit) {
  static const char* const NAME[4] = {"THR", "YAW", "PIT", "ROL"};
  GimbalState g;
  gimbals_get(g);
  const int raw[4] = {g.throttle, g.yaw, g.pitch, g.roll};
  // Landscape: one line per channel (cursor, name, bar, %, flag). Portrait: the text
  // fields share a line and the bar runs full-width under it.
  const int top = PORTRAIT ? 30 : 16, rowh = PORTRAIT ? 34 : 15;
  const int bx = PORTRAIT ? 2 : 30, bw = PORTRAIT ? LCD_W - 4 : 150;
  const int pctx = PORTRAIT ? 52 : bx + bw + 34;  // right edge of the percent field
  char b[10];
  for (int i = 0; i < 4; ++i) {
    bool sel = edit && (i == cursor);
    bool fl = gconfig_flip(i);
    float pct = stickPct(i, raw[i]);  // flip-aware position
    int y = top + i * rowh;
    int cy = PORTRAIT ? y + 5 : y + rowh / 2;                  // text line center
    int by = PORTRAIT ? y + 13 : y + 2, bh = PORTRAIT ? 12 : rowh - 4;  // bar
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
    tft.fillRect(bx, by, fw, bh, TFT_GREEN);
    tft.fillRect(bx + fw, by, bw - fw, bh, TFT_DARKGREY);
    // Percent.
    snprintf(b, sizeof(b), "%3d%%", (int)(pct + 0.5f));
    tft.setTextDatum(MR_DATUM);
    tft.setTextPadding(PORTRAIT ? 24 : 30);  // portrait: must not reach back into the name
    tft.setTextColor(TFT_WHITE, COL_BG);
    tft.drawString(b, pctx, cy, 1);
    // Reverse flag.
    tft.setTextPadding(PORTRAIT ? 18 : 28);
    tft.setTextColor(fl ? TFT_RED : TFT_DARKGREY, COL_BG);
    tft.drawString(fl ? "REV" : "NOR", LCD_W - 2, cy, 1);
  }
}

void drawRebootConfirm() {
  tft.fillScreen(COL_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_RED, COL_BG);
  tft.drawString("REBOOT?", LCD_W / 2, PORTRAIT ? 110 : 24, fitFont("REBOOT?", 4, LCD_W - 4));
  tft.setTextColor(TFT_WHITE, COL_BG);
  drawYesNo(PORTRAIT ? 150 : 56);
}

void doReboot() {
  tft.fillScreen(COL_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_YELLOW, COL_BG);
  tft.drawString("REBOOTING", LCD_W / 2, LCD_H / 2, fitFont("REBOOTING", 4, LCD_W - 4));
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
  tft.fillRect(x + w, y + (h - nubh) / 2, 3, nubh, TFT_WHITE);  // terminal (short tip)
  int ix = x + 3, iy = y + 3, iw = w - 6, ih = h - 6;  // inner
  int fw = iw * constrain(pct, 0, 100) / 100;
  tft.fillRect(ix, iy, fw, ih, fill);
  tft.fillRect(ix + fw, iy, iw - fw, ih, COL_BG);
}

void drawBatteryStatic() {
  tft.fillScreen(COL_BG);
  drawHeaderTitle("BATTERY", 6);
  drawHeaderHint("CXL=back", TFT_DARKGREY, 0, 6);

  // Lifetime total flight time (accumulated armed seconds from NVS), below the icon
  // (landscape) / at the foot of the screen on two lines (portrait).
  uint32_t tot = storage_get_total_flight_secs();
  char b[28];
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_DARKGREY, COL_BG);
  if (PORTRAIT) {
    snprintf(b, sizeof(b), "%luh%02lum", (unsigned long)(tot / 3600),
             (unsigned long)((tot % 3600) / 60));
    tft.drawString("TOTAL FLIGHT", 2, LCD_H - 24, 1);
    tft.drawString(b, 2, LCD_H - 12, 1);
  } else {
    snprintf(b, sizeof(b), "TOTAL FLIGHT %luh%02lum", (unsigned long)(tot / 3600),
             (unsigned long)((tot % 3600) / 60));
    tft.drawString(b, 4, 70, 1);
  }
}

// Redraw only what changed (force = redraw all, e.g. on page entry) to avoid the
// erase/redraw flash — especially the % which sits transparently over the icon fill.
void drawBatteryDynamic(bool force) {
  static int last_pct = -999, last_v = -1, last_i = -2000000, last_p = -1;
  PowerReading pr;
  bool ok = power_get(pr);
  int pct = ok ? batteryPct(pr.voltage_V) : -1;

  // Landscape: icon left, V/A/W column right. Portrait: icon on top, V/A/W under it.
  const int bx = PORTRAIT ? 6 : 8, by = PORTRAIT ? 32 : 24;
  const int bw = PORTRAIT ? 60 : 96, bh = PORTRAIT ? 34 : 40;
  const int rx = LCD_W - 4;
  const int vy = PORTRAIT ? 88 : 24, vstep = PORTRAIT ? 22 : 20;  // V / A / W lines
  char b[16];

  if (force || pct != last_pct) {  // icon fill + % only on change
    last_pct = pct;
    int p = pct < 0 ? 0 : pct;
    drawBatteryIcon(bx, by, bw, bh, p, batteryColor(p));
    snprintf(b, sizeof(b), ok ? "%d%%" : "--", p);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE);  // transparent over the freshly-drawn fill
    tft.drawString(b, bx + bw / 2, by + bh / 2, PORTRAIT ? 2 : 4);
  }

  int vi = ok ? (int)(pr.voltage_V * 100 + 0.5f) : -1;
  int ii = ok ? (int)pr.current_mA : -2000000;
  int pi = ok ? (int)(pr.power_mW / 10.0f + 0.5f) : -1;  // 0.01 W units

  tft.setTextDatum(MR_DATUM);
  tft.setTextPadding(PORTRAIT ? LCD_W - 6 : 96);
  if (force || vi != last_v) {
    last_v = vi;
    tft.setTextColor(TFT_WHITE, COL_BG);
    if (ok) snprintf(b, sizeof(b), "%.2f V", pr.voltage_V);
    else snprintf(b, sizeof(b), "-- V");
    tft.drawString(b, rx, vy, 2);
  }
  if (force || ii != last_i) {
    last_i = ii;
    tft.setTextColor(ok && pr.near_full ? TFT_RED : TFT_CYAN, COL_BG);
    if (!ok) snprintf(b, sizeof(b), "-- A");
    else if (fabsf(pr.current_mA) >= 1000.0f) snprintf(b, sizeof(b), "%.2f A", pr.current_mA / 1000.0f);
    else snprintf(b, sizeof(b), "%.0f mA", pr.current_mA);
    tft.drawString(b, rx, vy + vstep, 2);
  }
  if (force || pi != last_p) {
    last_p = pi;
    tft.setTextColor(TFT_GREENYELLOW, COL_BG);
    if (ok) snprintf(b, sizeof(b), "%.2f W", pr.power_mW / 1000.0f);
    else snprintf(b, sizeof(b), "-- W");
    tft.drawString(b, rx, vy + 2 * vstep, 2);
  }
}

// --- Menu item 2: Toggle -> MCP23017 pin + invert + CRSF channel + function. All 10
// toggles are listed; in the standard build Toggle6 (menu-mode switch) only has a pin
// and invert to assign.
const int EDIT_TOGGLE[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};  // toggle index per list row
constexpr int EDIT_TOGGLE_N = 10;

// "A0".."B7" for MCP bit 0..15, "--" when unassigned.
void pinName(uint8_t pin, char* out, size_t n) {
  if (pin <= 15) snprintf(out, n, "%c%d", pin < 8 ? 'A' : 'B', pin % 8);
  else snprintf(out, n, "--");
}

// Function labels a toggle can be assigned (index stored in gconfig). ARM is special
// (drives arm-safety + flight timer); the rest are labels the flight controller acts on.
const char* const FUNC_NAMES[] = {"-",   "ARM", "ANGLE", "HORIZ", "ACRO",
                                  "AIR", "TURTLE", "RTH", "BEEP", "PREARM"};
constexpr int FUNC_COUNT = 10;
constexpr uint8_t FUNC_ARM = GFUNC_ARM;  // shared with crsf.cpp arm-safety

// Scrolling list geometry. Landscape shares the main-menu rows (toggle, channel, function
// across one line); portrait uses double-height rows (toggle + channel over function).
constexpr int TM_ROW_H = PORTRAIT ? 2 * LIST_ROW_H : LIST_ROW_H;
constexpr int TM_VISIBLE = (LCD_H - HDR_H) / TM_ROW_H;  // 4 / 8
constexpr int TM_ROW_Y0 = HDR_H + TM_ROW_H / 2;

void drawToggleHeader(bool editing, int field) {
  drawHeaderTitle("TOGGLE MAP");  // clears the header strip only
  const char* hint = !editing       ? lp("OK:edit  CXL:back", "OK:edit")
                     : (field == 0) ? lp("PIN: flip switch or UP/DN", "flip sw/UPDN")
                     : (field == 1) ? lp("INVERT: UP/DN  OK:next", "INV  OK:next")
                     : (field == 2) ? lp("edit CH   OK:next", "CH   OK:next")
                                    : lp("edit FUNC OK:save", "FUNC OK:save");
  drawHeaderHint(hint, TFT_DARKGREY);
}

// Draw one visible row. slot = visual position 0..TM_VISIBLE-1; eidx = index into
// EDIT_TOGGLE (or out of range for a blank slot on the last short page).
// field: 0 = pin, 1 = invert, 2 = channel, 3 = function (highlighted green when sel &&
// editing). pin_edit / inv_edit >= 0: the not-yet-committed pin / invert flag being edited
// on the selected row. Landscape shows invert as its own NOR/INV column; portrait has no
// room, so an inverted pin is written "!B3".
// Rendered into an off-screen sprite and blitted once, so a moving cursor never
// flashes the row as a solid bar between clear and text.
void drawToggleRow(int slot, int eidx, bool sel, bool editing, int field, int pin_edit = -1,
                   int inv_edit = -1) {
  int y = TM_ROW_Y0 + slot * TM_ROW_H;
  uint16_t bg = sel ? TFT_NAVY : COL_BG;
  TFT_eSprite spr(&tft);
  spr.createSprite(LCD_W, TM_ROW_H);
  spr.setTextFont(1);  // init inherited text state (gfxFont/size are garbage in a new sprite)
  spr.setTextSize(1);
  spr.fillSprite(bg);
  if (eidx >= 0 && eidx < EDIT_TOGGLE_N) {
    int t = EDIT_TOGGLE[eidx];
    uint8_t c = gconfig_toggle_chan(t);
    uint8_t fn = gconfig_toggle_func(t);
    char ts[6], ps[6], cs[6], fs[10];
    snprintf(ts, sizeof(ts), "T%d", t + 1);
    bool inv = (sel && editing && inv_edit >= 0) ? (inv_edit != 0) : gconfig_toggle_inv(t);
    ps[0] = '!';  // portrait prefix for an inverted pin; skipped otherwise
    char* pn = ps + 1;
    pinName((sel && editing && pin_edit >= 0) ? (uint8_t)pin_edit : gconfig_toggle_pin(t), pn,
            sizeof(ps) - 1);
    if (c) snprintf(cs, sizeof(cs), "CH%u", c);
    else snprintf(cs, sizeof(cs), "--");
    if (gconfig_toggle_reserved(t)) snprintf(fs, sizeof(fs), "MENU");  // the mode switch
    else snprintf(fs, sizeof(fs), "%s", FUNC_NAMES[fn < FUNC_COUNT ? fn : 0]);

    uint16_t base = sel ? TFT_YELLOW : TFT_WHITE;
    uint16_t pinCol = (sel && editing && field == 0) ? TFT_GREEN : base;
    uint16_t invCol = (sel && editing && field == 1) ? TFT_GREEN : base;
    uint16_t chCol = (sel && editing && field == 2) ? TFT_GREEN : base;
    uint16_t fnCol = (sel && editing && field == 3) ? TFT_GREEN : base;
    spr.setTextDatum(ML_DATUM);
    spr.setTextPadding(0);
    spr.setTextColor(base, bg);
    if (PORTRAIT) {  // line 1: toggle ... channel; line 2: [!]pin ... function
      spr.drawString(ts, 2, 8, 2);
      spr.setTextColor((invCol == TFT_GREEN) ? invCol : pinCol, bg);
      spr.drawString(inv ? ps : pn, 2, 24, 2);
      spr.setTextDatum(MR_DATUM);
      spr.setTextColor(chCol, bg);
      spr.drawString(cs, LCD_W - 2, 8, 2);
      spr.setTextColor(fnCol, bg);
      spr.drawString(fs, LCD_W - 2, 24, 2);
    } else {
      int cy = TM_ROW_H / 2;  // vertical center inside the sprite
      spr.drawString(ts, 8, cy, 2);
      spr.setTextColor(pinCol, bg);
      spr.drawString(pn, 44, cy, 2);
      spr.setTextColor(inv && invCol == base ? TFT_RED : invCol, bg);
      spr.drawString(inv ? "INV" : "NOR", 76, cy, 2);
      spr.setTextColor(chCol, bg);
      spr.drawString(cs, 118, cy, 2);
      spr.setTextColor(fnCol, bg);
      spr.drawString(fs, 174, cy, 2);
    }
  }
  spr.pushSprite(0, y - TM_ROW_H / 2);
  spr.deleteSprite();
}

void drawToggleListFull(int scroll, int cursor, bool editing, int field) {
  // Header + sprite rows fully cover the panel, so no fillScreen (which would flash).
  drawToggleHeader(editing, field);
  for (int s = 0; s < TM_VISIBLE; ++s) {
    int eidx = scroll + s;
    drawToggleRow(s, eidx, eidx == cursor, editing, field);
  }
}

// --- Menu item 4: CRSF module status (BetaFPV Nano TX V2).
// CRSF MODULE submenu: OUTPUT toggle + links to the module-config pages + WiFi setup.
const char* const CRSF_ITEMS[] = {
    "OUTPUT",                        "TX POWER",                    lp("FAN THRESHOLD", "FAN"),
    lp("PACKET RATE", "PKT RATE"),   lp("SWITCH MODE", "SW MODE"),  "BAUD RATE",
    lp("BLE JOYSTICK", "BLE JOY"),   lp("BIND RECEIVER", "BIND RX"), "WIFI SETUP"};
constexpr int CRSF_N = 9;
constexpr int CRSF_ITEM_OUTPUT = 0;
constexpr int CRSF_ITEM_BLE = 6;
constexpr int CRSF_ITEM_BIND = 7;
constexpr int CRSF_ITEM_WIFI = 8;
bool g_bindSent = false;  // BIND row shows "SENT" after OK (cleared on entering the page)

// Compact live status (right side): module online + baud / actual RF rate. Change-driven
// with text padding (no fillRect) so the periodic refresh doesn't flicker.
void drawCrsfMenuStatus(bool force) {
  static uint32_t prev_rx = 0;
  static int last_state = -2;
  CrsfLink l;
  crsf_get(l);
  int online = (l.rx_frames != prev_rx) ? 1 : 0;
  prev_rx = l.rx_frames;
  bool ble = ble_on();
  int pi = online ? pktCurrentIndex((uint8_t)l.rf_mode, l.valid && l.up_lq > 0) : -2;
  int state = ble ? 200 : (online ? (100 + pi) : -1);  // change detection
  if (!force && state == last_state) return;
  last_state = state;
  char b[24];
  uint16_t col;
  if (ble) {
    snprintf(b, sizeof(b), "BLE JOYSTICK");
    col = TFT_CYAN;
  } else if (online) {
    char rate[10];
    if (pi >= 0) snprintf(rate, sizeof(rate), "%s", PKT_OPTS[pi]);
    else snprintf(rate, sizeof(rate), "--");
    // Portrait drops the "k " to stay within 12 chars: "921/333 FULL".
    snprintf(b, sizeof(b), PORTRAIT ? "%lu/%s" : "%luk %s",
             (unsigned long)(crsf_get_baud() / 1000), rate);
    col = TFT_GREEN;
  } else {
    snprintf(b, sizeof(b), "OFFLINE");
    col = TFT_RED;
  }
  drawHeaderHint(b, col, 150);  // padded: overwrites the old status in one draw (no clear)
}

void drawCrsfMenuRow(int slot, int item, bool cursor) {
  int y = LIST_ROW_Y0 + slot * LIST_ROW_H;
  uint16_t bg = cursor ? TFT_NAVY : COL_BG;
  TFT_eSprite spr(&tft);
  spr.createSprite(LCD_W, LIST_ROW_H);
  spr.setTextFont(1);
  spr.setTextSize(1);
  spr.fillSprite(bg);
  if (item >= 0 && item < CRSF_N) {
    int cy = LIST_ROW_H / 2;
    spr.setTextDatum(ML_DATUM);
    spr.setTextPadding(0);
    // Portrait: the ON/OFF state drops to font 1 so it fits beside a font-2 label.
    const int tx = PORTRAIT ? LIST_TEXT_X : 10;
    const int vx = PORTRAIT ? LCD_W - 1 : LCD_W - 8, vfont = PORTRAIT ? 1 : 2;
    const bool has_val = (item == CRSF_ITEM_OUTPUT || item == CRSF_ITEM_BLE ||
                          (item == CRSF_ITEM_BIND && g_bindSent));
    spr.setTextColor(cursor ? TFT_WHITE : TFT_DARKGREY, bg);
    spr.drawString(CRSF_ITEMS[item], tx, cy,
                   fitFont(CRSF_ITEMS[item], 2, LCD_W - tx - 1 - (has_val ? 20 : 0)));
    if (item == CRSF_ITEM_OUTPUT) {  // OUTPUT ON/OFF
      bool on = crsf_output_enabled();
      spr.setTextDatum(MR_DATUM);
      spr.setTextColor(on ? TFT_GREEN : TFT_RED, bg);
      spr.drawString(on ? "ON" : "OFF", vx, cy, vfont);
    } else if (item == CRSF_ITEM_BLE) {  // BLE JOYSTICK ON/OFF
      bool on = ble_on();
      spr.setTextDatum(MR_DATUM);
      spr.setTextColor(on ? TFT_CYAN : TFT_DARKGREY, bg);
      spr.drawString(on ? "ON" : "OFF", vx, cy, vfont);
    } else if (item == CRSF_ITEM_BIND && g_bindSent) {  // bind progress (crsf_bind_state)
      int st = crsf_bind_state();
      const char* txt = st == BIND_SCAN   ? ".."
                        : st == BIND_RX   ? "RX.."
                        : st == BIND_TX   ? "TX.."
                        : st == BIND_DONE ? "OK"
                                          : lp("FAIL", "ERR");
      spr.setTextDatum(MR_DATUM);
      spr.setTextColor(st == BIND_DONE ? TFT_GREEN : st == BIND_FAILED ? TFT_RED : TFT_CYAN, bg);
      spr.drawString(txt, vx, cy, vfont);
    }
  }
  spr.pushSprite(0, y - LIST_ROW_H / 2);
  spr.deleteSprite();
}

void drawCrsfMenu(int scroll, int cursor) {
  // Full redraw (entry / scroll): header strip + title once, then status + rows.
  drawHeaderTitle("CRSF MODULE");
  drawCrsfMenuStatus(true);
  for (int s = 0; s < LIST_VISIBLE; ++s) {
    int item = scroll + s;
    drawCrsfMenuRow(s, item, item == cursor);
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
  if (PORTRAIT) {
    tft.drawString("MODULE", LCD_W / 2, 84, 2);
    tft.drawString("WIFI", LCD_W / 2, 102, 2);
    tft.drawString("SETUP?", LCD_W / 2, 120, 2);
    tft.setTextColor(TFT_WHITE, COL_BG);
    tft.drawString("reboots with", LCD_W / 2, 144, 1);
    tft.drawString("CRSF off", LCD_W / 2, 156, 1);
    drawYesNo(184);
    return;
  }
  tft.drawString("MODULE WIFI SETUP?", LCD_W / 2, 16, 2);
  tft.setTextColor(TFT_WHITE, COL_BG);
  tft.drawString("reboots with CRSF off", LCD_W / 2, 38, 1);
  drawYesNo(58);
}

// Shown at boot while the WiFi-boot flag is set (CRSF never started).
void drawWifiSetup() {
  tft.fillScreen(COL_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_CYAN, COL_BG);
  // Handset TX is silent in this mode; the module auto-starts WiFi after its
  // "Auto On WiFi" interval IF it powers up seeing no CRSF (so power-cycle it now).
  if (PORTRAIT) {  // same steps, wrapped to 12 chars per line
    tft.drawString("MODULE WIFI", LCD_W / 2, 8, 1);
    tft.drawString("SETUP", LCD_W / 2, 20, 1);
    static const char* const STEPS[] = {
        "1.Power-",    "  cycle the", "  module now", "",
        "2.Wait ~60s", "  TX silent", "",
        "3.Join AP",   "  ExpressLRS", "  TX",        "  password:", "  expresslrs", "",
        "4.Browse to", "  10.0.0.1"};
    tft.setTextDatum(ML_DATUM);
    tft.setTextColor(TFT_WHITE, COL_BG);
    int y = 40;
    for (const char* line : STEPS) {
      tft.drawString(line, 2, y, 1);
      y += line[0] ? 12 : 6;  // blank entry = half-line gap between steps
    }
    tft.setTextColor(TFT_YELLOW, COL_BG);
    tft.drawString(MENU_ON_BOOT_BTN ? "Press BOOT" : "Toggle6 OFF", 2, LCD_H - 24, 1);
    tft.drawString("= exit & fly", 2, LCD_H - 12, 1);
    return;
  }
  tft.drawString("MODULE WIFI SETUP", LCD_W / 2, 8, 1);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_WHITE, COL_BG);
  tft.drawString("1. Power-cycle module now", 6, 20, 1);
  tft.drawString("2. Wait ~60s (TX stays silent)", 6, 32, 1);
  tft.drawString("3. AP: ExpressLRS TX  pw:expresslrs", 6, 44, 1);
  tft.drawString("4. http://10.0.0.1", 6, 56, 1);
  tft.setTextColor(TFT_YELLOW, COL_BG);
  tft.drawString("Toggle6 OFF = exit & fly", 6, 68, 1);
}

// Config menu loop — runs while menu mode is on (Toggle6 / pocket BOOT button).
void configLoop() {
  // Main-menu pages come first (List..Reboot map to menu index via 1+sel); the CRSF
  // sub-pages follow and are reached only from the CRSF MODULE submenu.
  enum class Page : uint8_t {
    List, Gimbals, Toggles, Battery, Telemetry, Crsf, Reboot, Power, Fan, PktRate, SwMode,
    Baud
  };
  Page page = Page::List;
  int sel = 0;
  int menu_scroll = 0;  // index of the top visible main-menu row
  int gcursor = 0;   // selected channel on the Gimbals page
  int tmcursor = 0;  // selected toggle on the Toggle Mapping page
  int tm_scroll = 0;  // index (into EDIT_TOGGLE) of the top visible row
  bool tm_editing = false;
  int tm_field = 0;  // 0 = pin, 1 = invert, 2 = channel, 3 = function (OK steps through)
  uint8_t tm_pin = GPIN_NONE;  // pin being edited; written to gconfig only on save
  bool tm_inv = false;         // invert flag being edited; likewise applied on save
  uint16_t tm_lastbits = 0;    // MCP inputs last tick (pin field: detect a flipped switch)
  uint8_t tm_prevchan = 0;  // channel before entering edit (for CANCEL revert)
  uint8_t tm_prevfunc = 0;  // function before entering edit (for CANCEL revert)
  int te_cursor = 0;   // selected telemetry item
  int te_scroll = 0;   // top visible telemetry row
  int te_tick = 0;     // throttles live value refresh
  int pw_cursor = 0;   // selected TX-power level
  int pw_scroll = 0;   // top visible power row
  int pw_tick = 0;     // throttles the "NOW" telemetry refresh
  // Cyan "sent" markers start on the stored (boot-restored) setting, cursors on top of it.
  int pw_sent = g_elrs[ELRS_TX_POWER] < POWER_N ? g_elrs[ELRS_TX_POWER] : -1;
  int fan_cursor = 0, fan_scroll = 0, fan_sent = -1;  // FAN page (blind write to field 8)
  int fan_current = -2;  // last-written threshold (green dot); -2 = not yet loaded from NVS
  int pkt_cursor = 0, pkt_scroll = 0, pkt_tick = 0;  // PACKET RATE page
  int pkt_sent = g_elrs[ELRS_PKT_RATE] < PKT_N ? g_elrs[ELRS_PKT_RATE] : -1;
  int swm_sent = g_elrs[ELRS_SWITCH_MODE] < SWMODE_N ? g_elrs[ELRS_SWITCH_MODE] : -1;
  int swm_cursor = swm_sent >= 0 ? swm_sent : 0;  // SWITCH MODE page
  if (pw_sent >= 0) pw_cursor = pw_sent;
  if (pkt_sent >= 0) pkt_cursor = pkt_sent;
  int baud_cursor = 0, baud_scroll = 0, baud_sent = -1;            // BAUD RATE page
  bool crsf_confirm = false;  // CRSF page: WiFi-setup confirm dialog
  int crsf_sel = 0, crsf_scroll = 0, crsf_tick = 0;  // CRSF submenu cursor/scroll/status-tick
  bool redraw = true;
  EdgeKey ku, kd, kok, kcx;
  const TickType_t period = pdMS_TO_TICKS(50);

  while (menuModeOn()) {
    flightTimerTick();  // keep banking flight time if armed while in config mode
    RawKeys r = readRawKeys();
    bool up = ku.poll(r.up), dn = kd.poll(r.down), ok = kok.poll(r.ok),
         cx = kcx.poll(r.cancel);

    switch (page) {
      case Page::List:
        if (redraw) {
          if (sel < menu_scroll) menu_scroll = sel;
          if (sel >= menu_scroll + LIST_VISIBLE) menu_scroll = sel - LIST_VISIBLE + 1;
          drawMenuListFull(menu_scroll, sel);
          redraw = false;
        }
        if (up && sel > 0) {
          int old = sel;
          sel--;
          if (sel < menu_scroll) {  // scrolled up: repaint window
            menu_scroll = sel;
            drawMenuListFull(menu_scroll, sel);
          } else {  // same window: repaint just the two rows
            drawMenuRow(old - menu_scroll, old, false);
            drawMenuRow(sel - menu_scroll, sel, true);
          }
        }
        if (dn && sel < MENU_N - 1) {
          int old = sel;
          sel++;
          if (sel >= menu_scroll + LIST_VISIBLE) {  // scrolled down
            menu_scroll = sel - LIST_VISIBLE + 1;
            drawMenuListFull(menu_scroll, sel);
          } else {
            drawMenuRow(old - menu_scroll, old, false);
            drawMenuRow(sel - menu_scroll, sel, true);
          }
        }
        if (ok) {
          page = (Page)(1 + sel);
          g_bindSent = false;
          redraw = true;
        }
        break;
      case Page::Gimbals: {
        if (redraw) {
          drawGimbalPosStatic();
          redraw = false;
        }
        bool edit = toggleOn(TOGGLE5);  // Toggle5 gates editing AND back
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
          // Keep the cursor inside the visible window before the full paint.
          if (tmcursor < tm_scroll) tm_scroll = tmcursor;
          if (tmcursor >= tm_scroll + TM_VISIBLE) tm_scroll = tmcursor - TM_VISIBLE + 1;
          drawToggleListFull(tm_scroll, tmcursor, tm_editing, tm_field);
          redraw = false;
        }
        int t = EDIT_TOGGLE[tmcursor];
        if (!tm_editing) {
          // Selection mode: move cursor; scroll only when it leaves the window.
          if (up && tmcursor > 0) {
            int old = tmcursor;
            tmcursor--;
            if (tmcursor < tm_scroll) {  // scrolled up: repaint the window
              tm_scroll = tmcursor;
              drawToggleListFull(tm_scroll, tmcursor, false, 0);
            } else {  // same window: repaint just the two affected rows
              drawToggleRow(old - tm_scroll, old, false, false, 0);
              drawToggleRow(tmcursor - tm_scroll, tmcursor, true, false, 0);
            }
          }
          if (dn && tmcursor < EDIT_TOGGLE_N - 1) {
            int old = tmcursor;
            tmcursor++;
            if (tmcursor >= tm_scroll + TM_VISIBLE) {  // scrolled down
              tm_scroll = tmcursor - TM_VISIBLE + 1;
              drawToggleListFull(tm_scroll, tmcursor, false, 0);
            } else {
              drawToggleRow(old - tm_scroll, old, false, false, 0);
              drawToggleRow(tmcursor - tm_scroll, tmcursor, true, false, 0);
            }
          }
          if (ok) {
            tm_editing = true;
            tm_field = 0;  // start on the pin field
            tm_pin = gconfig_toggle_pin(t);
            tm_inv = gconfig_toggle_inv(t);
            tm_prevchan = gconfig_toggle_chan(t);
            tm_prevfunc = gconfig_toggle_func(t);
            SwitchState sw;
            switches_get(sw);
            tm_lastbits = sw.bits;
            drawToggleHeader(true, tm_field);
            drawToggleRow(tmcursor - tm_scroll, tmcursor, true, true, tm_field, tm_pin, tm_inv);
          }
          if (cx) {
            page = Page::List;
            redraw = true;
          }
          break;
        }

        // ---- Editing the selected toggle: PIN -> INV -> CH -> FUNC, saved on the last OK ----
        auto drawRow = [&]() {
          drawToggleRow(tmcursor - tm_scroll, tmcursor, true, tm_editing, tm_field,
                        tm_editing ? tm_pin : -1, tm_editing ? (int)tm_inv : -1);
        };
        auto endEdit = [&]() {
          tm_editing = false;
          tm_field = 0;
          drawToggleHeader(false, 0);
          drawRow();
        };
        auto cancel = [&]() {  // revert every field (pin + invert were never applied)
          gconfig_set_toggle_chan(t, tm_prevchan);
          gconfig_set_toggle_func(t, tm_prevfunc);
          endEdit();
        };
        auto commit = [&]() {
          // The standard build's menu switch must stay reachable: only accept a pin +
          // invert that reads ON right now (we are in menu mode, so its real switch is
          // ON). Anything else would drop out of the menu with no way back in.
          if (!MENU_ON_BOOT_BTN && t == TOGGLE6) {
            SwitchState sw;
            bool on = tm_pin <= 15 && switches_get(sw) &&
                      (((sw.bits >> tm_pin) & 1u) != (tm_inv ? 1u : 0u));
            if (!on) {
              cancel();
              drawHeaderTitle("TOGGLE MAP");
              drawHeaderHint(lp("REJECTED: T6 must read ON", "T6 not ON!"), TFT_RED);
              return;
            }
          }
          gconfig_set_toggle_pin(t, tm_pin);
          gconfig_set_toggle_inv(t, tm_inv);
          safeSaveGconfig();
          endEdit();
        };

        if (tm_field == 0) {
          // Edit PIN: up/down step through A0..B7 and "--" (none); flipping a physical
          // switch jumps straight to the pin it is wired to. OK = next field.
          uint8_t before = tm_pin;
          if (up) tm_pin = (tm_pin > 15) ? 0 : (tm_pin == 15 ? GPIN_NONE : tm_pin + 1);
          if (dn) tm_pin = (tm_pin > 15) ? 15 : (tm_pin == 0 ? GPIN_NONE : tm_pin - 1);
          SwitchState sw;
          if (switches_get(sw)) {
            uint16_t diff = sw.bits ^ tm_lastbits;
            tm_lastbits = sw.bits;
            if (diff && !(diff & (diff - 1))) tm_pin = (uint8_t)__builtin_ctz(diff);
          }
          if (tm_pin != before) drawRow();
          if (ok) {
            tm_field = 1;
            drawToggleHeader(true, tm_field);
            drawRow();
          }
          if (cx) cancel();
        } else if (tm_field == 1) {
          // Edit INVERT: up/down flip normal <-> inverted. OK = next field.
          if (up || dn) {
            tm_inv = !tm_inv;
            drawRow();
          }
          if (ok) {
            if (gconfig_toggle_reserved(t)) {  // menu switch: pin + invert are all there is
              commit();
            } else {
              tm_field = 2;
              drawToggleHeader(true, tm_field);
              drawRow();
            }
          }
          if (cx) cancel();
        } else if (tm_field == 2) {
          // Edit CHANNEL: up/down cycle channel; OK advances to function; CXL reverts.
          uint8_t c = gconfig_toggle_chan(t);
          if (up) {  // none -> 5..16 -> none
            gconfig_set_toggle_chan(t, (c == 0) ? 5 : (c >= 16 ? 0 : c + 1));
            drawRow();
          }
          if (dn) {  // none -> 16..5 -> none
            gconfig_set_toggle_chan(t, (c == 0) ? 16 : (c <= 5 ? 0 : c - 1));
            drawRow();
          }
          if (ok) {  // advance to the function field
            tm_field = 3;
            drawToggleHeader(true, tm_field);
            drawRow();
          }
          if (cx) cancel();
        } else {
          // Edit FUNCTION: up/down cycle label; OK saves all four fields; CXL reverts.
          uint8_t f = gconfig_toggle_func(t);
          if (up) {
            gconfig_set_toggle_func(t, (uint8_t)((f + 1) % FUNC_COUNT));
            drawRow();
          }
          if (dn) {
            gconfig_set_toggle_func(t, (uint8_t)((f + FUNC_COUNT - 1) % FUNC_COUNT));
            drawRow();
          }
          if (ok) commit();
          if (cx) cancel();
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
      case Page::Telemetry: {
        CrsfLink l;
        bool lok = crsf_get(l);
        if (redraw) {
          if (te_cursor < te_scroll) te_scroll = te_cursor;
          if (te_cursor >= te_scroll + TE_VISIBLE) te_scroll = te_cursor - TE_VISIBLE + 1;
          drawTeleFull(te_scroll, te_cursor, l, lok);
          redraw = false;
          te_tick = 0;
        }
        if (up && te_cursor > 0) {
          te_cursor--;
          if (te_cursor < te_scroll) te_scroll = te_cursor;
          drawTeleVisible(te_scroll, te_cursor, l, lok);
        }
        if (dn && te_cursor < TELE_N - 1) {
          te_cursor++;
          if (te_cursor >= te_scroll + TE_VISIBLE) te_scroll = te_cursor - TE_VISIBLE + 1;
          drawTeleVisible(te_scroll, te_cursor, l, lok);
        }
        if (ok) {  // toggle "show on main screen" and persist
          g_telemSel ^= (uint16_t)(1u << te_cursor);
          safeSaveTelemSel(g_telemSel);
          drawTeleVisible(te_scroll, te_cursor, l, lok);
        }
        if (++te_tick >= 4) {  // ~5 Hz live value refresh
          te_tick = 0;
          drawTeleVisible(te_scroll, te_cursor, l, lok);
        }
        if (cx) {
          page = Page::List;
          redraw = true;
        }
        break;
      }
      case Page::Power: {
        CrsfLink l;
        bool lok = crsf_get(l);
        uint16_t now = lok ? crsf_tx_power_mw(l.tx_power) : 0;
        if (redraw) {
          if (pw_cursor < pw_scroll) pw_scroll = pw_cursor;
          if (pw_cursor >= pw_scroll + LIST_VISIBLE) pw_scroll = pw_cursor - LIST_VISIBLE + 1;
          drawPowerFull(pw_scroll, pw_cursor, now, pw_sent);
          redraw = false;
          pw_tick = 0;
        }
        if (up && pw_cursor > 0) {
          pw_cursor--;
          if (pw_cursor < pw_scroll) pw_scroll = pw_cursor;
          drawPowerVisible(pw_scroll, pw_cursor, now, pw_sent);
        }
        if (dn && pw_cursor < POWER_N - 1) {
          pw_cursor++;
          if (pw_cursor >= pw_scroll + LIST_VISIBLE) pw_scroll = pw_cursor - LIST_VISIBLE + 1;
          drawPowerVisible(pw_scroll, pw_cursor, now, pw_sent);
        }
        if (ok) {  // send the power write to the module (confirmed via telemetry NOW)
          safeSaveElrs(ELRS_TX_POWER, (uint8_t)pw_cursor);  // re-sent at every boot
          crsf_set_power(pw_cursor);
          pw_sent = pw_cursor;
          drawPowerHeader(now, pw_sent, false);  // change-driven, no flicker
          drawPowerVisible(pw_scroll, pw_cursor, now, pw_sent);
        }
        if (++pw_tick >= 4) {  // ~5 Hz refresh of the telemetry-reported power
          pw_tick = 0;
          drawPowerHeader(now, pw_sent, false);  // only redraws NOW/SET when it changes
          drawPowerVisible(pw_scroll, pw_cursor, now, pw_sent);
        }
        if (cx) {
          page = Page::Crsf;
          redraw = true;
        }
        break;
      }
      case Page::Fan: {
        if (redraw) {
          if (fan_current == -2) {  // first entry: load the remembered threshold
            uint8_t v = g_elrs[ELRS_FAN];
            fan_current = (v < FAN_N) ? v : -1;
            if (fan_current >= 0) fan_cursor = fan_current;  // start on the current one
          }
          if (fan_cursor < fan_scroll) fan_scroll = fan_cursor;
          if (fan_cursor >= fan_scroll + LIST_VISIBLE) fan_scroll = fan_cursor - LIST_VISIBLE + 1;
          drawSettingList(lp("FAN THRESHOLD", "FAN THRESH"), fan_scroll, fan_cursor, FAN_OPTS, FAN_N, fan_current, fan_sent);
          redraw = false;
        }
        if (up && fan_cursor > 0) {
          fan_cursor--;
          if (fan_cursor < fan_scroll) fan_scroll = fan_cursor;
          drawSettingRows(fan_scroll, fan_cursor, FAN_OPTS, FAN_N, fan_current, fan_sent);
        }
        if (dn && fan_cursor < FAN_N - 1) {
          fan_cursor++;
          if (fan_cursor >= fan_scroll + LIST_VISIBLE) fan_scroll = fan_cursor - LIST_VISIBLE + 1;
          drawSettingRows(fan_scroll, fan_cursor, FAN_OPTS, FAN_N, fan_current, fan_sent);
        }
        if (ok) {  // blind write the selected threshold (field 8, raw option index) + remember it
          safeSaveElrs(ELRS_FAN, (uint8_t)fan_cursor);
          crsf_param_set(CRSF_FIELD_FAN, fan_cursor);
          fan_sent = fan_cursor;
          fan_current = fan_cursor;  // green dot moves to the new setting
          drawSettingList(lp("FAN THRESHOLD", "FAN THRESH"), fan_scroll, fan_cursor, FAN_OPTS, FAN_N, fan_current, fan_sent);
        }
        if (cx) {
          page = Page::Crsf;
          redraw = true;
        }
        break;
      }
      case Page::PktRate: {
        CrsfLink l;
        bool lok = crsf_get(l);
        int cur = pktCurrentIndex(l.rf_mode, lok);  // actual rate from telemetry rf_Mode
        if (redraw) {
          if (pkt_cursor < pkt_scroll) pkt_scroll = pkt_cursor;
          if (pkt_cursor >= pkt_scroll + LIST_VISIBLE) pkt_scroll = pkt_cursor - LIST_VISIBLE + 1;
          drawSettingList("PACKET RATE", pkt_scroll, pkt_cursor, PKT_OPTS, PKT_N, cur, pkt_sent);
          redraw = false;
          pkt_tick = 0;
        }
        if (up && pkt_cursor > 0) {
          pkt_cursor--;
          if (pkt_cursor < pkt_scroll) pkt_scroll = pkt_cursor;
          drawSettingRows(pkt_scroll, pkt_cursor, PKT_OPTS, PKT_N, cur, pkt_sent);
        }
        if (dn && pkt_cursor < PKT_N - 1) {
          pkt_cursor++;
          if (pkt_cursor >= pkt_scroll + LIST_VISIBLE) pkt_scroll = pkt_cursor - LIST_VISIBLE + 1;
          drawSettingRows(pkt_scroll, pkt_cursor, PKT_OPTS, PKT_N, cur, pkt_sent);
        }
        if (ok) {  // write field 1 = list index (module may clamp at 400k baud; see rf_Mode)
          safeSaveElrs(ELRS_PKT_RATE, (uint8_t)pkt_cursor);  // re-sent at every boot
          crsf_param_set(CRSF_FIELD_PKT_RATE, pkt_cursor);
          pkt_sent = pkt_cursor;
          drawSettingList("PACKET RATE", pkt_scroll, pkt_cursor, PKT_OPTS, PKT_N, cur, pkt_sent);
        }
        if (++pkt_tick >= 4) {  // refresh the green "actual rate" dot from telemetry (rows only)
          pkt_tick = 0;
          drawSettingRows(pkt_scroll, pkt_cursor, PKT_OPTS, PKT_N, cur, pkt_sent);
        }
        if (cx) {
          page = Page::Crsf;
          redraw = true;
        }
        break;
      }
      case Page::SwMode: {
        if (redraw) {
          drawSettingList("SWITCH MODE", 0, swm_cursor, SWMODE_OPTS, SWMODE_N, -1, swm_sent);
          redraw = false;
        }
        if (up && swm_cursor > 0) {
          swm_cursor--;
          drawSettingRows(0, swm_cursor, SWMODE_OPTS, SWMODE_N, -1, swm_sent);
        }
        if (dn && swm_cursor < SWMODE_N - 1) {
          swm_cursor++;
          drawSettingRows(0, swm_cursor, SWMODE_OPTS, SWMODE_N, -1, swm_sent);
        }
        if (ok) {  // blind write field 3 = option index (meaning depends on the packet rate)
          safeSaveElrs(ELRS_SWITCH_MODE, (uint8_t)swm_cursor);  // re-sent at every boot
          crsf_param_set(CRSF_FIELD_SWITCH_MODE, swm_cursor);
          swm_sent = swm_cursor;
          drawSettingList("SWITCH MODE", 0, swm_cursor, SWMODE_OPTS, SWMODE_N, -1, swm_sent);
        }
        if (cx) {
          page = Page::Crsf;
          redraw = true;
        }
        break;
      }
      case Page::Baud: {
        // Current = the option matching the active baud.
        int cur = -1;
        for (int i = 0; i < BAUD_N; ++i)
          if (BAUD_VALS[i] == crsf_get_baud()) cur = i;
        if (redraw) {
          baud_scroll = 0;
          if (cur >= 0) baud_cursor = cur;
          drawSettingList("BAUD RATE", baud_scroll, baud_cursor, BAUD_OPTS, BAUD_N, cur, baud_sent);
          redraw = false;
        }
        if (up && baud_cursor > 0) {
          baud_cursor--;
          drawSettingRows(baud_scroll, baud_cursor, BAUD_OPTS, BAUD_N, cur, baud_sent);
        }
        if (dn && baud_cursor < BAUD_N - 1) {
          baud_cursor++;
          drawSettingRows(baud_scroll, baud_cursor, BAUD_OPTS, BAUD_N, cur, baud_sent);
        }
        if (ok) {  // persist + apply live (module autobaud re-locks in ~1-2 s)
          uint32_t b = BAUD_VALS[baud_cursor];
          crsf_pause();
          gimbals_pause();
          switches_pause();
          storage_set_crsf_baud(b);
          switches_resume();
          gimbals_resume();
          crsf_resume();
          crsf_set_baud(b);
          baud_sent = baud_cursor;
          drawSettingList("BAUD RATE", baud_scroll, baud_cursor, BAUD_OPTS, BAUD_N, baud_cursor,
                          baud_sent);
        }
        if (cx) {
          page = Page::Crsf;
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
        if (redraw) {
          if (crsf_confirm) {
            drawWifiConfirm();
          } else {
            if (crsf_sel < crsf_scroll) crsf_scroll = crsf_sel;
            if (crsf_sel >= crsf_scroll + LIST_VISIBLE) crsf_scroll = crsf_sel - LIST_VISIBLE + 1;
            drawCrsfMenu(crsf_scroll, crsf_sel);
          }
          redraw = false;
          crsf_tick = 0;
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
          break;
        }
        if (up && crsf_sel > 0) {
          int old = crsf_sel;
          crsf_sel--;
          if (crsf_sel < crsf_scroll) {  // scrolled: repaint window; else just two rows
            crsf_scroll = crsf_sel;
            drawCrsfMenu(crsf_scroll, crsf_sel);
          } else {
            drawCrsfMenuRow(old - crsf_scroll, old, false);
            drawCrsfMenuRow(crsf_sel - crsf_scroll, crsf_sel, true);
          }
        }
        if (dn && crsf_sel < CRSF_N - 1) {
          int old = crsf_sel;
          crsf_sel++;
          if (crsf_sel >= crsf_scroll + LIST_VISIBLE) {
            crsf_scroll = crsf_sel - LIST_VISIBLE + 1;
            drawCrsfMenu(crsf_scroll, crsf_sel);
          } else {
            drawCrsfMenuRow(old - crsf_scroll, old, false);
            drawCrsfMenuRow(crsf_sel - crsf_scroll, crsf_sel, true);
          }
        }
        if (ok) {
          switch (crsf_sel) {
            case 0:  // OUTPUT toggle
              crsf_set_output(!crsf_output_enabled());
              drawCrsfMenuRow(crsf_sel - crsf_scroll, crsf_sel, true);
              break;
            case 1: page = Page::Power; redraw = true; break;
            case 2: page = Page::Fan; redraw = true; break;
            case 3: page = Page::PktRate; redraw = true; break;
            case 4: page = Page::SwMode; redraw = true; break;
            case 5: page = Page::Baud; redraw = true; break;
            case CRSF_ITEM_BLE:  // BLE JOYSTICK: on -> start BLE + silence CRSF; off -> resume
              if (!ble_on()) {
                ble_set(true);
                crsf_set_output(false);
              } else {
                ble_set(false);
                crsf_set_output(true);
              }
              drawCrsfMenu(crsf_scroll, crsf_sel);  // OUTPUT + BLE rows both changed
              break;
            case CRSF_ITEM_BIND:  // put the TX module into bind mode
              crsf_bind();
              g_bindSent = true;
              drawCrsfMenuRow(crsf_sel - crsf_scroll, crsf_sel, true);
              break;
            case CRSF_ITEM_WIFI: crsf_confirm = true; redraw = true; break;
          }
        }
        if (cx) {
          page = Page::List;
          redraw = true;
        }
        if (!redraw && ++crsf_tick >= 6) {  // refresh status only (change-driven, no flicker)
          crsf_tick = 0;
          drawCrsfMenuStatus(false);
          static int bindShown = -1;  // bind row follows the bind state machine
          if (g_bindSent && crsf_bind_state() != bindShown) {
            bindShown = crsf_bind_state();
            int r = CRSF_ITEM_BIND - crsf_scroll;
            if (r >= 0 && r < LIST_VISIBLE)
              drawCrsfMenuRow(r, CRSF_ITEM_BIND, crsf_sel == CRSF_ITEM_BIND);
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
bool toggleOn(int toggle) {  // toggle index 0..9, read through its assigned MCP pin
  SwitchState sw;
  if (!switches_get(sw)) return false;  // sensor missing -> treat as OFF
  return gconfig_toggle_on(toggle, sw.bits);
}

// ---- Debug mode (Toggle3 + Toggle4 + Toggle5 all ON at power-up) ----
// A set of bench tools. Pitch stick DOWN = next tool, UP = previous (wraps). CRSF output
// is silenced. The toggles can't double as an exit (they are under test), so the BOOT
// button (GPIO0) reboots out of it — flip one of the three toggles OFF first or the
// combo re-enters on the way up.
//   1. MCP23017 inputs  — all 16 expander pins live
//   2. CRSF 1k resistor — is the series resistor on the CRSF wire fitted?
bool debugModeRequested() { return toggleOn(TOGGLE3) && toggleOn(TOGGLE4) && toggleOn(TOGGLE5); }

// Polled by every tool each tick: handles the BOOT exit and returns the tool step from a
// fresh pitch-stick push (+1 next, -1 previous, 0 stay).
int debugNav() {
  static bool prev_up = false, prev_dn = false;
  if (digitalRead(PIN_BOOT_BTN) == LOW) {  // exit: reboot back into the normal flow
    Serial.println("[debug] BOOT pressed - rebooting");
    doReboot();  // does not return
  }
  RawKeys k = readRawKeys();
  int step = 0;
  if (k.down && !prev_dn) step = 1;
  else if (k.up && !prev_up) step = -1;
  prev_up = k.up;
  prev_dn = k.down;
  return step;
}

// Clear the screen and draw a tool's title + the shared navigation hint. Landscape keeps
// the right end of the header line free for the tool's own status; portrait puts the
// hint in a two-line footer.
void debugHeader(const char* title) {
  tft.fillScreen(COL_BG);
  tft.setTextPadding(0);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_YELLOW, COL_BG);
  tft.drawString(title, PORTRAIT ? 2 : 4, 5, 1);
  tft.setTextColor(TFT_DARKGREY, COL_BG);
  if (PORTRAIT) {
    tft.drawString("PITCH = next", 2, LCD_H - 20, 1);
    tft.drawString("BOOT = exit", 2, LCD_H - 9, 1);
  } else {
    tft.drawString("PIT:next BOOT:exit", 104, 5, 1);
  }
}

// Tool 1: all 16 MCP23017 inputs (A0-A7, B0-B7) live. A cell turns green while its pin
// is triggered (pulled to GND) and keeps a white outline once it has been seen changing,
// so a pin that works only intermittently still leaves a mark. Each cell also names the
// toggle assigned to that pin. Raw states only (toggle invert is not applied). Changes
// are logged to serial. Returns the tool step.
int mcpDebugTool() {
  // Landscape: port A on the top row, port B below, pins 0-7 left to right.
  // Portrait: port A in the left column, port B right, pins 0-7 top to bottom.
  constexpr int CW = PORTRAIT ? 37 : 35, CH = PORTRAIT ? 28 : 31;  // cell pitch
  constexpr int X0 = PORTRAIT ? 1 : 2, Y0 = PORTRAIT ? 30 : 13;

  debugHeader(lp("MCP23017", "MCP INPUTS"));

  // Toggle number assigned to each MCP bit (0 = none), from the current pin map.
  uint8_t bit_toggle[16] = {0};
  for (int t = 9; t >= 0; --t) {
    uint8_t pin = gconfig_toggle_pin(t);
    if (pin <= 15) bit_toggle[pin] = (uint8_t)(t + 1);
  }

  uint16_t last = 0, seen = 0;
  int last_valid = -1;
  bool first = true;
  for (;;) {
    SwitchState sw;
    bool valid = switches_get(sw);
    uint16_t bits = valid ? sw.bits : 0;
    uint16_t changed = first ? 0xFFFF : (uint16_t)(bits ^ last);
    if (!first) seen |= changed;

    if (changed || (int)valid != last_valid) {  // raw word: hex, or the fault
      char b[16];
      if (valid) snprintf(b, sizeof(b), "0x%04X", bits);
      else snprintf(b, sizeof(b), "NO MCP!");
      tft.setTextColor(valid ? TFT_CYAN : TFT_RED, COL_BG);
      if (PORTRAIT) {
        tft.setTextDatum(ML_DATUM);
        tft.setTextPadding(LCD_W - 2);
        tft.drawString(b, 2, 17, 1);
      } else {
        tft.setTextDatum(MR_DATUM);
        tft.setTextPadding(60);
        tft.drawString(b, LCD_W - 4, 5, 1);
      }
      tft.setTextPadding(0);
      Serial.printf("[mcpdbg] %s\n", b);
    }

    for (int i = 0; i < 16; ++i) {
      if (!(changed & (1u << i))) continue;
      int port = i / 8, pin = i % 8;
      int x = X0 + (PORTRAIT ? port : pin) * CW, y = Y0 + (PORTRAIT ? pin : port) * CH;
      bool on = bits & (1u << i);
      uint16_t bg = on ? TFT_GREEN : 0x2965;  // dark grey when idle
      tft.fillRoundRect(x, y, CW - 2, CH - 2, 3, bg);
      if (seen & (1u << i)) tft.drawRoundRect(x, y, CW - 2, CH - 2, 3, TFT_WHITE);
      char n[4];
      snprintf(n, sizeof(n), "%c%d", port ? 'B' : 'A', pin);
      tft.setTextDatum(MC_DATUM);
      tft.setTextColor(on ? TFT_BLACK : TFT_LIGHTGREY, bg);
      tft.drawString(n, x + (CW - 2) / 2, y + 9, 2);
      if (bit_toggle[i]) {  // the toggle assigned here
        snprintf(n, sizeof(n), "T%d", bit_toggle[i]);
        tft.setTextColor(on ? TFT_BLACK : TFT_DARKGREY, bg);
        tft.drawString(n, x + (CW - 2) / 2, y + CH - 8, 1);
      }
    }

    last = bits;
    last_valid = valid;
    first = false;
    if (int step = debugNav()) return step;
    vTaskDelay(pdMS_TO_TICKS(20));  // matches the 50 Hz switch poll
  }
}

// Tool 2: CRSF wire check — is the 1k series resistor (GPIO17 -> CRSF pad) fitted?
// Re-runs crsf_wire_test() once a second so a board can be probed/reworked live; the
// screen and serial log update only when the result changes. See crsf.h for what the
// three probes mean and why "OK" is a range, not a measurement. Returns the tool step.
int crsfWireTool() {
  struct Text {
    const char* land;     // landscape verdict (one line, font 4)
    const char* port[2];  // portrait verdict (two lines, font 2)
    const char* why;      // landscape explanation (one line, font 1)
    const char* whyp[3];  // portrait explanation (<= 12 chars per line)
    uint16_t col;
  };
  static const Text TEXT[] = {
      {"1K OK", {"1K", "OK"}, "TX->pad path present, not a short (~300R..10k)",
       {"path present", "not shorted", "~300R..10k"}, TFT_GREEN},
      {"1K MISSING", {"1K", "MISSING"}, "RX does not follow TX: resistor missing / open",
       {"RX does not", "follow TX:", "R missing"}, TFT_RED},
      {"1K SHORTED", {"1K", "SHORTED"}, "TX and RX fight: resistor bridged (0 ohm)",
       {"TX/RX fight:", "R bridged", "(0 ohm)"}, TFT_RED},
      {"LINE HELD LOW", {"LINE", "HELD LOW"}, "stuck LOW: short to GND / module unpowered",
       {"pad stuck LO", "GND short or", "module off"}, TFT_RED},
      {"LINE BUSY", {"LINE", "BUSY"}, "readings unstable: module transmitting?",
       {"unstable:", "module is", "talking?"}, TFT_YELLOW},
  };

  debugHeader(lp("CRSF 1K", "CRSF 1K RES"));

  int last_sig = -1;
  for (int tick = 0;; ++tick) {
    if (tick % 50 == 0) {  // ~1 Hz
      CrsfWireTest r = crsf_wire_test();
      int sig = (int)r.verdict << 6 | r.idle_pu << 5 | r.idle_pd << 4 | r.loop_hi << 3 |
                r.loop_lo << 2 | r.hold_a << 1 | r.hold_b;
      if (sig != last_sig) {
        last_sig = sig;
        const Text& t = TEXT[(int)r.verdict];
        Serial.printf("[crsfdbg] %s  idle pu=%d pd=%d  loop hi=%d lo=%d  hold a=%d b=%d\n",
                      t.land, r.idle_pu, r.idle_pd, r.loop_hi, r.loop_lo, r.hold_a, r.hold_b);
        char b[48];
        tft.setTextPadding(0);
        if (PORTRAIT) {
          tft.fillRect(0, 12, LCD_W, LCD_H - 12 - 28, COL_BG);  // body, above the footer
          tft.setTextDatum(MC_DATUM);
          tft.setTextColor(t.col, COL_BG);
          tft.drawString(t.port[0], LCD_W / 2, 44, 2);
          tft.drawString(t.port[1], LCD_W / 2, 64, 2);
          tft.setTextDatum(ML_DATUM);
          tft.setTextColor(TFT_WHITE, COL_BG);
          for (int i = 0; i < 3; ++i) tft.drawString(t.whyp[i], 2, 92 + i * 12, 1);
          const char* const NAME[6] = {"idle pu", "idle pd", "loop hi",
                                       "loop lo", "hold a",  "hold b"};
          const bool val[6] = {r.idle_pu, r.idle_pd, r.loop_hi, r.loop_lo, r.hold_a, r.hold_b};
          tft.setTextColor(TFT_CYAN, COL_BG);
          for (int i = 0; i < 6; ++i) {
            snprintf(b, sizeof(b), "%-8s %d", NAME[i], val[i]);
            tft.drawString(b, 2, 144 + i * 12, 1);
          }
        } else {
          tft.fillRect(0, 12, LCD_W, LCD_H - 12, COL_BG);
          tft.setTextDatum(MC_DATUM);
          tft.setTextColor(t.col, COL_BG);
          tft.drawString(t.land, LCD_W / 2, 30, 4);
          tft.setTextDatum(ML_DATUM);
          tft.setTextColor(TFT_WHITE, COL_BG);
          tft.drawString(t.why, 4, 54, 1);
          snprintf(b, sizeof(b), "idle pu=%d pd=%d  loop hi=%d lo=%d  hold a=%d b=%d", r.idle_pu,
                   r.idle_pd, r.loop_hi, r.loop_lo, r.hold_a, r.hold_b);
          tft.setTextColor(TFT_CYAN, COL_BG);
          tft.drawString(b, 4, 67, 1);
        }
      }
    }
    if (int step = debugNav()) return step;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void debugLoop() {
  crsf_set_output(false);  // bench mode: no throttle gate ran, so keep the link silent
  pinMode(PIN_BOOT_BTN, INPUT_PULLUP);  // polled here (pocket's menu ISR is irrelevant now)
  Serial.println("[debug] debug mode (pitch = next tool, BOOT button = reboot/exit)");

  int (*const TOOLS[])() = {mcpDebugTool, crsfWireTool};
  constexpr int TOOL_N = sizeof(TOOLS) / sizeof(TOOLS[0]);
  int tool = 0;
  for (;;) tool = (tool + TOOLS[tool]() + TOOL_N) % TOOL_N;
}

// RC screen loop — runs while menu mode is off; returns when it comes on.
void rcLoop() {
  drawRcStatic();
  bool first = true;  // force a full paint on the first frame (statics are change-driven)
  const TickType_t period = pdMS_TO_TICKS(200);  // ~5 Hz
  while (!menuModeOn()) {
    flightTimerTick();
    renderRc(first);
    first = false;
    vTaskDelay(period);
  }
}

// Live mode switch (menuModeOn): off = RC screen, on = configuration menu.
void modeLoop() {
  for (;;) {
    if (menuModeOn()) configLoop();
    else rcLoop();
  }
}

void appTask(void*) {
  // NVS first, while no peripheral tasks/ISRs exist yet — flash init may write, and a
  // busy ISR during a flash-cache-disabled window panics the chip.
  storage_init();
  gconfig_init();  // load per-channel flip flags (default none) before CRSF uses them
  g_telemSel = storage_get_telem_sel();  // which telemetry items show on the RC screen
  crsf_set_baud(storage_get_crsf_baud());  // seed the CRSF baud before crsf_start()
  for (int i = 0; i < ELRS_SETTING_N; ++i) g_elrs[i] = storage_get_elrs((ElrsSetting)i);
  crsf_set_restore(g_elrs);  // re-send the stored module settings once CRSF is up

  tft.init();
  tft.setRotation(PANEL_ROTATION);
  tft.invertDisplay(PANEL_INVERT);  // explicit: false actively disables inversion
  tft.fillScreen(COL_BG);
  tft.setViewport(VP_X, VP_Y, LCD_W, LCD_H);
  backlightOn();
  menuModeInit();

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
      // Pocket: the reboot reset menu mode to off, so the next BOOT press is the exit.
      bool leave = MENU_ON_BOOT_BTN
                       ? menuModeOn()
                       : (switches_get(sw) && !gconfig_toggle_on(TOGGLE6, sw.bits));
      if (leave) {
        safeSetWifiBoot(false);  // clear flag, reboot to normal
        doReboot();              // does not return
      }
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }

  // Stored calibration first: the debug tools navigate with the (calibrated) pitch stick.
  bool have_cal = storage_load_cal(g_cal);

  // Toggle3+4+5 all ON at power-up: debug mode (does not return). Checked before the
  // Toggle5 calibration trigger, which the combo would also satisfy.
  if (debugModeRequested()) debugLoop();

  // Toggle5 ON at power-up forces a fresh calibration; otherwise use the stored one.
  // First boot (nothing stored) always calibrates.
  if (toggleOn(TOGGLE5) || !have_cal) {
    calibrationWizard();
  } else {
    Serial.println("[cal] loaded from NVS");
  }
  gconfig_set_cal(g_cal);  // ensure CRSF maps against the just-loaded/captured calibration

  throttleGate();  // block while throttle > 15%

  modeLoop();  // live menu-mode switch: Config vs RC (does not return)
}

}  // namespace

bool ui_start() {
  // 10 KB stack: TFT_eSPI sprite text rendering + float formatting in renderRc need
  // well more than the old 6 KB (a deep drawString there overflowed it -> LoadStoreError).
  BaseType_t ok = xTaskCreatePinnedToCore(appTask, "app", 10240, nullptr,
                                          /*priority=*/1, nullptr, /*core=*/0);
  return ok == pdPASS;
}
