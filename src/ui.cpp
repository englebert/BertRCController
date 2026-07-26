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

bool safeSaveFanThresh(uint8_t v) {  // remember the fan threshold we last wrote
  crsf_pause();
  gimbals_pause();
  switches_pause();
  storage_set_fan_thresh(v);
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

  // Arm-safety banner (top-right). Red ARMED = motors live; green DISARMED = throttle
  // held at idle; grey NO ARM = no toggle assigned the ARM function.
  int as = crsf_arm_state();
  const char* at = (as < 0) ? "NO ARM" : (as == 0 ? "DISARMED" : "ARMED");
  uint16_t ac = (as < 0) ? TFT_DARKGREY : (as == 0 ? TFT_GREEN : TFT_RED);
  tft.setTextDatum(MR_DATUM);
  tft.setTextPadding(96);
  tft.setTextColor(ac, TFT_BLACK);
  tft.drawString(at, LCD_W - 4, 8, 2);

  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(120);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  if (ok) snprintf(b, sizeof(b), "%d%%", batteryPct(pr.voltage_V));
  else snprintf(b, sizeof(b), "--%%");
  tft.drawString(b, LCD_W / 2, 28, 4);

  // Voltage (left) + flight-session MM:SS (right, white while armed).
  tft.setTextDatum(ML_DATUM);
  tft.setTextPadding(90);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  if (ok) snprintf(b, sizeof(b), "%.2f V", pr.voltage_V);
  else snprintf(b, sizeof(b), "-- V");
  tft.drawString(b, 4, 50, 2);

  uint32_t s = flightTimerSession();
  tft.setTextDatum(MR_DATUM);
  tft.setTextPadding(70);
  tft.setTextColor(g_fltArmed ? TFT_WHITE : TFT_DARKGREY, TFT_BLACK);
  snprintf(b, sizeof(b), "%02lu:%02lu", (unsigned long)(s / 60), (unsigned long)(s % 60));
  tft.drawString(b, LCD_W - 4, 50, 2);

  // Bottom telemetry strip: selected items, blitted from a sprite so it never flashes.
  // Green while telemetry is live; RED (stale values, or "NO TELEMETRY") when offline.
  CrsfLink l;
  bool ever = crsf_get(l);            // have we ever received link stats?
  bool online = crsf_telem_online();  // ...and is it still fresh?
  char strip[64];
  buildTeleStrip(strip, sizeof(strip), l);
  TFT_eSprite spr(&tft);
  spr.createSprite(LCD_W, 12);
  spr.fillSprite(TFT_BLACK);
  spr.setTextFont(1);  // clears the inherited garbage gfxFont — else font-1 text crashes
  spr.setTextSize(1);
  spr.setTextDatum(ML_DATUM);
  spr.setTextPadding(0);
  if (g_telemSel == 0) {
    // nothing selected — leave the strip blank
  } else if (online) {
    spr.setTextColor(TFT_GREENYELLOW, TFT_BLACK);
    spr.drawString(strip, 2, 6, 1);
  } else {
    spr.setTextColor(TFT_RED, TFT_BLACK);
    spr.drawString(ever ? strip : "NO TELEMETRY", 2, 6, 1);
  }
  spr.pushSprite(0, 64);
  spr.deleteSprite();
}

// ---- Config-mode menu system ----
// Navigation via gimbal keys (TASK01.md): pitch >1800us = UP, <1200 = DOWN;
// roll >1800 = OK (enter), <1200 = CANCEL (back).
bool toggleOn(int bit);  // fwd decl (defined below)

const char* const MENU_ITEMS[] = {"GIMBALS & TRIM", "TOGGLE MAPPING", "BATTERY INFO",
                                   "TELEMETRY",      "CRSF MODULE",    "TX POWER",
                                   "FAN",            "PACKET RATE",    "BAUD RATE",
                                   "REBOOT"};
constexpr int MENU_N = 10;

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

// Shared scrolling-list geometry: 4 rows visible at font 2 (16px) on the 76px panel.
constexpr int LIST_VISIBLE = 4;
constexpr int LIST_ROW_H = 16;
constexpr int LIST_ROW_Y0 = 20;  // center-y of the first visible row (below header)

void drawMenuHeader() {
  tft.fillRect(0, 0, LCD_W, 12, COL_BG);
  tft.setTextDatum(ML_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_YELLOW, COL_BG);
  tft.drawString("CONFIG", 4, 5, 1);
  tft.setTextDatum(MR_DATUM);
  tft.setTextColor(TFT_DARKGREY, COL_BG);
  tft.drawString("OK:enter  CXL:exit", LCD_W - 4, 5, 1);
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
    spr.drawString(MENU_ITEMS[idx], 8, LIST_ROW_H / 2, 2);
  }
  spr.pushSprite(0, y - LIST_ROW_H / 2);
  spr.deleteSprite();
}

void drawMenuListFull(int scroll, int sel) {
  // Header + 4 sprite rows cover the panel, so no fillScreen (which would flash).
  drawMenuHeader();
  for (int s = 0; s < LIST_VISIBLE; ++s) {
    int idx = scroll + s;
    drawMenuRow(s, idx, idx == sel);
  }
}

// --- Menu item: Telemetry — live CRSF link stats; OK toggles "show on main screen". ---
void drawTeleHeader() {
  tft.fillRect(0, 0, LCD_W, 12, COL_BG);
  tft.setTextPadding(0);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_YELLOW, COL_BG);
  tft.drawString("TELEMETRY", 4, 5, 1);
  tft.setTextDatum(MR_DATUM);
  tft.setTextColor(TFT_DARKGREY, COL_BG);
  tft.drawString("OK:main  CXL:back", LCD_W - 4, 5, 1);
}

// One telemetry row: selection bullet + name (left) + live value (right). Sprite blit.
void drawTeleRow(int slot, int item, bool cursor, const CrsfLink& l, bool lok) {
  int y = LIST_ROW_Y0 + slot * LIST_ROW_H;
  uint16_t bg = cursor ? TFT_NAVY : COL_BG;
  TFT_eSprite spr(&tft);
  spr.createSprite(LCD_W, LIST_ROW_H);
  spr.setTextFont(1);  // init inherited text state (gfxFont/size are garbage in a new sprite)
  spr.setTextSize(1);
  spr.fillSprite(bg);
  if (item >= 0 && item < TELE_N) {
    int cy = LIST_ROW_H / 2;
    bool on = g_telemSel & (1u << item);
    // selection bullet: filled yellow = shown on main; hollow grey = not.
    if (on) spr.fillCircle(9, cy, 4, TFT_YELLOW);
    else spr.drawCircle(9, cy, 4, TFT_DARKGREY);
    spr.setTextDatum(ML_DATUM);
    spr.setTextPadding(0);
    spr.setTextColor(TFT_WHITE, bg);
    spr.drawString(TELE_ITEMS[item].name, 22, cy, 2);
    char v[16];
    teleValue(item, l, lok, v, sizeof(v));
    spr.setTextDatum(MR_DATUM);
    spr.setTextColor(lok ? TFT_CYAN : TFT_DARKGREY, bg);
    spr.drawString(v, LCD_W - 6, cy, 2);
  }
  spr.pushSprite(0, y - LIST_ROW_H / 2);
  spr.deleteSprite();
}

void drawTeleVisible(int scroll, int cursor, const CrsfLink& l, bool lok) {
  for (int s = 0; s < LIST_VISIBLE; ++s) {
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

void drawPowerHeader(uint16_t now_mw, int sent) {
  tft.fillRect(0, 0, LCD_W, 12, COL_BG);
  tft.setTextPadding(0);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_YELLOW, COL_BG);
  tft.drawString("TX POWER", 4, 5, 1);
  char nows[8];
  if (now_mw) snprintf(nows, sizeof(nows), "%u", now_mw);
  else snprintf(nows, sizeof(nows), "--");
  char b[28];
  if (sent >= 0) snprintf(b, sizeof(b), "NOW:%s SET:%u", nows, POWER_MW[sent]);
  else snprintf(b, sizeof(b), "NOW:%s  OK:set", nows);
  tft.setTextDatum(MR_DATUM);
  tft.setTextColor(sent >= 0 ? TFT_GREEN : TFT_DARKGREY, COL_BG);
  tft.drawString(b, LCD_W - 4, 5, 1);
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
    if (now_mw == POWER_MW[idx]) spr.fillCircle(9, cy, 4, TFT_GREEN);
    else spr.drawCircle(9, cy, 4, TFT_DARKGREY);
    char b[12];
    snprintf(b, sizeof(b), "%u mW", POWER_MW[idx]);
    spr.setTextDatum(ML_DATUM);
    spr.setTextColor(idx == sent ? TFT_CYAN : TFT_WHITE, bg);
    spr.drawString(b, 22, cy, 2);
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
  drawPowerHeader(now_mw, sent);
  drawPowerVisible(scroll, cursor, now_mw, sent);
}

// --- Menu item: FAN threshold — the TX power at/above which the module's fan runs.
// Blind CRSF parameter write to "Fan Thresh" (ELRS 3.4.x field 8; value = raw option
// index, no min-shift). No telemetry read-back for the fan, so no live "current" marker.
const char* const FAN_OPTS[] = {"10mW",  "25mW",  "50mW",   "100mW", "250mW",
                                "500mW", "1000mW", "2000mW", "NEVER"};
constexpr int FAN_N = 9;
constexpr int FAN_FIELD = 8;  // Max Power(6) -> Dynamic(7) -> Fan Thresh(8) on ELRS 3.4.x

void drawSettingHeader(const char* title, int sent) {
  tft.fillRect(0, 0, LCD_W, 12, COL_BG);
  tft.setTextPadding(0);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_YELLOW, COL_BG);
  tft.drawString(title, 4, 5, 1);
  tft.setTextDatum(MR_DATUM);
  tft.setTextColor(sent >= 0 ? TFT_GREEN : TFT_DARKGREY, COL_BG);
  tft.drawString(sent >= 0 ? "SAVED" : "OK:set  CXL:back", LCD_W - 4, 5, 1);
}

// green bullet = module's current threshold; cyan text = the option we just wrote.
void drawFanRow(int slot, const char* label, bool cursor, bool is_current, bool is_sent) {
  int y = LIST_ROW_Y0 + slot * LIST_ROW_H;
  uint16_t bg = cursor ? TFT_NAVY : COL_BG;
  TFT_eSprite spr(&tft);
  spr.createSprite(LCD_W, LIST_ROW_H);
  spr.setTextFont(1);
  spr.setTextSize(1);
  spr.fillSprite(bg);
  if (label) {
    int cy = LIST_ROW_H / 2;
    if (is_current) spr.fillCircle(9, cy, 4, TFT_GREEN);
    else spr.drawCircle(9, cy, 4, TFT_DARKGREY);
    spr.setTextDatum(ML_DATUM);
    spr.setTextColor(is_sent ? TFT_CYAN : TFT_WHITE, bg);
    spr.drawString(label, 22, cy, 2);
  }
  spr.pushSprite(0, y - LIST_ROW_H / 2);
  spr.deleteSprite();
}

void drawSettingList(const char* title, int scroll, int cursor, const char* const* labels,
                     int n, int current, int sent) {
  drawSettingHeader(title, sent);
  for (int s = 0; s < LIST_VISIBLE; ++s) {
    int idx = scroll + s;
    drawFanRow(s, idx < n ? labels[idx] : nullptr, idx == cursor, idx == current, idx == sent);
  }
}

// --- Menu item: PACKET RATE (ELRS field 1). value = list index; verified via telemetry
// rf_Mode. NOTE: at 400000 baud on this half-duplex module, rates above 333Hz are capped by
// the module — the green dot (actual rf_Mode) reveals it. Use BAUD 921600 to unlock them. ---
const char* const PKT_OPTS[] = {"50Hz",  "100Hz FULL", "150Hz", "250Hz",  "333Hz FULL",
                                "500Hz", "D250",       "D500",  "F500",   "F1000"};
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

// --- Menu item: BAUD RATE (handset UART; module auto-detects). ---
const char* const BAUD_OPTS[] = {"400000", "921600"};
const uint32_t BAUD_VALS[] = {400000, 921600};
constexpr int BAUD_N = 2;

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

  // Lifetime total flight time (accumulated armed seconds from NVS), below the icon.
  uint32_t tot = storage_get_total_flight_secs();
  char b[28];
  snprintf(b, sizeof(b), "TOTAL FLIGHT %luh%02lum", (unsigned long)(tot / 3600),
           (unsigned long)((tot % 3600) / 60));
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_DARKGREY, COL_BG);
  tft.drawString(b, 4, 70, 1);
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

// --- Menu item 2: Toggle -> CRSF channel + function (Toggle6 reserved for menu mode).
const int EDIT_TOGGLE[9] = {0, 1, 2, 3, 4, 6, 7, 8, 9};  // toggle indices, skip Toggle6
constexpr int EDIT_TOGGLE_N = 9;

// Function labels a toggle can be assigned (index stored in gconfig). ARM is special
// (drives arm-safety + flight timer); the rest are labels the flight controller acts on.
const char* const FUNC_NAMES[] = {"-",   "ARM", "ANGLE", "HORIZ", "ACRO",
                                  "AIR", "TURTLE", "RTH", "BEEP", "PREARM"};
constexpr int FUNC_COUNT = 10;
constexpr uint8_t FUNC_ARM = GFUNC_ARM;  // shared with crsf.cpp arm-safety

// Scrolling list geometry (shares the main-menu list geometry).
constexpr int TM_VISIBLE = LIST_VISIBLE;
constexpr int TM_ROW_H = LIST_ROW_H;
constexpr int TM_ROW_Y0 = LIST_ROW_Y0;

void drawToggleHeader(bool editing, int field) {
  tft.fillRect(0, 0, LCD_W, 12, COL_BG);  // clear header row only
  tft.setTextPadding(0);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_YELLOW, COL_BG);
  tft.drawString("TOGGLE MAP", 4, 5, 1);
  tft.setTextDatum(MR_DATUM);
  tft.setTextColor(TFT_DARKGREY, COL_BG);
  const char* hint = !editing       ? "OK:edit  CXL:back"
                     : (field == 0) ? "edit CH   OK:next"
                                    : "edit FUNC OK:save";
  tft.drawString(hint, LCD_W - 4, 5, 1);
}

// Draw one visible row. slot = visual position 0..TM_VISIBLE-1; eidx = index into
// EDIT_TOGGLE (or out of range for a blank slot on the last short page).
// field: 0 = channel, 1 = function (highlighted green when sel && editing).
// Rendered into an off-screen sprite and blitted once, so a moving cursor never
// flashes the row as a solid bar between clear and text.
void drawToggleRow(int slot, int eidx, bool sel, bool editing, int field) {
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
    char ts[6], cs[6], fs[10];
    snprintf(ts, sizeof(ts), "T%d", t + 1);
    if (c) snprintf(cs, sizeof(cs), "CH%u", c);
    else snprintf(cs, sizeof(cs), "--");
    snprintf(fs, sizeof(fs), "%s", FUNC_NAMES[fn < FUNC_COUNT ? fn : 0]);

    uint16_t base = sel ? TFT_YELLOW : TFT_WHITE;
    uint16_t chCol = (sel && editing && field == 0) ? TFT_GREEN : base;
    uint16_t fnCol = (sel && editing && field == 1) ? TFT_GREEN : base;
    int cy = TM_ROW_H / 2;  // vertical center inside the sprite
    spr.setTextDatum(ML_DATUM);
    spr.setTextPadding(0);
    spr.setTextColor(base, bg);
    spr.drawString(ts, 8, cy, 2);
    spr.setTextColor(chCol, bg);
    spr.drawString(cs, 72, cy, 2);
    spr.setTextColor(fnCol, bg);
    spr.drawString(fs, 150, cy, 2);
  }
  spr.pushSprite(0, y - TM_ROW_H / 2);
  spr.deleteSprite();
}

void drawToggleListFull(int scroll, int cursor, bool editing, int field) {
  // Header + 4 sprite rows fully cover the panel, so no fillScreen (which would flash).
  drawToggleHeader(editing, field);
  for (int s = 0; s < TM_VISIBLE; ++s) {
    int eidx = scroll + s;
    drawToggleRow(s, eidx, eidx == cursor, editing, field);
  }
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
  tft.drawString("RATE", 4, 68, 1);  // value (baud + actual RF rate) drawn dynamically
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
  static uint32_t last_baud = 0;
  static int last_rf = -999;
  CrsfLink l;
  crsf_get(l);
  char b[20];

  // RATE: actual UART baud + the RF rate the module reports (rf_Mode). Updates when the
  // BAUD menu changes the wire speed or the PACKET RATE menu changes the RF rate.
  uint32_t baud = crsf_get_baud();
  int rf = (l.valid && l.up_lq > 0) ? l.rf_mode : -1;
  if (force || baud != last_baud || rf != last_rf) {
    last_baud = baud;
    last_rf = rf;
    int pi = pktCurrentIndex((uint8_t)l.rf_mode, l.valid && l.up_lq > 0);
    char rate[10];
    if (pi >= 0) snprintf(rate, sizeof(rate), "%s", PKT_OPTS[pi]);
    else snprintf(rate, sizeof(rate), "--");
    snprintf(b, sizeof(b), "%luk %s", (unsigned long)(baud / 1000), rate);
    drawCrsfVal(68, b, TFT_WHITE);
  }

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
  // Handset TX is silent in this mode; the module auto-starts WiFi after its
  // "Auto On WiFi" interval IF it powers up seeing no CRSF (so power-cycle it now).
  tft.drawString("1. Power-cycle module now", 6, 20, 1);
  tft.drawString("2. Wait ~60s (TX stays silent)", 6, 32, 1);
  tft.drawString("3. AP: ExpressLRS TX  pw:expresslrs", 6, 44, 1);
  tft.drawString("4. http://10.0.0.1", 6, 56, 1);
  tft.setTextColor(TFT_YELLOW, COL_BG);
  tft.drawString("Toggle6 OFF = exit & fly", 6, 68, 1);
}

// Config menu loop — runs while Toggle6 is ON; returns when it goes OFF.
void configLoop() {
  enum class Page : uint8_t {
    List, Gimbals, Toggles, Battery, Telemetry, Crsf, Power, Fan, PktRate, Baud, Reboot
  };
  Page page = Page::List;
  int sel = 0;
  int menu_scroll = 0;  // index of the top visible main-menu row
  int gcursor = 0;   // selected channel on the Gimbals page
  int tmcursor = 0;  // selected toggle on the Toggle Mapping page
  int tm_scroll = 0;  // index (into EDIT_TOGGLE) of the top visible row
  bool tm_editing = false;
  int tm_field = 0;         // 0 = channel, 1 = function (which field OK cycles)
  uint8_t tm_prevchan = 0;  // channel before entering edit (for CANCEL revert)
  uint8_t tm_prevfunc = 0;  // function before entering edit (for CANCEL revert)
  int te_cursor = 0;   // selected telemetry item
  int te_scroll = 0;   // top visible telemetry row
  int te_tick = 0;     // throttles live value refresh
  int pw_cursor = 0;   // selected TX-power level
  int pw_scroll = 0;   // top visible power row
  int pw_tick = 0;     // throttles the "NOW" telemetry refresh
  int pw_sent = -1;    // last power level we sent (for the cyan marker)
  int fan_cursor = 0, fan_scroll = 0, fan_sent = -1;  // FAN page (blind write to field 8)
  int fan_current = -2;  // last-written threshold (green dot); -2 = not yet loaded from NVS
  int pkt_cursor = 0, pkt_scroll = 0, pkt_sent = -1, pkt_tick = 0;  // PACKET RATE page
  int baud_cursor = 0, baud_scroll = 0, baud_sent = -1;            // BAUD RATE page
  bool crsf_confirm = false;  // CRSF page: WiFi-setup confirm dialog
  bool redraw = true;
  EdgeKey ku, kd, kok, kcx;
  const TickType_t period = pdMS_TO_TICKS(50);

  while (toggleOn(TOGGLE6_BIT)) {
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
            tm_field = 0;  // start on the channel field
            tm_prevchan = gconfig_toggle_chan(t);
            tm_prevfunc = gconfig_toggle_func(t);
            drawToggleHeader(true, tm_field);
            drawToggleRow(tmcursor - tm_scroll, tmcursor, true, true, tm_field);
          }
          if (cx) {
            page = Page::List;
            redraw = true;
          }
        } else if (tm_field == 0) {
          // Edit CHANNEL: up/down cycle channel; OK advances to function; CXL reverts.
          uint8_t c = gconfig_toggle_chan(t);
          if (up) {  // none -> 5..16 -> none
            gconfig_set_toggle_chan(t, (c == 0) ? 5 : (c >= 16 ? 0 : c + 1));
            drawToggleRow(tmcursor - tm_scroll, tmcursor, true, true, tm_field);
          }
          if (dn) {  // none -> 16..5 -> none
            gconfig_set_toggle_chan(t, (c == 0) ? 16 : (c <= 5 ? 0 : c - 1));
            drawToggleRow(tmcursor - tm_scroll, tmcursor, true, true, tm_field);
          }
          if (ok) {  // advance to the function field
            tm_field = 1;
            drawToggleHeader(true, tm_field);
            drawToggleRow(tmcursor - tm_scroll, tmcursor, true, true, tm_field);
          }
          if (cx) {  // cancel the whole edit, revert both fields
            gconfig_set_toggle_chan(t, tm_prevchan);
            gconfig_set_toggle_func(t, tm_prevfunc);
            tm_editing = false;
            drawToggleHeader(false, 0);
            drawToggleRow(tmcursor - tm_scroll, tmcursor, true, false, 0);
          }
        } else {
          // Edit FUNCTION: up/down cycle label; OK saves both fields; CXL reverts.
          uint8_t f = gconfig_toggle_func(t);
          if (up) {
            gconfig_set_toggle_func(t, (uint8_t)((f + 1) % FUNC_COUNT));
            drawToggleRow(tmcursor - tm_scroll, tmcursor, true, true, tm_field);
          }
          if (dn) {
            gconfig_set_toggle_func(t, (uint8_t)((f + FUNC_COUNT - 1) % FUNC_COUNT));
            drawToggleRow(tmcursor - tm_scroll, tmcursor, true, true, tm_field);
          }
          if (ok) {  // commit channel + function
            safeSaveGconfig();
            tm_editing = false;
            tm_field = 0;
            drawToggleHeader(false, 0);
            drawToggleRow(tmcursor - tm_scroll, tmcursor, true, false, 0);
          }
          if (cx) {  // cancel the whole edit, revert both fields
            gconfig_set_toggle_chan(t, tm_prevchan);
            gconfig_set_toggle_func(t, tm_prevfunc);
            tm_editing = false;
            tm_field = 0;
            drawToggleHeader(false, 0);
            drawToggleRow(tmcursor - tm_scroll, tmcursor, true, false, 0);
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
      case Page::Telemetry: {
        CrsfLink l;
        bool lok = crsf_get(l);
        if (redraw) {
          if (te_cursor < te_scroll) te_scroll = te_cursor;
          if (te_cursor >= te_scroll + LIST_VISIBLE) te_scroll = te_cursor - LIST_VISIBLE + 1;
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
          if (te_cursor >= te_scroll + LIST_VISIBLE) te_scroll = te_cursor - LIST_VISIBLE + 1;
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
          crsf_set_power(pw_cursor);
          pw_sent = pw_cursor;
          drawPowerHeader(now, pw_sent);
          drawPowerVisible(pw_scroll, pw_cursor, now, pw_sent);
        }
        if (++pw_tick >= 4) {  // ~5 Hz refresh of the telemetry-reported power
          pw_tick = 0;
          drawPowerHeader(now, pw_sent);
          drawPowerVisible(pw_scroll, pw_cursor, now, pw_sent);
        }
        if (cx) {
          page = Page::List;
          redraw = true;
        }
        break;
      }
      case Page::Fan: {
        if (redraw) {
          if (fan_current == -2) {  // first entry: load the remembered threshold
            uint8_t v = storage_get_fan_thresh();
            fan_current = (v < FAN_N) ? v : -1;
            if (fan_current >= 0) fan_cursor = fan_current;  // start on the current one
          }
          if (fan_cursor < fan_scroll) fan_scroll = fan_cursor;
          if (fan_cursor >= fan_scroll + LIST_VISIBLE) fan_scroll = fan_cursor - LIST_VISIBLE + 1;
          drawSettingList("FAN THRESHOLD", fan_scroll, fan_cursor, FAN_OPTS, FAN_N, fan_current, fan_sent);
          redraw = false;
        }
        if (up && fan_cursor > 0) {
          fan_cursor--;
          if (fan_cursor < fan_scroll) fan_scroll = fan_cursor;
          drawSettingList("FAN THRESHOLD", fan_scroll, fan_cursor, FAN_OPTS, FAN_N, fan_current, fan_sent);
        }
        if (dn && fan_cursor < FAN_N - 1) {
          fan_cursor++;
          if (fan_cursor >= fan_scroll + LIST_VISIBLE) fan_scroll = fan_cursor - LIST_VISIBLE + 1;
          drawSettingList("FAN THRESHOLD", fan_scroll, fan_cursor, FAN_OPTS, FAN_N, fan_current, fan_sent);
        }
        if (ok) {  // blind write the selected threshold (field 8, raw option index) + remember it
          safeSaveFanThresh((uint8_t)fan_cursor);
          crsf_param_set(FAN_FIELD, fan_cursor);
          fan_sent = fan_cursor;
          fan_current = fan_cursor;  // green dot moves to the new setting
          drawSettingList("FAN THRESHOLD", fan_scroll, fan_cursor, FAN_OPTS, FAN_N, fan_current, fan_sent);
        }
        if (cx) {
          page = Page::List;
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
          drawSettingList("PACKET RATE", pkt_scroll, pkt_cursor, PKT_OPTS, PKT_N, cur, pkt_sent);
        }
        if (dn && pkt_cursor < PKT_N - 1) {
          pkt_cursor++;
          if (pkt_cursor >= pkt_scroll + LIST_VISIBLE) pkt_scroll = pkt_cursor - LIST_VISIBLE + 1;
          drawSettingList("PACKET RATE", pkt_scroll, pkt_cursor, PKT_OPTS, PKT_N, cur, pkt_sent);
        }
        if (ok) {  // write field 1 = list index (module may clamp at 400k baud; see rf_Mode)
          crsf_param_set(1, pkt_cursor);
          pkt_sent = pkt_cursor;
          drawSettingList("PACKET RATE", pkt_scroll, pkt_cursor, PKT_OPTS, PKT_N, cur, pkt_sent);
        }
        if (++pkt_tick >= 4) {  // refresh the green "actual rate" dot from telemetry
          pkt_tick = 0;
          drawSettingList("PACKET RATE", pkt_scroll, pkt_cursor, PKT_OPTS, PKT_N, cur, pkt_sent);
        }
        if (cx) {
          page = Page::List;
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
          drawSettingList("BAUD RATE", baud_scroll, baud_cursor, BAUD_OPTS, BAUD_N, cur, baud_sent);
        }
        if (dn && baud_cursor < BAUD_N - 1) {
          baud_cursor++;
          drawSettingList("BAUD RATE", baud_scroll, baud_cursor, BAUD_OPTS, BAUD_N, cur, baud_sent);
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
    flightTimerTick();
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
  g_telemSel = storage_get_telem_sel();  // which telemetry items show on the RC screen
  crsf_set_baud(storage_get_crsf_baud());  // seed the CRSF baud before crsf_start()

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
  gconfig_set_cal(g_cal);  // ensure CRSF maps against the just-loaded/captured calibration

  throttleGate();  // block while throttle > 15%

  modeLoop();  // live Toggle6: Config vs RC (does not return)
}

}  // namespace

bool ui_start() {
  // 10 KB stack: TFT_eSPI sprite text rendering + float formatting in renderRc need
  // well more than the old 6 KB (a deep drawString there overflowed it -> LoadStoreError).
  BaseType_t ok = xTaskCreatePinnedToCore(appTask, "app", 10240, nullptr,
                                          /*priority=*/1, nullptr, /*core=*/0);
  return ok == pdPASS;
}
