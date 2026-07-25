// gconfig.cpp — shared channel config: gimbal reverse (flip) + toggle->channel map.
#include "gconfig.h"

#include <Arduino.h>

#include "storage.h"

namespace {
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
bool g_flip[4] = {false, false, false, false};
uint8_t g_tchan[10] = {0};  // toggle -> CRSF channel (5..16), 0 = unassigned
uint8_t g_tfunc[10] = {0};  // toggle -> function index, 0 = none
GimbalCal g_cal = {};       // gimbal calibration endpoints (raw ADC -> CRSF)

constexpr int TOGGLE6 = 5;  // reserved for menu mode; never mapped

// Calibration mapping constants (must match ui.cpp calPct + CRSF range in crsf.cpp).
constexpr int CAL_MARGIN = 5;
constexpr int CRSF_MIN = 172;
constexpr int CRSF_MAX = 1811;

template <typename T>
T clampv(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

void gconfig_init() {
  // Flip flags.
  bool f[4];
  if (storage_load_flip(f)) {
    portENTER_CRITICAL(&g_mux);
    for (int i = 0; i < 4; ++i) g_flip[i] = f[i];
    portEXIT_CRITICAL(&g_mux);
    Serial.printf("[gconfig] flip loaded T=%d Y=%d P=%d R=%d\n", f[0], f[1], f[2], f[3]);
  } else {
    Serial.println("[gconfig] flip: default (none stored)");
  }

  // Toggle -> channel map. Default: Toggle_i -> CH(5+i), except Toggle6 (menu) = none.
  uint8_t t[10];
  if (storage_load_toggles(t)) {
    portENTER_CRITICAL(&g_mux);
    for (int i = 0; i < 10; ++i) g_tchan[i] = t[i];
    portEXIT_CRITICAL(&g_mux);
    Serial.println("[gconfig] toggle map loaded");
  } else {
    portENTER_CRITICAL(&g_mux);
    for (int i = 0; i < 10; ++i) g_tchan[i] = (i == TOGGLE6) ? 0 : (uint8_t)(5 + i);
    portEXIT_CRITICAL(&g_mux);
    Serial.println("[gconfig] toggle map: default");
  }

  // Toggle -> function labels (default none).
  uint8_t fn[10];
  if (storage_load_toggle_funcs(fn)) {
    portENTER_CRITICAL(&g_mux);
    for (int i = 0; i < 10; ++i) g_tfunc[i] = fn[i];
    portEXIT_CRITICAL(&g_mux);
    Serial.println("[gconfig] toggle funcs loaded");
  }

  // Gimbal calibration (for the raw-ADC -> CRSF map). Leaves g_cal.valid=false (linear
  // fallback) when nothing is stored; the UI refreshes this after the wizard.
  GimbalCal cal;
  if (storage_load_cal(cal)) {
    portENTER_CRITICAL(&g_mux);
    g_cal = cal;
    portEXIT_CRITICAL(&g_mux);
    Serial.println("[gconfig] calibration loaded");
  }
}

void gconfig_set_cal(const GimbalCal& cal) {
  portENTER_CRITICAL(&g_mux);
  g_cal = cal;
  portEXIT_CRITICAL(&g_mux);
}

uint16_t gconfig_map_crsf(int idx, int raw) {
  if (idx < 0 || idx > 3) return (CRSF_MIN + CRSF_MAX) / 2;
  GimbalCal c;
  bool flip;
  portENTER_CRITICAL(&g_mux);
  c = g_cal;
  flip = g_flip[idx];
  portEXIT_CRITICAL(&g_mux);

  float pct;
  if (!c.valid || c.max[idx] <= c.min[idx]) {
    pct = clampv(raw, 0, 4095) * 100.0f / 4095.0f;  // uncalibrated linear fallback
  } else {
    int lo = (int)c.min[idx] - CAL_MARGIN;
    int hi = (int)c.max[idx] + CAL_MARGIN;
    int mid = c.mid[idx];
    if (mid <= lo || mid >= hi) mid = (lo + hi) / 2;
    if (raw <= mid) pct = (raw - lo) * 50.0f / (mid - lo);
    else pct = 50.0f + (raw - mid) * 50.0f / (hi - mid);
    pct = clampv(pct, 0.0f, 100.0f);
  }
  if (flip) pct = 100.0f - pct;
  int out = CRSF_MIN + (int)(pct * (CRSF_MAX - CRSF_MIN) / 100.0f + 0.5f);
  return (uint16_t)clampv(out, CRSF_MIN, CRSF_MAX);
}

bool gconfig_flip(int idx) {
  if (idx < 0 || idx > 3) return false;
  portENTER_CRITICAL(&g_mux);
  bool v = g_flip[idx];
  portEXIT_CRITICAL(&g_mux);
  return v;
}

void gconfig_set_flip(int idx, bool v) {
  if (idx < 0 || idx > 3) return;
  portENTER_CRITICAL(&g_mux);
  g_flip[idx] = v;
  portEXIT_CRITICAL(&g_mux);
}

uint8_t gconfig_toggle_chan(int toggle) {
  if (toggle < 0 || toggle > 9 || toggle == TOGGLE6) return 0;
  portENTER_CRITICAL(&g_mux);
  uint8_t v = g_tchan[toggle];
  portEXIT_CRITICAL(&g_mux);
  return v;
}

void gconfig_set_toggle_chan(int toggle, uint8_t ch) {
  if (toggle < 0 || toggle > 9 || toggle == TOGGLE6) return;
  portENTER_CRITICAL(&g_mux);
  g_tchan[toggle] = ch;
  portEXIT_CRITICAL(&g_mux);
}

uint8_t gconfig_toggle_func(int toggle) {
  if (toggle < 0 || toggle > 9 || toggle == TOGGLE6) return 0;
  portENTER_CRITICAL(&g_mux);
  uint8_t v = g_tfunc[toggle];
  portEXIT_CRITICAL(&g_mux);
  return v;
}

void gconfig_set_toggle_func(int toggle, uint8_t f) {
  if (toggle < 0 || toggle > 9 || toggle == TOGGLE6) return;
  portENTER_CRITICAL(&g_mux);
  g_tfunc[toggle] = f;
  portEXIT_CRITICAL(&g_mux);
}

int gconfig_toggle_with_func(uint8_t f) {
  if (f == 0) return -1;
  portENTER_CRITICAL(&g_mux);
  int found = -1;
  for (int i = 0; i < 10; ++i)
    if (g_tfunc[i] == f) {
      found = i;
      break;
    }
  portEXIT_CRITICAL(&g_mux);
  return found;
}

bool gconfig_save() {
  bool f[4];
  uint8_t t[10], fn[10];
  portENTER_CRITICAL(&g_mux);
  for (int i = 0; i < 4; ++i) f[i] = g_flip[i];
  for (int i = 0; i < 10; ++i) {
    t[i] = g_tchan[i];
    fn[i] = g_tfunc[i];
  }
  portEXIT_CRITICAL(&g_mux);
  bool ok1 = storage_save_flip(f);
  bool ok2 = storage_save_toggles(t);
  bool ok3 = storage_save_toggle_funcs(fn);
  return ok1 && ok2 && ok3;
}
