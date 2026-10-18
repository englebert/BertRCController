// storage.cpp — NVS-backed settings (Preferences).
#include "storage.h"

#include <Arduino.h>
#include <Preferences.h>

namespace {

Preferences prefs;
bool g_open = false;

constexpr const char* NS = "bertrc";
constexpr const char* KEY_CAL = "gcal";
constexpr const char* KEY_FLIP = "gflip";
constexpr const char* KEY_TMAP = "tmap";
constexpr const char* KEY_TFUNC = "tfunc";
constexpr const char* KEY_TPIN = "tpin";
constexpr const char* KEY_TINV = "tinv";
constexpr const char* KEY_WIFI_BOOT = "wifiboot";
constexpr const char* KEY_CRSF_OUT = "crsfout";
constexpr const char* KEY_TOTAL_FLIGHT = "tflight";
constexpr const char* KEY_TELEM_SEL = "telemsel";
constexpr const char* KEY_FAN_THRESH = "fanthr";
constexpr const char* KEY_CRSF_BAUD = "crsfbaud";

}  // namespace

bool storage_init() {
  g_open = prefs.begin(NS, false);
  if (!g_open) Serial.println("[storage] NVS open FAILED");
  return g_open;
}

bool storage_load_cal(GimbalCal& out) {
  out.valid = false;
  if (!g_open) return false;
  // Fixed blob: min[4], max[4], mid[4] (validity = key presence + exact size, so an
  // old 8-value blob from a previous layout reads as "not calibrated").
  uint16_t buf[12];
  if (prefs.getBytes(KEY_CAL, buf, sizeof(buf)) != sizeof(buf)) return false;
  for (int i = 0; i < 4; ++i) {
    out.min[i] = buf[i];
    out.max[i] = buf[4 + i];
    out.mid[i] = buf[8 + i];
  }
  out.valid = true;
  return true;
}

bool storage_save_cal(const GimbalCal& cal) {
  if (!g_open) return false;
  uint16_t buf[12];
  for (int i = 0; i < 4; ++i) {
    buf[i] = cal.min[i];
    buf[4 + i] = cal.max[i];
    buf[8 + i] = cal.mid[i];
  }
  return prefs.putBytes(KEY_CAL, buf, sizeof(buf)) == sizeof(buf);
}

bool storage_load_flip(bool out[4]) {
  if (!g_open) return false;
  uint8_t buf[4];
  if (prefs.getBytes(KEY_FLIP, buf, sizeof(buf)) != sizeof(buf)) return false;
  for (int i = 0; i < 4; ++i) out[i] = (buf[i] != 0);
  return true;
}

bool storage_save_flip(const bool in[4]) {
  if (!g_open) return false;
  uint8_t buf[4];
  for (int i = 0; i < 4; ++i) buf[i] = in[i] ? 1 : 0;
  return prefs.putBytes(KEY_FLIP, buf, sizeof(buf)) == sizeof(buf);
}

bool storage_load_toggles(uint8_t out[10]) {
  if (!g_open) return false;
  return prefs.getBytes(KEY_TMAP, out, 10) == 10;
}

bool storage_save_toggles(const uint8_t in[10]) {
  if (!g_open) return false;
  return prefs.putBytes(KEY_TMAP, in, 10) == 10;
}

bool storage_load_toggle_pins(uint8_t out[10]) {
  if (!g_open) return false;
  if (!prefs.isKey(KEY_TPIN)) return false;  // never saved yet: defaults, no NVS error log
  return prefs.getBytes(KEY_TPIN, out, 10) == 10;
}

bool storage_save_toggle_pins(const uint8_t in[10]) {
  if (!g_open) return false;
  return prefs.putBytes(KEY_TPIN, in, 10) == 10;
}

bool storage_load_toggle_inv(uint8_t out[10]) {
  if (!g_open) return false;
  if (!prefs.isKey(KEY_TINV)) return false;  // never saved yet: defaults, no NVS error log
  return prefs.getBytes(KEY_TINV, out, 10) == 10;
}

bool storage_save_toggle_inv(const uint8_t in[10]) {
  if (!g_open) return false;
  return prefs.putBytes(KEY_TINV, in, 10) == 10;
}

bool storage_load_toggle_funcs(uint8_t out[10]) {
  if (!g_open) return false;
  return prefs.getBytes(KEY_TFUNC, out, 10) == 10;
}

bool storage_save_toggle_funcs(const uint8_t in[10]) {
  if (!g_open) return false;
  return prefs.putBytes(KEY_TFUNC, in, 10) == 10;
}

bool storage_get_wifi_boot() {
  if (!g_open) return false;
  return prefs.getBool(KEY_WIFI_BOOT, false);
}

void storage_set_wifi_boot(bool v) {
  if (g_open) prefs.putBool(KEY_WIFI_BOOT, v);
}

bool storage_get_crsf_output() {
  if (!g_open || !prefs.isKey(KEY_CRSF_OUT)) return true;  // never set: on (no NVS error log)
  return prefs.getBool(KEY_CRSF_OUT, true);
}

void storage_set_crsf_output(bool on) {
  if (g_open) prefs.putBool(KEY_CRSF_OUT, on);
}

uint16_t storage_get_telem_sel() {
  if (!g_open) return 0;
  return prefs.getUShort(KEY_TELEM_SEL, 0);
}

void storage_set_telem_sel(uint16_t v) {
  if (g_open) prefs.putUShort(KEY_TELEM_SEL, v);
}

// NVS key per ElrsSetting (the fan keeps its original key so a stored value carries over).
constexpr const char* KEY_ELRS[ELRS_SETTING_N] = {"elrspkt", "elrsswm", "elrspwr",
                                                  KEY_FAN_THRESH, "elrstlm", "elrsmm"};

uint8_t storage_get_elrs(ElrsSetting s) {
  if (!g_open || s >= ELRS_SETTING_N) return 0xFF;
  if (!prefs.isKey(KEY_ELRS[s])) return 0xFF;  // never saved: no NVS error log
  return prefs.getUChar(KEY_ELRS[s], 0xFF);
}

void storage_set_elrs(ElrsSetting s, uint8_t v) {
  if (g_open && s < ELRS_SETTING_N) prefs.putUChar(KEY_ELRS[s], v);
}

uint32_t storage_get_crsf_baud() {
  if (!g_open) return 921600;
  return prefs.getULong(KEY_CRSF_BAUD, 921600);  // default 921600 to allow 500Hz/F1000
}

void storage_set_crsf_baud(uint32_t v) {
  if (g_open) prefs.putULong(KEY_CRSF_BAUD, v);
}

uint32_t storage_get_total_flight_secs() {
  if (!g_open) return 0;
  return prefs.getULong(KEY_TOTAL_FLIGHT, 0);
}

void storage_add_flight_secs(uint32_t secs) {
  if (!g_open || secs == 0) return;
  prefs.putULong(KEY_TOTAL_FLIGHT, prefs.getULong(KEY_TOTAL_FLIGHT, 0) + secs);
}
