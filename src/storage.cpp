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
constexpr const char* KEY_TOTAL_FLIGHT = "tflight";

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

uint32_t storage_get_total_flight_secs() {
  if (!g_open) return 0;
  return prefs.getULong(KEY_TOTAL_FLIGHT, 0);
}

void storage_add_flight_secs(uint32_t secs) {
  if (!g_open || secs == 0) return;
  prefs.putULong(KEY_TOTAL_FLIGHT, prefs.getULong(KEY_TOTAL_FLIGHT, 0) + secs);
}
