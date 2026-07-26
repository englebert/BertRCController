// storage.h — persistent settings in ESP32 NVS (Preferences).
//
// Holds gimbal calibration endpoints now; total flight time and toggle function
// assignments land here in later phases (see TASK01.md).
#pragma once

#include <stdbool.h>
#include <stdint.h>

// Per-stick raw ADC endpoints + center, captured by the calibration wizard
// (min/max from circling the corners; center derived as (min+max)/2).
// Index order matches GimbalState: 0=Throttle, 1=Yaw, 2=Pitch, 3=Roll.
struct GimbalCal {
  uint16_t min[4];
  uint16_t max[4];
  uint16_t mid[4];
  bool valid;
};

// Open the NVS namespace. Call once from the app task before other storage_* calls.
bool storage_init();

// Gimbal calibration. load returns out.valid=false when nothing is stored yet.
bool storage_load_cal(GimbalCal& out);
bool storage_save_cal(const GimbalCal& cal);

// Per-gimbal channel reverse (flip) flags. Index 0=T,1=Y,2=P,3=R.
// load returns false (leaving defaults) when nothing is stored yet.
bool storage_load_flip(bool out[4]);
bool storage_save_flip(const bool in[4]);

// Toggle -> CRSF channel map. Index 0=Toggle1..9=Toggle10; value = channel 5..16
// (0 = unassigned). load returns false (leaving defaults) when nothing is stored.
bool storage_load_toggles(uint8_t out[10]);
bool storage_save_toggles(const uint8_t in[10]);

// Toggle -> function label map (index into the UI's function name list; 0 = none).
bool storage_load_toggle_funcs(uint8_t out[10]);
bool storage_save_toggle_funcs(const uint8_t in[10]);

// "Boot into module-WiFi-setup" flag: when set, the app boots without starting CRSF
// so the ELRS module (seeing no handset) enters its WiFi config mode.
bool storage_get_wifi_boot();
void storage_set_wifi_boot(bool v);

// Bitmask of telemetry items shown on the main RC screen (bit i = item i). Default 0.
uint16_t storage_get_telem_sel();
void storage_set_telem_sel(uint16_t v);

// Last fan-threshold option index we wrote to the module (for the UI's current marker).
// Returns 0xFF when never set. The module itself is the source of truth.
uint8_t storage_get_fan_thresh();
void storage_set_fan_thresh(uint8_t v);

// Total accumulated flight time (persisted). Used by Phase 2.
uint32_t storage_get_total_flight_secs();
void storage_add_flight_secs(uint32_t secs);
