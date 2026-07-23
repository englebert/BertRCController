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

// Total accumulated flight time (persisted). Used by Phase 2.
uint32_t storage_get_total_flight_secs();
void storage_add_flight_secs(uint32_t secs);
