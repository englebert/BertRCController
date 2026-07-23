// gimbals.h — 4-channel analog gimbal sampling task.
//
// Reads the 4 sticks via analogContinuous() (DMA scan on ADC1), filters each channel
// (median-of-11 -> light EMA), and publishes the latest filtered raw counts (0..4095)
// to a mutex-guarded struct. Runs on core 1 at the highest priority (see CLAUDE.md).
#pragma once

#include <stdbool.h>
#include <stdint.h>

struct GimbalState {
  int16_t throttle;  // GPIO32
  int16_t yaw;       // GPIO33
  int16_t pitch;     // GPIO34
  int16_t roll;      // GPIO35
  bool valid;        // false until the first DMA batch has been filtered
};

// Configure the ADC + start the gimbal task. Call once from setup().
bool gimbals_start();

// Copy the latest filtered state into `out`. Returns out.valid. Safe from any task.
bool gimbals_get(GimbalState& out);
