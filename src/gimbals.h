// gimbals.h — 4-channel analog gimbal sampling task.
//
// Polls the 4 sticks with oneshot analogRead() (~1 kHz on ADC1 — continuous/DMA scan
// was dropped, it misbehaves with >3 channels on this board), filters each channel
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

// Suspend/resume the polling task. Wrap NVS/flash writes with these so no task runs
// flash-cached ADC code while the cache is disabled on the other core during a write.
void gimbals_pause();
void gimbals_resume();
