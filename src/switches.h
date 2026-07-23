// switches.h — MCP23017 switch reader task.
//
// 10 switches on the MCP23017 (GPA0-GPA7 + GPB0-GPB1), each wired pin->GND with the
// chip's internal pull-ups enabled, so they read active-low. This task polls + debounces
// on core 0 and publishes a debounced bitmask (bit i set = switch i CLOSED/ON).
#pragma once

#include <stdbool.h>
#include <stdint.h>

static constexpr int SWITCH_COUNT = 10;

struct SwitchState {
  uint16_t bits;  // bit i = switch i, 1 = ON (closed to GND)
  bool valid;     // false until the MCP23017 has been read successfully
};

// Init the MCP23017 and start the switches task. Call once from setup().
bool switches_start();

// Copy the latest debounced state into `out`. Returns out.valid. Safe from any task.
bool switches_get(SwitchState& out);
