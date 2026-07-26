// power.h — INA226 pack voltage / current monitoring task.
//
// Runs on core 0 at low priority, reads the INA226 at a few Hz, and publishes the
// latest reading to a mutex-guarded struct that other tasks (UI, telemetry) can read.
#pragma once

#include <stdbool.h>

struct PowerReading {
  float voltage_V;   // pack bus voltage
  float current_mA;  // pack current
  float power_mW;    // pack power
  bool near_full;    // within ~10% of the INA226 ~3.28 A full-scale ceiling
  bool valid;        // false until a good read has happened (sensor present)
};

// Init I2C + INA226 and start the power task. Call once from setup().
bool power_start();

// Copy the latest reading into `out`. Returns out.valid. Safe from any task.
bool power_get(PowerReading& out);
