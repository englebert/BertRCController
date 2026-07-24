// gconfig.h — shared gimbal channel config (reverse/flip flags).
//
// Runtime cache read by the CRSF task (output) and the UI (display + editing).
// Loaded from NVS at startup; defaults to no-flip when nothing is stored.
// Index order matches GimbalState: 0=Throttle, 1=Yaw, 2=Pitch, 3=Roll.
#pragma once

#include <stdbool.h>

// Load flip flags from NVS into the runtime cache (default all false). Call once at
// startup after storage_init() and before CRSF/UI use them.
void gconfig_init();

// Thread-safe read (called from the CRSF task at frame rate).
bool gconfig_flip(int idx);

// Update the runtime cache (takes effect immediately). Persist with gconfig_save().
void gconfig_set_flip(int idx, bool v);

// Persist the current flip cache to NVS. Caller must quiesce ADC/CRSF around this
// (flash write) as with other NVS saves.
bool gconfig_save();
