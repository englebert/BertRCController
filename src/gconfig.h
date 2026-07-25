// gconfig.h — shared channel config: gimbal reverse (flip) + toggle->channel map.
//
// Runtime cache read by the CRSF task (output) and the UI (display + editing).
// Loaded from NVS at startup; sensible defaults when nothing is stored.
// Gimbal index: 0=Throttle, 1=Yaw, 2=Pitch, 3=Roll. Toggle index: 0=Toggle1..9=Toggle10.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "storage.h"  // GimbalCal

// Load flip flags + toggle map + calibration from NVS into the runtime cache. Call once
// at startup after storage_init() and before CRSF/UI use them.
void gconfig_init();

// --- Gimbal reverse (flip) ---
bool gconfig_flip(int idx);            // thread-safe read (CRSF task, frame rate)
void gconfig_set_flip(int idx, bool v);  // update cache; persist with gconfig_save()

// --- Gimbal calibration (raw ADC -> CRSF mapping) ---
// Update the cached calibration (call after the calibration wizard). The wizard persists
// the cal to NVS itself; this just refreshes the copy the CRSF task maps against.
void gconfig_set_cal(const GimbalCal& cal);
// Map a filtered raw ADC value (0..4095) for gimbal idx (0=T,1=Y,2=P,3=R) to a CRSF value
// (172..1811) using the calibrated endpoints (min/center/max) + the channel reverse flag.
// Falls back to a linear map when no valid calibration is loaded. Matches the UI's calPct.
uint16_t gconfig_map_crsf(int idx, int raw);

// --- Toggle -> CRSF channel map. Value = channel 5..16, 0 = unassigned ---
uint8_t gconfig_toggle_chan(int toggle);          // read (CRSF task)
void gconfig_set_toggle_chan(int toggle, uint8_t ch);  // update cache

// Toggle function-label indices. MUST match the UI's FUNC_NAMES[] order. 0 = none.
// ARM is special: it drives the throttle-hold arm-safety in crsf.cpp.
enum {
  GFUNC_NONE = 0,
  GFUNC_ARM = 1,
};

// --- Toggle -> function label. Value = index into the UI function list, 0 = none ---
uint8_t gconfig_toggle_func(int toggle);
void gconfig_set_toggle_func(int toggle, uint8_t f);
// First toggle assigned the given function index, or -1 if none (e.g. find the ARM toggle).
int gconfig_toggle_with_func(uint8_t f);

// Persist the whole config (flip + toggle map) to NVS. Caller must quiesce ADC/CRSF
// around this (flash write) as with other NVS saves.
bool gconfig_save();
