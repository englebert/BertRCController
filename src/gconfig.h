// gconfig.h — shared channel config: gimbal reverse (flip) + toggle->channel map.
//
// Runtime cache read by the CRSF task (output) and the UI (display + editing).
// Loaded from NVS at startup; sensible defaults when nothing is stored.
// Gimbal index: 0=Throttle, 1=Yaw, 2=Pitch, 3=Roll. Toggle index: 0=Toggle1..9=Toggle10.
#pragma once

#include <stdbool.h>
#include <stdint.h>

// Load flip flags + toggle map from NVS into the runtime cache. Call once at startup
// after storage_init() and before CRSF/UI use them.
void gconfig_init();

// --- Gimbal reverse (flip) ---
bool gconfig_flip(int idx);            // thread-safe read (CRSF task, frame rate)
void gconfig_set_flip(int idx, bool v);  // update cache; persist with gconfig_save()

// --- Toggle -> CRSF channel map. Value = channel 5..16, 0 = unassigned ---
uint8_t gconfig_toggle_chan(int toggle);          // read (CRSF task)
void gconfig_set_toggle_chan(int toggle, uint8_t ch);  // update cache

// --- Toggle -> function label. Value = index into the UI function list, 0 = none ---
uint8_t gconfig_toggle_func(int toggle);
void gconfig_set_toggle_func(int toggle, uint8_t f);
// First toggle assigned the given function index, or -1 if none (e.g. find the ARM toggle).
int gconfig_toggle_with_func(uint8_t f);

// Persist the whole config (flip + toggle map) to NVS. Caller must quiesce ADC/CRSF
// around this (flash write) as with other NVS saves.
bool gconfig_save();
