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

// --- Toggle -> MCP23017 input pin (assigned in the TOGGLE MAP menu) ---
// Value = MCP bit 0..15 (A0-A7 = 0-7, B0-B7 = 8-15), GPIN_NONE = unassigned (reads OFF).
static constexpr uint8_t GPIN_NONE = 0xFF;
uint8_t gconfig_toggle_pin(int toggle);
void gconfig_set_toggle_pin(int toggle, uint8_t pin);  // update cache
// Per-toggle invert: when set, the toggle reads ON while its pin is open and OFF while
// it is pulled to GND. An unassigned toggle is always OFF, inverted or not.
bool gconfig_toggle_inv(int toggle);
void gconfig_set_toggle_inv(int toggle, bool inv);  // update cache
// Is this toggle ON in a switches bitmask (SwitchState.bits)? Every reader of a toggle's
// state goes through this, so the pin map + invert flag are the single source of truth.
bool gconfig_toggle_on(int toggle, uint16_t bits);
// Toggle6 is the menu-mode switch in the standard build: it has a pin but no channel or
// function. POCKETVERSION uses the BOOT button instead, so there it is a normal toggle.
bool gconfig_toggle_reserved(int toggle);

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

// Persist the whole config (flip + toggle pins/invert/channels/functions) to NVS. Caller must quiesce ADC/CRSF
// around this (flash write) as with other NVS saves.
bool gconfig_save();
