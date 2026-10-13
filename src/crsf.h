// crsf.h — CRSF link to the ExpressLRS module over half-duplex UART2 (module #5).
//
// Sends 16-channel RC frames (AETR) to the BetaFPV Nano TX at 250 Hz and parses
// telemetry coming back on the same wire. Echo of our own TX (frame type 0x16) is
// discarded; real telemetry (link stats, etc.) is kept. Runs on core 0, medium priority.
#pragma once

#include <stdbool.h>
#include <stdint.h>

struct CrsfLink {
  bool valid;          // at least one valid telemetry frame received from the module
  uint32_t tx_frames;  // RC frames we have sent
  uint32_t rx_frames;  // valid non-echo frames received from the module
  uint8_t up_lq;       // uplink link quality (0..100), from link-stats frame
  int16_t up_rssi;     // uplink RSSI in dBm (negative)
  uint8_t rf_mode;     // reported RF mode index
  int8_t up_snr;       // uplink SNR (dB)
  uint8_t dn_lq;       // downlink link quality (0..100)
  int16_t dn_rssi;     // downlink RSSI in dBm (negative)
  int8_t dn_snr;       // downlink SNR (dB)
  uint8_t tx_power;    // uplink TX power index (map with crsf_tx_power_mw)
  uint8_t ant;         // active antenna
  // Downlink FC battery telemetry (CRSF battery-sensor frame 0x08), if the FC sends it.
  bool batt_valid;     // a battery-sensor frame has been received
  uint16_t batt_dv;    // pack voltage in 0.1 V units
  uint16_t batt_da;    // pack current in 0.1 A units
  uint32_t batt_mah;   // used capacity (mAh)
  uint8_t batt_pct;    // battery remaining (%)
};

// Map the CRSF TX-power index to milliwatts (0 for unknown index).
uint16_t crsf_tx_power_mw(uint8_t idx);

// Set the module's Max Power via a CRSF Lua parameter write (no discovery needed).
// level_index = index into the module's filtered power table (0 = its min power, 25 mW on
// the Nano TX V2). Queued; the CRSF task holds RC and sends it on a quiet wire. Confirm via
// crsf_get().tx_power.
void crsf_set_power(int level_index);

// --- ELRS Lua parameter discovery/write (for settings without a fixed field ID, e.g. the
// cooling-fan threshold). These BLOCK for up to ~2 s and suspend the CRSF task internally
// (RC held) so the request/response isn't corrupted on the shared wire. Call from the UI
// task while not armed.
struct CrsfParam {
  int field;          // discovered field id (1..count), -1 if not found / no reply
  uint8_t value;      // current option index
  uint8_t max;        // highest option index (option count = max + 1)
  char options[96];   // ';'-separated option labels, from the module
  uint8_t dbg_count;  // params reported by DEVICE_INFO (0 = no ping reply)
  uint8_t dbg_read;   // entries successfully read during the walk
};
// Walk the parameter tree and return the first field whose name contains `name_substr`.
bool crsf_param_scan(const char* name_substr, CrsfParam& out);
// Write an option index to a parameter field. Returns false if the field is invalid.
bool crsf_param_write(int field, uint8_t value);
// Queue a fire-and-forget parameter write via the CRSF task's proven RC-hold path (same as
// crsf_set_power). value = raw option index. Use for settings like the fan threshold.
void crsf_param_set(int field, int value);

// CRSF UART baud to the module. crsf_set_baud() before crsf_start() seeds the initial baud;
// after start it changes the UART live (module re-detects via autobaud in ~1-2 s). Use a
// value the module knows: 400000 or 921600. (Not persisted here — save via storage.)
uint32_t crsf_get_baud();
void crsf_set_baud(uint32_t baud);

// Enable/disable RC transmission. When off the CRSF task sends nothing (the drone fails
// safe); the task keeps running so telemetry RX and re-enable still work. Not persisted.
void crsf_set_output(bool on);
bool crsf_output_enabled();

// Bitmask of the 10 toggles' physical state (bit i = Toggle(i+1) ON). For the RC screen.
uint16_t crsf_toggle_mask();

// True while link-stats telemetry is fresh (a link-stats frame within the last ~1 s);
// false when telemetry has never arrived or has gone stale (link/module offline).
bool crsf_telem_online();

// Init UART2 + start the CRSF task. Call once from setup().
bool crsf_start();

// Copy the latest link state into `out`. Returns out.valid. Safe from any task.
bool crsf_get(CrsfLink& out);

// Copy the 16 CRSF channel values (172..1811) last sent. Safe from any task.
void crsf_get_channels(uint16_t out[16]);

// Arm-safety state for the UI: -1 = no ARM toggle mapped (throttle passes through),
// 0 = disarmed (throttle held at idle), 1 = armed. Safe from any task.
int crsf_arm_state();

// ---- CRSF wire self-test (debug mode): is the 1k series resistor fitted? ----
// Wiring under test: GPIO17 (TX) --1k--> CRSF pad <-- GPIO16 (RX, direct). Neither pin has
// an ADC, so this is three digital probes with the UART temporarily released:
//   idle     - TX hi-Z: does the line follow RX's internal pulls, or is something holding it?
//   loopback - TX driven HIGH then LOW against an opposing RX pull: RX must follow, which
//              proves a path from TX to the pad (missing resistor / broken trace = OPEN).
//   contend  - both pins driven to opposite levels at the weakest drive strength: with a
//              series resistor each pin keeps its own level; with a 0-ohm bridge they fight
//              to a common voltage and one of them reads wrong (= SHORT).
// OK therefore means "a resistance of very roughly 300 ohm .. 10 k" - consistent with the
// 1k part, not a measurement of it.
enum class CrsfWire : uint8_t {
  Ok,       // path present and not a short: resistor fitted
  Open,     // RX does not follow TX: resistor missing / open circuit
  Short,    // pins fight: resistor bridged (0 ohm) or far too low
  HeldLow,  // line stuck low even with pull-up: pad shorted to GND or module unpowered
  Busy,     // readings changed between passes (module transmitting / noise)
};
struct CrsfWireTest {
  CrsfWire verdict;
  bool idle_pu, idle_pd;  // idle: level read with RX pull-up / pull-down (TX hi-Z)
  bool loop_hi, loop_lo;  // loopback: RX level with TX HIGH / TX LOW (want 1 / 0)
  bool hold_a, hold_b;    // contend: both pins kept their level, RX=H/TX=L and RX=L/TX=H
};
// Run the test (~15 ms). Stops the CRSF UART for the duration and restores it afterwards;
// call from the UI task only, with RC output already off (it glitches the wire).
CrsfWireTest crsf_wire_test();

// Pause/resume the CRSF task (stops UART traffic). Wrap NVS/flash writes with these —
// UART interrupt activity during a flash-cache-disabled window can panic.
void crsf_pause();
void crsf_resume();
