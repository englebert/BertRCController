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
};

// Map the CRSF TX-power index to milliwatts (0 for unknown index).
uint16_t crsf_tx_power_mw(uint8_t idx);

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

// Pause/resume the CRSF task (stops UART traffic). Wrap NVS/flash writes with these —
// UART interrupt activity during a flash-cache-disabled window can panic.
void crsf_pause();
void crsf_resume();
