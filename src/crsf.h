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
};

// Init UART2 + start the CRSF task. Call once from setup().
bool crsf_start();

// Copy the latest link state into `out`. Returns out.valid. Safe from any task.
bool crsf_get(CrsfLink& out);

// Copy the 16 CRSF channel values (172..1811) last sent. Safe from any task.
void crsf_get_channels(uint16_t out[16]);

// Pause/resume the CRSF task (stops UART traffic). Wrap NVS/flash writes with these —
// UART interrupt activity during a flash-cache-disabled window can panic.
void crsf_pause();
void crsf_resume();
