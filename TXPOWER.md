# TXPOWER.md — Setting ELRS TX Power over CRSF

How the handset firmware changes the BetaFPV Nano TX V2's transmit power from
the ESP32, over the existing single-wire half-duplex CRSF link. Same mechanism
the ExpressLRS Lua script uses.

## Mechanism

ELRS exposes its Lua menu as a numbered **parameter tree**. Settings are changed
with `CRSF_FRAMETYPE_PARAMETER_WRITE (0x2D)` — an extended-header frame with
payload `[field_index, value]`. This is NOT the `0x32` command frame (that one
is for actions like enter-WiFi / bind).

Constants (as used by SimpleTX and other DIY ELRS handsets):

| Name | Value | Meaning |
|------|-------|---------|
| `ELRS_ADDRESS` | `0xEE` | TX module (dest) |
| `ADDR_RADIO` | `0xEA` | radio handset (src) |
| `TYPE_SETTINGS_WRITE` | `0x2D` | parameter write frame type |
| `ELRS_PKT_RATE_COMMAND` | `0x01` | field: Packet Rate |
| `ELRS_TLM_RATIO_COMMAND` | `0x02` | field: Telem Ratio |
| `ELRS_SWITCH_MODE_COMMAND` | `0x03` | field: Switch Mode |
| `ELRS_MODEL_MATCH_COMMAND` | `0x04` | field: Model Match |
| `ELRS_POWER_COMMAND` | `0x06` | field: **Max Power** |
| `ELRS_WIFI_COMMAND` | `0xFE` | command: enter WiFi mode |
| `ELRS_BIND_COMMAND` | `0xFF` | command: enter bind mode |

## Frame layout (8 bytes total)

```
[0xEE] [len=6] [0x2D] [0xEE] [0xEA] [0x06] [value] [CRC8]
 dest    len    type   ext    ext    field   power   poly 0xD5
                       dest   src            index   over bytes 2..6
```

CRC: CRC8 poly `0xD5` computed over type..payload (bytes 2–6) — the same
routine already used for RC frames.

## Power index values

ELRS power enum (module snaps to nearest supported level):

| Index | Power |
|-------|-------|
| 0 | 10 mW |
| 1 | 25 mW |
| 2 | 50 mW |
| 3 | 100 mW |
| 4 | 250 mW |
| 5 | 500 mW |
| 6 | 1000 mW |

Nano TX V2 max = 1000 mW.

**2S power warning:** do not select 500 mW / 1 W when the pack is below 7 V —
the module browns out and reboots, dropping the link. Gate high-power
selection on INA226 bus voltage.

## Implementation

```cpp
// Set TX power: field 0x06, value = power index (see table)
void crsf_set_power(uint8_t pwr_idx) {
  uint8_t f[8];
  f[0] = 0xEE;              // dest: TX module
  f[1] = 6;                 // len: bytes after this (type..crc)
  f[2] = 0x2D;              // PARAMETER_WRITE
  f[3] = 0xEE;              // ext dest: TX module
  f[4] = 0xEA;              // ext src: radio handset
  f[5] = 0x06;              // field: Max Power
  f[6] = pwr_idx;           // value
  f[7] = crsf_crc8(&f[2], 5);
  while (CRSF_UART.available()) CRSF_UART.read();
  CRSF_UART.write(f, 8);
  echo_pending += 8;        // half-duplex echo bookkeeping
}
```

## Rules

1. **Pause the RC stream around parameter traffic.** Known ELRS issue:
   parameter responses get corrupted when interleaved with RC channel frames —
   worse on a single shared wire. Hold the channels task for a few frame
   periods after sending, drain the response, then resume. Never change
   parameters while armed.
2. **The setting persists** in module config. Send on user action (settings
   menu), not on every boot.
3. **Field index 0x06 is a version assumption.** It is the Max Power position
   in the ELRS 2.x/3.x parameter tree and is what DIY handsets hardcode. The
   fully robust method is the Lua approach: DEVICE_PING (0x28) → read
   PARAMETER_SETTINGS_ENTRY (0x2B) responses → locate "Max Power" by name.
   For one known module: hardcode 6, verify once.
4. **Verification is built in:** read INA226 current at 10 mW, set 250 mW,
   read again. The current jump is the acknowledgment.

## Related actions (same frame, different field)

- Packet rate: field `0x01` (e.g. 250Hz mode index per module's rate table)
- Telem ratio: field `0x02`
- WiFi mode: field `0xFE`, value 1 — kills RF link immediately; gate behind
  explicit user action. (Module also auto-enters WiFi after ~60 s of no CRSF.)
- Bind mode: field `0xFF`, value 1
