# FAN.md — ELRS Module Fan Control

How the BetaFPV Nano TX V2's cooling fan works and how to configure it from the
ESP32 handset over CRSF. Companion to TXPOWER.md and crsf_param.cpp.

## How the fan works

The module has a built-in 1804 fan + heatsink, managed **entirely by the
module's own firmware**. The fan is not a device on the handset side of the
CRSF link and there is no direct fan-on/fan-off command.

The only knob is **Fan Threshold**: the TX power level at or above which the
fan runs. Typical option list: `Off; 10mW; 25mW; 50mW; 100mW; 250mW; 500mW; …`
Set to 250 mW → fan silent at low power, spins up when power ≥ 250 mW.

The setting persists in module config — set once, not every boot.

## Configuration paths

**Path A — WiFi UI (recommended for a set-once value).**
Power the module without CRSF for ~60 s → it starts its AP
(`ExpressLRS TX`, password `expresslrs`) → set Fan Threshold in the browser.
Zero firmware work.

**Path B — over CRSF (this project: crsf_param.cpp).**
Unlike Max Power (well-known field `0x06`), Fan Threshold has **no stable
hardcoded field index** — its position in the parameter tree shifts with
firmware version and module features. It must be discovered by name.

## CRSF discovery flow (crsf_param.cpp)

```
1. DEVICE_PING (0x28, broadcast)      → DEVICE_INFO (0x29): parameter count
2. for field = 1..count:
     PARAMETER_READ (0x2C) [field, chunk]
       → PARAMETER_SETTINGS_ENTRY (0x2B): [field][chunks_remaining][parent][type][name\0][data…]
     reassemble chunks while chunks_remaining > 0
3. match entry name == "Fan Threshold"  → field index
4. PARAMETER_WRITE (0x2D) [field, option_idx]
5. cache index in NVS (Preferences, namespace "elrs", key = param name)
```

API:
```cpp
uint8_t idx = crsf_param_find("Fan Threshold");  // discovery, cached after first run
crsf_param_write(idx, option_idx);
// or the convenience wrapper:
crsf_set_fan_threshold(option_idx);
```

## Implementation rules

1. **Pause the RC stream** for the whole discovery/write sequence. Parameter
   responses corrupt when interleaved with RC channel frames (known ELRS
   issue), and the single shared wire makes it worse. Natural home: a module
   settings screen that suspends the channels task while open. Never armed.
2. **Sync byte:** module→handset responses arrive addressed `0xEA` (radio),
   not `0xC8`. A parser that resyncs on `0xC8` only will silently drop every
   parameter frame — use the dedicated `recv_frame()` (accepts 0xEA/0xC8/0xEE).
3. **Chunking is mandatory here.** Fan Threshold is a text-selection field
   whose option list rarely fits one CRSF frame; entries arrive chunked via
   `chunks_remaining`. Skipping reassembly makes the name match fail on
   exactly this parameter.
4. **Option indices are firmware-dependent.** 0=Off, then ascending power
   levels is typical, but VERIFY against the entry's own ';'-separated option
   string (or the WiFi UI) before hardcoding a value in the handset menu.
5. **Cache invalidation:** reflashing module firmware can renumber the tree.
   If a cached write stops taking effect after a module update, clear the
   `elrs` NVS namespace to force rediscovery.

## Interactions worth knowing

- The fan draws from the module's 7–13 V supply, adding load at exactly the
  high-power moments — one more reason to gate 500 mW/1 W selection on INA226
  bus voltage (module browns out below 7 V on a sagging 2S).
- Discovery is generic: once it works, every Lua parameter (Packet Rate,
  Telem Ratio, Dynamic Power, …) is settable via `crsf_param_find(name)` —
  Fan Threshold is just the first customer.
- A handset case fan would be a separate, ESP32-driven thing (spare MCP23017
  pin + MOSFET) — not needed at this build's ~2 W dissipation.
