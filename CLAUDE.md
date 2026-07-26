# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

# DIY ELRS RC Controller Firmware

ESP32 handheld RC transmitter. Reads 4 analog gimbals and 10 switches, renders a
status UI on an SPI TFT, monitors pack voltage/current, and sends CRSF to an
ExpressLRS module over a single-wire half-duplex UART.

> **Repo status:** greenfield. Only this `CLAUDE.md` exists so far — no `platformio.ini`,
> no `src/`. It is a design spec, not yet implemented. When scaffolding, follow
> [Suggested Layout](#suggested-layout) and treat [Decided](#decided) as fixed and
> [Open](#open--ask-before-assuming) as blocking questions to resolve with the user.

## Build / Commands

PlatformIO (CLI); assumes an env named `esp32` in `platformio.ini` (add one when scaffolding).

```bash
pio run                      # build
pio run -t upload            # build + flash over USB
pio device monitor           # serial monitor (set monitor_speed in platformio.ini)
pio run -t upload -t monitor # flash then monitor
pio run -t clean             # clean build artifacts
pio pkg update               # resolve/update platform + libs
```

- **Verify the arduino-esp32 core is ≥ 3.0.0 after the first `pio run`** (see BUILD GOTCHA
  below) — 2.x builds will fail on `analogContinuous()`/`ledcAttach()`.
- No test suite yet. If adding host-side unit tests (e.g. filter logic), use PlatformIO's
  `pio test -e native` with a `native` env; run one with `pio test -e native -f <test_name>`.
- TFT_eSPI and other board config lives in `platformio.ini` `build_flags` (see VSPI section),
  **not** in edited library headers, so config stays in the repo.

## Target
- MCU: **ESP32-WROOM-32** (classic, dual-core LX6)
- Framework: **arduino-esp32 ≥ 3.0.0** — REQUIRED for `analogContinuous()` and `ledcAttach()`
- Build: **PlatformIO**, Arduino C++
- RTOS: FreeRTOS (bundled)

> BUILD GOTCHA: PlatformIO's official `espressif32` platform historically shipped
> arduino-esp32 2.x, which lacks `analogContinuous()`. Verify the resolved core
> version at build time. If it is 2.x, either switch to a platform package that
> provides core 3.x (e.g. the pioarduino `platform-espressif32` fork) or drop to
> the ESP-IDF `adc_continuous` driver directly.

## Pin Map

| GPIO | Function | Peripheral | ADC ch | Notes |
|------|----------|-----------|--------|-------|
| 16 | ELRS CRSF RX | UART2 RX | — | direct to module CRSF pad |
| 17 | ELRS CRSF TX | UART2 TX | — | via **1k series** to same CRSF pad |
| 18 | Display SCK | VSPI | — | |
| 23 | Display MOSI | VSPI | — | module silkscreen may say "SDA" |
| 5 | Display CS | VSPI | — | strapping pin; idles HIGH → safe |
| 19 | Display DC | GPIO out | — | |
| 4 | Display RST | GPIO out | — | may instead tie to EN (Open #5) |
| 27 | Display backlight | LEDC PWM | — | ADC2 pin, but output-only use = safe |
| 21 | I2C SDA | I2C0 (Wire) | — | shared: MCP23017 + INA226 |
| 22 | I2C SCL | I2C0 (Wire) | — | |
| 34 | Gimbal — throttle | ADC1 | CH6 | input-only, no internal pullup |
| 36 | Gimbal — yaw | ADC1 | CH0 | input-only |
| 33 | Gimbal — pitch | ADC1 | CH5 | |
| 35 | Gimbal — roll | ADC1 | CH7 | input-only, no internal pullup |
| 32 | **DEAD** | ADC1 | CH4 | ADC input stuck ~540 regardless of drive — do not use on this board |
| 39 | *unused* | ADC1 | CH3 | input-only spare; read ADC notes before reusing |

**15 GPIO used.** Switches are NOT on ESP32 pins — all on the MCP23017.

**Do not use:** ADC2 for analog reads (WiFi conflict) · strapping pins 0/2/12/15 ·
UART0 pins 1/3 · display MISO (panel is write-only).

## Peripherals

### UART2 — ELRS / CRSF (half-duplex, single wire)
- Module: **BetaFPV Nano TX V2** (ExpressLRS 2.4 GHz). Bay pads: `NC · 7–13V · GND · CRSF`
- One bidirectional CRSF pin. Wiring: `GPIO17 →1k→ CRSF`, `GPIO16 → CRSF` (passive half-duplex)
- `Serial2`, **400000 baud**, 8N1. Non-inverted 3.3V logic — no inverter, no level shifter
- Higher rates possible (up to 921600) for F1000 packet rates; both ends must match
- **Echo:** RX sees its own TX on the shared wire. The CRSF driver MUST discard echoed
  frames (or flush RX after each TX). This is the most likely failure mode in this design.
- Telemetry (LQ, RSSI, link stats) returns on the same wire → feed to the UI task
- Channels: 11-bit packed CRSF, 16 available, **14 used** (4 gimbals + 10 switches)
- **Module config over CRSF — writes YES, discovery NO (timing, not a gate).** A blind
  **parameter WRITE (0x2D) works** — fire-and-forget, no reply needed. But **discovery
  (ping 0x28→info 0x29, read 0x2C→entry 0x2B) never gets a reply** here. Reason (from ELRS
  source, not a permission gate): the module only clocks queued telemetry out in the return
  window *right after it receives a frame* (`handleOutput()` runs only after `ProcessPacket()`).
  Pausing RC to "listen" opens no window, so the reply is generated but never sent. So to
  read params you'd have to keep RC flowing and read replies interleaved (not yet done); the
  `crsf_param_scan`/ping-probe scaffolding in `crsf.cpp` is kept for that future fix.
  **Write musts** (`TXPOWER.md`): origin **0xEA** (not 0xEF); **hold RC ~24 ms** and send on a
  quiet wire; field IDs are firmware-version dependent. `crsf_param_set(field,value)` is the
  generic queued write; `crsf_set_power()` routes through it.
- **TX power:** 0x2D write to field **6** ("Max Power"), value = index into the module's
  *filtered* table (0 = its min power). Nano TX V2 min = **25 mW** (no 10 mW), max 1000 mW.
  TX POWER menu; confirmed live from the telemetry TX-power byte. Persists in module config.
- **Fan:** 0x2D write to field **8** ("Fan Thresh") = the TX power at/above which the fan
  runs; value = raw option index (NO min-shift, unlike power): 0=10mW … 7=2000mW, 8=NEVER.
  FAN menu (blind write, no telemetry read-back → last value cached in NVS for the green dot).
  Field IDs assume **ELRS 3.4.x** (Max Power=6 ⇒ Fan=8); 3.5.x+ adds "Link Mode" shifting to
  Max Power=7, Fan=9 — re-derive from the working Max Power field if the module is updated.
- **Module WiFi mode:** no command path (that route needs the discovery the module refuses).
  Triggered ONLY by the no-CRSF timeout — power-cycle the module while the handset sends zero
  CRSF, and it auto-starts WiFi after its "Auto On WiFi" interval (~60 s; must be enabled,
  not -1, in the ExpressLRS Configurator/Web UI). `MODULE WIFI SETUP` mode just skips
  `crsf_start()` so GPIO17 stays idle; AP = `ExpressLRS TX`, pw `expresslrs`, http://10.0.0.1.

### I2C0 — `Wire`, SDA=21, SCL=22, 400 kHz
Single 4.7k pull-up pair for the whole bus (remove per-module pull-ups).

| Device | Addr | Role |
|--------|------|------|
| MCP23017 | `0x20` | 10 switches (A0/A1/A2 → GND) |
| INA226 | `0x40` | pack voltage + current + power (A0/A1 → GND) |

**MCP23017** — 10 switches on GPA0–GPA7 + GPB0–GPB1, 6 pins free.
Each switch wired pin→GND; enable internal pull-ups (GPPU) via `INPUT_PULLUP`;
read **active-low**. RESET must be held HIGH or the chip stays in reset.
INT A/B available if interrupt-driven reads are preferred over polling.
Lib: `Adafruit_MCP23X17`.

**INA226** — high-side sense on the main power path.
- Shunt **0.025 Ω** (two 0.05 Ω in **parallel**, for double the range) → full-scale
  **~3.28 A**, current LSB **~0.1 mA**. Configured in `power.cpp`: `SHUNT_OHMS=0.025`,
  `MAX_CURRENT_A=3.274` (library caps shunt×maxCurrent at 81.9 mV, so 3.274 A ≈ 81.85 mV).
- **~3.28 A is the ceiling.** Handset TX bursts sit well under it now; readings pin rather
  than clip gracefully. `near_full` flags within ~10% of full-scale.
- Set the Calibration register from `R_shunt = 0.025` and `current_LSB ≈ 100 µA`;
  set averaging + conversion time in CONFIG.
- Bus voltage read directly (0–36 V range). There is **no** ADC voltage divider.
- Lib: `INA226` (Rob Tillaart) or direct register access.

### VSPI — ST7789 TFT
- Lib: **TFT_eSPI**. Under PlatformIO, configure via `build_flags`
  (`-DUSER_SETUP_LOADED`, `-DST7789_DRIVER`, `-DTFT_MOSI=23`, …) rather than editing
  `User_Setup.h`, so the config lives in the repo.
- SCK=18, MOSI=23, CS=5, DC=19, RST=4. MISO unused.
- SPI clock 40 MHz default.
- Backlight: GPIO27 on LEDC, ~5 kHz, 8-bit — `ledcAttach(27, 5000, 8)`. Full on boot.
- 3.3V native logic, direct connection.

## ADC (gimbals)
- Driver: **oneshot `analogRead()` polling** at ~1 kHz. `analogContinuous()` (DMA scan)
  was tried first but **misbehaves with >3 channels on this board** — individual channels
  read stuck/ghosted (roll, then pitch). Oneshot polling reads each pin cleanly; the
  filter and CRSF rates are unaffected. Do not reintroduce continuous mode here.
- Pins **34, 36, 33, 35** (throttle, yaw, pitch, roll). Task loops all four per tick.
- Attenuation **11 dB** (full 0–3.3V pot swing). Run `esp_adc_cal` to linearize.
- Startup calibration: capture per-stick min/max endpoints (standard RC practice) —
  this also trims the ADC's dead zones near the rails.
- HW already on the board: 1k series + 100nF→GND per wiper; pots fed from the clean
  3.3V reference rail.

**GPIO36/39 are unused here.** If ever repurposed for analog:
- Erratum 3.11 — powering on SARADC1/SARADC2/Hall/AMP pulls GPIO36 and GPIO39 low for
  ~80 ns. Mitigate with `adc_power_acquire()` (+~1 mA); continuous mode keeps the ADC
  powered anyway.
- Never call `hall_sensor_read()` — the Hall sensor occupies ADC1 CH0 and CH3.
- Both pins carry internal 270 pF caps to GPIO37/38.

## Filtering (per gimbal channel)
Pipeline: **oversample(8) → median-of-11 → EMA(α=0.12) → hysteresis deadband(35)**
Tuned on hardware to kill ESP32 ADC jitter. The **deadband** is the key at-rest fix:
the output holds until it moves >35 counts (~0.85%), so a still stick shows zero jitter
with no lag on real movement. Oversample/median/EMA reduce the jitter amplitude; the
deadband freezes what's left. All four knobs are constants at the top of `gimbals.cpp`.

- Median: sliding ring buffer kept in *time* order. Copy to a scratch array, sort the
  **copy**, take index `[5]`. Never sort the ring buffer in place — that destroys the
  write position. Easiest bug to introduce in this codebase.
- Median before EMA: spikes are rejected before they can pollute the smoothed value.
- Group delay ≈ 5 samples → ~5 ms at 1 kHz. Keep the sample loop ≥1 kHz so filter
  latency stays imperceptible.
- Median rejects up to 5 corrupt samples of 11. Sustained bursts beyond that are a
  wiring problem — fix at the source, don't filter harder.

## RTOS Architecture

| Task | Core | Priority | Behavior |
|------|------|----------|----------|
| Gimbal sampling | 1 | highest | polls 4x analogRead() per ~1ms tick, filters, publishes |
| CRSF / ELRS | 0 | medium | TX 250–500 Hz, independent of sample rate |
| Display / UI | 0 | low | redraws at ~10–30 Hz |

- Gimbal task: poll `analogRead()` for each of the 4 pins per tick → median+EMA filter
  → publish to shared struct. (No ADC ISR — continuous/DMA mode was dropped, see ADC.)
- Shared gimbal struct (4× `int16_t`): guard writes and reads with `portENTER_CRITICAL`
  or a mutex so CRSF never transmits a half-updated channel set.
- Rates are decoupled: CRSF reads the latest filtered values whenever it builds a frame.
- WiFi/BT assumed off. ADC1 has no WiFi conflict regardless.

## Suggested Layout
```
src/
  main.cpp          task creation, core pinning
  pins.h            single source of truth for GPIO — mirror the pin map above
  gimbals.cpp/.h    analogContinuous setup, ISR notify, median + EMA
  switches.cpp/.h   MCP23017 init, debounce, bitmask
  power.cpp/.h      INA226 init, V/I/mAh accumulation
  crsf.cpp/.h       half-duplex UART, echo rejection, channel packing, telemetry parse
  ui.cpp/.h         TFT_eSPI screens
```

## Decided
- Gimbals on internal ADC1 (ADS1115 dropped) — final: throttle **34**, yaw **36**,
  pitch **33**, roll **35** (all verified tracking). GPIO32 is dead; GPIO39 spare.
- Battery sensing via INA226 only — no resistor divider, GPIO36 free
- Switches on MCP23017 over I2C — no ESP32 GPIO, no external pull-ups needed
- INA226 shunt: 0.025 Ω (2× 0.05 Ω in parallel) → ~3.28 A full-scale
- CRSF series resistor: 1k
- Backlight on GPIO27 with PWM dimming
- Display: **76x284 ST7789** bar panel. Confirmed on hardware. Rotation 1 (90° CW) →
  logical 284x76 landscape. No CGRAM_OFFSET; visible glass carved from the 240x320 GRAM
  via a viewport. Portrait offset (col 82, row 18); landscape swaps to (18, 82). Colors
  need `invertDisplay(true)`. Geometry lives in `src/display_cfg.h`.
- Platform: pioarduino `platform-espressif32` 53.03.13 resolves arduino-esp32 **3.1.3** (≥3.0 ✓)
- CRSF: hand-rolled TX+telemetry, AETR, 400000 baud. Half-duplex echo rejected by
  ignoring inbound type-0x16 frames. All 5 peripheral modules bring-up complete + verified.

## Channel Map (CRSF, AETR)
Built in `crsf.cpp` `buildChannels()`. Gimbal ADC 0..4095 → CRSF 172..1811.
Toggles are 2-position (172/1811); the toggle→MCP-pin map is `TOGGLE_BIT[]`.

| CH | Source | MCP pin (bit) | CH | Source | MCP pin (bit) |
|----|--------|---------------|----|--------|---------------|
| 1 | Roll (gimbal) | — | 9 | Toggle5 | B1 (9) |
| 2 | Pitch (gimbal) | — | 10 | Toggle6 | B2 (10) |
| 3 | Throttle (gimbal) | — | 11 | Toggle7 | A0 (0) |
| 4 | Yaw (gimbal) | — | 12 | Toggle8 | B7 (15) |
| 5 | Toggle1 | B3 (11) | 13 | Toggle9 | B6 (14) |
| 6 | Toggle2 | B5 (13) | 14 | Toggle10 | A7 (7) |
| 7 | Toggle3 | B4 (12) | 15 | unused (992) | — |
| 8 | Toggle4 | B0 (8) | 16 | unused (992) | — |

MCP bit numbering: A0–A7 = bits 0–7, B0–B7 = bits 8–15.

Special toggles (read directly by the app/UI task, in addition to their channels):
- **Toggle5 (B1)** ON at power-up → force the gimbal calibration wizard (circle sticks,
  min/max captured, center = (min+max)/2, stored in NVS).
- **Toggle6 (B2)** live mode switch: ON = configuration mode (pitch = UP/DOWN keys,
  roll = OK/CANCEL, thresholds 1800/1200 on the 1000–2000 µs scale), OFF = RC screen.

NVS/flash writes MUST be wrapped with gimbals_pause()/crsf_pause() (see safeSaveCal in
ui.cpp) — a busy ADC-DMA/UART ISR during a flash-cache-disabled window panics the chip
("Cache disabled but cached memory region accessed").

## Open — ask before assuming
1. ~~Display resolution + x/y offset~~ — **RESOLVED**: 76x284, rot 1, offsets in `display_cfg.h`.
2. ~~CRSF channel order + mapping~~ — **RESOLVED**: AETR (CH1=Roll, CH2=Pitch, CH3=Throttle,
   CH4=Yaw; CH5-14 = switches). In `crsf.cpp`.
3. ~~CRSF library~~ — **RESOLVED**: hand-rolled (`crsf.cpp`). Echo rejection = ignore
   incoming type-0x16 frames (our own TX on the shared wire). Verified: module returns
   0x3A OpenTX-sync frames, crcerr=0.
4. ~~CRSF baud~~ — **RESOLVED**: 400000.
5. Display RST: keep GPIO4, or tie to EN and free the pin
6. SPI clock: 40 vs 80 MHz
7. Oversample depth: 8× vs 16×
8. Switch semantics: which are 2-position vs 3-position, and their channel values
9. WiFi/BT enabled? (affects core pinning and telemetry path)

## Hardware notes (context, not code)
- ELRS module needs **7–13 V from the pack**, not 3.3V. On 2S, avoid 500 mW/1 W when
  the pack sags below 7 V — the module reboots and the link drops.
- Shunt: **two** 0.05 Ω 2512 (1 W, ≤50 ppm/°C) in parallel = 0.025 Ω, ~2 W handling.
- INA226 IN+ = supply side, IN− = load side. Reversed only flips the current sign.
