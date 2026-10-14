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
pio run -e pocket -t upload  # POCKETVERSION variant (portrait display) — see below
```

- **Two envs:** `esp32` (default, landscape 284x76) and `pocket` (`-DPOCKETVERSION`, same
  panel mounted portrait 76x284). The define only flips `PANEL_ROTATION` in
  `src/display_cfg.h`; `ui.cpp` branches each screen's geometry on `PORTRAIT`
  (`!PANEL_LANDSCAPE`) and uses `lp(landscape, portrait)` for labels that need a short
  portrait form. Landscape numbers are hardware-tuned — don't change them when editing the
  portrait side, and build **both** envs (`pio run -e esp32 -e pocket`) after any UI change.
- **Pocket battery = 1S** (max 4.2 V): battery % maps 3.0–4.2 V (`VBAT_CELLS` in `ui.cpp`);
  the standard build stays 2S (6.0–8.4 V).
- **Pocket menu mode = BOOT button, not Toggle6.** Each press of BOOT (GPIO0, `PIN_BOOT_BTN`)
  flips RC screen ↔ configuration mode (latched in a GPIO ISR, 250 ms debounce; always
  boots to the RC screen). `menuModeOn()` in `ui.cpp` is the single mode source for both
  builds. In MODULE WIFI SETUP a BOOT press exits. Toggle6 is still reserved/unmapped in
  `gconfig.cpp` and does nothing in the pocket build.

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
- `Serial2`, **921600 baud** default (configurable; module auto-detects), 8N1. Non-inverted
  3.3V logic — no inverter, no level shifter. 921600 unlocks 500Hz/F1000 (400000 caps ~333Hz).
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
- **Packet rate:** 0x2D write to field **1**, value = list index (no shift). SX1280 options:
  0=50Hz,1=100Full,2=150,3=250,4=333Full,5=500,6=D250,7=D500,8=F500,9=F1000. Verify via
  telemetry **rf_Mode** (LinkStatistics byte 6 = enum_rate: {2,4,5,7,8,9,10,11,12,13} per index).
- **Switch mode / channel count:** ELRS only carries **CH1–12** over the air in its normal
  switch modes (Wide/Hybrid), so toggles mapped to CH13–16 (default Toggle9/10) never reach
  the receiver. All 16 need a FULL packet rate (100Hz FULL / 333Hz FULL) **and** Switch Mode
  "16ch Rate/2". Value = option index: normal rates {0 Wide, 1 Hybrid}, FULL rates {0 8ch,
  1 16ch Rate/2, 2 12ch Mixed}. The module ignores the change while a receiver is connected.
- **SWITCH MODE / TELEM RATIO / MODEL MATCH** (CRSF MODULE menu, one shared `Page::ModSel`,
  `SEL_PAGES` in `ui.cpp`): writes go through `crsf_set_setting(ElrsSetting, idx)` to the
  field found by name ("Switch Mode", "Telem Ratio", "Model Match"). These three are **read
  back** from the module (full multi-chunk PARAM_ENTRY, `modReadStep()`) after the boot
  restore, ~600 ms after each write, and on entering the page: the list shows the module's
  own option labels (switch-mode options follow the packet rate) and the green dot is its
  actual value (`crsf_get_setting`); log `[crsf] module <name> = <v> (<options>)`.
  Telem Ratio (3.5): Std;Off;1:128;1:64;1:32;1:16;1:8;1:4;1:2;Race. **Model Match never set =
  Off** and is sent at boot (with Off the RX accepts any model ID). Stored in NVS like the
  others (`elrstlm`, `elrsmm`); restore order rate → switch → tlm → model match → power → fan.
  ELRS 3.5.6 ids here: rate 1, tlm 2, switch 3, mm 5, power 7, fan 9, bind 31.
- **Bind:** CRSF MODULE → BIND RECEIVER runs `crsf_bind()`, a non-blocking state machine
  (`bindStep()` in the CRSF task). Frame: CRSF command (type **0x32**, origin 0xEA,
  `0x10 0x01` = CRSF/BIND, inner CRC poly **0xBA**, then the frame CRC). ELRS 3.4 facts:
  the bind command to the **module** (dest 0xEE) only makes it broadcast its UID for
  ~0.3 s (6 bind packets, then it exits) — the RX must already be listening in bind mode.
  Sent to the **receiver** (dest 0xEC), the module relays it over the air and a linked RX
  drops into bind mode (ELRS `lua.cpp`; EdgeTX does the same when telemetry is live). So:
  link up → RX bind every 300 ms until the link drops (≤5 s) → module bind ×(up to 4, 2.5 s
  apart) until link stats return. Row shows .. / RX.. / TX.. / OK / FAIL; log `[crsf] bind: ...`.
  With no link, put the RX in bind mode by hand (3× power-cycle) first.
  **Module bind on ELRS < 3.4:** the CRSF bind command only exists from 3.4.0 — this
  handset's module reports **ELRS 3.0.0** and silently ignores it. Every version has a
  "Bind" COMMAND in its Lua menu, clicked by a 0x2D write of 1 (lcsClick); its field id is
  build-dependent, so the first bind per boot reads fields 1..N (PARAM_READ 0x2C, chunk 0,
  interleaved with RC) and matches the PARAM_ENTRY (0x2B) named "Bind" (type 0x0D). Falls
  back to the CRSF command if not found. The RX-first step needs ELRS >= 3.4 on the RX too.
- **Module identity at boot:** a device ping goes out right after the model ID; the
  DEVICE_INFO reply is logged as `[crsf] module: '<name>' ELRS x.y.z, N params`. Pings and
  param reads DO get answered now that frames are synced to the module (the old "discovery
  never replies" note predates sync). The receiver may answer the broadcast ping too
  (logged as `receiver: ...`; this one is 'SB Nano 2G4 RX' ELRS 3.5.6).
- **Field ids are looked up by name, not hard-coded.** They shift between ELRS versions
  (3.5 inserts "Link Mode" after Switch Mode: Max Power 6→7, Fan 8→9, and 8 becomes
  Dynamic Power). At boot, after the ping and before the settings restore, the task reads
  chunk 0 of every field (`fieldScanStart/Step`, ~1.5 s) and matches "Packet Rate",
  "Switch Mode", "Max Power", "Fan Thresh", "Bind". `queueParamWrite()` translates the
  `CRSF_FIELD_*` defaults (1/3/6/8, used before the scan finishes) to the found ids and
  **skips** a setting the module doesn't have. Rescanned when the module comes back after a
  reset (it may have been reflashed). Logged as `[crsf] module fields: rate=.. bind=..`
  (ELRS 3.0.0 here: 1/3/6/8, bind 30). Option *indices* are still assumed unchanged.
  Replies get lost occasionally, so the scan makes up to 3 passes over fields that
  didn't answer (a single pass once missed Max Power and skipped the TX power restore).
- **RX SETTINGS (CRSF MODULE menu):** reads the receiver's own Lua menu over the air
  (`crsf_rx_read()`): PARAM_READ to dest **0xEC**, which the module relays as MSP; replies
  come back as telemetry, chunked, **addressed to the module (0xEE) with origin 0xEC** —
  not to 0xEA, so filter on origin. ~3 s for 12 fields; 2 s timeout + retries per chunk;
  field count from the RX's ping reply. Read-only list (folders/commands hidden), OK
  re-reads; also logged as `[crsf] rx <n> <name> = <value>`. This RX ('SB Nano 2G4 RX',
  ELRS 3.5.6): Protocol CRSF, Bind Storage **Persistent**, Model Id Off.
- **Module settings are re-sent at every boot.** PACKET RATE, SWITCH MODE, TX POWER and FAN
  choices are stored in NVS (`storage_get/set_elrs`, keys `elrspkt`/`elrsswm`/`elrspwr`/
  `fanthr`) and the CRSF task writes them back to the module once it first answers (or
  after 3 s), one at a time: rate → switch mode → power → fan (`restoreStep()` in
  `crsf.cpp`). Never-set settings are skipped. The menus' cyan marker = the stored value.
- **Model ID at boot:** first thing in `restoreStep()`, before the settings, the handset
  sends the CRSF model-select command (0x32, `0x10 0x05 <id>`, same inner-CRC framing as
  bind) with `MODEL_ID = 0`, as EdgeTX does. ELRS (3.4 `tx_main.cpp`) holds RF in
  `awaitingModelId` until a model ID arrives or a timeout passes, and keeps rate/power/
  switch-mode **per model ID** — 0 is the slot the module used when none was sent, so the
  stored settings stay where they were. Model Match on the RX compares against this ID.
- **Module reset -> re-send:** if the module goes silent > 500 ms and then talks again
  (its own power blip/reboot — it has a separate supply), the model ID and stored settings
  are sent again, as EdgeTX does (`MODULE_ALIVE_TIMEOUT`); otherwise it would come back on
  model 0 with RF held. Logged as `[crsf] module came back ...`.
- **Frame pacing follows the module (EdgeTX mixer sync):** the module streams 0x3A
  OpenTX-sync frames `[0xEA][origin][0x10][interval u32][offset s32]` (0.1 us units). The
  CRSF task is driven by a us `esp_timer` (not the 1 ms tick) with period = interval + lag,
  lag consumed once (EdgeTX `getAdjustedRefreshRate`), so each RF packet gets fresh
  channels. Fixed 250 Hz fallback when no sync within 2 s; param-write/command slots keep
  the fixed 4 ms spacing. Unsynced at 333Hz+ the module flagged every frame late and spammed
  ~80 sync frames/s; synced it's back to ~5-9/s with lag within +-250 us.
- **Baud (CRSF wire):** ELRS 3.4.x has **no CRSF baud-set command** (speed-proposal frames are
  dropped). Change our own `Serial2` baud (400000/921600) → the module's autobaud re-locks in
  ~1-2 s (it re-detects on any bad-packet/disconnect, not just power-on). Persisted in NVS,
  seeded before `crsf_start()`. **Coupling:** at 400000 on this half-duplex single-wire module
  the module **caps packet rate at ~333 Hz** (`adjustPacketRateForBaud`) — 500Hz/F1000 need
  **921600**. The PACKET RATE green dot (actual rf_Mode) and the CRSF page RATE line show it.
- **BLE gamepad (FPV sim):** `ble.cpp` — the handset presents as a BLE HID gamepad "BertRC"
  (4 calibrated gimbal axes + 10 toggle buttons, ~50 Hz) via ESP32-BLE-Gamepad + NimBLE 2.x
  (builds on arduino-esp32 3.1.3). Enabled from CRSF MODULE → BLE JOYSTICK; ON silences CRSF
  output to save power. NimBLE starts on first enable and stays resident (runtime teardown is
  crash-prone) — OFF just stops reporting. BT on ADC1 has no conflict (only ADC2 clashes WiFi).
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
| CRSF / ELRS | 0 | medium | TX at the module's packet rate (0x3A sync), 250 Hz fallback |
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
- CRSF: hand-rolled TX+telemetry, AETR, 921600 baud (default; configurable). Half-duplex echo rejected by
  ignoring inbound type-0x16 frames. All 5 peripheral modules bring-up complete + verified.

## Channel Map (CRSF, AETR)
Built in `crsf.cpp` `buildChannels()`. Gimbal ADC 0..4095 → CRSF 172..1811.
Toggles are 2-position (172/1811). The toggle→MCP-pin map is **dynamic**: `g_tpin[]` in
`gconfig.cpp` (NVS key `tpin`), edited in the TOGGLE MAP menu. The pins in the table below
are only the defaults (original handset wiring) used when nothing is stored. Every reader
of a toggle (CRSF channels, arm state, BLE buttons, UI special toggles, MCP debug labels)
goes through `gconfig_toggle_on()` / `gconfig_toggle_pin()` — never hard-code an MCP bit.

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

**TOGGLE MAP menu** — each toggle row edits PIN → INV → CH → FUNC, saved on the last OK
(CXL reverts all). PIN: UP/DOWN steps A0..B7/`--`, or just flip the physical switch and its
pin is captured. INV: per-toggle invert (`g_tinv[]`, NVS key `tinv`) — the toggle reads ON
while its pin is open; applied inside `gconfig_toggle_on()` so every reader sees it
(landscape shows NOR/INV, portrait writes an inverted pin as `!B3`). Standard build:
Toggle6 (menu switch) has only pin + invert, and a new setting is rejected
unless it reads ON at save time (prevents locking yourself out of the menu). Pocket build:
Toggle6 is an ordinary toggle (BOOT button does the menu).

Special toggles (read directly by the app/UI task, in addition to their channels; they
follow the pin map above, the pins named here are the defaults):
- **Toggle5 (B1)** ON at power-up → force the gimbal calibration wizard (circle sticks,
  min/max captured, center = (min+max)/2, stored in NVS).
- **Toggle3 + Toggle4 + Toggle5 (B4, B0, B1)** all ON at power-up → debug mode
  (`debugLoop()` in `ui.cpp`, both builds). Pitch stick DOWN/UP = next/previous tool; BOOT
  button (GPIO0) reboots out (flip a combo toggle OFF first). CRSF output is silenced.
  Takes precedence over the Toggle5 calibration trigger. Tools:
  1. **MCP23017 inputs** — all 16 pins live, green = triggered, white outline = seen
     changing; raw word logged as `[mcpdbg] 0x....`.
  2. **CRSF 1k resistor** — `crsf_wire_test()` in `crsf.cpp` releases UART2 and probes
     GPIO16/17 digitally (neither has an ADC): loopback (RX must follow TX → else 1K
     MISSING) and weak-drive contention (pins must hold opposite levels → else 1K SHORTED).
     "OK" means roughly 300 Ω–10 kΩ, not a measurement of 1k. Logged as `[crsfdbg] ...`.
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
4. ~~CRSF baud~~ — **RESOLVED**: 921600 default (was 400000; raised to allow F1000), set in
   BAUD RATE menu + NVS, module auto-detects.
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
