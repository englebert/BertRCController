# BertRCController

Firmware for a **DIY ExpressLRS (ELRS) handheld RC transmitter** built on an ESP32.
It reads 4 analog gimbals and 10 switches, renders a status UI on a small SPI TFT,
monitors pack voltage/current, and sends **CRSF** to an ExpressLRS 2.4 GHz module over a
single-wire half-duplex UART. It can also configure the ELRS module (power, fan, packet
rate, baud, WiFi) over the link, and double as a **BLE gamepad** for FPV simulators.

> Target module: **BetaFPV Nano TX V2** (ExpressLRS 2.4 GHz, SX1280). Firmware written for
> **ESP32-WROOM-32** + **arduino-esp32 ≥ 3.0** under **PlatformIO**.

---

## Features

- **4 gimbals** on the internal ADC (oneshot polling, calibrated, jitter-filtered).
- **10 toggle switches** via an MCP23017 I²C expander.
- **Pack telemetry** from an INA226 (voltage / current / power, mAh).
- **76×284 ST7789 TFT** UI, flicker-free (off-screen sprite rendering).
- **Hand-rolled CRSF** TX + telemetry over half-duplex UART (AETR, up to F1000).
- **Live link telemetry** (LQ, RSSI, SNR, TX power, RF mode) + relayed **FC battery**.
- **Module configuration over CRSF** — TX power, cooling-fan threshold, packet rate, UART
  baud, and WiFi-update mode, all from the handset.
- **Per-toggle channel + function mapping** (ARM, ANGLE, HORIZON, ACRO, AIR, TURTLE, RTH …).
- **Arm safety** state + **flight-session timer** and lifetime total (in flash).
- **Gimbal calibration wizard** + per-channel reverse (flip).
- **BLE HID gamepad** mode ("BertRC") for FPV simulators (silences CRSF to save power).

---

## Hardware

- **MCU:** ESP32-WROOM-32 (dual-core LX6), FreeRTOS.
- **ELRS module:** BetaFPV Nano TX V2 — needs **7–13 V** from the pack (not 3.3 V).
- **Display:** 76×284 ST7789 bar panel (rotation 1 → 284×76 landscape).
- **Expander:** MCP23017 @ `0x20` (10 switches).
- **Power sensor:** INA226 @ `0x40`, **0.025 Ω** shunt (two 0.05 Ω in parallel) → ~3.28 A FS.

### Pin map

| GPIO | Function | Notes |
|------|----------|-------|
| 16 | CRSF RX (UART2) | direct to module CRSF pad |
| 17 | CRSF TX (UART2) | via **1 kΩ** series to the same pad |
| 18 / 23 / 5 / 19 / 4 | TFT SCK / MOSI / CS / DC / RST | VSPI |
| 27 | TFT backlight | LEDC PWM |
| 21 / 22 | I²C SDA / SCL | MCP23017 + INA226 (single 4.7 kΩ pull-ups) |
| 34 / 36 / 33 / 35 | Gimbals: throttle / yaw / pitch / roll | ADC1 |

*GPIO32 is dead on this board; GPIO39 is a spare. Switches are all on the MCP23017 — not
ESP32 pins.*

---

## Wiring / schematic

> 📎 **Visual version:** [`docs/wiring.html`](docs/wiring.html) — a color-coded (by net)
> connection diagram; open it in a browser. The ASCII/tables below carry the same info.

### Overview

```
              2S LiPo (7.4–8.4 V nominal)
                     │ +           │ −
                     ▼             │
             ┌───────────────┐     │
             │    INA226      │     │      high-side sense on the main + rail
             │  IN+     IN−   │     │      shunt 0.025 Ω (two 0.05 Ω in parallel)
             └───┬───────┬────┘     │
              +  │       │  switched-+ (load side)
                 │       ├───────────────► ELRS TX module  "7–13 V" pad
                 │       │
                 │       └──► 3.3 V regulator ──► ESP32 · TFT · MCP23017 · INA226(Vs)
                 │                                 · gimbal reference rail (clean 3.3 V)
                 │
        (INA226 VBUS senses the bus voltage; IN+ = supply side, IN− = load side)

        GND is common to everything (pack −, ESP32, module, TFT, sensors, pots).
```

The **ELRS module runs directly off the pack (7–13 V)** — *not* 3.3 V. Everything else runs
from a 3.3 V regulator. All grounds are tied together.

### ESP32 ↔ ELRS module (CRSF, half-duplex single wire)

Module bay pads: `NC · 7–13 V · GND · CRSF`. Both ESP32 UART pins meet at the one CRSF pad:

```
  GPIO17 ──[ 1 kΩ ]──┐
                     ├──► module CRSF pad
  GPIO16 ────────────┘
  pack + ───────────────► module 7–13 V
  GND ──────────────────► module GND
```

Non-inverted 3.3 V logic — no inverter, no level shifter. The 1 kΩ series on TX + the shared
wire form a passive half-duplex bus; the driver rejects its own TX echo.

### ESP32 ↔ TFT (ST7789, VSPI)

| ESP32 | TFT | | ESP32 | TFT |
|-------|-----|-|-------|-----|
| GPIO18 | SCK | | GPIO4 | RST |
| GPIO23 | MOSI (may be silk "SDA") | | GPIO27 | BLK / LED |
| GPIO5 | CS | | 3V3 | VCC |
| GPIO19 | DC | | GND | GND |

MISO unused (panel is write-only). 3.3 V native logic — direct connection.

### ESP32 ↔ I²C bus (MCP23017 + INA226)

```
  GPIO21 (SDA) ──┬── MCP23017 SDA ── INA226 SDA
  GPIO22 (SCL) ──┼── MCP23017 SCL ── INA226 SCL
                 └── one 4.7 kΩ pull-up pair to 3.3 V for the whole bus
```

- **MCP23017** @ `0x20`: `A0/A1/A2 → GND` (address), **`RESET → 3.3 V`** (else it stays in
  reset), `VDD → 3.3 V`, `VSS → GND`. INTA/INTB unused (polled).
- **INA226** @ `0x40`: `A0/A1 → GND` (address), `VS → 3.3 V`, `GND → GND`, `IN+ = supply`,
  `IN− = load` (reversing only flips the current sign).
- Remove any per-module pull-ups so only the single 4.7 kΩ pair loads the bus.

### Gimbals (4 pots → ADC1)

Each potentiometer: outer legs across the **clean 3.3 V reference rail** and GND; wiper
through an RC low-pass into the ADC pin:

```
  3.3 V ──[ pot ]── GND
             │ wiper
             ├──[ 1 kΩ ]──► GPIO (34 thr / 36 yaw / 33 pit / 35 roll)
             │
           100 nF
             │
            GND
```

11 dB attenuation (full 0–3.3 V swing). Endpoints/center captured by the calibration wizard.

### Switches (10 toggles → MCP23017)

Each switch wired **pin → GND**, with the MCP23017's internal pull-ups enabled
(`INPUT_PULLUP`), so they read **active-low**:

```
  MCP GPA0–GPA7, GPB0, GPB1  ──[ switch ]── GND
```

Toggle → MCP pin map (A0–A7 = bits 0–7, B0–B7 = bits 8–15):

| Toggle | MCP | Toggle | MCP |
|--------|-----|--------|-----|
| 1 | B3 | 6 | B2 |
| 2 | B5 | 7 | A0 |
| 3 | B4 | 8 | B7 |
| 4 | B0 | 9 | B6 |
| 5 | B1 | 10 | A7 |

6 MCP pins remain free.

### Bill of parts (core)

ESP32-WROOM-32 dev board · BetaFPV Nano TX V2 · 76×284 ST7789 · MCP23017 · INA226 ·
2× 0.05 Ω 1 W 2512 shunt · 4× gimbals · 10× switches · 3.3 V regulator · 1 kΩ (CRSF) ·
4.7 kΩ ×2 (I²C) · per-gimbal 1 kΩ + 100 nF.

---

## Build & Flash

Requires [PlatformIO](https://platformio.org/) (CLI). The `esp32` env uses the pioarduino
`platform-espressif32` fork to get arduino-esp32 3.x.

```bash
pio run                       # build
pio run -t upload             # build + flash over USB
pio device monitor            # serial monitor (115200)
pio run -t upload -t monitor  # flash then monitor
```

Library and TFT_eSPI configuration live in `platformio.ini` (`lib_deps`, `build_flags`) so
the config stays in the repo — do not edit `User_Setup.h`.

---

## Using the handset

### Boot
`BOOTING UP` → sequential module init (INA226, MCP23017, gimbals, CRSF) → calibration (if
forced/needed) → throttle-safety gate (blocks while throttle > 15 %) → RC screen.

### Two live modes — **Toggle6**
Toggle6 is the mode switch (not a channel):
- **OFF → RC screen** (flying view).
- **ON → Configuration menu.**

### Calibration / flip — **Toggle5**
- **Toggle5 ON at power-up** forces the **gimbal calibration wizard** (circle the sticks to
  capture min/max; center is captured at rest; stored in flash). First boot always calibrates.
- Inside **GIMBALS & TRIM**, Toggle5 gates edit mode for per-channel **reverse (flip)**.

### Navigation (gimbal keys)
In menus the sticks act as keys: **pitch** up/down = UP/DOWN, **roll** right/left = OK/CANCEL
(thresholds on the 1000–2000 µs scale, flip-aware).

### Main RC screen
- Small **battery icon with % inside** + pack **voltage**.
- Top-right **banner**: `ARMED` (red) / `DISARMED` (green) / `NO ARM` / `TX OFF`.
- **Flight mode** (center): `ACRO` / `ANGLE` / `HORIZ` (+ `AIR`), or `TURTLE` / `RTH`.
- **Telemetry** line (middle): selected link/battery items — green live, **red when offline**.
- **Flight timer** (session MM:SS).
- **10 toggle-state cells** along the bottom (green = ON).

### Configuration menu
| Item | What it does |
|------|--------------|
| **GIMBALS & TRIM** | View live stick %, reverse channels (Toggle5). |
| **TOGGLE MAPPING** | Map each toggle → CRSF channel **and** a function label. |
| **BATTERY INFO** | Full INA226 V / A / W + lifetime total flight time. |
| **TELEMETRY** | Live CRSF link stats + FC battery; pick which show on the RC screen. |
| **CRSF MODULE** | Module hub (submenu, below). |
| **REBOOT** | Restart the handset. |

### CRSF MODULE submenu
Live status header (`ONLINE 921k F1000` / `OFFLINE` / `BLE JOYSTICK`):

| Item | What it does |
|------|--------------|
| **OUTPUT** | Toggle RC transmission on/off (off = drone fails safe, saves power). |
| **TX POWER** | 25 / 50 / 100 / 250 / 500 / 1000 mW — confirmed live via telemetry. |
| **FAN THRESHOLD** | TX power at/above which the module fan runs (or NEVER). |
| **PACKET RATE** | 50 Hz … F1000 (needs 921600 baud for > 333 Hz). |
| **BAUD RATE** | 400000 / 921600 (module auto-detects). |
| **BLE JOYSTICK** | Start BLE gamepad mode (silences CRSF). |
| **WIFI SETUP** | Boot the module into its WiFi config AP. |

---

## Channel map (CRSF, AETR)

Gimbal ADC `0..4095` → CRSF `172..1811` (calibrated + flip). Toggles are 2-position
(`172`/`1811`).

| CH | Source | CH | Source |
|----|--------|----|--------|
| 1 | Roll | 9 | Toggle5 |
| 2 | Pitch | 10 | Toggle6 |
| 3 | Throttle | 11 | Toggle7 |
| 4 | Yaw | 12 | Toggle8 |
| 5 | Toggle1 | 13 | Toggle9 |
| 6 | Toggle2 | 14 | Toggle10 |
| 7 | Toggle3 | 15 | unused (center) |
| 8 | Toggle4 | 16 | unused (center) |

Each toggle also carries an optional **function label** (ARM, ANGLE, HORIZ, ACRO, AIR,
TURTLE, RTH, BEEP, PREARM). The label is handset-side (drives the arm state, flight-mode
display, etc.); the **channel** is what the flight controller actually reads.

---

## Signal chain notes

- **Gimbal filter:** oversample(8) → median-of-11 → EMA(α=0.12) → hysteresis deadband(35).
  Kills ESP32 ADC jitter with imperceptible lag.
- **CRSF link:** UART2, **921600 baud** default (400000 also supported; module auto-detects),
  250 Hz frames, AETR. Half-duplex echo (frame type 0x16) is discarded.
- **Module config = parameter writes, not discovery.** This module streams sync frames but
  never answers device pings, so reading the parameter tree by name doesn't work here — but a
  blind CRSF **parameter write (0x2D)** does (that's how power / fan / packet-rate are set),
  as long as the RC stream is briefly held and the origin address is `0xEA`. See
  `TXPOWER.md` / `FAN.md` and `CLAUDE.md` for the field IDs and details.
- **Baud ↔ packet-rate coupling:** at 400000 baud the half-duplex module caps at ~333 Hz;
  921600 unlocks 500 Hz / F1000.

## Gotchas

- ELRS module WiFi is triggered **only** by the no-CRSF timeout (power-cycle the module while
  the handset sends nothing) — there is no CRSF WiFi command on this module.
- INA226 full-scale is ~**3.28 A**; readings pin (not clip) near the ceiling.
- On 2S, avoid 500 mW / 1 W when the pack sags below 7 V — the module browns out.
- NVS/flash writes quiesce the CRSF/gimbal/switch tasks first (a busy ISR during a
  flash-cache-disabled window panics the chip).

---

## Repo layout

```
src/
  main.cpp          task creation, core pinning
  pins.h            GPIO map (single source of truth)
  display_cfg.h     panel geometry / viewport offsets
  gimbals.*         ADC polling, median + EMA filter
  switches.*        MCP23017 read + debounce
  power.*           INA226 V/I/mAh
  gconfig.*         shared channel config (flip, toggle map, functions, calibration)
  storage.*         NVS-backed settings
  crsf.*            CRSF TX + telemetry, param writes, arm state, toggle mask
  ble.*             BLE HID gamepad
  ui.cpp            screens, menus, app task
CLAUDE.md           deep implementation notes / decisions
TXPOWER.md FAN.md   CRSF parameter-write references
```
