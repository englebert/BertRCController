# CRSF Handset — BetaFPV Nano TX V2 Integration Notes

Working reference for the BERT RC Controller (ESP32 transmitter → ELRS Nano TX V2).

## Hardware

| Item | Value |
|---|---|
| Module | BetaFPV Nano TX V2, ExpressLRS 2.4G (stock FW 3.3.x) |
| Power | JR-bay VBAT pin, 2S pack (do **not** feed 3.3V) |
| Signal | Single CRSF pin, 3.3V logic, non-inverted UART, half-duplex |
| ESP32 wiring | UART2 — TX GPIO17 via **1k series resistor**, RX GPIO16 direct on same wire, common GND |
| Antenna | Always attach before power-on (PA damage otherwise) |

## UART / Protocol

- **400,000 baud, 8N1, non-inverted**
- Frame: `[addr][len][type][payload...][crc8]`
  - `len` = type + payload + crc (payload + 2)
  - CRC8 DVB-S2, poly `0xD5`, computed over **type + payload only**
- Addresses: `0xEE` = to TX module (our RC frames), `0xEA` = to handset (telemetry back; some frames use `0xC8` — accept both)

## RC Channels Frame (we send)

- Type `0x16` RC_CHANNELS_PACKED, every **4 ms (250 Hz)**
- Payload: 16 channels × 11 bits = 22 bytes, LSB-first bit-packed
- Total frame = 26 bytes
- Channel scale: **172–1811** (988–2012 µs), center **992**
  - `crsf = (us − 1500) × 8/5 + 992`
- Ch1–4 = AETR (roll, pitch, throttle, yaw); AUX from MCP23017 switches
- Boot/failsafe: throttle at 172 (min) always

## Telemetry (we receive)

| Type | Frame | Key fields |
|---|---|---|
| `0x14` | Link Statistics | uplink RSSI (−dBm), LQ %, SNR, TX power enum, downlink stats |
| `0x08` | Battery | voltage (0.1 V), current (0.1 A), mAh used, % — **big-endian** |
| `0x3A` | Radio sync | module's desired interval + phase offset (optional, later) |

## Half-Duplex Rule

Every TX byte echoes back on RX. After each send, **discard exactly N echoed bytes** (N = frame length) before parsing telemetry. If telemetry dies, echo counter desync is suspect #1.

## FreeRTOS Plan

- ADC task (existing median+EMA pipeline) → publishes filtered channels to shared struct
- CRSF task: `vTaskDelayUntil` @ 4 ms → pack → send → drain telemetry
- Pin CRSF task to core 1, high priority — the 4 ms cadence is the hard constraint

## Module Ops (from manual)

- **Bind**: 3× quick press left button (Button1), or via Lua `[Bind]`; solid LED = bound. FW 3.3.0 ships with no binding phrase
- **WiFi update**: LED slow green flash → AP `ExpressLRS TX` / pw `expresslrs` → http://10.0.0.1
- LED: rainbow=power-on, green flash=WiFi, blue flash=BT joystick, red fast=RF chip fault, orange flash=no handset connection; solid color = connected, color maps to packet rate
- Don't use 500 mW/1 W below 7 V (2S) — module brownout-reboots → link loss
