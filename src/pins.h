// pins.h — single source of truth for GPIO assignments.
// Mirrors the pin map in CLAUDE.md. If you change a pin, change it here.
#pragma once

// ---- ELRS / CRSF (UART2, half-duplex single wire) ----
static constexpr int PIN_CRSF_RX = 16;  // UART2 RX  <- CRSF pad
static constexpr int PIN_CRSF_TX = 17;  // UART2 TX  -> 1k series -> CRSF pad

// ---- Display (VSPI, ST7789) ----
static constexpr int PIN_TFT_SCK = 18;
static constexpr int PIN_TFT_MOSI = 23;  // silkscreen may say "SDA"
static constexpr int PIN_TFT_CS = 5;     // strapping pin; idles HIGH -> safe
static constexpr int PIN_TFT_DC = 19;
static constexpr int PIN_TFT_RST = 4;
static constexpr int PIN_TFT_BL = 27;    // LEDC PWM backlight (ADC2 pin, output-only = safe)

// ---- I2C0 (Wire): MCP23017 switches + INA226 power ----
static constexpr int PIN_I2C_SDA = 21;
static constexpr int PIN_I2C_SCL = 22;

// ---- Gimbals (ADC1, oneshot analogRead polling) ----
// Final arrangement, all four verified tracking on hardware (2026-07). GPIO32 is a
// dead ADC input on this board — do not use. GPIO39 is the remaining spare.
// GPIO36 erratum 3.11 glitch is mitigated by continuous mode keeping the ADC powered.
static constexpr int PIN_GIMBAL_THROTTLE = 34;  // ADC1_CH6 (input-only)
static constexpr int PIN_GIMBAL_YAW = 36;       // ADC1_CH0 (input-only)
static constexpr int PIN_GIMBAL_PITCH = 33;     // ADC1_CH5
static constexpr int PIN_GIMBAL_ROLL = 35;      // ADC1_CH7 (input-only)

// ---- BOOT button (POCKETVERSION: menu-mode toggle; both builds: exit MCP debug) ----
// GPIO0 is a strapping pin: fine as a button input once running (devkit has an external
// pull-up, pressed = LOW), but holding it during reset enters the ROM download mode.
static constexpr int PIN_BOOT_BTN = 0;
