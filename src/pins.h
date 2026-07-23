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

// ---- Gimbals (ADC1, analogContinuous) ----
// Throttle moved 32 -> 36 (GPIO32 pot channel was dead). GPIO36 = ADC1_CH0, input-only.
// Erratum 3.11 glitch is mitigated by continuous mode keeping the ADC powered.
static constexpr int PIN_GIMBAL_THROTTLE = 36;  // ADC1_CH0 (input-only)
static constexpr int PIN_GIMBAL_YAW = 33;       // ADC1_CH5
static constexpr int PIN_GIMBAL_PITCH = 34;     // ADC1_CH6 (input-only)
static constexpr int PIN_GIMBAL_ROLL = 35;      // ADC1_CH7 (input-only)
