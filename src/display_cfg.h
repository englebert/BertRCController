// display_cfg.h — panel geometry for the 76x284 ST7789 bar display.
//
// TFT_eSPI has no built-in offset table for a 76x284 panel, so the visible glass
// sits inside the controller's 240x320 GRAM. We keep TFT_WIDTH/HEIGHT at the full
// controller size (see platformio.ini) and carve out the visible strip with a
// viewport, applying the correct offset for the chosen rotation.
//
// CALIBRATION: the two NATIVE (portrait) offsets below place the glass. Tune them
// on hardware using the UI task's calibration frame (1px border + colored corner
// ticks): if a border edge is cut off -> DECREASE that offset; if there's a black
// gap before the border -> INCREASE it. The landscape offsets are derived by swapping
// these (the ST7789 MV bit exchanges the column/row address axes in landscape).
#pragma once

// ---- Native (rotation 0, portrait) panel geometry ----
static constexpr int PANEL_NAT_W = 76;   // visible glass width  (portrait)
static constexpr int PANEL_NAT_H = 284;  // visible glass height (portrait)

// Where the visible glass begins inside the 240x320 GRAM, in PORTRAIT.
// Die is centered: (240-76)/2 = 82, (320-284)/2 = 18.
static constexpr int PANEL_COL_OFFSET = 82;  // X / column start (portrait)
static constexpr int PANEL_ROW_OFFSET = 18;  // Y / row start    (portrait)

// ---- Orientation ----
// TFT_eSPI rotations: 0/2 = portrait (76x284), 1/3 = landscape (284x76).
// 1 = 90 deg clockwise. If it comes out upside-down for your mounting, use 3.
static constexpr int PANEL_ROTATION = 1;

// ---- Derived logical screen + viewport (do not edit; follows from the above) ----
static constexpr bool PANEL_LANDSCAPE = (PANEL_ROTATION & 1) != 0;
// Logical drawing surface after rotation.
static constexpr int LCD_W = PANEL_LANDSCAPE ? PANEL_NAT_H : PANEL_NAT_W;  // 284 : 76
static constexpr int LCD_H = PANEL_LANDSCAPE ? PANEL_NAT_W : PANEL_NAT_H;  // 76 : 284
// Viewport origin into GRAM. In landscape the MV bit swaps the axes, so the
// column/row offsets swap too.
static constexpr int VP_X = PANEL_LANDSCAPE ? PANEL_ROW_OFFSET : PANEL_COL_OFFSET;  // 18 : 82
static constexpr int VP_Y = PANEL_LANDSCAPE ? PANEL_COL_OFFSET : PANEL_ROW_OFFSET;  // 82 : 18

// Set to true if colors look photo-negative (many ST7789 IPS panels need this).
static constexpr bool PANEL_INVERT = true;

// ---- Backlight PWM (LEDC): ~5 kHz, 8-bit, 0..255 ----
static constexpr int BL_PWM_FREQ = 5000;
static constexpr int BL_PWM_BITS = 8;
static constexpr int BL_FULL = 255;
