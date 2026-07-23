// ui.cpp — display bring-up + UI task. CRSF toggle-channel monitor (CH5..CH16).
#include "ui.h"

#include <Arduino.h>
#include <TFT_eSPI.h>

#include "crsf.h"
#include "display_cfg.h"
#include "pins.h"

namespace {

TFT_eSPI tft;  // pins come from build_flags (platformio.ini)

// CRSF 11-bit channel range.
constexpr int CRSF_MIN = 172;
constexpr int CRSF_MAX = 1811;

// Show the non-gimbal channels: CH5..CH16 (indices 4..15) = 12 channels, 6 cols x 2 rows.
constexpr int FIRST_IDX = 4;   // CH5
constexpr int NSHOW = 12;      // CH5..CH16
constexpr int COLS = 6;
constexpr int ROWS = 2;
constexpr int CW = LCD_W / COLS;  // 47
constexpr int CH = LCD_H / ROWS;  // 38

void backlightOn() {
  ledcAttach(PIN_TFT_BL, BL_PWM_FREQ, BL_PWM_BITS);  // arduino-esp32 3.x API
  ledcWrite(PIN_TFT_BL, BL_FULL);
}

int cellX(int k) { return (k % COLS) * CW; }
int cellY(int k) { return (k / COLS) * CH; }

// Static labels (channel numbers) drawn once.
void drawGrid() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  char b[6];
  for (int k = 0; k < NSHOW; ++k) {
    snprintf(b, sizeof(b), "CH%d", FIRST_IDX + k + 1);  // CH5..CH16
    tft.drawString(b, cellX(k) + CW / 2, cellY(k) + 6, 1);
  }
}

// Dynamic per-channel: position bar + value.
void drawChannel(int k, int v) {
  int x = cellX(k);
  int y = cellY(k);
  int bx = x + 3, bw = CW - 6;
  int by = y + 14, bh = 8;
  int fw = constrain((int)((long)(v - CRSF_MIN) * bw / (CRSF_MAX - CRSF_MIN)), 0, bw);

  // High = green, low = grey; makes 2-position toggles obvious.
  uint16_t col = (v > (CRSF_MIN + CRSF_MAX) / 2) ? TFT_GREEN : TFT_DARKGREY;
  tft.fillRect(bx, by, fw, bh, col);
  tft.fillRect(bx + fw, by, bw - fw, bh, TFT_NAVY);

  char b[6];
  snprintf(b, sizeof(b), "%4d", v);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(CW - 4);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(b, x + CW / 2, y + 30, 1);
}

void renderChannels() {
  uint16_t ch[16];
  crsf_get_channels(ch);
  for (int k = 0; k < NSHOW; ++k) drawChannel(k, ch[FIRST_IDX + k]);
}

void uiTask(void*) {
  tft.init();
  tft.setRotation(PANEL_ROTATION);
  if (PANEL_INVERT) tft.invertDisplay(true);

  // Clear full GRAM once (no viewport yet), then carve out the visible strip.
  tft.fillScreen(TFT_BLACK);
  tft.setViewport(VP_X, VP_Y, LCD_W, LCD_H);
  backlightOn();
  drawGrid();

  Serial.printf("[ui] display up: %dx%d @ viewport(%d,%d) rot=%d\n", LCD_W, LCD_H, VP_X,
                VP_Y, PANEL_ROTATION);

  const TickType_t period = pdMS_TO_TICKS(50);  // ~20 Hz refresh
  for (;;) {
    renderChannels();
    vTaskDelay(period);
  }
}

}  // namespace

bool ui_start() {
  BaseType_t ok = xTaskCreatePinnedToCore(uiTask, "ui", 4096, nullptr,
                                          /*priority=*/1, nullptr, /*core=*/0);
  return ok == pdPASS;
}
