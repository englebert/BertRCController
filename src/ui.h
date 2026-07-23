// ui.h — display / UI task.
//
// Per the RTOS architecture in CLAUDE.md this task runs on core 0 at low priority
// and redraws at ~10-30 Hz. For this first "hello world" module it just brings up
// the ST7789, sets the backlight, and draws a calibration frame + greeting.
#pragma once

// Create and start the UI task (pinned to core 0, low priority).
// Call once from setup(). Returns true if the task was created.
bool ui_start();
