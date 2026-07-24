// gconfig.cpp — shared gimbal channel config (reverse/flip).
#include "gconfig.h"

#include <Arduino.h>

#include "storage.h"

namespace {
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
bool g_flip[4] = {false, false, false, false};
}  // namespace

void gconfig_init() {
  bool f[4];
  if (storage_load_flip(f)) {
    portENTER_CRITICAL(&g_mux);
    for (int i = 0; i < 4; ++i) g_flip[i] = f[i];
    portEXIT_CRITICAL(&g_mux);
    Serial.printf("[gconfig] flip loaded T=%d Y=%d P=%d R=%d\n", f[0], f[1], f[2], f[3]);
  } else {
    Serial.println("[gconfig] flip: default (none stored)");
  }
}

bool gconfig_flip(int idx) {
  if (idx < 0 || idx > 3) return false;
  portENTER_CRITICAL(&g_mux);
  bool v = g_flip[idx];
  portEXIT_CRITICAL(&g_mux);
  return v;
}

void gconfig_set_flip(int idx, bool v) {
  if (idx < 0 || idx > 3) return;
  portENTER_CRITICAL(&g_mux);
  g_flip[idx] = v;
  portEXIT_CRITICAL(&g_mux);
}

bool gconfig_save() {
  bool f[4];
  portENTER_CRITICAL(&g_mux);
  for (int i = 0; i < 4; ++i) f[i] = g_flip[i];
  portEXIT_CRITICAL(&g_mux);
  return storage_save_flip(f);
}
