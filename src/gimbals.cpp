// gimbals.cpp — 4-channel gimbal sampling via oneshot analogRead polling (module #4).
//
// NOTE: analogContinuous() (DMA scan) was dropped — on this board it misbehaves with
// more than 3 channels (individual channels read stuck/ghosted). We poll all four pins
// with plain analogRead() at ~1 kHz instead, then apply the same median-of-11 + EMA
// filter. Simpler, and no DMA ISR to collide with flash-cache-disabled windows.
#include "gimbals.h"

#include <Arduino.h>

#include "pins.h"

namespace {

// ADC1 pins: throttle, yaw, pitch, roll.
const uint8_t ADC_PINS[] = {PIN_GIMBAL_THROTTLE, PIN_GIMBAL_YAW, PIN_GIMBAL_PITCH,
                            PIN_GIMBAL_ROLL};
constexpr int NCH = 4;

constexpr float EMA_ALPHA = 0.2f;  // light smoothing after the median

// --- Median-of-11: ring buffer kept in TIME order; sort a COPY, never the ring.
struct Median11 {
  int16_t ring[11] = {0};
  int widx = 0;   // next write position (time order preserved)
  int count = 0;  // valid samples so far

  int16_t push(int16_t v) {
    ring[widx] = v;
    widx = (widx + 1) % 11;
    if (count < 11) count++;

    int16_t tmp[11];
    for (int i = 0; i < count; ++i) tmp[i] = ring[i];  // copy, then sort the copy
    for (int i = 1; i < count; ++i) {                  // insertion sort (n<=11)
      int16_t key = tmp[i];
      int j = i - 1;
      while (j >= 0 && tmp[j] > key) {
        tmp[j + 1] = tmp[j];
        --j;
      }
      tmp[j + 1] = key;
    }
    return tmp[count / 2];
  }
};

Median11 g_median[NCH];
float g_ema[NCH] = {0, 0, 0, 0};
bool g_have_ema[NCH] = {false, false, false, false};

TaskHandle_t g_task = nullptr;

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
GimbalState g_state = {0, 0, 0, 0, false};

void publish(const int16_t v[NCH], bool valid) {
  portENTER_CRITICAL(&g_mux);
  g_state.throttle = v[0];
  g_state.yaw = v[1];
  g_state.pitch = v[2];
  g_state.roll = v[3];
  g_state.valid = valid;
  portEXIT_CRITICAL(&g_mux);
}

void gimbalsTask(void*) {
  analogReadResolution(12);  // 0..4095
  for (int i = 0; i < NCH; ++i) {
    analogSetPinAttenuation(ADC_PINS[i], ADC_11db);  // full 0-3.3 V stick swing
  }

  const TickType_t period = pdMS_TO_TICKS(1);  // ~1 kHz (4 oneshot reads per loop)
  for (;;) {
    int16_t out[NCH];
    bool all = true;
    for (int i = 0; i < NCH; ++i) {
      int16_t raw = (int16_t)analogRead(ADC_PINS[i]);
      int16_t med = g_median[i].push(raw);
      if (!g_have_ema[i]) {
        g_ema[i] = med;
        g_have_ema[i] = true;
      } else {
        g_ema[i] += EMA_ALPHA * (med - g_ema[i]);
      }
      out[i] = (int16_t)(g_ema[i] + 0.5f);
      if (!g_have_ema[i]) all = false;
    }
    publish(out, all);
    vTaskDelay(period);
  }
}

}  // namespace

bool gimbals_start() {
  BaseType_t ok = xTaskCreatePinnedToCore(gimbalsTask, "gimbal", 4096, nullptr,
                                          /*priority=*/10, &g_task, /*core=*/1);
  if (ok != pdPASS) {
    Serial.println("[gimbals] task create FAILED");
    return false;
  }
  Serial.println("[gimbals] oneshot polling started (4ch, 11dB, ~1 kHz)");
  return true;
}

bool gimbals_get(GimbalState& out) {
  portENTER_CRITICAL(&g_mux);
  out = g_state;
  portEXIT_CRITICAL(&g_mux);
  return out.valid;
}

// Suspend/resume the polling task. Wrap NVS/flash writes with these so no task is
// executing flash-cached ADC code while the cache is disabled on the other core.
void gimbals_pause() {
  if (g_task) vTaskSuspend(g_task);
}

void gimbals_resume() {
  if (g_task) vTaskResume(g_task);
}
