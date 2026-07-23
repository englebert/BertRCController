// gimbals.cpp — analogContinuous ADC sampling + filtering (module #4).
#include "gimbals.h"

#include <Arduino.h>

#include "pins.h"

namespace {

// ADC1 pins in scan order: throttle, yaw, pitch, roll.
const uint8_t ADC_PINS[] = {PIN_GIMBAL_THROTTLE, PIN_GIMBAL_YAW, PIN_GIMBAL_PITCH,
                            PIN_GIMBAL_ROLL};
constexpr int NCH = 4;

// analogContinuous() hardware-averages this many conversions per pin per batch
// (the "oversample" stage). 20 kHz is the ESP32 continuous-ADC minimum.
constexpr uint32_t CONVERSIONS_PER_PIN = 8;
constexpr uint32_t SAMPLE_FREQ_HZ = 20000;  // 20000/(4*8) ~= 625 batches/s

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

int pinToIndex(uint8_t pin) {
  for (int i = 0; i < NCH; ++i)
    if (ADC_PINS[i] == pin) return i;
  return -1;
}

// ADC conversion-done ISR: notify the gimbal task ONLY. Nothing blocking here.
void ARDUINO_ISR_ATTR adcCallback() {
  BaseType_t hpw = pdFALSE;
  vTaskNotifyGiveFromISR(g_task, &hpw);
  if (hpw) portYIELD_FROM_ISR();
}

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
  adc_continuous_data_t* result = nullptr;
  for (;;) {
    // Block until the ADC ISR notifies a full DMA batch is ready.
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0) {
      continue;  // no data within 1 s; keep last published values
    }
    if (!analogContinuousRead(&result, 0) || result == nullptr) continue;

    int16_t out[NCH];
    bool all = true;
    for (int i = 0; i < NCH; ++i) {
      int idx = pinToIndex(result[i].pin);
      if (idx < 0) continue;
      int16_t med = g_median[idx].push((int16_t)result[i].avg_read_raw);
      if (!g_have_ema[idx]) {
        g_ema[idx] = med;
        g_have_ema[idx] = true;
      } else {
        g_ema[idx] += EMA_ALPHA * (med - g_ema[idx]);
      }
    }
    for (int i = 0; i < NCH; ++i) {
      out[i] = (int16_t)(g_ema[i] + 0.5f);
      if (!g_have_ema[i]) all = false;
    }
    publish(out, all);
  }
}

}  // namespace

bool gimbals_start() {
  analogContinuousSetAtten(ADC_11db);  // full 0-3.3 V stick swing
  if (!analogContinuous(ADC_PINS, NCH, CONVERSIONS_PER_PIN, SAMPLE_FREQ_HZ, adcCallback)) {
    Serial.println("[gimbals] analogContinuous() setup FAILED");
    return false;
  }

  BaseType_t ok = xTaskCreatePinnedToCore(gimbalsTask, "gimbal", 4096, nullptr,
                                          /*priority=*/10, &g_task, /*core=*/1);
  if (ok != pdPASS) {
    Serial.println("[gimbals] task create FAILED");
    return false;
  }

  analogContinuousStart();
  Serial.println("[gimbals] analogContinuous started (4ch, 11dB, DMA)");
  return true;
}

bool gimbals_get(GimbalState& out) {
  portENTER_CRITICAL(&g_mux);
  out = g_state;
  portEXIT_CRITICAL(&g_mux);
  return out.valid;
}

void gimbals_pause() { analogContinuousStop(); }

void gimbals_resume() { analogContinuousStart(); }
