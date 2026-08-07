#pragma once
#include <cstdint>
#include <esp_adc/adc_oneshot.h>

class AudioCapture {
public:
  bool begin();
  void start();
  // Called frequently from loop; emits at most one sample per call.
  bool poll(uint8_t& outSample);
private:
  adc_oneshot_unit_handle_t adc_ = nullptr;
  uint32_t nextSampleAtUs_ = 0;
  float dcEstimate_ = 0.0f;
  bool dcReady_ = false;
};
