#include "audio_capture.h"
#include "config.h"
#include <walkie_core/codec.h>
#include <Arduino.h>

bool AudioCapture::begin() {
  adc_oneshot_unit_init_cfg_t unitCfg{};
  unitCfg.unit_id = ADC_UNIT_1;
  if (adc_oneshot_new_unit(&unitCfg, &adc_) != ESP_OK) return false;
  adc_oneshot_chan_cfg_t channelCfg{};
  channelCfg.bitwidth = ADC_BITWIDTH_12;
  channelCfg.atten = ADC_ATTEN_DB_12;
  return adc_oneshot_config_channel(adc_, ADC_CHANNEL_6, &channelCfg) == ESP_OK;
}

void AudioCapture::start() {
  dcReady_ = false;
  nextSampleAtUs_ = micros();
}

bool AudioCapture::poll(uint8_t& outSample) {
  constexpr uint32_t intervalUs = 1000000UL / config::kSampleRate;
  const uint32_t now = micros();
  if (static_cast<int32_t>(now - nextSampleAtUs_) < 0) return false;
  nextSampleAtUs_ += intervalUs; // Does not accumulate ADC read/loop overhead.
  // Prevent a late loop iteration from creating an artificial catch-up burst.
  if (static_cast<int32_t>(now - nextSampleAtUs_) > 4 * static_cast<int32_t>(intervalUs))
    nextSampleAtUs_ = now + intervalUs;

  int raw = 0;
  if (adc_oneshot_read(adc_, ADC_CHANNEL_6, &raw) != ESP_OK) return false;
  if (!dcReady_) {
    dcEstimate_ = static_cast<float>(raw);
    dcReady_ = true;
  }
  dcEstimate_ += 0.001f * (static_cast<float>(raw) - dcEstimate_);
  const int centered = raw - static_cast<int>(dcEstimate_);
  outSample = walkie::dcCentered12ToU8(static_cast<int16_t>(centered),
                                       config::kAudioGain);
  return true;
}
