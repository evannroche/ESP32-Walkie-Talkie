#include "audio_playback.h"
#include "config.h"
#include <walkie_core/codec.h>
#include <Arduino.h>
#include <algorithm>

bool AudioPlayback::begin() {
  pinMode(config::kAmpShutdownPin, OUTPUT);
  digitalWrite(config::kAmpShutdownPin, LOW);

  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chanCfg.dma_desc_num = 8;
  chanCfg.dma_frame_num = 64; // ~64 ms total buffered stereo audio at 8 kHz
  if (i2s_new_channel(&chanCfg, &tx_, nullptr) != ESP_OK) return false;
  i2s_std_config_t stdCfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(config::kSampleRate),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                      I2S_SLOT_MODE_STEREO),
      .gpio_cfg = {
          .mclk = I2S_GPIO_UNUSED,
          .bclk = static_cast<gpio_num_t>(config::kI2sBclkPin),
          .ws = static_cast<gpio_num_t>(config::kI2sLrclkPin),
          .dout = static_cast<gpio_num_t>(config::kI2sDataOutPin),
          .din = I2S_GPIO_UNUSED,
          .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
      },
  };
  if (i2s_channel_init_std_mode(tx_, &stdCfg) != ESP_OK) return false;
  return i2s_channel_enable(tx_) == ESP_OK;
}

bool AudioPlayback::play(const uint8_t* samples, uint16_t count) {
  if (!samples || count == 0 || !tx_) return false;
  constexpr size_t kFrames = 256;
  int16_t stereo[kFrames * 2];
  digitalWrite(config::kAmpShutdownPin, HIGH);
  delay(5);
  bool ok = true;
  for (size_t offset = 0; offset < count && ok; offset += kFrames) {
    const size_t frames = std::min(kFrames, static_cast<size_t>(count) - offset);
    for (size_t i = 0; i < frames; ++i) {
      const int16_t s = walkie::u8ToPcm16(samples[offset + i]);
      stereo[2 * i] = s;
      stereo[2 * i + 1] = s;
    }
    const size_t totalBytes = frames * 2 * sizeof(int16_t);
    size_t written = 0;
    if (i2s_channel_write(tx_, stereo, totalBytes, &written, 1000) != ESP_OK ||
        written != totalBytes) ok = false;
  }
  // Send a quiet tail, then let DMA drain before disabling the amplifier.
  for (auto& s : stereo) s = 0;
  size_t written = 0;
  if (i2s_channel_write(tx_, stereo, sizeof(stereo), &written, 1000) != ESP_OK) ok = false;
  delay(80); // Allow the <=64 ms DMA backlog to drain before muting.
  digitalWrite(config::kAmpShutdownPin, LOW);
  return ok;
}
