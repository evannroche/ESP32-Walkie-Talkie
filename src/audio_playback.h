#pragma once
#include <cstdint>
#include <driver/i2s_std.h>

class AudioPlayback {
public:
  bool begin();
  // Synchronous, clock-paced block writes; the caller owns samples until done.
  bool play(const uint8_t* samples, uint16_t count);
private:
  i2s_chan_handle_t tx_ = nullptr;
};
