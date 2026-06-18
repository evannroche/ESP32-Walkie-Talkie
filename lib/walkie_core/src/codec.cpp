#include <walkie_core/codec.h>

namespace walkie {
uint8_t adc12ToU8(uint16_t raw) {
  if (raw > 4095) raw = 4095;
  return static_cast<uint8_t>(raw >> 4);
}

uint8_t dcCentered12ToU8(int16_t centeredRaw, uint8_t gain) {
  int32_t value = 128 + (static_cast<int32_t>(centeredRaw) * gain) / 16;
  if (value < 0) value = 0;
  if (value > 255) value = 255;
  return static_cast<uint8_t>(value);
}

int16_t u8ToPcm16(uint8_t sample) {
  return static_cast<int16_t>((static_cast<int32_t>(sample) - 128) * 256);
}

uint8_t pcm16ToU8(int16_t sample) {
  return static_cast<uint8_t>((static_cast<int32_t>(sample) + 32768) / 256);
}
}
