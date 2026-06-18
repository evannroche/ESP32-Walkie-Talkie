#pragma once
#include <cstdint>

namespace walkie {
// Uniform 12-bit ADC raw value to unsigned 8-bit; quantization is lossy.
uint8_t adc12ToU8(uint16_t raw);
// Quantize a DC-centered signed 12-bit sample into unsigned 8-bit silence=128.
uint8_t dcCentered12ToU8(int16_t centeredRaw, uint8_t gain = 1);
// 128 represents silence; bipolar signed 16-bit PCM for I2S.
int16_t u8ToPcm16(uint8_t sample);
// Inverse of u8ToPcm16 for representable 8-bit source samples.
uint8_t pcm16ToU8(int16_t sample);
}
