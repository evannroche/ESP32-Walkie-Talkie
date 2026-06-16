#pragma once
#include <cstdint>
#include <walkie_core/protocol.h>

namespace config {
constexpr int kSampleRate = 8000;
constexpr uint16_t kMaxSamples = walkie::kMaxSamples;
constexpr int kAudioInPin = 34;  // ESP32 ADC1_CH6
constexpr int kButtonPin = 23;   // Active HIGH, internal pull-down
constexpr int kLedPin = 2;
constexpr int kI2sBclkPin = 26;
constexpr int kI2sLrclkPin = 25;
constexpr int kI2sDataOutPin = 22;
constexpr int kAmpShutdownPin = 27;  // MAX98357A SD; HIGH=enabled
constexpr uint32_t kButtonDebounceMs = 30;
constexpr int kAudioGain = 1;  // Digital gain on DC-centered ADC signal

// Build with -DDEVICE_RIGHT=1 for the RIGHT device (which sends to LEFT).
// The default (LEFT device) sends to the RIGHT device.
#ifdef DEVICE_RIGHT
constexpr uint8_t kPeerMac[6] = {0xF4, 0x65, 0x0B, 0x48, 0xB9, 0x18};
#else
constexpr uint8_t kPeerMac[6] = {0xF4, 0x65, 0x0B, 0x4A, 0x73, 0xCC};
#endif
}
