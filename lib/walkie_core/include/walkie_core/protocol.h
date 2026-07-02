#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace walkie {

constexpr uint8_t kMagic = 0x57;
constexpr uint8_t kVersion = 1;
constexpr size_t kMaxEspNowPayload = 250;  // Conservative ESP-NOW v1 payload size
constexpr size_t kHeaderBytes = 12;
constexpr size_t kAudioBytesPerPacket = kMaxEspNowPayload - kHeaderBytes;
constexpr uint16_t kMaxSamples = 8000 * 3;
constexpr size_t kMaxPackets = (kMaxSamples + kAudioBytesPerPacket - 1) / kAudioBytesPerPacket;
constexpr uint8_t kSilence = 128;

enum class PacketType : uint8_t { Start = 1, Audio = 2 };

struct PacketView {
  PacketType type;
  uint16_t messageId;
  uint16_t sequence;
  uint16_t packetCount;
  uint16_t sampleCount;
  const uint8_t* payload;
  size_t payloadBytes;
};

// Fixed 12-byte little-endian header on EVERY packet:
// [magic, version, type, reserved, msg_id:u16, seq:u16, count:u16, samples:u16].
// Start has sequence=0 and no payload. Audio seq runs 0..count-1.
size_t encodeStart(uint16_t messageId, uint16_t sampleCount, uint8_t* out, size_t cap);
size_t encodeAudio(uint16_t messageId, uint16_t sampleCount, uint16_t sequence,
                   const uint8_t* audio, uint8_t* out, size_t cap);
bool decodePacket(const uint8_t* bytes, size_t len, PacketView& out);

class Reassembler {
public:
  Reassembler() = default;
  void configure(uint8_t* buffer, size_t capacity);
  void reset();
  // Accepts START or AUDIO (even if START was lost); duplicates are harmless.
  bool accept(const uint8_t* packet, size_t len, uint32_t nowMs);
  bool isComplete() const;
  // Wait for missing packets briefly after the last packet, or idle timeout.
  bool shouldFlush(uint32_t nowMs, uint32_t tailWaitMs = 150,
                   uint32_t idleWaitMs = 350) const;
  bool active() const { return active_; }
  uint16_t messageId() const { return messageId_; }
  uint16_t sampleCount() const { return sampleCount_; }
  uint16_t receivedPackets() const { return receivedPackets_; }
  uint16_t missingPackets() const { return packetCount_ - receivedPackets_; }
  const uint8_t* data() const { return data_; }

private:
  bool hasPacket(uint16_t seq) const;
  void markPacket(uint16_t seq);
  uint8_t* data_ = nullptr;
  size_t capacity_ = 0;
  std::array<uint8_t, (kMaxPackets + 7) / 8> received_{};
  uint16_t messageId_ = 0;
  uint16_t sampleCount_ = 0;
  uint16_t packetCount_ = 0;
  uint16_t receivedPackets_ = 0;
  uint32_t lastPacketMs_ = 0;
  bool active_ = false;
  bool sawLastPacket_ = false;
};

}
