#include <walkie_core/protocol.h>
#include <algorithm>
#include <cstring>

namespace walkie {
namespace {
uint16_t get16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}
void put16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
}
uint16_t packetCountFor(uint16_t samples) {
  return static_cast<uint16_t>((samples + kAudioBytesPerPacket - 1) / kAudioBytesPerPacket);
}
void header(uint8_t* out, PacketType type, uint16_t id, uint16_t seq,
            uint16_t count, uint16_t samples) {
  out[0] = kMagic;
  out[1] = kVersion;
  out[2] = static_cast<uint8_t>(type);
  out[3] = 0;
  put16(out + 4, id);
  put16(out + 6, seq);
  put16(out + 8, count);
  put16(out + 10, samples);
}
}

size_t encodeStart(uint16_t id, uint16_t samples, uint8_t* out, size_t cap) {
  if (!out || samples == 0 || samples > kMaxSamples || cap < kHeaderBytes) return 0;
  header(out, PacketType::Start, id, 0, packetCountFor(samples), samples);
  return kHeaderBytes;
}

size_t encodeAudio(uint16_t id, uint16_t samples, uint16_t seq,
                   const uint8_t* audio, uint8_t* out, size_t cap) {
  if (!out || !audio || samples == 0 || samples > kMaxSamples) return 0;
  const uint16_t count = packetCountFor(samples);
  if (seq >= count) return 0;
  const size_t offset = static_cast<size_t>(seq) * kAudioBytesPerPacket;
  const size_t len = std::min(kAudioBytesPerPacket, static_cast<size_t>(samples) - offset);
  if (cap < kHeaderBytes + len) return 0;
  header(out, PacketType::Audio, id, seq, count, samples);
  std::memcpy(out + kHeaderBytes, audio + offset, len);
  return kHeaderBytes + len;
}

bool decodePacket(const uint8_t* bytes, size_t len, PacketView& out) {
  if (!bytes || len < kHeaderBytes || len > kMaxEspNowPayload ||
      bytes[0] != kMagic || bytes[1] != kVersion || bytes[3] != 0) return false;
  const auto type = static_cast<PacketType>(bytes[2]);
  const uint16_t samples = get16(bytes + 10);
  const uint16_t count = get16(bytes + 8);
  const uint16_t seq = get16(bytes + 6);
  if (samples == 0 || samples > kMaxSamples || count != packetCountFor(samples)) return false;
  if (type == PacketType::Start) {
    if (len != kHeaderBytes || seq != 0) return false;
  } else if (type == PacketType::Audio) {
    if (seq >= count) return false;
    const size_t offset = static_cast<size_t>(seq) * kAudioBytesPerPacket;
    const size_t expected = std::min(kAudioBytesPerPacket, static_cast<size_t>(samples) - offset);
    if (len != kHeaderBytes + expected) return false;
  } else return false;
  out = {type, get16(bytes + 4), seq, count, samples,
         bytes + kHeaderBytes, len - kHeaderBytes};
  return true;
}

void Reassembler::configure(uint8_t* buffer, size_t capacity) {
  data_ = buffer;
  capacity_ = capacity;
  reset();
}

void Reassembler::reset() {
  received_.fill(0);
  active_ = false;
  sawLastPacket_ = false;
  messageId_ = sampleCount_ = packetCount_ = receivedPackets_ = 0;
  lastPacketMs_ = 0;
}

bool Reassembler::hasPacket(uint16_t seq) const {
  return (received_[seq >> 3] & (1u << (seq & 7))) != 0;
}
void Reassembler::markPacket(uint16_t seq) {
  received_[seq >> 3] |= static_cast<uint8_t>(1u << (seq & 7));
}

bool Reassembler::accept(const uint8_t* packet, size_t len, uint32_t nowMs) {
  PacketView view{};
  if (!data_ || !decodePacket(packet, len, view) || view.sampleCount > capacity_) return false;
  if (!active_) {
    active_ = true;
    messageId_ = view.messageId;
    sampleCount_ = view.sampleCount;
    packetCount_ = view.packetCount;
    receivedPackets_ = 0;
    sawLastPacket_ = false;
    received_.fill(0);
    // Unsupplied chunks are always silence, including an absent last packet.
    std::memset(data_, kSilence, sampleCount_);
  }
  if (view.messageId != messageId_ || view.sampleCount != sampleCount_ ||
      view.packetCount != packetCount_) return false;
  lastPacketMs_ = nowMs;
  if (view.type == PacketType::Audio) {
    if (!hasPacket(view.sequence)) {
      const size_t offset = static_cast<size_t>(view.sequence) * kAudioBytesPerPacket;
      std::memcpy(data_ + offset, view.payload, view.payloadBytes);
      markPacket(view.sequence);
      ++receivedPackets_;
    }
    if (view.sequence == packetCount_ - 1) sawLastPacket_ = true;
  }
  return true;
}

bool Reassembler::isComplete() const {
  return active_ && packetCount_ > 0 && receivedPackets_ == packetCount_;
}

bool Reassembler::shouldFlush(uint32_t nowMs, uint32_t tailWaitMs, uint32_t idleWaitMs) const {
  if (!active_) return false;
  // An orphan Start packet must not monopolize a receive buffer forever.
  if (receivedPackets_ == 0) return nowMs - lastPacketMs_ >= idleWaitMs;
  if (isComplete()) return true;
  const uint32_t elapsed = nowMs - lastPacketMs_; // unsigned wrap-safe for short intervals
  return elapsed >= (sawLastPacket_ ? tailWaitMs : idleWaitMs);
}
}
