#include <unity.h>
#include <walkie_core/codec.h>
#include <walkie_core/protocol.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

using namespace walkie;

void setUp() {}
void tearDown() {}

void test_header_and_corrupt_packets() {
  uint8_t packet[kMaxEspNowPayload]{};
  PacketView v{};
  auto n = encodeStart(0x1234, 242, packet, sizeof(packet));
  TEST_ASSERT_EQUAL_UINT32(kHeaderBytes, n);
  TEST_ASSERT_TRUE(decodePacket(packet, n, v));
  TEST_ASSERT_EQUAL_UINT8(static_cast<int>(PacketType::Start), static_cast<int>(v.type));
  TEST_ASSERT_EQUAL_HEX16(0x1234, v.messageId);
  TEST_ASSERT_EQUAL_UINT16(242, v.sampleCount);
  TEST_ASSERT_EQUAL_UINT16(2, v.packetCount);
  packet[1] = 0xFF; // Invalid version
  TEST_ASSERT_FALSE(decodePacket(packet, n, v));
  packet[1] = kVersion;
  TEST_ASSERT_FALSE(decodePacket(packet, 4, v));
  TEST_ASSERT_FALSE(decodePacket(packet, n - 1, v));
}

void test_short_four_byte_final_chunk() {
  // 238 + 4 => final packet has FOUR audio bytes, not a new metadata header.
  constexpr uint16_t count = static_cast<uint16_t>(kAudioBytesPerPacket + 4);
  std::array<uint8_t, count> input{};
  for (size_t i = 0; i < input.size(); ++i) input[i] = static_cast<uint8_t>(i);
  std::array<uint8_t, count> output{};
  Reassembler rx;
  rx.configure(output.data(), output.size());
  uint8_t packet[kMaxEspNowPayload];
  auto n = encodeStart(5, count, packet, sizeof(packet));
  TEST_ASSERT_TRUE(rx.accept(packet, n, 1));
  n = encodeAudio(5, count, 0, input.data(), packet, sizeof(packet));
  TEST_ASSERT_EQUAL_UINT32(kMaxEspNowPayload, n);
  TEST_ASSERT_TRUE(rx.accept(packet, n, 2));
  n = encodeAudio(5, count, 1, input.data(), packet, sizeof(packet));
  TEST_ASSERT_EQUAL_UINT32(kHeaderBytes + 4, n);
  TEST_ASSERT_TRUE(rx.accept(packet, n, 3));
  TEST_ASSERT_TRUE(rx.isComplete());
  TEST_ASSERT_EQUAL_MEMORY(input.data(), rx.data(), count);
}

void test_out_of_order_and_duplicates() {
  constexpr uint16_t count = 600;
  std::array<uint8_t, count> input{};
  std::array<uint8_t, count> output{};
  for (size_t i = 0; i < count; ++i) input[i] = static_cast<uint8_t>(i * 7);
  Reassembler rx;
  rx.configure(output.data(), output.size());
  uint8_t packet[kMaxEspNowPayload];
  // No START: recovered using redundant framing carried by every AUDIO packet.
  for (int seq : {2, 0, 2, 1}) {
    const size_t n = encodeAudio(9, count, seq, input.data(), packet, sizeof(packet));
    TEST_ASSERT_TRUE(rx.accept(packet, n, 3 + seq));
  }
  TEST_ASSERT_TRUE(rx.isComplete());
  TEST_ASSERT_EQUAL_UINT16(3, rx.receivedPackets());
  TEST_ASSERT_EQUAL_MEMORY(input.data(), rx.data(), count);
}

void test_dropped_packet_replaced_by_silence() {
  constexpr uint16_t count = 600;
  std::array<uint8_t, count> input{};
  std::array<uint8_t, count> output{};
  input.fill(200);
  Reassembler rx;
  rx.configure(output.data(), output.size());
  uint8_t packet[kMaxEspNowPayload];
  for (int seq : {0, 2}) {
    const size_t n = encodeAudio(10, count, seq, input.data(), packet, sizeof(packet));
    TEST_ASSERT_TRUE(rx.accept(packet, n, 100));
  }
  TEST_ASSERT_FALSE(rx.isComplete());
  TEST_ASSERT_EQUAL_UINT16(1, rx.missingPackets());
  TEST_ASSERT_FALSE(rx.shouldFlush(249));
  TEST_ASSERT_TRUE(rx.shouldFlush(250));
  for (size_t i = 0; i < count; ++i)
    TEST_ASSERT_EQUAL_UINT8(i >= kAudioBytesPerPacket &&
                            i < 2 * kAudioBytesPerPacket ? 128 : 200, output[i]);
}

void test_missing_final_packet_times_out() {
  std::array<uint8_t, 500> input{};
  std::array<uint8_t, 500> output{};
  Reassembler rx;
  rx.configure(output.data(), output.size());
  uint8_t packet[kMaxEspNowPayload];
  const auto n = encodeAudio(7, 500, 0, input.data(), packet, sizeof(packet));
  TEST_ASSERT_TRUE(rx.accept(packet, n, 42));
  TEST_ASSERT_FALSE(rx.shouldFlush(391));
  TEST_ASSERT_TRUE(rx.shouldFlush(392));
  TEST_ASSERT_EQUAL_UINT16(2, rx.missingPackets());
}

void test_full_three_second_recording() {
  std::vector<uint8_t> input(kMaxSamples);
  std::vector<uint8_t> output(kMaxSamples);
  for (size_t i = 0; i < input.size(); ++i) input[i] = static_cast<uint8_t>(i % 256);
  Reassembler rx;
  rx.configure(output.data(), output.size());
  uint8_t packet[kMaxEspNowPayload];
  // Deliver all 101 audio packets in reverse order; START is deliberately lost.
  for (int seq = static_cast<int>(kMaxPackets) - 1; seq >= 0; --seq) {
    auto n = encodeAudio(0xABCD, kMaxSamples, static_cast<uint16_t>(seq),
                         input.data(), packet, sizeof(packet));
    TEST_ASSERT_TRUE(n > kHeaderBytes);
    TEST_ASSERT_TRUE(rx.accept(packet, n, 100));
  }
  TEST_ASSERT_TRUE(rx.isComplete());
  TEST_ASSERT_EQUAL_UINT16(0, rx.missingPackets());
  TEST_ASSERT_EQUAL_MEMORY(input.data(), rx.data(), input.size());
}

void test_orphan_header_releases_buffer() {
  std::array<uint8_t, 240> output{};
  Reassembler rx;
  rx.configure(output.data(), output.size());
  uint8_t packet[kMaxEspNowPayload];
  const auto n = encodeStart(11, 240, packet, sizeof(packet));
  TEST_ASSERT_TRUE(rx.accept(packet, n, 1));
  TEST_ASSERT_FALSE(rx.shouldFlush(350));
  TEST_ASSERT_TRUE(rx.shouldFlush(351));
}

void test_codec_roundtrip_and_quantization() {
  TEST_ASSERT_EQUAL_UINT8(128, dcCentered12ToU8(0));
  TEST_ASSERT_EQUAL_UINT8(129, dcCentered12ToU8(16));
  TEST_ASSERT_EQUAL_UINT8(0, dcCentered12ToU8(-4095));
  TEST_ASSERT_EQUAL_UINT8(255, dcCentered12ToU8(4095));
  TEST_ASSERT_EQUAL_UINT8(0, adc12ToU8(0));
  TEST_ASSERT_EQUAL_UINT8(255, adc12ToU8(4095));
  TEST_ASSERT_EQUAL_UINT8(255, adc12ToU8(65535));
  TEST_ASSERT_EQUAL_INT16(-32768, u8ToPcm16(0));
  TEST_ASSERT_EQUAL_INT16(0, u8ToPcm16(128));
  TEST_ASSERT_EQUAL_INT16(32512, u8ToPcm16(255));
  for (int i = 0; i < 256; ++i) {
    const uint8_t sample = static_cast<uint8_t>(i);
    TEST_ASSERT_EQUAL_UINT8(sample, pcm16ToU8(u8ToPcm16(sample)));
  }
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_header_and_corrupt_packets);
  RUN_TEST(test_short_four_byte_final_chunk);
  RUN_TEST(test_out_of_order_and_duplicates);
  RUN_TEST(test_dropped_packet_replaced_by_silence);
  RUN_TEST(test_missing_final_packet_times_out);
  RUN_TEST(test_full_three_second_recording);
  RUN_TEST(test_orphan_header_releases_buffer);
  RUN_TEST(test_codec_roundtrip_and_quantization);
  return UNITY_END();
}
