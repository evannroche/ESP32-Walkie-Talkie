#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <walkie_core/protocol.h>

struct ReadyAudio {
  uint8_t slot;
  uint16_t samples;
  uint16_t lostPackets;
  uint32_t epoch;
};

class Transport {
public:
  bool begin(uint8_t* buffer0, uint8_t* buffer1);
  void setAccepting(bool enabled) { accepting_.store(enabled); }
  void clearReceive();  // Invalidates old and in-flight messages without touching playback memory.
  bool receiving() const { return receiving_.load(); }
  bool nextReady(ReadyAudio& ready);
  bool isCurrent(const ReadyAudio& ready) const { return ready.epoch == epoch_.load(); }
  const uint8_t* samplesFor(uint8_t slot) const;
  void release(uint8_t slot);
  bool sendAudio(const uint8_t* audio, uint16_t sampleCount);
  uint32_t droppedPackets() const { return dropped_.load(); }

private:
  struct RxPacket {
    uint32_t epoch;
    uint16_t length;
    uint8_t bytes[walkie::kMaxEspNowPayload];
  };
  enum SlotState : uint8_t { FREE, ASSEMBLING, READY };
  static Transport* instance_;
  static void onReceive(const esp_now_recv_info_t* info, const uint8_t* bytes, int length);
  static void onSent(const esp_now_send_info_t* info, esp_now_send_status_t result);
  static void taskEntry(void* context);
  void runReceiver();
  void publishActive(uint32_t epoch);
  void resetActive();
  bool sendPacket(const uint8_t* data, size_t length);

  QueueHandle_t packets_ = nullptr;
  QueueHandle_t ready_ = nullptr;
  QueueHandle_t released_ = nullptr;
  SemaphoreHandle_t txComplete_ = nullptr;
  TaskHandle_t task_ = nullptr;
  uint8_t* buffers_[2] = {nullptr, nullptr};
  walkie::Reassembler assembly_[2];
  SlotState slotState_[2] = {FREE, FREE}; // Owned by receiver task only
  int activeSlot_ = -1;                  // Owned by receiver task only
  std::atomic<bool> accepting_{false};
  std::atomic<bool> receiving_{false};
  std::atomic<uint32_t> epoch_{1};
  std::atomic<uint32_t> dropped_{0};
  std::atomic<uint32_t> sentOk_{0};
  std::atomic<uint32_t> sentFailed_{0};
  std::atomic<bool> lastTxSucceeded_{false};
  uint16_t lastFinishedMessageId_ = 0; // Receiver task; filters late duplicate packets
  bool hasFinishedMessage_ = false;
  uint16_t nextMessageId_ = 0;         // Called from main task only
};
