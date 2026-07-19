#include "transport.h"
#include "config.h"
#include <Arduino.h>
#include <WiFi.h>
#include <cstring>
#include <esp_system.h>

Transport* Transport::instance_ = nullptr;

bool Transport::begin(uint8_t* buffer0, uint8_t* buffer1) {
  if (!buffer0 || !buffer1) return false;
  buffers_[0] = buffer0;
  buffers_[1] = buffer1;
  nextMessageId_ = static_cast<uint16_t>(esp_random());
  assembly_[0].configure(buffer0, config::kMaxSamples);
  assembly_[1].configure(buffer1, config::kMaxSamples);

  packets_ = xQueueCreate(16, sizeof(RxPacket));
  ready_ = xQueueCreate(2, sizeof(ReadyAudio));
  released_ = xQueueCreate(2, sizeof(uint8_t));
  txComplete_ = xSemaphoreCreateBinary();
  if (!packets_ || !ready_ || !released_ || !txComplete_) return false;

  WiFi.mode(WIFI_STA);
  Serial.printf("Local MAC: %s\n", WiFi.macAddress().c_str());
  if (esp_now_init() != ESP_OK) return false;
  instance_ = this;
  if (esp_now_register_send_cb(onSent) != ESP_OK ||
      esp_now_register_recv_cb(onReceive) != ESP_OK) return false;

  esp_now_peer_info_t peer{};
  std::memcpy(peer.peer_addr, config::kPeerMac, 6);
  peer.channel = 0;  // Use current station channel; ensure both devices use same channel.
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) return false;

  if (xTaskCreatePinnedToCore(taskEntry, "walkie_rx", 6144, this, 2, &task_, 1) != pdPASS)
    return false;
  accepting_.store(true);
  return true;
}

void Transport::onReceive(const esp_now_recv_info_t* info,
                          const uint8_t* bytes, int length) {
  Transport* t = instance_;
  if (!t || !t->accepting_.load() || !info || !bytes ||
      length <= 0 || length > static_cast<int>(walkie::kMaxEspNowPayload) ||
      std::memcmp(info->src_addr, config::kPeerMac, 6) != 0) return;
  RxPacket packet{};
  packet.epoch = t->epoch_.load();
  packet.length = static_cast<uint16_t>(length);
  std::memcpy(packet.bytes, bytes, length);
  // ESP-NOW callback is a Wi-Fi task, not an interrupt: non-blocking queue send.
  if (xQueueSend(t->packets_, &packet, 0) != pdTRUE) ++t->dropped_;
}

void Transport::onSent(const esp_now_send_info_t*, esp_now_send_status_t result) {
  Transport* t = instance_;
  if (!t) return;
  t->lastTxSucceeded_.store(result == ESP_NOW_SEND_SUCCESS);
  if (result == ESP_NOW_SEND_SUCCESS) ++t->sentOk_;
  else ++t->sentFailed_;
  xSemaphoreGive(t->txComplete_);
}

void Transport::taskEntry(void* context) {
  static_cast<Transport*>(context)->runReceiver();
}

void Transport::resetActive() {
  if (activeSlot_ >= 0) {
    assembly_[activeSlot_].reset();
    slotState_[activeSlot_] = FREE;
    activeSlot_ = -1;
  }
  receiving_.store(false);
}

void Transport::publishActive(uint32_t epoch) {
  if (activeSlot_ < 0) return;
  const int index = activeSlot_;
  activeSlot_ = -1;
  receiving_.store(false);
  if (assembly_[index].receivedPackets() == 0) {
    assembly_[index].reset();
    slotState_[index] = FREE;
    return;
  }
  slotState_[index] = READY;
  lastFinishedMessageId_ = assembly_[index].messageId();
  hasFinishedMessage_ = true;
  ReadyAudio ready{static_cast<uint8_t>(index), assembly_[index].sampleCount(),
                   assembly_[index].missingPackets(), epoch};
  if (xQueueSend(ready_, &ready, 0) != pdTRUE) {
    assembly_[index].reset();
    slotState_[index] = FREE;
  }
}

void Transport::runReceiver() {
  uint32_t taskEpoch = epoch_.load();
  for (;;) {
    // Main thread releases buffers only after playback ends.
    uint8_t slot;
    while (xQueueReceive(released_, &slot, 0) == pdTRUE) {
      if (slot < 2 && slotState_[slot] == READY) {
        assembly_[slot].reset();
        slotState_[slot] = FREE;
      }
    }
    if (taskEpoch != epoch_.load()) {
      resetActive();
      taskEpoch = epoch_.load();
      hasFinishedMessage_ = false;
    }

    RxPacket packet{};
    if (xQueueReceive(packets_, &packet, pdMS_TO_TICKS(10)) == pdTRUE &&
        packet.epoch == taskEpoch) {
      walkie::PacketView parsed{};
      if (walkie::decodePacket(packet.bytes, packet.length, parsed) &&
          !(hasFinishedMessage_ && parsed.messageId == lastFinishedMessageId_)) {
        if (activeSlot_ >= 0 && parsed.messageId != assembly_[activeSlot_].messageId())
          publishActive(taskEpoch);
        if (activeSlot_ < 0) {
          for (int i = 0; i < 2; ++i) {
            if (slotState_[i] == FREE) {
              activeSlot_ = i;
              slotState_[i] = ASSEMBLING;
              receiving_.store(true);
              break;
            }
          }
        }
        if (activeSlot_ >= 0) {
          auto& receiver = assembly_[activeSlot_];
          receiver.accept(packet.bytes, packet.length, millis());
          if (receiver.isComplete()) publishActive(taskEpoch);
        } else ++dropped_; // Two occupied buffers: reject without overwriting playback.
      }
    }
    if (activeSlot_ >= 0 && assembly_[activeSlot_].shouldFlush(millis()))
      publishActive(taskEpoch);
  }
}

void Transport::clearReceive() {
  ++epoch_; // Task discards prior packets by epoch; any published old event is stale.
}

bool Transport::nextReady(ReadyAudio& ready) {
  return xQueueReceive(ready_, &ready, 0) == pdTRUE;
}

const uint8_t* Transport::samplesFor(uint8_t slot) const {
  return slot < 2 ? buffers_[slot] : nullptr;
}

void Transport::release(uint8_t slot) {
  if (slot < 2) xQueueSend(released_, &slot, portMAX_DELAY);
}

bool Transport::sendPacket(const uint8_t* data, size_t length) {
  // Never queue the next packet before the last send callback; throttles radio TX.
  xSemaphoreTake(txComplete_, 0);
  if (esp_now_send(config::kPeerMac, data, length) != ESP_OK) return false;
  return xSemaphoreTake(txComplete_, pdMS_TO_TICKS(250)) == pdTRUE &&
         lastTxSucceeded_.load();
}

bool Transport::sendAudio(const uint8_t* audio, uint16_t count) {
  if (!audio || count == 0 || count > config::kMaxSamples) return false;
  const uint32_t successBefore = sentOk_.load();
  const uint32_t failureBefore = sentFailed_.load();
  const uint16_t id = ++nextMessageId_;
  uint8_t packet[walkie::kMaxEspNowPayload];
  bool ok = true;
  size_t length = walkie::encodeStart(id, count, packet, sizeof(packet));
  if (!length || !sendPacket(packet, length)) ok = false;

  const uint16_t packets = static_cast<uint16_t>(
      (count + walkie::kAudioBytesPerPacket - 1) / walkie::kAudioBytesPerPacket);
  for (uint16_t seq = 0; ok && seq < packets; ++seq) {
    length = walkie::encodeAudio(id, count, seq, audio, packet, sizeof(packet));
    if (!length || !sendPacket(packet, length)) ok = false;
  }
  // No printing in either Wi-Fi callback; one summary per recording.
  Serial.printf("TX msg=%u packets=%u link-ok=%lu link-failed=%lu queue-drop=%lu %s\n",
                id, packets + 1,
                static_cast<unsigned long>(sentOk_.load() - successBefore),
                static_cast<unsigned long>(sentFailed_.load() - failureBefore),
                static_cast<unsigned long>(dropped_.load()),
                ok ? "completed" : "aborted");
  return ok;
}
