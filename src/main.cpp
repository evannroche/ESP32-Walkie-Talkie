#include <Arduino.h>
#include <cstdlib>
#include "audio_capture.h"
#include "audio_playback.h"
#include "config.h"
#include "transport.h"

namespace {
enum class State { IDLE, RECORDING, TRANSMITTING, RECEIVING, PLAYING };
State state = State::IDLE;
AudioCapture capture;
AudioPlayback playback;
Transport radio;
uint8_t* recordBuffer = nullptr;
uint16_t recorded = 0;
bool requireButtonRelease = false;

class DebouncedButton {
public:
  void begin() {
    pinMode(config::kButtonPin, INPUT_PULLDOWN);
    lastRaw_ = digitalRead(config::kButtonPin) == HIGH;
    stable_ = lastRaw_;
    changedAt_ = millis();
  }
  void update() {
    const bool raw = digitalRead(config::kButtonPin) == HIGH;
    if (raw != lastRaw_) {
      lastRaw_ = raw;
      changedAt_ = millis();
    }
    if (millis() - changedAt_ >= config::kButtonDebounceMs) stable_ = raw;
  }
  bool pressed() const { return stable_; }
private:
  bool lastRaw_ = false;
  bool stable_ = false;
  uint32_t changedAt_ = 0;
} button;

uint8_t* allocateSamples() {
  if (psramFound()) {
    if (auto* p = static_cast<uint8_t*>(ps_malloc(config::kMaxSamples))) return p;
  }
  return static_cast<uint8_t*>(malloc(config::kMaxSamples));
}

[[noreturn]] void fatalError(const char* reason) {
  Serial.printf("FATAL: %s\n", reason);
  for (;;) { // Startup failure is visible, never silently continue without radio/audio.
    digitalWrite(config::kLedPin, HIGH);
    delay(120);
    digitalWrite(config::kLedPin, LOW);
    delay(120);
  }
}

void discardReady() {
  ReadyAudio r{};
  while (radio.nextReady(r)) radio.release(r.slot);
}

void startRecording() {
  radio.setAccepting(false);
  radio.clearReceive();
  discardReady();
  digitalWrite(config::kAmpShutdownPin, LOW);
  digitalWrite(config::kLedPin, HIGH);
  recorded = 0;
  capture.start();
  state = State::RECORDING;
  Serial.println("Recording");
}

void sendRecording() {
  state = State::TRANSMITTING;
  if (recorded > 100) radio.sendAudio(recordBuffer, recorded);
  Serial.printf("Captured %u samples (%.2f seconds)\n",
                recorded, static_cast<double>(recorded) / config::kSampleRate);
  // Prevent an old partial incoming message from being played after TX.
  radio.clearReceive();
  discardReady();
  radio.setAccepting(true);
  digitalWrite(config::kLedPin, LOW);
  requireButtonRelease = true; // One recording per press, even after hitting the cap.
  state = State::IDLE;
}

void playMessage(const ReadyAudio& ready) {
  radio.setAccepting(false);
  radio.clearReceive();
  state = State::PLAYING;
  digitalWrite(config::kLedPin, HIGH);
  Serial.printf("Playing: %u samples, %u packet(s) concealed with silence\n",
                ready.samples, ready.lostPackets);
  const bool ok = playback.play(radio.samplesFor(ready.slot), ready.samples);
  radio.release(ready.slot); // Receiver task may now reuse this buffer.
  discardReady();           // Half-duplex: discard anything queued during playback.
  if (!ok) Serial.println("I2S playback error");
  digitalWrite(config::kLedPin, LOW);
  radio.setAccepting(true);
  state = State::IDLE;
}
}

void setup() {
  Serial.begin(115200);
  pinMode(config::kLedPin, OUTPUT);
  digitalWrite(config::kLedPin, LOW);
  button.begin();

  recordBuffer = allocateSamples();
  uint8_t* receive0 = allocateSamples();
  uint8_t* receive1 = allocateSamples();
  if (!recordBuffer || !receive0 || !receive1) fatalError("Audio buffer allocation");
  if (!capture.begin()) fatalError("ADC init");
  if (!playback.begin()) fatalError("I2S init");
  if (!radio.begin(receive0, receive1)) fatalError("ESP-NOW init/peer/task");
  Serial.println("Ready. Hold to record (max 3 seconds), release to transmit.");
}

void loop() {
  button.update();
  switch (state) {
    case State::IDLE:
    case State::RECEIVING: {
      // A press takes precedence over playback; ignore any partial received audio.
      if (!button.pressed()) requireButtonRelease = false;
      if (button.pressed() && !requireButtonRelease) {
        startRecording();
        break;
      }
      ReadyAudio next{};
      if (radio.nextReady(next)) {
        if (radio.isCurrent(next)) playMessage(next);
        else radio.release(next.slot);
      } else {
        state = radio.receiving() ? State::RECEIVING : State::IDLE;
        delay(1);
      }
      break;
    }
    case State::RECORDING: {
      if (!button.pressed() || recorded >= config::kMaxSamples) {
        sendRecording();
        break;
      }
      uint8_t sample;
      if (capture.poll(sample)) recordBuffer[recorded++] = sample;
      break;
    }
    case State::TRANSMITTING:
    case State::PLAYING:
      // These transitions execute synchronously in sendRecording/playMessage.
      break;
  }
}
