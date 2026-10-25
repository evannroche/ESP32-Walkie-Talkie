# ESP32 Walkie-Talkie

[![Firmware and tests](https://github.com/evannroche/ESP32-Walkie-Talkie/actions/workflows/ci.yml/badge.svg)](https://github.com/evannroche/ESP32-Walkie-Talkie/actions/workflows/ci.yml)

A **push-to-talk, half-duplex** voice radio built using two classic ESP32 boards and ESP-NOW. The ESP32 devices exchange voice recordings directly without a Wi-Fi router. Hold the button to capture up to **3 seconds** of audio; release it to transmit. The other board reassembles the recording and plays it over a MAX98357A I2S amplifier.

```
Microphone -> ADC1 -> 8 kHz / 8-bit recording buffer -> packet framing
                                                        |
Speaker <- MAX98357A <- I2S <- playback buffer <- ESP-NOW reassembly
                                                        ^
                                                 ESP-NOW wireless link
```

## Hardware and wiring

This pinout is sourced from [`src/config.h`](src/config.h), and assumes a **classic ESP32-WROOM-32** (`esp32dev`) with an analog microphone output and a MAX98357A I2S amplifier. Supply correct power and ground to the modules; **do not connect a raw electret capsule directly to GPIO34 without a suitable biased analog frontend**.

| ESP32 GPIO | Function | Connect to |
| --- | --- | --- |
| GPIO34 | ADC1_CH6 analog input | Conditioned microphone analog output |
| GPIO23 | Active-high push-to-talk button | Button to 3.3 V; internal pulldown enabled |
| GPIO2 | Status LED | Onboard LED, if present |
| GPIO26 | I2S BCLK | MAX98357A BCLK |
| GPIO25 | I2S LRCLK / WS | MAX98357A LRC |
| GPIO22 | I2S DOUT | MAX98357A DIN |
| GPIO27 | Amplifier enable | MAX98357A SD (HIGH = enabled) |

The paired MAC addresses currently configured in `config.h` are:

| Unit being flashed | Partner MAC address |
| --- | --- |
| Left | `F4:65:0B:4A:73:CC` (right board) |
| Right | `F4:65:0B:48:B9:18` (left board) |

Confirm both addresses from the Serial Monitor, as firmware or replacement boards may use different addresses. The firmware filters incoming packets by the configured peer address, but this is **not authentication or encryption**.

## Build, upload, test

To migrate the existing repository, copy this project's contents into its root and remove the obsolete root-level `walkie_talkie.ino`; PlatformIO uses `src/main.cpp` instead. Keep your existing hardware photos or verified demo assets separately.

Install [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html), then:

```sh
pio run                           # Builds left and right firmware
pio run -e esp32-left -t upload   # Upload left board (connect its USB cable)
pio run -e esp32-right -t upload  # Upload right board (connect its USB cable)
pio device monitor -b 115200
pio test -e native                # Laptop-only packet/codec tests
```

`platformio.ini` pins the ESP32 platform to `espressif32 @ 8.0.0`, which contains Arduino-ESP32 3.x and ESP-IDF 5.x drivers. The `native` environment uses the host compiler without any hardware. CI builds both firmware variants and runs the native tests on pushes and pull requests. The badge reports the GitHub Actions workflow state once this refactor is applied to the repository.

## Design decisions

- **ESP-NOW** transports short voice messages directly between peers without an access point. No application-level acknowledgements or retries are implemented; ESP-NOW's send status is **not** a guarantee of receiver playback.
- **8 kHz / 8-bit mono audio** uses 8 KB per recorded second, with 16-bit stereo samples generated only for I2S playback. This is basic speech quality, not high-fidelity audio or lossless compression.
- **Packet format** uses a fixed **12-byte header on every packet**: magic (1), version (1), type (1), reserved (1), message ID (2), sequence (2), packet count (2), sample count (2), all multibyte fields little-endian. A Start packet has no audio payload. Audio packets contain up to **238 bytes**, fitting within a conservative 250-byte ESP-NOW payload. The maximum recording is 24,000 samples = 101 audio packets plus one Start packet.
- **Loss and reordering** are handled by sequence number, duplicate suppression, and a bitmap of received packets. Receiving can start from an audio packet if Start was dropped. After the final packet, the receiver waits up to 150 ms for gaps; otherwise, a 350 ms idle timeout flushes partial messages. Missing chunks play as silence, rather than shifting subsequent audio.
- **Task ownership** keeps the ESP-NOW callbacks short. The Wi-Fi callback only copies packets into a FreeRTOS queue. A receiver task reassembles into one of two buffers and passes ready-message descriptors to `loop()`. The playback buffer is not reusable until released. New receptions are rejected while capturing, sending, or playing (half-duplex).
- **Audio drivers** use `adc_oneshot` and `i2s_std`. ADC capture applies an adjustable gain and running DC baseline removal. I2S playback writes stereo frames in blocks, with the driver pacing samples via DMA rather than per-sample software delays.

## Known limitations

- Fixed **3-second** recording cap and **half-duplex** operation; playback currently finishes before another recording begins.
- **Not encrypted** (`peer.encrypt = false`), and no secure pairing, receiver-level ACK, retransmissions, or CRC beyond lower-layer checks.
- Sampling is still **polling-based** using `micros()` and may exhibit jitter under CPU load; hardware-timer or ADC continuous DMA capture is a future improvement.
- Timeout-based loss concealment simply fills missing 8-bit samples with midpoint silence (`128`). There is no jitter buffer, forward error correction, or packet retransmission.
- The receive queue is bounded; high packet rates or processing stalls can drop packets. Both devices must use the same Wi-Fi channel, and Wi-Fi coexistence and range have not been benchmarked.
- This README does **not** claim measured latency, range, delivery percentage, or audio fidelity. Those require repeatable hardware tests.

## Demo

Add a short **real hardware video or GIF** showing a recording on one board and playback on the other. A demo is not included because it has not been provided or verified.

## Project layout

```text
src/main.cpp               Button debounce and state transitions
src/config.h               Pinout, sample rate, board pair addresses
src/audio_capture.*        ADC capture and DC-offset cancellation
src/audio_playback.*       Block-based I2S output
src/transport.*            ESP-NOW callbacks, FreeRTOS queues, TX and RX tasks
lib/walkie_core/           Hardware-independent packet codec and reassembler
test/test_protocol/        Native Unity tests
.github/workflows/ci.yml  Firmware build and test workflow
```
