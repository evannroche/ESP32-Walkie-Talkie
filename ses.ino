#define CONFIG_ADC_DISABLE_DRIVER 1  
#include <Arduino.h>
#include "driver/i2s.h"
#include <driver/adc.h>
#include <esp_now.h>
#include <WiFi.h>

// Audio configuration 
#define SAMPLE_RATE 8000        // 8 kHz sampling
#define MAX_RECORD_TIME 3       // Recording Time (3 Sec)
#define MAX_BUFFER_SIZE (SAMPLE_RATE * MAX_RECORD_TIME)
#define MAX_PACKET_SIZE 250     // MAX ESP NOW packet size

// I2S configuration for MAX98357 amplifier 
#define I2S_BCLK_PIN 26         
#define I2S_LRCLK_PIN 25    
#define I2S_DATA_OUT_PIN 22   
#define I2S_SD_PIN 27           
#define I2S_PORT_TX I2S_NUM_0  

// Analog input 
#define AUDIO_IN_PIN 34         
#define BUTTON 23              

// Status LED
#define LED_PIN 2              

uint8_t* audioBuffer;      
uint8_t* receivedBuffer;        
uint32_t sampleIndex = 0;
bool isRecording = false;
bool isPlaying = false;
bool isTransmitting = false;
bool receivedAudio = false;



// MAC ADDRESSES
// Left ESP32 f4:65:0b:48:b9:18
//uint8_t partnerMac[] = {0xF4, 0x65, 0x0B, 0x48, 0xB9, 0x18};  
// Right small board ESP32 f4:65:0b:4a:73:cc
 uint8_t partnerMac[] = {0xF4, 0x65, 0x0B, 0x4A, 0x73, 0xCC}; 

esp_now_peer_info_t peerInfo;
uint32_t receivedSamples = 0;

// Initialize Analog to DIgital Input
void init_analog_input() {
  adc1_config_width(ADC_WIDTH_BIT_12);        
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);  
}

// Initialize I2S for audio output only
void i2s_init_output() {
  pinMode(I2S_SD_PIN, OUTPUT);
  digitalWrite(I2S_SD_PIN, LOW);  
 
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK_PIN,
    .ws_io_num = I2S_LRCLK_PIN,
    .data_out_num = I2S_DATA_OUT_PIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  // Install and configure the I2S driver for output
  i2s_driver_install(I2S_PORT_TX, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT_TX, &pin_config);
}

// ESP-NOW callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Audio packet delivered successfully");
  } else {
    Serial.println("Error delivering audio packet");
  }
  
  if (!isTransmitting) {
    digitalWrite(LED_PIN, LOW); 
  }
}

// ESP-NOW callback when data is received 
void OnDataReceived(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  static uint32_t currentPacketIndex = 0;
  static uint32_t expectedPackets = 0;
  static uint32_t totalReceived = 0;
  
  // First packet contains metadata 
  if (len == sizeof(uint32_t)) {
    receivedSamples = *((uint32_t*)incomingData);
    currentPacketIndex = 0;
    totalReceived = 0;
    
    // Make sure we don't exceed our buffer size
    if (receivedSamples > MAX_BUFFER_SIZE) {
      receivedSamples = MAX_BUFFER_SIZE;
    }
    
    expectedPackets = (receivedSamples + MAX_PACKET_SIZE - 1) / MAX_PACKET_SIZE;
    
    Serial.print("Receiving audio: ");
    Serial.print(receivedSamples);
    Serial.println(" samples");
    
    digitalWrite(LED_PIN, HIGH); 
    return;
  }
  
  // Regular audio data packet
  if (totalReceived + len <= MAX_BUFFER_SIZE) {
    memcpy(receivedBuffer + totalReceived, incomingData, len);
    totalReceived += len;
    currentPacketIndex++;
    
    // If this is the last expected packet
    if (currentPacketIndex >= expectedPackets) {
      digitalWrite(LED_PIN, LOW); // Turn LED off when reception is complete
      
      // Only play if not currently recording or playing
      if (!isRecording && !isPlaying) {
        receivedAudio = true;
      }
    }
  }
}

// Initialize ESP-NOW for wireless communication
void init_espnow() {
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  
  // Print MAC address (useful for setting up the other ESP32)
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataReceived);
  
  // Register peer
  memcpy(peerInfo.peer_addr, partnerMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  Serial.println("ESP-NOW initialized");
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000); 
  
  // Initialize memory buffers
  if (psramFound()) {
    Serial.println("PSRAM found, using it for audio buffers");
    audioBuffer = (uint8_t*)ps_malloc(MAX_BUFFER_SIZE);
    receivedBuffer = (uint8_t*)ps_malloc(MAX_BUFFER_SIZE);
  } else {
    Serial.println("No PSRAM found, using heap for audio buffers");
    audioBuffer = (uint8_t*)malloc(MAX_BUFFER_SIZE);
    receivedBuffer = (uint8_t*)malloc(MAX_BUFFER_SIZE);
  }
  
  // Check if memory allocation was successful
  if (!audioBuffer || !receivedBuffer) {
    Serial.println("Failed to allocate memory for audio buffers!");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  
  // Initialize GPIO pins
  pinMode(BUTTON, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize GPIO for audio input
  init_analog_input();
  
  // Initialize I2S for audio output
  i2s_init_output();
  
  // Initialize ESP-NOW
  init_espnow();
  
  Serial.println("ESP32 Walkie-Talkie Ready");
  Serial.println("- Press and hold button to record and transmit");
  Serial.println("- Release to stop recording and listen");
}

void startRecording() {
  // Don't start if already recording
  if (isRecording) {
    return;
  }
  
  // Don't start if currently playing
  if (isPlaying) {
    return;
  }
  
  // Disable amplifier
  digitalWrite(I2S_SD_PIN, LOW);
  
  // Reset recording state
  sampleIndex = 0;
  isRecording = true;
  
  digitalWrite(LED_PIN, HIGH);
  Serial.println("Recording started");
}

void stopRecording() {
  if (!isRecording) {
    return;
  }
  
  isRecording = false;
  
  Serial.print("Recorded ");
  Serial.print(sampleIndex);
  Serial.print(" samples (");
  Serial.print((float)sampleIndex / SAMPLE_RATE, 2);
  Serial.println(" seconds)");
  
  // Transmit the recorded audio if we have enough samples
  if (sampleIndex > 100) {
    transmitAudio();
  } else {
    digitalWrite(LED_PIN, LOW); // Turn LED off if not transmitting
  }
}

void recordSample() {
  static unsigned long lastSampleTime = 0;
  const unsigned long samplingInterval = 1000000 / SAMPLE_RATE;

  if (micros() - lastSampleTime >= samplingInterval) {
    if (sampleIndex < MAX_BUFFER_SIZE) {
      int rawValue = adc1_get_raw(ADC1_CHANNEL_6);  // Direct ADC read
      uint8_t sample = (rawValue >> 4);             // 12-bit → 8-bit
      audioBuffer[sampleIndex++] = sample;
    } else {
      stopRecording();
    }
    lastSampleTime = micros();
  }
}

void transmitAudio() {
  if (sampleIndex == 0) {
    return;
  }
  
  isTransmitting = true;
  digitalWrite(LED_PIN, HIGH); // Ensure LED is on during transmission
  
  // First send metadata (number of samples)
  uint32_t metaData = sampleIndex;
  esp_err_t result = esp_now_send(partnerMac, (uint8_t*)&metaData, sizeof(metaData));
  
  if (result != ESP_OK) {
    Serial.println("Error sending metadata");
    isTransmitting = false;
    digitalWrite(LED_PIN, LOW);
    return;
  }
  
  delay(5);
  
  // Then send audio data in chunks
  uint32_t packetCount = 0;
  for (uint32_t offset = 0; offset < sampleIndex; offset += MAX_PACKET_SIZE) {
    // Calculate size of this packet
    uint32_t packetSize = (MAX_PACKET_SIZE < (sampleIndex - offset)) ? MAX_PACKET_SIZE : (sampleIndex - offset);
    
    // Send packet
    result = esp_now_send(partnerMac, audioBuffer + offset, packetSize);
    
    if (result != ESP_OK) {
      Serial.print("Failed to send packet ");
      Serial.println(packetCount);
    }
    
    packetCount++;
    delay(5); // Small delay between packets to avoid overwhelming receiver
  }
  
  Serial.print("Transmitted ");
  Serial.print(packetCount);
  Serial.println(" audio packets");
  
  isTransmitting = false;
}

void playAudio(uint8_t* buffer, uint32_t numSamples) {
  if (numSamples == 0) {
    return;
  }
  
  if (isRecording) {
    return;
  }
  
  if (isPlaying) {
    return;
  }
  
  isPlaying = true;
  
  // Enable the amplifier
  digitalWrite(I2S_SD_PIN, HIGH);
  delay(5);  // Brief delay for amplifier to stabilize
  
  Serial.print("Playing audio: ");
  Serial.print(numSamples);
  Serial.println(" samples");
  
  // Loop through the buffer and send each sample to the I2S output
  for (uint32_t i = 0; i < numSamples && isPlaying; i++) {
    // Convert 8-bit sample to 16-bit (scale up)
    int16_t sample16 = (buffer[i] - 128) * 256; // Convert to signed and scale
    
    // Create stereo sample (same value for left and right channels)
    uint32_t stereoSample = (sample16 << 16) | (sample16 & 0xFFFF);
    size_t bytes_written;
    
    // Write the sample to the I2S driver
    i2s_write(I2S_PORT_TX, &stereoSample, sizeof(stereoSample), &bytes_written, portMAX_DELAY);
    
    // Maintain correct playback speed
    delayMicroseconds(1000000 / SAMPLE_RATE);
    
    // Check for button press that might want to stop playback
    if (digitalRead(BUTTON) == HIGH) {
      isPlaying = false;
      break;
    }
  }
  
  // Disable the amplifier when not in use
  digitalWrite(I2S_SD_PIN, LOW);
  
  isPlaying = false;
  Serial.println("Playback complete");
}

void loop() {
  int buttonState = digitalRead(BUTTON);
  
  // Button is pressed (HIGH) - RECORD
  if (buttonState == HIGH) {
    if (!isRecording && !isPlaying) {
      startRecording();
    }
    
    if (isRecording) {
      recordSample();  // Record multiple samples efficiently
    }
  }
  // Button is released (LOW) - STOP and TRANSMIT
  else {
    if (isRecording) {
      stopRecording();
      // transmitAudio() is called from stopRecording() if needed
    }
  }
  
  // Check if we have received audio to play
  if (receivedAudio && !isRecording && !isPlaying) {
    // Reset the flag
    receivedAudio = false;
    
    // Play the received audio
    playAudio(receivedBuffer, receivedSamples);
  }
}