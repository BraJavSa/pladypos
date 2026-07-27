#include <Arduino.h>

// Radio variables
uint16_t spektrum_channels[6] = {1024, 1024, 1024, 1024, 1024, 1024};
volatile uint16_t spektrum_rx_count = 0;

// Watchdog heartbeat variables
unsigned long last_watchdog_time = 0;
bool watchdog_state = false;

// PC communications fallback
unsigned long last_cmd_time = 0;

void setup() {
  // Serial to PC (USB)
  Serial.begin(115200);
  
  // Serial1 to Spektrum Receiver (Hardware pins RX/TX)
  Serial1.begin(115200);

  // Enable the Murata DC-DC converter (Negative logic: LOW = ON)
  // This is critical as the converter provides power to onboard devices, which may include the radio receiver.
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);

  // Initialize watchdog pin
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
}

void loop() {
  // Read PC commands (mainly for handshake 0xF0)
  readSerialPC();

  // Read radio receiver continuously
  readSpektrum();

}

void readSerialPC() {
  static int state = 0;
  static uint8_t p_type = 0;
  static uint8_t p_len = 0;
  static uint8_t payload[32];
  static uint8_t payload_idx = 0;

  while (Serial.available() > 0) {
    uint8_t val = Serial.read();

    if (state == 0) {
      if (val == 0xFF)
        state = 1;
    } else if (state == 1) {
      if (val == 0xFF)
        state = 2;
      else
        state = 0;
    } else if (state == 2) {
      p_type = val;
      state = 3;
    } else if (state == 3) {
      p_len = val;
      payload_idx = 0;
      if (p_len > sizeof(payload)) {
        state = 0;
      } else {
        state = 4;
      }
    } else if (state == 4) {
      payload[payload_idx++] = val;
      if (payload_idx == p_len) {
        state = 5;
      }
    } else if (state == 5) {
      uint8_t checksum = p_type + p_len;
      for (uint8_t i = 0; i < p_len; ++i) {
        checksum += payload[i];
      }
      if (checksum == val) {
        processPCPacket(p_type, payload, p_len);
      }
      state = 0;
    }
  }
}

void processPCPacket(uint8_t type, uint8_t *data, uint8_t len) {
  if (type == 0x00 && len == 8) {
    // Ignore motor outputs, just track command receipt
    last_cmd_time = millis();
  } else if (type == 0xF0) {
    // Port detection signature
    const char *sig = "PLADYPOS_BRIDGE";
    uint8_t sig_len = 15;
    uint8_t header[4] = {0xFF, 0xFF, 0xF0, sig_len};
    uint8_t checksum = 0xF0 + sig_len;
    
    if (Serial.availableForWrite() >= 20) {
      Serial.write(header, 4);
      for (uint8_t i = 0; i < sig_len; ++i) {
        checksum += sig[i];
        Serial.write(sig[i]);
      }
      Serial.write(checksum);
    }
  }
}

void readSpektrum() {
  static uint8_t packet[16];
  static uint8_t packet_idx = 0;
  static unsigned long last_available_time = 0;
  static bool gap_detected = false;

  while (Serial1.available() > 0) {
    // If we've detected a long pause in data arrival, this byte marks the beginning of a new packet.
    if (gap_detected) {
      packet_idx = 0;
      gap_detected = false;
    }

    packet[packet_idx++] = Serial1.read();
    last_available_time = micros();

    if (packet_idx == 16) {
      // Decode Spektrum DSMX satellite packet (16 bytes)
      // Byte 0-1: system header info
      // Byte 2-15: 7 channels (2 bytes each)
      for (int i = 1; i < 8; ++i) {
        uint8_t b1 = packet[2 * i];
        uint8_t b2 = packet[2 * i + 1];
        uint8_t chan = (b1 >> 3) & 0x0F;
        uint16_t val = ((b1 & 0x07) << 8) | b2;
        if (chan < 6) {
          spektrum_channels[chan] = val;
        }
      }
      spektrum_rx_count++;
      packet_idx = 0;

      // Send the decoded channels to the PC immediately for minimum latency
      sendSpektrumToPC();
    }
  }

  // Check if we have been idle (no incoming bytes) for more than 5000 microseconds (5ms).
  // The Spektrum frame gap is at least 9ms (9000us) for 11ms frame rate and 20ms for 22ms.
  if (micros() - last_available_time > 5000) {
    gap_detected = true;
  }
}

void sendSpektrumToPC() {
  // Size: 4 bytes header + 14 bytes payload (12 channels + 2 freq) + 1 checksum = 19 bytes
  if (Serial.availableForWrite() >= 19) {
    uint8_t header[4] = {0xFF, 0xFF, 0x01, 14};
    uint8_t checksum = 0x01 + 14;

    Serial.write(header, 4);

    uint8_t *data_ptr = (uint8_t *)spektrum_channels;
    for (int i = 0; i < 12; ++i) {
      checksum += data_ptr[i];
      Serial.write(data_ptr[i]);
    }

    // Compute active RX frequency over the last second
    static unsigned long last_freq_calc = 0;
    static uint16_t current_freq = 0;
    unsigned long now = millis();
    if (now - last_freq_calc >= 1000) {
      current_freq = spektrum_rx_count;
      spektrum_rx_count = 0;
      last_freq_calc = now;
    }

    uint8_t *freq_ptr = (uint8_t *)&current_freq;
    checksum += freq_ptr[0] + freq_ptr[1];
    Serial.write(freq_ptr[0]);
    Serial.write(freq_ptr[1]);

    Serial.write(checksum);
  }
}
