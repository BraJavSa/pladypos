#include <Servo.h>
#include <Arduino.h>

Servo thrusters[4];
const int thruster_pins[4] = {A0, A1, A2, A3};

// PC Commands
uint16_t last_received_motors[4] = {1500, 1500, 1500, 1500};
unsigned long last_cmd_time = 0;

// Spektrum Radio variables
uint16_t spektrum_channels[6] = {1024, 1024, 1024, 1024, 1024, 1024};
volatile uint16_t spektrum_rx_count = 0;
bool spektrum_received = false;
unsigned long last_spektrum_time = 0;

// Watchdog heartbeat variables
unsigned long last_watchdog_time = 0;
bool watchdog_state = false;

// Motor telemetry/feedback variables (normalized thrust [-1.0, 1.0])
float normalized_motors[4] = {0.0, 0.0, 0.0, 0.0};
unsigned long last_feedback_time = 0;

float scale_input(uint16_t val, uint16_t min_val, uint16_t max_val);
void readSerialPC();
void processPCPacket(uint8_t type, uint8_t *data, uint8_t len);
void readSpektrum();
void sendSpektrumToPC();
void sendFeedbackToPC();

void setup() {
  // Serial to PC (USB)
  Serial.begin(115200);

  // Serial1 to Spektrum Receiver (Hardware pins RX/TX)
  Serial1.begin(115200);

  // Initialize thrusters
  for (int i = 0; i < 4; ++i) {
    thrusters[i].attach(thruster_pins[i]);
    thrusters[i].writeMicroseconds(1500); // Neutral on startup
  }

  // Enable the Murata DC-DC converter (Negative logic: LOW = ON)
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);

  // Initialize watchdog pin
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
}

void loop() {
  unsigned long now = millis();



  // Read commands from PC and Spektrum receiver
  readSerialPC();
  readSpektrum();

  // Mode Selection: Channel 4 (index 3 of spektrum_channels)
  // > 1000 (e.g. 1705) -> Radio Priority
  // < 1000 (e.g. 343) -> PC/Joy Priority
  // Default to PC mode if no radio signal received in last 500ms
  bool is_radio_mode = spektrum_received && (now - last_spektrum_time < 500) && (spektrum_channels[3] > 1000);

  if (is_radio_mode) {
    // 1. Scale Radio Inputs to [-1.0, 1.0]
    // Ch1 (surge / u) -> min 342, max 1702, neutral 1023
    float surge = scale_input(spektrum_channels[0], 342, 1702);
    // Ch2 (sway / v) -> right 343, left 1702, neutral 1023 (right negative, left positive)
    float sway = scale_input(spektrum_channels[1], 343, 1702);
    // Ch3 (yaw / r) -> CW 343, CCW 1702, neutral 1023 (CW negative, CCW positive)
    float yaw = scale_input(spektrum_channels[2], 343, 1702);

    // 2. Mix Motor Signals (X-configuration)
    float u_FR = surge - sway - yaw;
    float u_FL = surge + sway + yaw;
    float u_BR = -surge - sway + yaw;
    float u_BL = -surge + sway - yaw;

    // 3. Max Normalize to prevent thruster saturation
    float max_val = abs(u_FR);
    if (abs(u_FL) > max_val) max_val = abs(u_FL);
    if (abs(u_BR) > max_val) max_val = abs(u_BR);
    if (abs(u_BL) > max_val) max_val = abs(u_BL);

    if (max_val > 1.0) {
      u_FR /= max_val;
      u_FL /= max_val;
      u_BR /= max_val;
      u_BL /= max_val;
    }

    // 4. Save normalized outputs
    normalized_motors[0] = u_FR;
    normalized_motors[1] = u_FL;
    normalized_motors[2] = u_BR;
    normalized_motors[3] = u_BL;

    // 5. Convert to PWM and Write to Motors
    thrusters[0].writeMicroseconds(1500 + (int)(400 * u_FR));
    thrusters[1].writeMicroseconds(1500 + (int)(400 * u_FL));
    thrusters[2].writeMicroseconds(1500 + (int)(400 * u_BR));
    thrusters[3].writeMicroseconds(1500 + (int)(400 * u_BL));

  } else {
    // Joy/PC mode
    // Safety timeout: if no command received from PC for 500ms, set motors to neutral
    if (now - last_cmd_time > 500) {
      for (int i = 0; i < 4; ++i) {
        thrusters[i].writeMicroseconds(1500);
        normalized_motors[i] = 0.0;
      }
    } else {
      // Use PC commands and write to thrusters
      for (int i = 0; i < 4; ++i) {
        thrusters[i].writeMicroseconds(last_received_motors[i]);
        normalized_motors[i] = ((float)last_received_motors[i] - 1500.0) / 400.0;
      }
    }
  }

  // Publish normalized motor feedback to PC at 30 Hz (every ~33 ms)
  if (now - last_feedback_time >= 33) {
    last_feedback_time = now;
    sendFeedbackToPC();
  }
}

float scale_input(uint16_t val, uint16_t min_val, uint16_t max_val) {
  if (val < min_val) val = min_val;
  if (val > max_val) val = max_val;
  return 2.0 * ((float)val - min_val) / (max_val - min_val) - 1.0;
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
    last_cmd_time = millis();
    for (int i = 0; i < 4; ++i) {
      uint16_t pwm = ((uint16_t)data[2*i+1] << 8) | data[2*i];
      pwm = constrain(pwm, 1100, 1900);
      last_received_motors[i] = pwm;
    }
  } else if (type == 0xF0) {
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
    if (gap_detected) {
      packet_idx = 0;
      gap_detected = false;
    }

    packet[packet_idx++] = Serial1.read();
    last_available_time = micros();

    if (packet_idx == 16) {
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
      spektrum_received = true;
      last_spektrum_time = millis();
      packet_idx = 0;

      sendSpektrumToPC();
    }
  }

  if (micros() - last_available_time > 5000) {
    gap_detected = true;
  }
}

void sendSpektrumToPC() {
  if (Serial.availableForWrite() >= 19) {
    uint8_t header[4] = {0xFF, 0xFF, 0x01, 14};
    uint8_t checksum = 0x01 + 14;

    Serial.write(header, 4);

    uint8_t *data_ptr = (uint8_t *)spektrum_channels;
    for (int i = 0; i < 12; ++i) {
      checksum += data_ptr[i];
      Serial.write(data_ptr[i]);
    }

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

void sendFeedbackToPC() {
  if (Serial.availableForWrite() >= 21) {
    uint8_t header[4] = {0xFF, 0xFF, 0x04, 16};
    uint8_t checksum = 0x04 + 16;

    Serial.write(header, 4);

    uint8_t *data_ptr = (uint8_t *)normalized_motors;
    for (int i = 0; i < 16; ++i) {
      checksum += data_ptr[i];
      Serial.write(data_ptr[i]);
    }
    Serial.write(checksum);
  }
}
