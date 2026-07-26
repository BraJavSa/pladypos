#include <Servo.h>

Servo thrusters[4];
const int thruster_pins[4] = {A0, A1, A2, A3};

uint16_t last_received_motors[4] = {1500, 1500, 1500, 1500};
unsigned long last_cmd_time = 0;

// Watchdog heartbeat variables
unsigned long last_watchdog_time = 0;
bool watchdog_state = false;

void setup() {
  Serial.begin(115200);

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
  digitalWrite(7, LOW);
}

void loop() {
  unsigned long now = millis();

  // Read commands from the PC
  readSerialPC();

  // Safety timeout: if no command received from PC for 500ms, set motors to neutral
  if (now - last_cmd_time > 500) {
    for (int i = 0; i < 4; ++i) {
      thrusters[i].writeMicroseconds(1500);
      last_received_motors[i] = 1500;
    }
  }

  // Send motor feedback to PC at 10 Hz (every 100 ms)
  static unsigned long last_feedback_time = 0;
  if (now - last_feedback_time >= 100) {
    last_feedback_time = now;
    sendFeedbackToPC();
  }

  // Watchdog heartbeat (toggles Pin 7 at 1 Hz)
  if (now - last_watchdog_time >= 1000) {
    last_watchdog_time = now;
    watchdog_state = !watchdog_state;
    digitalWrite(7, watchdog_state ? HIGH : LOW);
  }
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
      thrusters[i].writeMicroseconds(pwm);
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

void sendFeedbackToPC() {
  if (Serial.availableForWrite() >= 13) {
    uint8_t header[4] = {0xFF, 0xFF, 0x04, 8};
    uint8_t checksum = 0x04 + 8;

    Serial.write(header, 4);

    uint8_t *data_ptr = (uint8_t *)last_received_motors;
    for (int i = 0; i < 8; ++i) {
      checksum += data_ptr[i];
      Serial.write(data_ptr[i]);
    }
    Serial.write(checksum);
  }
}
