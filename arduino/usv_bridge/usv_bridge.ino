#include <Servo.h>

const int DRIVER_NUM = 4;
const int MOTOR_BASE = 1500;
const int MAX_IDLE = 1000;
uint8_t motorPWM[DRIVER_NUM] = {13, 12, 11, 10};
Servo motors[DRIVER_NUM];

uint16_t spektrum_channels[6] = {1024, 1024, 1024, 1024, 1024, 1024};
unsigned long last_cmd_time = 0;
unsigned long last_spektrum_time = 0;
unsigned long last_telemetry_time = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  for (int i = 0; i < DRIVER_NUM; ++i) {
    motors[i].attach(motorPWM[i]);
    motors[i].writeMicroseconds(MOTOR_BASE);
  }
}

void loop() {
  readSerialPC();
  readSpektrum();
  sendTelemetry();

  if (millis() - last_cmd_time > MAX_IDLE) {
    for (int i = 0; i < DRIVER_NUM; ++i) {
      motors[i].writeMicroseconds(MOTOR_BASE);
    }
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
    uint16_t pwm_vals[4];
    memcpy(pwm_vals, data, 8);
    for (int i = 0; i < DRIVER_NUM; ++i) {
      if (pwm_vals[i] >= 1000 && pwm_vals[i] <= 2000) {
        motors[i].writeMicroseconds(pwm_vals[i]);
      }
    }
    last_cmd_time = millis();
  } else if (type == 0xF0) {
    const char *sig = "PLADYPOS_BRIDGE";
    uint8_t sig_len = 15;
    uint8_t header[4] = {0xFF, 0xFF, 0xF0, sig_len};
    uint8_t checksum = 0xF0 + sig_len;
    Serial.write(header, 4);
    for (uint8_t i = 0; i < sig_len; ++i) {
      checksum += sig[i];
      Serial.write(sig[i]);
    }
    Serial.write(checksum);
  }
}

void readSpektrum() {
  static uint8_t packet[16];
  static uint8_t packet_idx = 0;
  static unsigned long last_byte_time = 0;

  if (Serial1.available() > 0) {
    unsigned long now = millis();
    if (now - last_byte_time > 5) {
      packet_idx = 0;
    }
    last_byte_time = now;

    packet[packet_idx++] = Serial1.read();

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
      sendSpektrumToPC();
      packet_idx = 0;
    }
  }
}

void sendSpektrumToPC() {
  uint8_t header[4] = {0xFF, 0xFF, 0x01, 12};
  uint8_t checksum = 0x01 + 12;

  Serial.write(header, 4);

  uint8_t *data_ptr = (uint8_t *)spektrum_channels;
  for (int i = 0; i < 12; ++i) {
    checksum += data_ptr[i];
    Serial.write(data_ptr[i]);
  }
  Serial.write(checksum);
}

void sendTelemetry() {
  unsigned long now = millis();
  if (now - last_telemetry_time < 200) {
    return;
  }
  last_telemetry_time = now;

  float voltage = analogRead(A0) * (5.0 / 1023.0) * 4.0;

  uint8_t header[4] = {0xFF, 0xFF, 0x02, 4};
  uint8_t checksum = 0x02 + 4;

  Serial.write(header, 4);

  uint8_t *data_ptr = (uint8_t *)&voltage;
  for (int i = 0; i < 4; ++i) {
    checksum += data_ptr[i];
    Serial.write(data_ptr[i]);
  }
  Serial.write(checksum);
}
