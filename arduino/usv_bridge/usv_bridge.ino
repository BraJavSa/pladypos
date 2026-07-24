#include <Servo.h>

const int DRIVER_NUM = 4;
uint8_t motorPWM[DRIVER_NUM] = {13, 12, 11, 10};
Servo motors[DRIVER_NUM];

uint16_t spektrum_channels[6] = {1024, 1024, 1024, 1024, 1024, 1024};
unsigned long last_print_time = 0;
unsigned long last_spektrum_byte_time = 0;
bool spektrum_active = false;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  for (int i = 0; i < DRIVER_NUM; ++i) {
    motors[i].attach(motorPWM[i]);
    motors[i].writeMicroseconds(1500);
  }
}

void loop() {
  readSpektrum();
  readSerialCommands();

  unsigned long now = millis();
  if (now - last_print_time >= 1000) {
    last_print_time = now;
    printDebugInfo();
  }
}

void readSpektrum() {
  static uint8_t packet[16];
  static uint8_t packet_idx = 0;

  while (Serial1.available() > 0) {
    unsigned long now = millis();
    if (now - last_spektrum_byte_time > 5) {
      packet_idx = 0;
    }
    last_spektrum_byte_time = now;

    packet[packet_idx++] = Serial1.read();

    if (packet_idx == 16) {
      spektrum_active = true;
      for (int i = 1; i < 8; ++i) {
        uint8_t b1 = packet[2 * i];
        uint8_t b2 = packet[2 * i + 1];
        uint8_t chan = (b1 >> 3) & 0x0F;
        uint16_t val = ((b1 & 0x07) << 8) | b2;
        if (chan < 6) {
          spektrum_channels[chan] = val;
        }
      }
      packet_idx = 0;
    }
  }
}

void readSerialCommands() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      // Expected command format: "m0 1600" or "m2 1450"
      if (cmd.startsWith("m") && cmd.length() >= 6) {
        int motor_idx = cmd.substring(1, 2).toInt();
        int pwm_val = cmd.substring(3).toInt();
        if (motor_idx >= 0 && motor_idx < DRIVER_NUM) {
          if (pwm_val >= 1000 && pwm_val <= 2000) {
            motors[motor_idx].writeMicroseconds(pwm_val);
            Serial.print("SUCCESS: Set motor ");
            Serial.print(motor_idx);
            Serial.print(" (pin ");
            Serial.print(motorPWM[motor_idx]);
            Serial.print(" (pin ");
            Serial.print(motorPWM[motor_idx]);
            Serial.print(") to ");
            Serial.println(pwm_val);
          } else {
            Serial.println("ERROR: PWM must be between 1000 and 2000");
          }
        } else {
          Serial.println("ERROR: Motor index must be between 0 and 3");
        }
      } else {
        Serial.println("ERROR: Unknown command. Use 'm<idx> <pwm>' (e.g. 'm0 1600')");
      }
    }
  }
}

void printDebugInfo() {
  Serial.println("==================================================");
  
  // 1. Spektrum Radio State
  Serial.print("Radio Active: ");
  if (spektrum_active && (millis() - last_spektrum_byte_time < 1000)) {
    Serial.println("YES");
  } else {
    Serial.println("NO (No signal)");
  }
  Serial.print("Radio Channels: ");
  for (int i = 0; i < 6; ++i) {
    Serial.print("Ch");
    Serial.print(i);
    Serial.print(":");
    Serial.print(spektrum_channels[i]);
    Serial.print("  ");
  }
  Serial.println();

  // 2. Analog Pins (for voltage sensing)
  Serial.print("Analog Pins: ");
  Serial.print("A0:"); Serial.print(analogRead(A0)); Serial.print("  ");
  Serial.print("A1:"); Serial.print(analogRead(A1)); Serial.print("  ");
  Serial.print("A2:"); Serial.print(analogRead(A2)); Serial.print("  ");
  Serial.print("A3:"); Serial.print(analogRead(A3)); Serial.print("  ");
  Serial.print("A4:"); Serial.print(analogRead(A4)); Serial.print("  ");
  Serial.print("A5:"); Serial.print(analogRead(A5)); Serial.print("  ");
  Serial.print("A6:"); Serial.print(analogRead(A6)); Serial.print("  ");
  Serial.print("A7:"); Serial.print(analogRead(A7)); Serial.print("  ");
  Serial.print("A8:"); Serial.print(analogRead(A8)); Serial.print("  ");
  Serial.print("A9:"); Serial.print(analogRead(A9)); Serial.print("  ");
  Serial.print("A10:"); Serial.print(analogRead(A10)); Serial.print("  ");
  Serial.print("A11:"); Serial.println(analogRead(A11));

  // 3. Servo Outputs Info
  Serial.println("Motors (Pins: 13, 12, 11, 10). To test, type e.g.: 'm0 1600'");
}
