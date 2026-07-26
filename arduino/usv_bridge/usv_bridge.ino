unsigned long last_cmd_time = 0;
unsigned long last_telemetry_time = 0;

// Watchdog heartbeat variables
unsigned long last_watchdog_time = 0;
bool watchdog_state = false;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  // Enable the Murata UWE-12/10 DC-DC converter (Negative logic: LOW = ON)
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);

  // Initialize watchdog pin
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
}

void loop() {
  readSerialPC();
  sendTelemetry();

  // Watchdog heartbeat (toggles Pin 7 at 1 Hz)
  unsigned long now = millis();
  if (now - last_watchdog_time >= 1000) {
    last_watchdog_time = now;
    watchdog_state = !watchdog_state;
    digitalWrite(7, watchdog_state ? HIGH : LOW);
  }

  // Send a counter from 1 to 10000 at 10 Hz
  static uint16_t counter = 1;
  static unsigned long last_counter_time = 0;
  if (now - last_counter_time >= 100) {
    last_counter_time = now;
    sendCounterToPC(counter);
    counter++;
    if (counter > 10000) {
      counter = 1;
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

void sendCounterToPC(uint16_t val) {
  uint8_t header[4] = {0xFF, 0xFF, 0x03, 2};
  uint8_t checksum = 0x03 + 2;

  Serial.write(header, 4);

  uint8_t *data_ptr = (uint8_t *)&val;
  checksum += data_ptr[0] + data_ptr[1];
  Serial.write(data_ptr[0]);
  Serial.write(data_ptr[1]);
  Serial.write(checksum);
}

void sendTelemetry() {
  unsigned long now = millis();
  if (now - last_telemetry_time < 200) {
    return;
  }
  last_telemetry_time = now;

  // Read A0 (battery voltage sensor)
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
