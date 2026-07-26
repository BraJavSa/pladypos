uint16_t spektrum_channels[6] = {1024, 1024, 1024, 1024, 1024, 1024};
volatile uint16_t spektrum_rx_count = 0;

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
  // readSerialPC();    // Disabled for diagnostic test
  readSpektrum();
  // sendTelemetry();   // Disabled for diagnostic test

  // Watchdog heartbeat (toggles Pin 7 at 1 Hz)
  unsigned long now = millis();
  if (now - last_watchdog_time >= 1000) {
    last_watchdog_time = now;
    watchdog_state = !watchdog_state;
    digitalWrite(7, watchdog_state ? HIGH : LOW);
  }
}

void readSpektrum() {
  static uint8_t packet[16];
  static uint8_t packet_idx = 0;
  static unsigned long last_byte_time = 0;

  while (Serial1.available() > 0) {
    unsigned long now = millis();
    if (now - last_byte_time > 8) {
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
      spektrum_rx_count++;
      packet_idx = 0;
    }
  }

  // Throttle sending Spektrum data to PC to 10 Hz (every 100 ms)
  static unsigned long last_spektrum_pc_time = 0;
  unsigned long now = millis();
  if (now - last_spektrum_pc_time >= 100) {
    last_spektrum_pc_time = now;
    uint16_t freq = spektrum_rx_count * 10;
    spektrum_rx_count = 0;
    sendSpektrumToPC(freq);
  }
}

void sendSpektrumToPC(uint16_t freq) {
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

    uint8_t *freq_ptr = (uint8_t *)&freq;
    checksum += freq_ptr[0] + freq_ptr[1];
    Serial.write(freq_ptr[0]);
    Serial.write(freq_ptr[1]);

    Serial.write(checksum);
  }
}
