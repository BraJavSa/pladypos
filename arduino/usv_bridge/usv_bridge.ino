uint16_t spektrum_channels[6] = {1024, 1024, 1024, 1024, 1024, 1024};
volatile uint16_t spektrum_rx_count = 0;

unsigned long last_cmd_time = 0;
unsigned long last_telemetry_time = 0;

// Watchdog heartbeat variables
unsigned long last_watchdog_time = 0;
bool watchdog_state = false;

void setup() {
  Serial.begin(115200);
  // Serial1.begin(115200); // Disconnected for control experiment

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

  // Generate a synthetic sweep to simulate stick movement
  static unsigned long last_sweep_time = 0;
  static uint16_t sweep_val = 1000;
  static int sweep_dir = 10;
  if (now - last_sweep_time >= 50) {
    last_sweep_time = now;
    sweep_val += sweep_dir;
    if (sweep_val >= 2000 || sweep_val <= 1000) {
      sweep_dir = -sweep_dir;
    }
    spektrum_channels[0] = sweep_val;
    spektrum_channels[1] = sweep_val + 50;
    spektrum_channels[2] = sweep_val - 50;
  }

  // Throttle sending Spektrum data to PC to 10 Hz (every 100 ms)
  static unsigned long last_spektrum_pc_time = 0;
  if (now - last_spektrum_pc_time >= 100) {
    last_spektrum_pc_time = now;
    uint16_t freq = 90; // Hardcoded mock radio frequency
    sendSpektrumToPC(freq);
  }

  // Watchdog heartbeat (toggles Pin 7 at 1 Hz)
  if (now - last_watchdog_time >= 1000) {
    last_watchdog_time = now;
    watchdog_state = !watchdog_state;
    digitalWrite(7, watchdog_state ? HIGH : LOW);
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
