uint16_t spektrum_channels[6] = {1024, 1024, 1024, 1024, 1024, 1024};
unsigned long last_print_time = 0;
unsigned long last_spektrum_byte_time = 0;
bool spektrum_active = false;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
}

void loop() {
  readSpektrum();

  unsigned long now = millis();
  if (now - last_print_time >= 500) {
    last_print_time = now;
    printRadioInfo();
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

void printRadioInfo() {
  Serial.print("Radio Active: ");
  if (spektrum_active && (millis() - last_spektrum_byte_time < 1000)) {
    Serial.print("YES  | Channels: ");
    for (int i = 0; i < 6; ++i) {
      Serial.print("Ch");
      Serial.print(i);
      Serial.print(":");
      Serial.print(spektrum_channels[i]);
      Serial.print("  ");
    }
    Serial.println();
  } else {
    Serial.println("NO (No signal on Serial1 RX)");
  }
}
