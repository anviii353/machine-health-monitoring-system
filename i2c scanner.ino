#include <Wire.h>

void setup() {
  Wire.begin(21, 22);
  Serial.begin(115200);
  delay(1000);

  Serial.println("Scanning for I2C devices...");

  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at address: 0x");
      Serial.println(i, HEX);
    }
  }

  Serial.println("Scan complete.");
}

void loop() {}