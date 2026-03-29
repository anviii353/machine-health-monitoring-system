#include <Wire.h>

#define MPU_ADDR 0x68
#define LED 2   // ESP32 LED pin

int16_t ax, ay, az;
float vibration;

float threshold = 15000;  // adjust this later

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  pinMode(LED, OUTPUT);

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.println("System Ready");
}

void loop() {
  // Read accelerometer data
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();

  // Calculate vibration magnitude
  vibration = sqrt(ax * ax + ay * ay + az * az);

  // Print value (for graph)
  Serial.println(vibration);

  // Fault detection
  if (vibration > threshold) {
    digitalWrite(LED, HIGH);   // LED ON → Fault
    Serial.println("FAULT DETECTED");
  } else {
    digitalWrite(LED, LOW);    // LED OFF → Normal
  }

  delay(100);
}