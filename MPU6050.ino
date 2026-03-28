#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);  // SDA, SCL for ESP32

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!");

  // Optional: Set accelerometer range
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // Optional: Set filter bandwidth
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("Accel X: ");
  Serial.print(a.acceleration.x);
  Serial.print(" m/s^2, ");

  Serial.print("Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(" m/s^2, ");

  Serial.print("Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  delay(200);  // adjust for faster/slower readings
}