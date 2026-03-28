#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200); // Make sure your Serial Monitor is set to 115200 baud!
  
  // Wait for serial monitor to open
  while (!Serial) {
    delay(10);
  }

  Serial.println("Initializing MPU6050...");

  // Try to initialize the sensor over I2C
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip. Check your wiring!");
    while (1) {
      delay(10); // Halt the program if the sensor isn't found
    }
  }
  Serial.println("MPU6050 Found and Connected!");

  // Configure the sensor's sensitivity
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // Good for motor vibrations
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);   // Smooths out extreme noise

  delay(100);
}

void loop() {
  // Create variables to hold the sensor data
  sensors_event_t a, g, temp;
  
  // Fetch the data from the sensor
  mpu.getEvent(&a, &g, &temp);

  // Print the Acceleration (Vibration) data 
  // We format it this way specifically for the Arduino Serial Plotter
  Serial.print("Vibration_X:");
  Serial.print(a.acceleration.x);
  Serial.print(",");
  
  Serial.print("Vibration_Y:");
  Serial.print(a.acceleration.y);
  Serial.print(",");
  
  Serial.print("Vibration_Z:");
  Serial.println(a.acceleration.z);

  // Delay for 50 milliseconds (reads 20 times per second)
  delay(50); 
}
