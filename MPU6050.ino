#include <Wire.h>          // I2C communication
#include <arduinoFFT.h>   // FFT library
#include <math.h>         // Math functions

#define MPU_ADDR 0x68     // MPU6050 I2C address
#define RED_LED 27
#define BUZZER 26

const uint16_t samples = 64;          // Number of samples for FFT
const double samplingFrequency = 100; // Sampling rate (Hz)

double vReal[samples]; // Real part of signal
double vImag[samples]; // Imaginary part (set to 0)

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

int16_t rawX, rawY, rawZ; // Raw accelerometer values
float ax, ay, az, magnitude;

const float FREQ_THRESHOLD = 6.0; // Fault threshold frequency

int fCount = 0; // Counter to confirm fault (avoid noise)
unsigned int sampling_period_us;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA = 21, SCL = 22

  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  // Wake up MPU6050 (disable sleep mode)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Calculate sampling interval
  sampling_period_us = round(1000000 * (1.0 / samplingFrequency));

  Serial.println("Vibration Test Ready");
}

void loop() {

  // Collect samples for FFT
  for (int i = 0; i < samples; i++) {
    unsigned long t = micros(); // Start time for consistent sampling

    // Read accelerometer data (X, Y, Z)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // Starting register for accel data
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);

    if (Wire.available() == 6) {
      rawX = Wire.read() << 8 | Wire.read();
      rawY = Wire.read() << 8 | Wire.read();
      rawZ = Wire.read() << 8 | Wire.read();
    }

    // Convert raw values to g (±2g range)
    ax = rawX / 16384.0;
    ay = rawY / 16384.0;
    az = rawZ / 16384.0;

    // Compute total vibration magnitude
    magnitude = sqrt(ax * ax + ay * ay + az * az);

    vReal[i] = magnitude; // Store signal
    vImag[i] = 0;         // Imaginary part is zero

    // Maintain fixed sampling rate
    while (micros() - t < sampling_period_us);
  }

  // Apply window to reduce spectral leakage
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);

  // Perform FFT
  FFT.compute(FFT_FORWARD);

  // Convert complex values to magnitude
  FFT.complexToMagnitude();

  // Find dominant frequency
  double freq = FFT.majorPeak();

  // Check if frequency crosses threshold consistently
  if (freq > FREQ_THRESHOLD) fCount++;
  else fCount = 0;

  bool fault = (fCount > 3); // Confirm fault after multiple detections

  digitalWrite(RED_LED, fault); // Turn LED ON if fault

  if (fault) {
    digitalWrite(BUZZER, HIGH); delay(100);
    digitalWrite(BUZZER, LOW);  delay(100);
  } else {
    digitalWrite(BUZZER, LOW);
  }

  // Print detected frequency
  Serial.print("Freq: ");
  Serial.println(freq);

  delay(200);
}