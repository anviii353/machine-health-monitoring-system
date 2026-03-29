#include <Wire.h>          // I2C communication (MPU6050)
#include <DHT.h>           // DHT22 temperature sensor
#include <arduinoFFT.h>    // FFT for vibration analysis
#include <math.h>          // Math functions

/******** FFT CONFIG ********/
const uint16_t samples = 64;          // Number of samples for FFT
const double samplingFrequency = 100; // Sampling frequency (Hz)

double vReal[samples]; // Real signal values
double vImag[samples]; // Imaginary part (initially 0)

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

/******** PINS ********/
#define DHTPIN 4
#define DHTTYPE DHT22
#define SOUND_PIN 34      // Analog sound input
#define MPU_ADDR 0x68     // MPU6050 I2C address

#define GREEN_LED 13
#define YELLOW_LED 14
#define RED_LED 27
#define BUZZER_PIN 26

/******** THRESHOLDS ********/
const int SOUND_THRESHOLD = 1500;   // Sound trigger level
const float TEMP_OFFSET = 0.3;      // Temp deviation from baseline
const float FREQ_THRESHOLD = 6.0;   // Vibration frequency threshold

/******** OBJECT ********/
DHT dht(DHTPIN, DHTTYPE);

/******** VARIABLES ********/
int16_t rawX, rawY, rawZ; // Raw accelerometer values
float ax, ay, az, magnitude;

/******** BASELINE ********/
float baseTemp = 0;  // Reference temperature (room)

/******** STABILITY COUNTERS ********/
int sCount = 0, tCount = 0, fCount = 0; // Avoid false triggers

/******** TIMING ********/
unsigned int sampling_period_us;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);  // SDA, SCL pins

  dht.begin();

  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Wake MPU6050 (disable sleep mode)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Calculate sampling interval for FFT
  sampling_period_us = round(1000000 * (1.0 / samplingFrequency));

  Serial.println("Calibrating temperature...");

  // Take average of 10 readings to get baseline temp
  for (int i = 0; i < 10; i++) {
    baseTemp += dht.readTemperature();
    delay(500);
  }
  baseTemp /= 10;

  Serial.print("Baseline Temp: ");
  Serial.println(baseTemp);

  Serial.println("System Ready");
}

void loop() {

  /******** 1. READ TEMP + SOUND ********/
  float tempC = dht.readTemperature();   // Read temperature
  if (isnan(tempC)) tempC = baseTemp;    // Handle sensor error

  int soundLvl = analogRead(SOUND_PIN);  // Read sound level

  /******** 2. FFT SAMPLING ********/
  for (int i = 0; i < samples; i++) {

    unsigned long t = micros(); // Start time for sampling

    // Read accelerometer data
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);

    if (Wire.available() == 6) {
      rawX = Wire.read() << 8 | Wire.read();
      rawY = Wire.read() << 8 | Wire.read();
      rawZ = Wire.read() << 8 | Wire.read();
    }

    // Convert raw values to g-force
    ax = rawX / 16384.0;
    ay = rawY / 16384.0;
    az = rawZ / 16384.0;

    // Calculate total vibration magnitude
    magnitude = sqrt(ax * ax + ay * ay + az * az);

    vReal[i] = magnitude; // Store sample
    vImag[i] = 0;

    // Maintain constant sampling rate
    while (micros() - t < sampling_period_us);
  }

  /******** 3. FFT ********/
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD); // Reduce noise
  FFT.compute(FFT_FORWARD);                        // Perform FFT
  FFT.complexToMagnitude();                        // Convert to magnitude

  double peakFreq = FFT.majorPeak(); // Get dominant frequency

  /******** 4. SMART FAULT LOGIC ********/
  // Count consecutive threshold crossings to avoid false alarms
  if (soundLvl > SOUND_THRESHOLD) sCount++; else sCount = 0;
  if (tempC > baseTemp + TEMP_OFFSET) tCount++; else tCount = 0;
  if (peakFreq > FREQ_THRESHOLD) fCount++; else fCount = 0;

  bool sFault = (sCount > 3);
  bool tFault = (tCount > 3);
  bool fFault = (fCount > 3);

  /******** 5. LED ********/
  digitalWrite(GREEN_LED, sFault);   // Sound fault
  digitalWrite(YELLOW_LED, tFault);  // Temp fault
  digitalWrite(RED_LED, fFault);     // Vibration fault

  /******** 6. BUZZER ********/
  // Different patterns for different faults
  if (fFault) {
    digitalWrite(BUZZER_PIN, HIGH); delay(100);
    digitalWrite(BUZZER_PIN, LOW);  delay(100);
  }
  else if (tFault) {
    digitalWrite(BUZZER_PIN, HIGH); delay(500);
    digitalWrite(BUZZER_PIN, LOW);  delay(500);
  }
  else if (sFault) {
    for (int i = 0; i < 2; i++) {
      digitalWrite(BUZZER_PIN, HIGH); delay(150);
      digitalWrite(BUZZER_PIN, LOW);  delay(150);
    }
    delay(300);
  }
  else {
    digitalWrite(BUZZER_PIN, LOW); // No fault
  }

  /******** 7. SERIAL OUTPUT ********/
  Serial.println("------ SYSTEM STATUS ------");

  Serial.print("Temp: ");
  Serial.print(tempC);
  Serial.print(" (Base: ");
  Serial.print(baseTemp);
  Serial.println(")");

  Serial.print("Sound: ");
  Serial.println(soundLvl);

  Serial.print("Freq: ");
  Serial.println(peakFreq);

  Serial.print("Status: ");

  if (!sFault && !tFault && !fFault) {
    Serial.println("NORMAL");
  } else {
    if (fFault) Serial.print("VIBRATION ");
    if (tFault) Serial.print("TEMP ");
    if (sFault) Serial.print("SOUND ");
    Serial.println();
  }

  Serial.println("----------------------------");

  delay(200); // Small delay for stability
}