#include <Wire.h>        // I2C communication (MPU6050)
#include <DHT.h>         // DHT22 temperature sensor
#include <arduinoFFT.h>  // FFT for vibration analysis
#include <math.h>        // Math functions
#include <WiFi.h>        // WiFi communication

/******** WIFI ********/
const char* ssid = "Mayur's A35";
const char* password = "mayurpswd22";
WiFiServer server(80);   // Web server on port 80

/******** FFT CONFIG ********/
const uint16_t samples = 64;          // Number of samples
const double samplingFrequency = 100; // Sampling frequency (Hz)

double vReal[samples]; // Real values
double vImag[samples]; // Imaginary values

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

/******** PINS ********/
#define DHTPIN 4
#define DHTTYPE DHT22
#define SOUND_PIN 34
#define MPU_ADDR 0x68

#define GREEN_LED 13
#define YELLOW_LED 14
#define RED_LED 27
#define BUZZER_PIN 26

/******** THRESHOLDS ********/
const float TEMP_OFFSET = 0.2;       // Allowed temp deviation
const float FREQ_THRESHOLD = 7.0;    // Vibration threshold
const float SOUND_DB_THRESHOLD = 35.0; // Sound threshold in dB

/******** OBJECT ********/
DHT dht(DHTPIN, DHTTYPE);

/******** VARIABLES ********/
int16_t rawX, rawY, rawZ;  // Raw accelerometer data
float ax, ay, az, magnitude;

/******** BASELINE ********/
float baseTemp = 0;  // Reference temperature

/******** STABILITY COUNTERS ********/
int sCount = 0, tCount = 0, fCount = 0; // Avoid false triggers

/******** TIMING ********/
unsigned int sampling_period_us;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA, SCL

  dht.begin();

  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Wake MPU6050 (disable sleep)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Sampling interval for FFT
  sampling_period_us = round(1000000 * (1.0 / samplingFrequency));

  Serial.println("Calibrating temperature...");

  // Take average temp as baseline
  for (int i = 0; i < 10; i++) {
    baseTemp += dht.readTemperature();
    delay(500);
  }
  baseTemp /= 10;

  Serial.print("Baseline Temp: ");
  Serial.println(baseTemp);

  // Connect to WiFi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.begin(); // Start web server

  Serial.println("System Ready");
}

void loop() {

  /******** 1. READ TEMPERATURE ********/
  float tempC = dht.readTemperature();  // Get temperature
  if (isnan(tempC)) tempC = baseTemp;   // Handle error

  /******** 2. SOUND (dB CONVERSION) ********/
  int soundRaw = analogRead(SOUND_PIN); // Raw ADC value

  float voltage = soundRaw * (3.3 / 4095.0); // Convert to voltage

  float reference = 0.02;  // Reference voltage
  float soundDB = 20 * log10(voltage / reference); // Convert to dB

  if (soundDB < 0) soundDB = 0; // Avoid negative values

  /******** 3. FFT SAMPLING ********/
  for (int i = 0; i < samples; i++) {

    unsigned long t = micros(); // Time for sampling control

    // Read accelerometer (MPU6050)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);

    if (Wire.available() == 6) {
      rawX = Wire.read() << 8 | Wire.read();
      rawY = Wire.read() << 8 | Wire.read();
      rawZ = Wire.read() << 8 | Wire.read();
    }

    // Convert to g-force
    ax = rawX / 16384.0;
    ay = rawY / 16384.0;
    az = rawZ / 16384.0;

    // Total vibration magnitude
    magnitude = sqrt(ax * ax + ay * ay + az * az);

    vReal[i] = magnitude;
    vImag[i] = 0;

    // Maintain constant sampling rate
    while (micros() - t < sampling_period_us);
  }

  /******** 4. FFT PROCESS ********/
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD); // Reduce noise
  FFT.compute(FFT_FORWARD);                        // Perform FFT
  FFT.complexToMagnitude();                        // Get magnitude

  double peakFreq = FFT.majorPeak(); // Dominant frequency

  /******** 5. FAULT LOGIC ********/
  // Use counters to confirm fault (avoid noise spikes)
  if (soundDB > SOUND_DB_THRESHOLD) sCount++; else sCount = 0;
  if (tempC > baseTemp + TEMP_OFFSET) tCount++; else tCount = 0;
  if (peakFreq > FREQ_THRESHOLD) fCount++; else fCount = 0;

  bool sFault = (sCount > 3);
  bool tFault = (tCount > 3);
  bool fFault = (fCount > 3);

  /******** 6. LED INDICATION ********/
  digitalWrite(GREEN_LED, sFault);   // Sound fault
  digitalWrite(YELLOW_LED, tFault);  // Temp fault
  digitalWrite(RED_LED, fFault);     // Vibration fault

  /******** 7. BUZZER ALERT ********/
  if (fFault) {  // Fast beep for vibration
    digitalWrite(BUZZER_PIN, HIGH); delay(100);
    digitalWrite(BUZZER_PIN, LOW);  delay(100);
  }
  else if (tFault) { // Slow beep for temp
    digitalWrite(BUZZER_PIN, HIGH); delay(500);
    digitalWrite(BUZZER_PIN, LOW);  delay(500);
  }
  else if (sFault) { // Double beep for sound
    for (int i = 0; i < 2; i++) {
      digitalWrite(BUZZER_PIN, HIGH); delay(150);
      digitalWrite(BUZZER_PIN, LOW);  delay(150);
    }
    delay(300);
  }
  else {
    digitalWrite(BUZZER_PIN, LOW); // No fault
  }

  /******** 8. SERIAL MONITOR ********/
  Serial.println("------ SYSTEM STATUS ------");

  Serial.print("Temp: ");
  Serial.print(tempC);
  Serial.print(" (Base: ");
  Serial.print(baseTemp);
  Serial.println(")");

  Serial.print("Sound (dB): ");
  Serial.println(soundDB);

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

  /******** 9. WIFI WEBPAGE ********/
  WiFiClient client = server.available(); // Check client

  if (client) {
    while (!client.available()) delay(1);
    client.readStringUntil('\r'); // Read request

    // Send webpage
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println("Connection: close");
    client.println();

    client.println("<!DOCTYPE html><html>");
    client.println("<head><meta http-equiv='refresh' content='1'></head>");
    client.println("<body><h2>ESP32 Monitoring</h2>");

    client.print("<p>Temp: "); client.print(tempC); client.println("</p>");
    client.print("<p>Base Temp: "); client.print(baseTemp); client.println("</p>");
    client.print("<p>Sound (dB): "); client.print(soundDB); client.println("</p>");
    client.print("<p>Freq: "); client.print(peakFreq); client.println("</p>");

    client.print("<p>Status: ");
    if (!sFault && !tFault && !fFault) client.print("NORMAL");
    else {
      if (fFault) client.print("VIBRATION ");
      if (tFault) client.print("TEMP ");
      if (sFault) client.print("SOUND ");
    }
    client.println("</p>");

    client.println("</body></html>");

    client.stop(); // Close connection
  }

  delay(200); // Stability delay
}