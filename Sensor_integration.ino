#include <Wire.h>
#include <DHT.h>
#include <arduinoFFT.h>
#include <math.h>
#include <WiFi.h>   // ADDED

/******** WIFI ********/
const char* ssid = "Mayur's A35";
const char* password = "mayurpswd22";
WiFiServer server(80);   // ADDED

/******** FFT CONFIG ********/
const uint16_t samples = 64;
const double samplingFrequency = 100;

double vReal[samples];
double vImag[samples];

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
const int SOUND_THRESHOLD = 1500;
const float TEMP_OFFSET = 0.3;
const float FREQ_THRESHOLD = 6.0;

/******** OBJECT ********/
DHT dht(DHTPIN, DHTTYPE);

/******** VARIABLES ********/
int16_t rawX, rawY, rawZ;
float ax, ay, az, magnitude;

/******** BASELINE ********/
float baseTemp = 0;

/******** STABILITY COUNTERS ********/
int sCount = 0, tCount = 0, fCount = 0;

/******** TIMING ********/
unsigned int sampling_period_us;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  dht.begin();

  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Wake MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  sampling_period_us = round(1000000 * (1.0 / samplingFrequency));

  Serial.println("Calibrating temperature...");

  // AUTO BASELINE
  for (int i = 0; i < 10; i++) {
    baseTemp += dht.readTemperature();
    delay(500);
  }
  baseTemp /= 10;

  Serial.print("Baseline Temp: ");
  Serial.println(baseTemp);

  // WIFI CONNECT
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.begin();  // START SERVER

  Serial.println("System Ready");
}

void loop() {

  /******** 1. READ TEMP + SOUND ********/
  float tempC = dht.readTemperature();
  if (isnan(tempC)) tempC = baseTemp;

  int soundLvl = analogRead(SOUND_PIN);

  /******** 2. FFT SAMPLING ********/
  for (int i = 0; i < samples; i++) {

    unsigned long t = micros();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);

    if (Wire.available() == 6) {
      rawX = Wire.read() << 8 | Wire.read();
      rawY = Wire.read() << 8 | Wire.read();
      rawZ = Wire.read() << 8 | Wire.read();
    }

    ax = rawX / 16384.0;
    ay = rawY / 16384.0;
    az = rawZ / 16384.0;

    magnitude = sqrt(ax * ax + ay * ay + az * az);

    vReal[i] = magnitude;
    vImag[i] = 0;

    while (micros() - t < sampling_period_us);
  }

  /******** 3. FFT ********/
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  double peakFreq = FFT.majorPeak();

  /******** 4. SMART FAULT LOGIC ********/
  if (soundLvl > SOUND_THRESHOLD) sCount++; else sCount = 0;
  if (tempC > baseTemp + TEMP_OFFSET) tCount++; else tCount = 0;
  if (peakFreq > FREQ_THRESHOLD) fCount++; else fCount = 0;

  bool sFault = (sCount > 3);
  bool tFault = (tCount > 3);
  bool fFault = (fCount > 3);

  /******** 5. LED ********/
  digitalWrite(GREEN_LED, sFault);
  digitalWrite(YELLOW_LED, tFault);
  digitalWrite(RED_LED, fFault);

  /******** 6. BUZZER ********/
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
    digitalWrite(BUZZER_PIN, LOW);
  }

  /******** 7. SERIAL ********/
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

  /******** 8. WIFI WEBPAGE ********/
  WiFiClient client = server.available();

  if (client) {
    while (!client.available()) delay(1);
    client.readStringUntil('\r');

    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println("Connection: close");
    client.println();

    client.println("<!DOCTYPE html><html>");
    client.println("<head><meta http-equiv='refresh' content='1'></head>");
    client.println("<body><h2>ESP32 Monitoring</h2>");

    client.print("<p>Temp: "); client.print(tempC); client.println("</p>");
    client.print("<p>Base Temp: "); client.print(baseTemp); client.println("</p>");
    client.print("<p>Sound: "); client.print(soundLvl); client.println("</p>");
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

    client.stop();
  }

  delay(200);
}