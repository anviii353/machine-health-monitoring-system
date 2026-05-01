#include <Wire.h>         // I2C communication for the MPU6050
#include <DHT.h>          // Library for the DHT22 Temperature/Humidity sensor
#include <arduinoFFT.h>   // Library to perform Fast Fourier Transform math
#include <math.h>         // Standard math library for logarithms (Decibel calc)
#include <WiFi.h>         // ESP32 WiFi library for the local dashboard

/******** WIFI CONFIG ********/
const char* ssid = "WIFI_name";
const char* password = "WIFI_pswd";
WiFiServer server(80); 

/******** FFT CONFIG ********/
const uint16_t samples = 64;           
const double samplingFrequency = 100;  

double vReal[samples]; 
double vImag[samples]; 
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

/******** PIN DEFINITIONS ********/
#define DHTPIN 4
#define DHTTYPE DHT22
#define SOUND_PIN 34   
#define MPU_ADDR 0x68  

#define GREEN_LED 13
#define YELLOW_LED 14
#define RED_LED 27
#define BUZZER_PIN 26

/******** FAULT THRESHOLDS ********/
const float TEMP_OFFSET = 0.2;        
const float FREQ_THRESHOLD = 7.40;    
const float SOUND_DB_THRESHOLD = 37.20; 

/******** SENSOR OBJECTS & TIMING ********/
DHT dht(DHTPIN, DHTTYPE);
unsigned int sampling_period_us; 

/******** GLOBAL DATA VARIABLES ********/
// These are stored globally so all our independent functions can share them easily
int16_t rawX, rawY, rawZ;
float ax, ay, az, magnitude;

float baseTemp = 0; 
float tempC = 0;
float soundDB = 0;
double peakFreq = 0;

int sCount = 0, tCount = 0, fCount = 0; 
bool sFault = false, tFault = false, fFault = false;


// ==========================================
// 1. SETUP ROUTINE (Runs Once)
// ==========================================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); 
  dht.begin();

  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);

  sampling_period_us = round(1000000 * (1.0 / samplingFrequency));

  Serial.println("Calibrating temperature...");
  for (int i = 0; i < 10; i++) {
    baseTemp += dht.readTemperature();
    delay(500);
  }
  baseTemp /= 10;
  Serial.print("Baseline Temp: "); Serial.println(baseTemp);

  // Initialize Wi-Fi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 20) {
    delay(500); Serial.print("."); wifiAttempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.print("IP Address: "); Serial.println(WiFi.localIP()); 
  } else {
    Serial.println("\nWiFi Failed to Connect. Running offline.");
  }

  server.begin(); 
  Serial.println("System Ready");
}

// ==========================================
// 2. MAIN LOOP (The "Table of Contents")
// ==========================================
void loop() {
  processTemperature();
  processSound();
  processVibrationFFT();
  evaluateFaults();
  printTelemetry();
  updateWebDashboard();
  
  delay(200); // Main loop pacing
}

// ==========================================
// 3. MODULAR FUNCTIONS (The Heavy Lifting)
// ==========================================

void processTemperature() {
  tempC = dht.readTemperature();
  if (isnan(tempC)) tempC = baseTemp; // Fallback to prevent math crashes
}

void processSound() {
  int soundRaw = analogRead(SOUND_PIN); 
  float voltage = soundRaw * (3.3 / 4095.0);
  float reference = 0.02;  
  soundDB = 20 * log10(voltage / reference);
  if (soundDB < 0) soundDB = 0; 
}

void processVibrationFFT() {
  // 1. Collect Kinetic Samples
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

    ax = rawX / 16384.0; ay = rawY / 16384.0; az = rawZ / 16384.0;
    magnitude = sqrt(ax * ax + ay * ay + az * az) - 1.0; // Subtract 1G gravity

    vReal[i] = magnitude; 
    vImag[i] = 0;

    while (micros() - t < sampling_period_us);
  }

  // 2. Compute FFT
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();
  peakFreq = FFT.majorPeak();
}

void evaluateFaults() {
  // 1. Stability Counters
  if (soundDB > SOUND_DB_THRESHOLD) sCount++; else sCount = 0;
  if (tempC > baseTemp + TEMP_OFFSET) tCount++; else tCount = 0;
  if (peakFreq > FREQ_THRESHOLD) fCount++; else fCount = 0;

  sFault = (sCount > 3);
  tFault = (tCount > 3);
  fFault = (fCount > 3);

  // 2. LED Triggers
  digitalWrite(GREEN_LED, sFault);   
  digitalWrite(YELLOW_LED, tFault);  
  digitalWrite(RED_LED, fFault);     

  // 3. Buzzer Logic
  if (fFault) {
    digitalWrite(BUZZER_PIN, HIGH); delay(100); digitalWrite(BUZZER_PIN, LOW); delay(100);
  }
  else if (tFault) {
    digitalWrite(BUZZER_PIN, HIGH); delay(500); digitalWrite(BUZZER_PIN, LOW); delay(500);
  }
  else if (sFault) {
    for (int i = 0; i < 2; i++) {
      digitalWrite(BUZZER_PIN, HIGH); delay(150); digitalWrite(BUZZER_PIN, LOW); delay(150);
    }
    delay(300);
  }
  else {
    digitalWrite(BUZZER_PIN, LOW); 
  }
}

void printTelemetry() {
  Serial.println("------ SYSTEM STATUS ------");
  Serial.print("Temp: "); Serial.print(tempC); Serial.print(" (Base: "); Serial.print(baseTemp); Serial.println(")");
  Serial.print("Sound (dB): "); Serial.println(soundDB);
  Serial.print("Freq: "); Serial.println(peakFreq);
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
}

void updateWebDashboard() {
  WiFiClient client = server.available(); 

  if (client) {
    int timeout = 0;
    while (!client.available() && timeout < 1000) {
      delay(1);
      timeout++;
    }

    if (client.available()) {
      client.readStringUntil('\r'); 

      client.println("HTTP/1.1 200 OK");
      client.println("Content-type:text/html");
      client.println("Connection: close");
      client.println();

      client.println("<!DOCTYPE html><html>");
      client.println("<head><meta http-equiv='refresh' content='1'>"); 
      client.println("<style>body{font-family: Arial, sans-serif; text-align: center; margin-top: 50px;}</style>");
      client.println("</head>");
      client.println("<body><h2>ESP32 PreSense Node</h2>");

      client.print("<h3>Temp: "); client.print(tempC); client.println(" &deg;C</h3>");
      client.print("<p>Base Temp: "); client.print(baseTemp); client.println(" &deg;C</p>");
      client.print("<h3>Sound: "); client.print(soundDB); client.println(" dB</h3>");
      client.print("<h3>Freq: "); client.print(peakFreq); client.println(" Hz</h3>");

      client.print("<h2>Status: ");
      if (!sFault && !tFault && !fFault) client.print("<span style='color:green;'>NORMAL</span>");
      else {
        client.print("<span style='color:red;'>");
        if (fFault) client.print("VIBRATION FAULT ");
        if (tFault) client.print("TEMP FAULT ");
        if (sFault) client.print("SOUND FAULT ");
        client.print("</span>");
      }
      client.println("</h2>");
      client.println("</body></html>");
    }
    client.stop();
  }
}
