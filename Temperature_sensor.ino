#include <DHT.h>   // Library for DHT temperature sensor

#define DHTPIN 4
#define DHTTYPE DHT22
#define YELLOW_LED 14
#define BUZZER 26

DHT dht(DHTPIN, DHTTYPE);

float baseTemp = 0;              // Baseline temperature
const float TEMP_OFFSET = 0.3;   // Threshold offset for fault detection

int tCount = 0; // Counter to confirm fault (avoid false triggers)

void setup() {
  Serial.begin(115200);
  dht.begin();

  pinMode(YELLOW_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  Serial.println("Calibrating...");

  // Take multiple readings to calculate stable baseline temperature
  for (int i = 0; i < 10; i++) {
    baseTemp += dht.readTemperature();
    delay(500);
  }
  baseTemp /= 10; // Average baseline

  Serial.print("Baseline: ");
  Serial.println(baseTemp);
}

void loop() {

  float temp = dht.readTemperature(); // Read current temperature

  // If sensor fails, use baseline value to avoid NaN issues
  if (isnan(temp)) temp = baseTemp;

  // Check if temperature exceeds threshold consistently
  if (temp > baseTemp + TEMP_OFFSET) tCount++;
  else tCount = 0;

  bool fault = (tCount > 3); // Confirm fault after multiple detections

  digitalWrite(YELLOW_LED, fault); // Turn LED ON if fault detected

  if (fault) {
    // Buzzer alert pattern
    digitalWrite(BUZZER, HIGH); delay(500);
    digitalWrite(BUZZER, LOW);  delay(500);
  } else {
    digitalWrite(BUZZER, LOW);
  }

  // Print temperature for monitoring
  Serial.print("Temp: ");
  Serial.println(temp);

  delay(500);
}