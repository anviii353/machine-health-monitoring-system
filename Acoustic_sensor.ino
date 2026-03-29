#define SOUND_PIN 34   // Analog pin connected to sound sensor
#define GREEN_LED 13
#define BUZZER 26

const int SOUND_THRESHOLD = 1500; // Threshold for abnormal sound level

int sCount = 0; // Counter to confirm fault (reduces noise false triggers)

void setup() {
  Serial.begin(115200);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  Serial.println("Sound Test Ready");
}

void loop() {

  int sound = analogRead(SOUND_PIN); // Read analog sound level

  // Check if sound exceeds threshold consistently
  if (sound > SOUND_THRESHOLD) sCount++;
  else sCount = 0;

  bool fault = (sCount > 3); // Confirm fault after multiple detections

  digitalWrite(GREEN_LED, fault); // Turn LED ON if fault detected

  if (fault) {
    // Short buzzer alert pattern
    for (int i = 0; i < 2; i++) {
      digitalWrite(BUZZER, HIGH); delay(150);
      digitalWrite(BUZZER, LOW);  delay(150);
    }
  } else {
    digitalWrite(BUZZER, LOW);
  }

  // Print sound value for monitoring/debugging
  Serial.print("Sound: ");
  Serial.println(sound);

  delay(200); // Small delay between readings
}