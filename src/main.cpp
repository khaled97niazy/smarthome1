// tasks:
// 1- smoke detector
// 2- fire detection
// 3- movement  detection
// 4- detection

#include <Arduino.h>

// Fire detection (analog flame sensor) + buzzer alarm
// Wiring (ESP32 dev board):
// - Flame sensor AO -> GPIO34 (ADC1, input-only)
// - Buzzer signal    -> GPIO25 (output; use a passive/piezo buzzer or a
//                     transistor driver module for best siren volume)
const int FIRE_SENSOR_PIN = 34;
const int BUZZER_PIN = 25;

// Reversed sensor: normal ~4095 (no flame), drops toward 0 when flame detected.
// Fire is detected when reading is BELOW this threshold.
const int FIRE_THRESHOLD = 500;

// Siren pattern (ESP32 LEDC tone generation, Arduino-ESP32 core 3.x).
// Works best with a passive buzzer. Active buzzers may only click/beep.
const int SIREN_RESOLUTION_BITS = 8;
const int SIREN_LOW_HZ = 700;
const int SIREN_HIGH_HZ = 1800;
const int SIREN_STEP_HZ = 60;
const unsigned long SIREN_STEP_MS = 20;
const unsigned long PRINT_INTERVAL_MS = 250;

int sirenHz = SIREN_LOW_HZ;
int sirenDirection = 1;
unsigned long lastSirenMs = 0;
unsigned long lastPrintMs = 0;
bool sirenActive = false;

void startSiren() {
  if (sirenActive) return;
  ledcAttach(BUZZER_PIN, sirenHz, SIREN_RESOLUTION_BITS);
  ledcWrite(BUZZER_PIN, 128); // 50% duty cycle
  sirenActive = true;
}

void stopSiren() {
  if (!sirenActive) return;
  ledcWrite(BUZZER_PIN, 0);
  ledcDetach(BUZZER_PIN);
  digitalWrite(BUZZER_PIN, LOW);
  sirenActive = false;
}

void updateSiren() {
  unsigned long now = millis();
  if (now - lastSirenMs < SIREN_STEP_MS) return;
  lastSirenMs = now;

  sirenHz += sirenDirection * SIREN_STEP_HZ;
  if (sirenHz >= SIREN_HIGH_HZ) {
    sirenHz = SIREN_HIGH_HZ;
    sirenDirection = -1;
  } else if (sirenHz <= SIREN_LOW_HZ) {
    sirenHz = SIREN_LOW_HZ;
    sirenDirection = 1;
  }
  ledcWriteTone(BUZZER_PIN, sirenHz);
}

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(FIRE_SENSOR_PIN, INPUT);

  Serial.begin(115200);
  delay(200);
  Serial.println("Fire detection started");
  Serial.print("Sensor pin GPIO");
  Serial.print(FIRE_SENSOR_PIN);
  Serial.print(", buzzer pin GPIO");
  Serial.print(BUZZER_PIN);
  Serial.print(", threshold <= ");
  Serial.println(FIRE_THRESHOLD);
}

void loop() {
  int sensorValue = analogRead(FIRE_SENSOR_PIN);
  bool fireDetected = sensorValue <= FIRE_THRESHOLD;

  if (fireDetected) {
    startSiren();
    updateSiren();
  } else {
    stopSiren();
  }

  unsigned long now = millis();
  if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;
    Serial.print("Flame ADC=");
    Serial.print(sensorValue);
    Serial.print(" fire=");
    Serial.print(fireDetected ? "YES" : "NO");
    Serial.print(" sirenHz=");
    Serial.println(fireDetected ? sirenHz : 0);
  }

  delay(5);
}
