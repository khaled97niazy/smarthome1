// tasks:
// 1- smoke detector
// 2- fire detection
// 3- movement  detection
// 4- detection

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------
// Flame sensor AO -> GPIO34 (ADC1, input-only)
// MQ-2 smoke AO   -> GPIO35 (ADC1, input-only)
// Buzzer signal   -> GPIO25 (passive/piezo buzzer or transistor driver module)
const int FIRE_SENSOR_PIN = 34;
const int MQ2_SENSOR_PIN = 35;
const int BUZZER_PIN = 25;

// Reversed flame sensor: normal ~4095 (no flame), drops toward 0 when flame detected.
const int FIRE_THRESHOLD = 500;

// MQ-2 smoke threshold. MQ-2 needs warm-up (~30-60s) before readings are stable.
// Tune this after observing smoke vs clean-air ADC values.
const int SMOKE_THRESHOLD = 2000;

// ---------------------------------------------------------------------------
// Siren (ESP32 LEDC tone generation, Arduino-ESP32 core 3.x)
// ---------------------------------------------------------------------------
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
  pinMode(MQ2_SENSOR_PIN, INPUT);

  Serial.begin(115200);
  delay(200);
  Serial.println("Smart home detection started");
  Serial.print("Flame sensor GPIO");
  Serial.print(FIRE_SENSOR_PIN);
  Serial.print(", smoke MQ-2 GPIO");
  Serial.print(MQ2_SENSOR_PIN);
  Serial.print(", buzzer GPIO");
  Serial.print(BUZZER_PIN);
  Serial.print(", fire threshold <= ");
  Serial.print(FIRE_THRESHOLD);
  Serial.print(", smoke threshold >= ");
  Serial.println(SMOKE_THRESHOLD);
}

void loop() {
  int flameValue = analogRead(FIRE_SENSOR_PIN);
  int smokeValue = analogRead(MQ2_SENSOR_PIN);

  bool fireDetected = flameValue <= FIRE_THRESHOLD;
  bool smokeDetected = smokeValue >= SMOKE_THRESHOLD;
  bool alarm = fireDetected || smokeDetected;

  if (alarm) {
    startSiren();
    updateSiren();
  } else {
    stopSiren();
  }

  unsigned long now = millis();
  if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;
    Serial.print("Flame ADC=");
    Serial.print(flameValue);
    Serial.print(" fire=");
    Serial.print(fireDetected ? "YES" : "NO");
    Serial.print(" | Smoke ADC=");
    Serial.print(smokeValue);
    Serial.print(" smoke=");
    Serial.print(smokeDetected ? "YES" : "NO");
    Serial.print(" sirenHz=");
    Serial.println(alarm ? sirenHz : 0);
  }

  delay(5);
}
