// Smart Home Automation & Safety System (ESP32)
// 1- Smoke detector (MQ-2 analog sensor -> buzzer alarm)
// 2- Fire detection (analog flame sensor -> buzzer alarm)
// 3- Motion detection (PIR sensor -> buzzer alarm)
// 4- Ultrasonic gate control (HC-SR04 -> 2 servo motors open to 180 deg)

#include <Arduino.h>
#include <ESP32Servo.h>

// ---------------------------------------------------------------------------
// Sensors & Actuators
// ---------------------------------------------------------------------------
// Flame sensor AO       -> GPIO34 (ADC1, input-only)
// MQ-2 smoke AO         -> GPIO35 (ADC1, input-only)
// PIR motion OUT        -> GPIO27 (digital input, active-HIGH)
// Ultrasonic TRIG       -> GPIO32 (digital output)
// Ultrasonic ECHO       -> GPIO33 (digital input; step down 5V ECHO to 3.3V)
// Buzzer signal         -> GPIO25 (passive/piezo buzzer or transistor driver module)
// Gate servo 1 (left)   -> GPIO18 (ESP32Servo 50 Hz PWM)
// Gate servo 2 (right)  -> GPIO19 (ESP32Servo 50 Hz PWM)
const int FIRE_SENSOR_PIN = 34;
const int MQ2_SENSOR_PIN = 35;
const int PIR_SENSOR_PIN = 27;
const int ULTRASONIC_TRIG_PIN = 32;
const int ULTRASONIC_ECHO_PIN = 33;
const int BUZZER_PIN = 25;
const int GATE_SERVO_1_PIN = 18;
const int GATE_SERVO_2_PIN = 19;

// Reversed flame sensor: normal ~4095 (no flame), drops toward 0 when flame detected.
const int FIRE_THRESHOLD = 500;

// MQ-2 smoke threshold. MQ-2 needs warm-up (~30-60s) before readings are stable.
// Tune this after observing smoke vs clean-air ADC values.
const int SMOKE_THRESHOLD = 2000;

// Ultrasonic gate detection settings.
// Opens the 2-servo gate when an object is detected within GATE_TRIGGER_DISTANCE_CM.
const float GATE_TRIGGER_DISTANCE_CM = 25.0f;
const unsigned long ULTRASONIC_INTERVAL_MS = 60;   // >= 60ms between HC-SR04 triggers
const unsigned long ULTRASONIC_TIMEOUT_US = 20000; // ~3.4m max range timeout (20ms)
const int GATE_CONFIRM_READS = 2;                  // consecutive detections to filter glitches
const unsigned long GATE_HOLD_OPEN_MS = 3000;      // hold gate open after path clears

// ---------------------------------------------------------------------------
// Gate Servos (ESP32Servo 50 Hz PWM, 500-2500 us for MG996R/DS3218 180-deg)
// ---------------------------------------------------------------------------
const int SERVO_PWM_FREQ_HZ = 50;
const int SERVO_PWM_RES_BITS = 16; // Fine 16-bit PWM steps for smooth ramping
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

const int SERVO1_CLOSED_ANGLE = 0;
const int SERVO1_OPEN_ANGLE = 180;
const int SERVO2_CLOSED_ANGLE = 0;
const int SERVO2_OPEN_ANGLE = 180;

// Smooth non-blocking ramp (100 deg/s -> ~1.8s full 0 <-> 180 deg travel)
const int SERVO_STEP_DEG = 2;
const unsigned long SERVO_STEP_INTERVAL_MS = 20;

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

Servo gateServo1;
Servo gateServo2;
int currentServo1Angle = SERVO1_CLOSED_ANGLE;
int currentServo2Angle = SERVO2_CLOSED_ANGLE;
int targetServo1Angle = SERVO1_CLOSED_ANGLE;
int targetServo2Angle = SERVO2_CLOSED_ANGLE;
unsigned long lastServoStepMs = 0;

unsigned long lastUltrasonicMs = 0;
unsigned long lastGateDetectionMs = 0;
int gateConfirmCount = 0;
float lastDistanceCm = -1.0f;
bool gateOpen = false;

int servoAngleToPulseUs(int angle) {
  int clamped = constrain(angle, 0, 180);
  long span = static_cast<long>(SERVO_MAX_US - SERVO_MIN_US);
  return SERVO_MIN_US + static_cast<int>((clamped * span + 90L) / 180L);
}

bool attachGateServo(Servo &servo, int pin, int initialAngle, const char *label) {
  servo.setPeriodHertz(SERVO_PWM_FREQ_HZ);
  servo.attach(pin, SERVO_MIN_US, SERVO_MAX_US);
  if (!servo.attached()) {
    Serial.print("WARNING: Failed to attach ");
    Serial.print(label);
    Serial.print(" on GPIO");
    Serial.println(pin);
    return false;
  }
  servo.setTimerWidth(SERVO_PWM_RES_BITS);
  servo.writeMicroseconds(servoAngleToPulseUs(initialAngle));
  return true;
}

int stepServoAngle(int current, int target, int maxStep) {
  if (current < target) {
    return min(current + maxStep, target);
  }
  if (current > target) {
    return max(current - maxStep, target);
  }
  return current;
}

void setGateOpen(bool open) {
  if (gateOpen == open) return;
  gateOpen = open;
  targetServo1Angle = gateOpen ? SERVO1_OPEN_ANGLE : SERVO1_CLOSED_ANGLE;
  targetServo2Angle = gateOpen ? SERVO2_OPEN_ANGLE : SERVO2_CLOSED_ANGLE;
}

void updateGateServos(unsigned long now) {
  if (currentServo1Angle == targetServo1Angle &&
      currentServo2Angle == targetServo2Angle) {
    return;
  }
  if (now - lastServoStepMs < SERVO_STEP_INTERVAL_MS) {
    return;
  }
  lastServoStepMs = now;

  int nextAngle1 = stepServoAngle(currentServo1Angle, targetServo1Angle, SERVO_STEP_DEG);
  int nextAngle2 = stepServoAngle(currentServo2Angle, targetServo2Angle, SERVO_STEP_DEG);

  if (nextAngle1 != currentServo1Angle) {
    currentServo1Angle = nextAngle1;
    if (gateServo1.attached()) {
      gateServo1.writeMicroseconds(servoAngleToPulseUs(currentServo1Angle));
    }
  }
  if (nextAngle2 != currentServo2Angle) {
    currentServo2Angle = nextAngle2;
    if (gateServo2.attached()) {
      gateServo2.writeMicroseconds(servoAngleToPulseUs(currentServo2Angle));
    }
  }
}

float readUltrasonicCm() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  unsigned long durationUs = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
  if (durationUs == 0) {
    return -1.0f; // Out of range / no echo received
  }
  return (durationUs * 0.0343f) / 2.0f;
}

void updateGate(unsigned long now) {
  if (now - lastUltrasonicMs >= ULTRASONIC_INTERVAL_MS) {
    lastUltrasonicMs = now;
    lastDistanceCm = readUltrasonicCm();

    bool objectNear = (lastDistanceCm > 0.0f && lastDistanceCm <= GATE_TRIGGER_DISTANCE_CM);
    if (objectNear) {
      if (gateConfirmCount < GATE_CONFIRM_READS) {
        gateConfirmCount++;
      }
      if (gateConfirmCount >= GATE_CONFIRM_READS) {
        lastGateDetectionMs = now;
        setGateOpen(true);
      }
    } else {
      gateConfirmCount = 0;
    }
  }

  if (gateOpen && (now - lastGateDetectionMs >= GATE_HOLD_OPEN_MS)) {
    setGateOpen(false);
  }

  updateGateServos(now);
}

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
  pinMode(PIR_SENSOR_PIN, INPUT_PULLDOWN);

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  Serial.begin(115200);
  delay(200);

  ESP32PWM::allocateTimer(0);
  attachGateServo(gateServo1, GATE_SERVO_1_PIN, currentServo1Angle, "gate servo 1");
  attachGateServo(gateServo2, GATE_SERVO_2_PIN, currentServo2Angle, "gate servo 2");

  Serial.println("Smart home detection started");
  Serial.print("Flame GPIO");
  Serial.print(FIRE_SENSOR_PIN);
  Serial.print(" (<= ");
  Serial.print(FIRE_THRESHOLD);
  Serial.print("), Smoke MQ-2 GPIO");
  Serial.print(MQ2_SENSOR_PIN);
  Serial.print(" (>= ");
  Serial.print(SMOKE_THRESHOLD);
  Serial.print("), PIR GPIO");
  Serial.print(PIR_SENSOR_PIN);
  Serial.print(", Buzzer GPIO");
  Serial.println(BUZZER_PIN);

  Serial.print("Ultrasonic TRIG GPIO");
  Serial.print(ULTRASONIC_TRIG_PIN);
  Serial.print(", ECHO GPIO");
  Serial.print(ULTRASONIC_ECHO_PIN);
  Serial.print(" (<= ");
  Serial.print(GATE_TRIGGER_DISTANCE_CM, 1);
  Serial.print("cm), Gate servos GPIO");
  Serial.print(GATE_SERVO_1_PIN);
  Serial.print(" & GPIO");
  Serial.println(GATE_SERVO_2_PIN);
}

void loop() {
  unsigned long now = millis();

  int flameValue = analogRead(FIRE_SENSOR_PIN);
  int smokeValue = analogRead(MQ2_SENSOR_PIN);
  bool motionDetected = (digitalRead(PIR_SENSOR_PIN) == HIGH);

  bool fireDetected = flameValue <= FIRE_THRESHOLD;
  bool smokeDetected = smokeValue >= SMOKE_THRESHOLD;
  bool alarm = fireDetected || smokeDetected || motionDetected;

  if (alarm) {
    startSiren();
    updateSiren();
  } else {
    stopSiren();
  }

  updateGate(now);

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
    Serial.print(" | PIR motion=");
    Serial.print(motionDetected ? "YES" : "NO");
    Serial.print(" | Dist=");
    if (lastDistanceCm < 0.0f) {
      Serial.print("---");
    } else {
      Serial.print(lastDistanceCm, 1);
      Serial.print("cm");
    }
    Serial.print(" gate=");
    if (currentServo1Angle == targetServo1Angle && currentServo2Angle == targetServo2Angle) {
      Serial.print(gateOpen ? "OPEN(" : "CLOSED(");
    } else {
      Serial.print(gateOpen ? "OPENING(" : "CLOSING(");
    }
    Serial.print(currentServo1Angle);
    if (currentServo2Angle != currentServo1Angle) {
      Serial.print("/");
      Serial.print(currentServo2Angle);
    }
    Serial.print("deg)");
    Serial.print(" | sirenHz=");
    Serial.println(alarm ? sirenHz : 0);
  }

  delay(5);
}
