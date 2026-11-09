// Smart Home Automation & Safety System (ESP32 Main Hub)
// 1- Smoke detector (MQ-2 analog sensor -> buzzer alarm)
// 2- Fire detection (analog flame sensor -> buzzer alarm)
// 3- Motion detection (PIR sensor -> buzzer alarm)
// 4- Ultrasonic gate control (HC-SR04 -> 2 servo motors open to 180 deg)
// 5- Wireless ESP-NOW + SoftAP hub for 2x ESP32-CAM (HW-297) nodes:
//    - Camera 1 (Face Recognition) -> opens 3rd door servo (GPIO26) or 4s siren
//    - Camera 2 (Plate OCR)        -> opens 2-servo car gate (GPIO18/19) or 4s siren

#include <Arduino.h>
#include <ESP32Servo.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_now.h>
#include "smart_home_protocol.h"

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
// Door servo 3 (face)   -> GPIO26 (ESP32Servo 50 Hz PWM)
const int FIRE_SENSOR_PIN = 34;
const int MQ2_SENSOR_PIN = 35;
const int PIR_SENSOR_PIN = 27;
const int ULTRASONIC_TRIG_PIN = 32;
const int ULTRASONIC_ECHO_PIN = 33;
const int BUZZER_PIN = 25;
const int GATE_SERVO_1_PIN = 18;
const int GATE_SERVO_2_PIN = 19;
const int DOOR_SERVO_3_PIN = 26;

// Reversed flame sensor: normal ~4095 (no flame), drops toward 0 when flame detected.
const int FIRE_THRESHOLD = 500;

// MQ-2 smoke threshold. MQ-2 needs warm-up (~30-60s) before readings are stable.
// Tune this after observing smoke vs clean-air ADC values.
const int SMOKE_THRESHOLD = 2000;

// Ultrasonic gate detection settings.
// Opens the 2-servo gate when an object is detected within GATE_TRIGGER_DISTANCE_CM.
const float GATE_TRIGGER_DISTANCE_CM = 25.0f;
const unsigned long ULTRASONIC_INTERVAL_MS = 60;      // >= 60ms between HC-SR04 triggers
const unsigned long ULTRASONIC_TIMEOUT_US = 20000;    // ~3.4m max range timeout (20ms)
const int GATE_CONFIRM_READS = 2;                     // consecutive detections to filter glitches
const unsigned long GATE_HOLD_OPEN_MS = 3000;         // hold gate open after ultrasonic clears
const unsigned long PLATE_GATE_HOLD_OPEN_MS = 5000;   // hold gate open after recognized plate
const unsigned long DOOR_HOLD_OPEN_MS = 3000;         // hold 3rd servo open after recognized face
const unsigned long CAM_ALARM_DURATION_MS = 4000;     // 4s siren pulse for unrecognized face/plate
const unsigned long CAM_ONLINE_TIMEOUT_MS = 15000;    // heartbeat freshness window

// ---------------------------------------------------------------------------
// Servos (ESP32Servo 50 Hz PWM, 500-2500 us for MG996R/DS3218 180-deg)
// ---------------------------------------------------------------------------
const int SERVO_PWM_FREQ_HZ = 50;
const int SERVO_PWM_RES_BITS = 16; // Fine 16-bit PWM steps for smooth ramping
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

const int SERVO1_CLOSED_ANGLE = 0;
const int SERVO1_OPEN_ANGLE = 180;
const int SERVO2_CLOSED_ANGLE = 0;
const int SERVO2_OPEN_ANGLE = 180;
const int DOOR3_CLOSED_ANGLE = 0;
const int DOOR3_OPEN_ANGLE = 180;

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
Servo doorServo3;
int currentServo1Angle = SERVO1_CLOSED_ANGLE;
int currentServo2Angle = SERVO2_CLOSED_ANGLE;
int currentDoor3Angle = DOOR3_CLOSED_ANGLE;
int targetServo1Angle = SERVO1_CLOSED_ANGLE;
int targetServo2Angle = SERVO2_CLOSED_ANGLE;
int targetDoor3Angle = DOOR3_CLOSED_ANGLE;
unsigned long lastServoStepMs = 0;

unsigned long lastUltrasonicMs = 0;
unsigned long lastGateDetectionMs = 0;
unsigned long plateGateUntilMs = 0;
int gateConfirmCount = 0;
float lastDistanceCm = -1.0f;
bool gateOpen = false;

unsigned long doorOpenUntilMs = 0;
bool doorOpen = false;

unsigned long camAlarmUntilMs = 0;
bool espNowReady = false;

struct CamNodeTelemetry {
  bool seen = false;
  unsigned long lastSeenMs = 0;
  unsigned long lastEventMs = 0;
  uint8_t lastEvent = 0;
  int16_t lastMatchId = -1;
  uint8_t lastConfidence = 0;
  uint32_t lastSeq = 0;
  char lastLabel[24] = "none";
};

CamNodeTelemetry faceCamState;
CamNodeTelemetry plateCamState;

int lastFlameAdc = 4095;
int lastSmokeAdc = 0;
bool lastFireDetected = false;
bool lastSmokeDetected = false;
bool lastMotionDetected = false;
bool lastAlarmActive = false;

struct IncomingCamPacket {
  SmartHomeNet::CamEventPacket pkt;
  uint8_t srcMac[6];
};

constexpr size_t RX_QUEUE_CAP = 8;
portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;
IncomingCamPacket rxQueue[RX_QUEUE_CAP];
volatile size_t rxHead = 0;
volatile size_t rxTail = 0;
volatile uint32_t rxDropped = 0;

WebServer server(80);

void onEspNowRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *data, int len) {
  if (!data || len != static_cast<int>(sizeof(SmartHomeNet::CamEventPacket))) {
    return;
  }
  SmartHomeNet::CamEventPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));
  if (pkt.magic != SmartHomeNet::PACKET_MAGIC) {
    return;
  }
  pkt.label[sizeof(pkt.label) - 1] = '\0';

  portENTER_CRITICAL(&rxMux);
  size_t nextHead = (rxHead + 1) % RX_QUEUE_CAP;
  if (nextHead == rxTail) {
    rxDropped = rxDropped + 1;
  } else {
    rxQueue[rxHead].pkt = pkt;
    if (recvInfo && recvInfo->src_addr) {
      memcpy(rxQueue[rxHead].srcMac, recvInfo->src_addr, 6);
    } else {
      memset(rxQueue[rxHead].srcMac, 0, 6);
    }
    rxHead = nextHead;
  }
  portEXIT_CRITICAL(&rxMux);
}

bool popCamPacket(IncomingCamPacket &out) {
  bool hasItem = false;
  portENTER_CRITICAL(&rxMux);
  if (rxTail != rxHead) {
    out = rxQueue[rxTail];
    rxTail = (rxTail + 1) % RX_QUEUE_CAP;
    hasItem = true;
  }
  portEXIT_CRITICAL(&rxMux);
  return hasItem;
}

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

void setDoorOpen(bool open) {
  if (doorOpen == open) return;
  doorOpen = open;
  targetDoor3Angle = doorOpen ? DOOR3_OPEN_ANGLE : DOOR3_CLOSED_ANGLE;
}

void updateAllServos(unsigned long now) {
  if (currentServo1Angle == targetServo1Angle &&
      currentServo2Angle == targetServo2Angle &&
      currentDoor3Angle == targetDoor3Angle) {
    return;
  }
  if (now - lastServoStepMs < SERVO_STEP_INTERVAL_MS) {
    return;
  }
  lastServoStepMs = now;

  int nextAngle1 = stepServoAngle(currentServo1Angle, targetServo1Angle, SERVO_STEP_DEG);
  int nextAngle2 = stepServoAngle(currentServo2Angle, targetServo2Angle, SERVO_STEP_DEG);
  int nextAngle3 = stepServoAngle(currentDoor3Angle, targetDoor3Angle, SERVO_STEP_DEG);

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
  if (nextAngle3 != currentDoor3Angle) {
    currentDoor3Angle = nextAngle3;
    if (doorServo3.attached()) {
      doorServo3.writeMicroseconds(servoAngleToPulseUs(currentDoor3Angle));
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

void updateGateAndDoor(unsigned long now) {
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

  bool ultrasonicHold = (lastGateDetectionMs != 0 && (now - lastGateDetectionMs < GATE_HOLD_OPEN_MS));
  bool plateHold = false;
  if (plateGateUntilMs != 0) {
    if (static_cast<long>(plateGateUntilMs - now) > 0) {
      plateHold = true;
    } else {
      plateGateUntilMs = 0;
    }
  }

  if (gateOpen && !ultrasonicHold && !plateHold) {
    setGateOpen(false);
  }

  if (doorOpen) {
    if (doorOpenUntilMs == 0 || static_cast<long>(doorOpenUntilMs - now) <= 0) {
      doorOpenUntilMs = 0;
      setDoorOpen(false);
    }
  }

  updateAllServos(now);
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

void updateSiren(unsigned long now) {
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

const char *eventTypeName(uint8_t ev) {
  switch (ev) {
    case SmartHomeNet::CAM_EVENT_RECOGNIZED:
      return "RECOGNIZED";
    case SmartHomeNet::CAM_EVENT_UNKNOWN:
      return "UNKNOWN";
    case SmartHomeNet::CAM_EVENT_HEARTBEAT:
      return "HEARTBEAT";
    default:
      return "NONE";
  }
}

void handleCamEvent(const IncomingCamPacket &item, unsigned long now) {
  const auto &pkt = item.pkt;
  CamNodeTelemetry *telemetry = nullptr;
  const char *srcName = "UNKNOWN";

  if (pkt.source == SmartHomeNet::CAM_SOURCE_FACE) {
    telemetry = &faceCamState;
    srcName = "FACE-CAM";
  } else if (pkt.source == SmartHomeNet::CAM_SOURCE_PLATE) {
    telemetry = &plateCamState;
    srcName = "PLATE-CAM";
  } else {
    return;
  }

  telemetry->seen = true;
  telemetry->lastSeenMs = now;

  if (pkt.event == SmartHomeNet::CAM_EVENT_HEARTBEAT) {
    return;
  }

  telemetry->lastEventMs = now;
  telemetry->lastEvent = pkt.event;
  telemetry->lastMatchId = pkt.matchId;
  telemetry->lastConfidence = pkt.confidence;
  telemetry->lastSeq = pkt.seq;
  strncpy(telemetry->lastLabel, pkt.label, sizeof(telemetry->lastLabel) - 1);
  telemetry->lastLabel[sizeof(telemetry->lastLabel) - 1] = '\0';

  Serial.printf("[ESP-NOW] %s -> %s (id=%d, conf=%u%%, label=\"%s\", seq=%lu)\n",
                srcName,
                eventTypeName(pkt.event),
                pkt.matchId,
                pkt.confidence,
                telemetry->lastLabel,
                static_cast<unsigned long>(pkt.seq));

  if (pkt.event == SmartHomeNet::CAM_EVENT_RECOGNIZED) {
    camAlarmUntilMs = 0; // Silence camera intruder siren on valid entry
    if (pkt.source == SmartHomeNet::CAM_SOURCE_FACE) {
      doorOpenUntilMs = now + DOOR_HOLD_OPEN_MS;
      setDoorOpen(true);
    } else if (pkt.source == SmartHomeNet::CAM_SOURCE_PLATE) {
      plateGateUntilMs = now + PLATE_GATE_HOLD_OPEN_MS;
      setGateOpen(true);
    }
  } else if (pkt.event == SmartHomeNet::CAM_EVENT_UNKNOWN) {
    camAlarmUntilMs = now + CAM_ALARM_DURATION_MS;
  }
}

String jsonEscape(const char *s) {
  String out;
  if (!s) return out;
  for (size_t i = 0; s[i] != '\0'; ++i) {
    char c = s[i];
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (static_cast<uint8_t>(c) >= 0x20) {
      out += c;
    }
  }
  return out;
}

void handleRoot() {
  static const char HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Smart Home Hub</title>
<style>
  body{font-family:system-ui,-apple-system,sans-serif;background:#0f172a;color:#e2e8f0;margin:0;padding:16px}
  .wrap{max-width:860px;margin:0 auto}
  h1{font-size:1.4rem;margin:0 0 12px}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:12px}
  .card{background:#1e293b;border:1px solid #334155;border-radius:10px;padding:14px}
  .k{color:#94a3b8;font-size:.85rem}
  .v{font-size:1.05rem;font-weight:600;margin-top:2px}
  .row{display:flex;justify-content:space-between;align-items:center;padding:5px 0;border-bottom:1px solid #334155}
  .row:last-child{border-bottom:0}
  .badge{padding:2px 8px;border-radius:999px;font-size:.78rem;font-weight:700}
  .ok{background:#14532d;color:#86efac}
  .warn{background:#7c2d12;color:#fdba74}
  .bad{background:#7f1d1d;color:#fca5a5}
  .off{background:#334155;color:#cbd5e1}
  a.btn{display:inline-block;margin-top:8px;padding:8px 12px;border-radius:8px;background:#2563eb;color:#fff;text-decoration:none;font-weight:600;font-size:.9rem}
</style>
</head>
<body>
<div class="wrap">
  <h1>Smart Home Security &amp; Access Hub</h1>
  <div class="grid">
    <div class="card">
      <div class="k">Safety Sensors &amp; Siren</div>
      <div class="row"><span>Flame (GPIO34)</span><span id="flame" class="badge off">---</span></div>
      <div class="row"><span>Smoke MQ-2 (GPIO35)</span><span id="smoke" class="badge off">---</span></div>
      <div class="row"><span>PIR Motion (GPIO27)</span><span id="pir" class="badge off">---</span></div>
      <div class="row"><span>Siren (GPIO25)</span><span id="siren" class="badge off">---</span></div>
    </div>
    <div class="card">
      <div class="k">Servos &amp; Ultrasonic Gate</div>
      <div class="row"><span>Ultrasonic (GPIO32/33)</span><span id="dist">---</span></div>
      <div class="row"><span>Car Gate Servos (GPIO18/19)</span><span id="gate" class="badge off">---</span></div>
      <div class="row"><span>Face Door Servo (GPIO26)</span><span id="door" class="badge off">---</span></div>
      <div class="row"><span>Wi-Fi AP</span><span>SmartHome-Net (192.168.4.1)</span></div>
    </div>
    <div class="card">
      <div class="k">Camera 1 — Face Recognition</div>
      <div class="row"><span>Link Status</span><span id="faceOn" class="badge off">OFFLINE</span></div>
      <div class="row"><span>Last Result</span><span id="faceEv">---</span></div>
      <div class="row"><span>Identity / Conf</span><span id="faceLbl">---</span></div>
      <a class="btn" href="http://192.168.4.101/" target="_blank">Open Face Cam UI (192.168.4.101)</a>
    </div>
    <div class="card">
      <div class="k">Camera 2 — Car Plate OCR</div>
      <div class="row"><span>Link Status</span><span id="plateOn" class="badge off">OFFLINE</span></div>
      <div class="row"><span>Last Result</span><span id="plateEv">---</span></div>
      <div class="row"><span>Plate / Conf</span><span id="plateLbl">---</span></div>
      <a class="btn" href="http://192.168.4.102/" target="_blank">Open Plate Cam UI (192.168.4.102)</a>
    </div>
  </div>
</div>
<script>
function setBadge(id, text, cls){const e=document.getElementById(id);e.textContent=text;e.className='badge '+cls;}
async function refresh(){
  try{
    const r=await fetch('/api/status'); const d=await r.json();
    setBadge('flame', (d.fire?'ALERT ':'OK ')+'('+d.flameAdc+')', d.fire?'bad':'ok');
    setBadge('smoke', (d.smoke?'ALERT ':'OK ')+'('+d.smokeAdc+')', d.smoke?'bad':'ok');
    setBadge('pir', d.pir?'MOTION':'CLEAR', d.pir?'warn':'ok');
    setBadge('siren', d.alarm?(d.camAlarm?'CAM SIREN ':'ALARM ')+d.sirenHz+'Hz':'OFF', d.alarm?'bad':'ok');
    document.getElementById('dist').textContent = d.distCm>=0 ? d.distCm.toFixed(1)+' cm' : '---';
    setBadge('gate', (d.gateOpen?'OPEN':'CLOSED')+' ('+d.servo1+'/' +d.servo2+' deg)', d.gateOpen?'ok':'off');
    setBadge('door', (d.doorOpen?'OPEN':'CLOSED')+' ('+d.door3+' deg)', d.doorOpen?'ok':'off');
    setBadge('faceOn', d.faceCam.online?'ONLINE':'OFFLINE', d.faceCam.online?'ok':'off');
    document.getElementById('faceEv').textContent = d.faceCam.event;
    document.getElementById('faceLbl').textContent = d.faceCam.label+' ('+d.faceCam.conf+'%)';
    setBadge('plateOn', d.plateCam.online?'ONLINE':'OFFLINE', d.plateCam.online?'ok':'off');
    document.getElementById('plateEv').textContent = d.plateCam.event;
    document.getElementById('plateLbl').textContent = d.plateCam.label+' ('+d.plateCam.conf+'%)';
  }catch(e){}
}
refresh(); setInterval(refresh, 1200);
</script>
</body>
</html>)HTML";
  server.send_P(200, "text/html", HTML);
}

void handleApiStatus() {
  unsigned long now = millis();
  bool camAlarmActive = (camAlarmUntilMs != 0 && static_cast<long>(camAlarmUntilMs - now) > 0);
  bool faceOnline = faceCamState.seen && (now - faceCamState.lastSeenMs <= CAM_ONLINE_TIMEOUT_MS);
  bool plateOnline = plateCamState.seen && (now - plateCamState.lastSeenMs <= CAM_ONLINE_TIMEOUT_MS);

  String json = "{";
  json += "\"flameAdc\":" + String(lastFlameAdc) + ",";
  json += "\"fire\":" + String(lastFireDetected ? "true" : "false") + ",";
  json += "\"smokeAdc\":" + String(lastSmokeAdc) + ",";
  json += "\"smoke\":" + String(lastSmokeDetected ? "true" : "false") + ",";
  json += "\"pir\":" + String(lastMotionDetected ? "true" : "false") + ",";
  json += "\"camAlarm\":" + String(camAlarmActive ? "true" : "false") + ",";
  json += "\"alarm\":" + String(lastAlarmActive ? "true" : "false") + ",";
  json += "\"sirenHz\":" + String(lastAlarmActive ? sirenHz : 0) + ",";
  json += "\"distCm\":" + String(lastDistanceCm, 1) + ",";
  json += "\"gateOpen\":" + String(gateOpen ? "true" : "false") + ",";
  json += "\"servo1\":" + String(currentServo1Angle) + ",";
  json += "\"servo2\":" + String(currentServo2Angle) + ",";
  json += "\"doorOpen\":" + String(doorOpen ? "true" : "false") + ",";
  json += "\"door3\":" + String(currentDoor3Angle) + ",";
  json += "\"rxDropped\":" + String(rxDropped) + ",";
  json += "\"faceCam\":{\"online\":" + String(faceOnline ? "true" : "false") +
          ",\"event\":\"" + String(eventTypeName(faceCamState.lastEvent)) + "\"" +
          ",\"id\":" + String(faceCamState.lastMatchId) +
          ",\"conf\":" + String(faceCamState.lastConfidence) +
          ",\"label\":\"" + jsonEscape(faceCamState.lastLabel) + "\"},";
  json += "\"plateCam\":{\"online\":" + String(plateOnline ? "true" : "false") +
          ",\"event\":\"" + String(eventTypeName(plateCamState.lastEvent)) + "\"" +
          ",\"id\":" + String(plateCamState.lastMatchId) +
          ",\"conf\":" + String(plateCamState.lastConfidence) +
          ",\"label\":\"" + jsonEscape(plateCamState.lastLabel) + "\"}";
  json += "}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
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
  attachGateServo(doorServo3, DOOR_SERVO_3_PIN, currentDoor3Angle, "face door servo 3");

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  IPAddress apIp(SmartHomeNet::HUB_IP[0], SmartHomeNet::HUB_IP[1],
                 SmartHomeNet::HUB_IP[2], SmartHomeNet::HUB_IP[3]);
  IPAddress subnet(SmartHomeNet::SUBNET_MASK[0], SmartHomeNet::SUBNET_MASK[1],
                   SmartHomeNet::SUBNET_MASK[2], SmartHomeNet::SUBNET_MASK[3]);
  WiFi.softAPConfig(apIp, apIp, subnet);
  WiFi.softAP(SmartHomeNet::AP_SSID, SmartHomeNet::AP_PASS, SmartHomeNet::WIFI_CHANNEL, 0, 4);

  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onEspNowRecv);
    espNowReady = true;
  } else {
    Serial.println("ERROR: esp_now_init failed on Hub!");
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.begin();

  Serial.println("Smart home hub started");
  Serial.print("SoftAP: ");
  Serial.print(SmartHomeNet::AP_SSID);
  Serial.print(" ch=");
  Serial.print(SmartHomeNet::WIFI_CHANNEL);
  Serial.print(" IP=");
  Serial.print(WiFi.softAPIP());
  Serial.print(" AP_MAC=");
  Serial.println(WiFi.softAPmacAddress());

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
  Serial.print(GATE_SERVO_2_PIN);
  Serial.print(", Face door servo GPIO");
  Serial.println(DOOR_SERVO_3_PIN);
}

void loop() {
  unsigned long now = millis();

  IncomingCamPacket item;
  while (popCamPacket(item)) {
    handleCamEvent(item, now);
  }

  server.handleClient();

  lastFlameAdc = analogRead(FIRE_SENSOR_PIN);
  lastSmokeAdc = analogRead(MQ2_SENSOR_PIN);
  lastMotionDetected = (digitalRead(PIR_SENSOR_PIN) == HIGH);

  lastFireDetected = lastFlameAdc <= FIRE_THRESHOLD;
  lastSmokeDetected = lastSmokeAdc >= SMOKE_THRESHOLD;

  bool camAlarmActive = false;
  if (camAlarmUntilMs != 0) {
    if (static_cast<long>(camAlarmUntilMs - now) > 0) {
      camAlarmActive = true;
    } else {
      camAlarmUntilMs = 0;
    }
  }

  lastAlarmActive = lastFireDetected || lastSmokeDetected || lastMotionDetected || camAlarmActive;

  if (lastAlarmActive) {
    startSiren();
    updateSiren(now);
  } else {
    stopSiren();
  }

  updateGateAndDoor(now);

  if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;
    Serial.print("Flame ADC=");
    Serial.print(lastFlameAdc);
    Serial.print(" fire=");
    Serial.print(lastFireDetected ? "YES" : "NO");
    Serial.print(" | Smoke ADC=");
    Serial.print(lastSmokeAdc);
    Serial.print(" smoke=");
    Serial.print(lastSmokeDetected ? "YES" : "NO");
    Serial.print(" | PIR motion=");
    Serial.print(lastMotionDetected ? "YES" : "NO");
    Serial.print(" | CamAlarm=");
    Serial.print(camAlarmActive ? "YES" : "NO");
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

    Serial.print(" | door3=");
    if (currentDoor3Angle == targetDoor3Angle) {
      Serial.print(doorOpen ? "OPEN(" : "CLOSED(");
    } else {
      Serial.print(doorOpen ? "OPENING(" : "CLOSING(");
    }
    Serial.print(currentDoor3Angle);
    Serial.print("deg)");

    Serial.print(" | sirenHz=");
    Serial.println(lastAlarmActive ? sirenHz : 0);
  }

  delay(5);
}
