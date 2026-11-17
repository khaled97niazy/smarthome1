# Smart Home Automation, Vision & Safety System (ESP32 + 2× HW-297 ESP32-CAM)

A multi-node ESP32 smart home safety and biometric/vehicle access-control system built with **PlatformIO**. It combines a **Main ESP32 Hub** (`esp32dev`) with **two `HW-297` (AI-Thinker ESP32-CAM) camera nodes** communicating wirelessly over a built-in Wi-Fi SoftAP + **ESP-NOW** link—no external PC, cloud server, or home router required.

---

## System Architecture

1. **Main Hub (`[env:esp32dev]`, [`src/main.cpp`](src/main.cpp))**
   - **Safety Sensors**: Digital Flame IR (`DO` → `GPIO34`), Digital Smoke/Gas MQ-2 (`DO` → `GPIO35`), PIR Motion (`GPIO27`), and HC-SR04 Ultrasonic (`TRIG=GPIO32`, `ECHO=GPIO33`, `<= 25.0 cm`) → sweeping **Siren Buzzer (`GPIO25`)**.
   - **Per-Sensor Alarm Bypass / Ignore**: Dedicated 1-click toggles and NVS settings allow bypassing/muting the siren trigger for individual sensors (e.g. ignore MQ-2 or Flame sensor alarm) while keeping real-time visual telemetry active and other alarms fully armed.
   - **PIR Scheduled Night Guard**: Restricts the loud siren activation on motion to the overnight window between **Midnight (00:00) and 7:00 AM (07:00)**; movement during the day displays in the UI but remains silent.
   - **Smart Lighting Controller (`GPIO21`)**: 3.3V digital output driving a relay module or MOSFET circuit with dashboard controls, automated event logging, and voice feedback.
   - **HW-104 Spoken Voice Speaker (`GPIO4`)**: PAM8403 2×3W Class-D audio amplifier driven via 312.5 kHz LEDC PWM carrier (4× oversampled) with non-blocking FreeRTOS streaming, providing **100% offline natural human spoken sentences** for completed actions (*"Light turned on/off"*, *"Door unlocked/locked"*, *"Car gate opened/closed"*, *"Access granted, welcome home"*, *"Access denied"*, *"Warning: smoke/flame detected"*, and *"Command confirmed"*), with an NVS toggle between natural speech and musical chimes.
   - **INMP441 I2S Digital Microphone (`GPIO17`, `GPIO16`, `GPIO23`)**: Hardware I2S Master RX sampling at 16 kHz, real-time RMS audio level meter, Voice Activity Detection (VAD), and extensible voice command dispatcher.
   - **MFRC522 RFID Card Access**: Hardware SPI reader (`SS=GPIO5`, `RST=GPIO22`, `SCK=GPIO14`, `MOSI=GPIO13`, `MISO=GPIO12`) opening the dual car gate on authorized UID `52411902`.
   - **Car Gate Servos (`GPIO18`, `GPIO19`)**: `0° ↔ 160°` smooth non-blocking PWM ramp, opened (`5 seconds`) when an authorized RFID card is presented.
   - **3rd Face Door Servo (`GPIO26`)**: `0° ↔ 160°` smooth non-blocking PWM ramp, opened exclusively (`3 seconds`) when Camera 1 recognizes an enrolled face.
   - **Intruder Siren Pulse**: Triggers a **`4-second`** siren burst (`GPIO25` + `GPIO4`) whenever Camera 1 reports an unrecognized face or an unauthorized RFID card is scanned.
   - **Wireless Hub & Web Admin Panel**: Hosts Wi-Fi SoftAP **`SmartHome-Net`** (password `smarthome123`, channel `1`, IP `http://192.168.4.1`) + optional Home Wi-Fi STA uplink, ESP-NOW receiver, and a **Full Web Admin Dashboard**:
     - **Live System Clock & Time Sync**: Displays live clock (default boot time: 12:00 PM), 1-click Browser Time Sync, and manual time setter.
     - **Smart Lighting Switch**: Interactive ON/OFF/Toggle dashboard buttons with live state feedback on `GPIO21`.
     - **Voice Assistant & VU Meter**: Real-time microphone VU level bar, speech activity indicator badge, speaker status, and instant voice verification test triggers.
     - **Live Camera Streams**: Embedded side-by-side viewports for Camera 1 (Face) and Camera 2 (Plate) with live FPS/latency and direct/proxy streaming.
     - **Sensors & Telemetry**: Dynamic visual gauges and alert badges for Flame (`AO`), Smoke (`MQ-2`), Motion (`PIR`), and Ultrasonic (`HC-SR04`).
     - **Manual Overrides**: Quick-action controls for Car Gate (open/close), Face Door (unlock/lock), Smart Light (toggle), and Siren Alarm (mute/test).
     - **NVS Settings & Event Log**: Save custom thresholds, PIR schedule hours, timers, and Home Wi-Fi credentials to Flash, and view the live rolling system event log.

2. **Camera 1 — On-Device Face Recognition (`[env:cam_face]`, [`src/cam_face/main.cpp`](src/cam_face/main.cpp))**
   - Runs Espressif ESP-DL **2-stage face detection (`HumanFaceDetectMSR01` + `HumanFaceDetectMNP01`)** + **MobileFaceNet (`FaceRecognition112V1S8`)** 100% locally on the `HW-297` (`240×240` RGB565 in PSRAM).
   - Stores enrolled face embeddings and person names in a dedicated Flash partition (`fr` in [`partitions_cam_face.csv`](partitions_cam_face.csv)) across reboots, plus threshold/settings in NVS Flash.
   - **Real-Time MJPEG Live Video Stream**: Dedicated FreeRTOS streaming server on **Port 81** (`/stream`) with zero polling, live FPS counter, latency HUD, and face reticle overlay. Automatic fallback to high-speed snapshot pipeline if port 81 is blocked.
   - **Enhanced Guided Face Enrollment**: Interactive wizard supporting **Single Capture** and **Multi-Angle Guided (3x samples: Front, Left 15°, Right 15°)**, real-time distance and positioning guidance, 12s countdown timer, and instant **Cancel Enrollment** button.
   - Hosts a Web UI at **`http://192.168.4.101`** (live face-box stream, multi-angle enroll, user list with `[IN VIEW]` badge, delete, clear, test buttons, calibration).
   - **Recognized Face** → ESP-NOW triggers **Door Servo 3 (`GPIO26`)** on the Main Hub.
   - **Unknown Face** (`2` consecutive detections, `4 s` cooldown) → ESP-NOW triggers the **4 s Siren Buzzer (`GPIO25`)**.

3. **Camera 2 — License Plate OCR, QR Code & Barcode Recognition (`[env:cam_plate]` / `[env:cam_plate_qr]`, [`src/cam_plate/main.cpp`](src/cam_plate/main.cpp))**
   - Runs 100% locally on-device (`320×240` grayscale in PSRAM):
     - **QR Code & 1D Barcode Decoder ([`plate_code.cpp`](src/cam_plate/plate_code.cpp) + PSRAM-backed [`quirc`](src/cam_plate/quirc/))**: Decodes **QR codes** (`quirc`, any rotation), **Code 128** (Sets A/B/C, Mod-103 checksum), and **Code 39** barcodes containing the plate number (e.g. `ABC1234` or `PLATE:ABC1234`).
     - **License Plate OCR Fallback ([`plate_ocr.cpp`](src/cam_plate/plate_ocr.cpp))**: Horizontal edge-energy ROI localization, Otsu adaptive thresholding, 2D connected-component character segmentation, and `16×16` zero-mean NCC template OCR (`0–9`, `A–Z`).
   - **Real-Time MJPEG Live Video Stream**: Dedicated FreeRTOS streaming server on **Port 81** (`/stream`), real-time FPS badge, and 3:1 aspect plate reticle HUD.
   - **1-Click Plate Authorization**: Floating HUD action button to instantly authorize any detected plate, QR, or barcode into NVS Flash with a single click.
   - Supports live **Recognition Mode** switching saved to NVS Flash: **`Auto (QR / Barcode → OCR)`** (default in `[env:cam_plate]`), **`QR Code / Barcode Only`** (default in `[env:cam_plate_qr]`), or **`Plate OCR Only`**.
   - Stores up to **20 authorized plate strings** and camera/recognition calibration in NVS Flash across reboots.
   - Hosts a Web UI at **`http://192.168.4.102`** (live quad/bounding-box stream, `QR CODE` / `BARCODE` / `OCR` source indicator, Add/Delete/Use Last Read plate manager with `[IN VIEW]` tracking, test buttons, mode/polarity/fuzzy/flash settings).
   - **Authorized Plate / QR / Barcode** (`2` consecutive valid reads) → ESP-NOW opens the **2-Servo Car Gate (`GPIO18` & `GPIO19`)**.
   - **Unknown Plate / QR / Barcode** (`2` consecutive valid reads, `4 s` cooldown) → ESP-NOW triggers the **4 s Siren Buzzer (`GPIO25`)**.

---

## Hardware Requirements

- **1× ESP32 DevKit** (`esp32dev`) — Main Hub
- **2× AI-Thinker ESP32-CAM (`HW-297`)** with OV2640 camera & 4 MB PSRAM
- **USB-to-TTL (`FTDI` / `CP2102` / `CH340`) Programmer** (3.3 V UART TX/RX, for flashing `HW-297` boards; or an ESP32-CAM-MB daughterboard)
- **Sensors & Actuators on Main Hub**:
  - Digital IR Flame Sensor (`DO` → `GPIO34`)
  - Digital MQ-2 Smoke / Gas Sensor (`DO` → `GPIO35`)
  - PIR Motion Sensor (`OUT` → `GPIO27`)
  - HC-SR04 Ultrasonic Sensor (`TRIG` → `GPIO32`, `ECHO` → `GPIO33` via 5 V → 3.3 V resistor divider)
  - Passive Piezo Buzzer (`SIG` → `GPIO25`)
  - **3× 180° Servo Motors** (e.g., MG996R / DS3218, `500–2500 µs`): **Car Gate Servo 1 (`GPIO18`)**, **Car Gate Servo 2 (`GPIO19`)**, **Face Door Servo 3 (`GPIO26`)**
  - **Dedicated 5 V ≥ 3 A Power Supply** (common `GND` shared with Main ESP32 and servos; `HW-297` boards should be powered cleanly at **5 V** on the `5V` pin rather than a weak `3.3 V` regulator)

---

## Wiring & Pinout

### Main ESP32 Hub (`esp32dev`)

| Component | Module Pin | ESP32 GPIO | Mode | Notes |
| --- | --- | --- | --- | --- |
| **Smart Light (Relay / MOSFET)** | `IN / Gate` | `GPIO21` | Digital Output (`3.3 V`) | Connect to 5V relay module IN or MOSFET gate to switch light loads |
| **HW-104 Spoken Voice Speaker** | `L / R` (Audio In) | `GPIO4` | LEDC 312.5 kHz PWM (8-bit, Timer 3 Ch 6) | PAM8403 2×3W Class-D amplifier; non-blocking FreeRTOS natural spoken sentences & dual siren alarms |
| **INMP441 I2S Microphone** | `SCK` (BCLK)<br>`WS` (LRCLK)<br>`SD` (Data)<br>`L/R` (Channel)<br>`VDD` | `GPIO17`<br>`GPIO16`<br>`GPIO23`<br>`GND`<br>`3V3` | Hardware I2S (`I2S_NUM_0`) | 16 kHz Master RX digital microphone; VDD connects strictly to 3.3V (NOT 5V) |
| **Flame Sensor** | `DO` | `GPIO34` | Digital Input (`INPUT`) | Active-LOW (0V = Flame, 3.3V = Safe); potentiometer calibrated on module; siren trigger can be ignored via dashboard |
| **MQ-2 Smoke Sensor** | `DO` | `GPIO35` | Digital Input (`INPUT`) | Active-LOW (0V = Smoke/Gas, 3.3V = Clean); potentiometer calibrated on module; siren trigger can be ignored via dashboard |
| **PIR Motion Sensor** | `OUT` | `GPIO27` | `INPUT_PULLDOWN` | Active `HIGH` (`3.3 V` logic); siren only activates during night window (**Midnight–7 AM**) |
| **Ultrasonic Sensor** | `TRIG` | `GPIO32` | Digital Output | 10 µs trigger pulse every `60 ms` (`<= 25.0 cm` triggers siren) |
| **Ultrasonic Sensor** | `ECHO` | `GPIO33` | Digital Input | **Use 5 V → 3.3 V voltage divider** |
| **Buzzer** | `SIG` / `+` | `GPIO25` | LEDC Tone PWM (8-bit, Timer 2 Ch 4) | `700 Hz ↔ 1800 Hz` alarm siren |
| **Car Gate Servo 1** | `PWM` | `GPIO18` | `ESP32Servo` 50 Hz (16-bit, Timer 0) | Opens `0° → 160°` (`5 s`) on valid RFID UID `52411902` |
| **Car Gate Servo 2** | `PWM` | `GPIO19` | `ESP32Servo` 50 Hz (16-bit, Timer 0) | Opens `0° → 160°` (`5 s`) on valid RFID UID `52411902` |
| **Face Door Servo 3** | `PWM` | `GPIO26` | `ESP32Servo` 50 Hz (16-bit, Timer 0) | Opens `0° → 160°` (`3 s`) on recognized face from Camera 1 |
| **MFRC522 RFID Reader** | `SDA` / `SS`<br>`SCK`<br>`MOSI`<br>`MISO`<br>`RST` | `GPIO5`<br>`GPIO14`<br>`GPIO13`<br>`GPIO12`<br>`GPIO22` | Hardware SPI (`MFRC522`) | Reads 13.56 MHz RFID tags; accepted UID: `52411902` |

### Camera 1 (`HW-297` Face Recognition) Runtime Wiring & Main ESP32 Hub Link

At runtime, Camera 1 (`HW-297` AI-Thinker ESP32-CAM) requires **only 5 V power and common ground**—all face recognition events, heartbeats, and live video streams are transmitted wirelessly to the Main ESP32 Hub:

| `HW-297` Pin / Subsystem | Connection / Internal GPIO | Function & Best-Practice Wiring Notes |
| --- | --- | --- |
| **`5V`** | **Dedicated `5 V` Rail (`≥ 2 A`)** | Power via the `5V` pin (feeds onboard AMS1117-3.3V). **Never** power from the Main ESP32's `3V3` pin; add a `470 µF–1000 µF` bulk capacitor across `5V` and `GND` near the camera to prevent Wi-Fi/flash brownouts. |
| **`GND`** | **Common `GND`** (Shared with Main ESP32 & Servos) | Tie all power supply, Main ESP32, servo, and `HW-297` grounds together. |
| **`IO0`** | **Disconnected (Floating) at Runtime** | Must **only** be tied to `GND` while flashing firmware. Leave disconnected for normal boot. |
| **`U0T` (`GPIO1`) / `U0R` (`GPIO3`)** | Optional USB-TTL `RX` / `TX` (`115200` baud) | Used for flashing or viewing `[FACE]` Serial debug logs; leave unconnected in standalone deployment. |
| **Onboard Flash LED** | Internal `GPIO4` (LEDC Ch 2, 5 kHz, 8-bit) | Auto-flash hysteresis in low light (`sceneLuma < 0.18` → ON at duty `48`, `> 0.40` → OFF) or manual slider (`0–180`). |
| **Onboard Red Status LED** | Internal `GPIO33` (Active-LOW) | Illuminates (`LOW`) when an enrolled face is recognized; turns off (`HIGH`) when idle or unknown. |
| **OV2640 Camera Bus** | Internal `XCLK=0` (LEDC Ch 0), `SIOD=26`, `SIOC=27`, `PWDN=32`, `VSYNC=25`, `HREF=23`, `PCLK=22`, `D0..D7=5,18,19,21,36,39,34,35` | Captures `240×240` `RGB565` frames in PSRAM (`CAMERA_GRAB_LATEST`, double-buffered) with automatic self-healing if frame grabs stall. |
| **Wireless Link → Main ESP32 Hub** | Wi-Fi SoftAP `SmartHome-Net` (`192.168.4.101` → `192.168.4.1`) | **Dual-Path Event Delivery**: Sends 37-byte `CamEventPacket` over **ESP-NOW** + fallback **HTTP POST `/api/cam/event`** with sequence deduplication (`seq`):<br>• `CAM_EVENT_RECOGNIZED` → Unlocks **Face Door Servo 3 (`GPIO26`, `0° → 160°` for `3 s`)** + speaks *"Door unlocked"* on `GPIO4`<br>• `CAM_EVENT_UNKNOWN` (confirmed over `2` frames) → Triggers **Intruder Siren (`GPIO25`, `4 s`)** + speaks *"Access denied"* on `GPIO4`<br>• `CAM_EVENT_HEARTBEAT` (every `5 s` + immediate on connect) → Keeps Camera 1 online status & Hub proxy (`/api/cam/1/preview.jpg`) active |

### Flashing the `HW-297` (ESP32-CAM) Boards via USB-TTL

`HW-297` has no onboard USB-UART bridge. Connect your 3.3 V USB-TTL adapter as follows:

| USB-TTL | `HW-297` Pin | Notes |
| --- | --- | --- |
| `5V` (or External 5 V PSU) | `5V` | Do **not** under-power; brownouts cause camera/Wi-Fi init failures |
| `GND` | `GND` | Common ground |
| `TX` | `U0R` (`GPIO3`) | Cross TX → RX |
| `RX` | `U0T` (`GPIO1`) | Cross RX → TX |
| *(Flash jumper only)* | **`IO0` → `GND`** | Tie `IO0` to `GND` and press **`RST`**, run `upload`, then **remove `IO0–GND`** and press **`RST`** to boot normally |

---

## Build & Upload (PlatformIO)

[`platformio.ini`](platformio.ini) defines four environments:

```bash
# Build all firmwares at once
pio run

# 1) Flash Main ESP32 Hub (esp32dev)
pio run -e esp32dev -t upload

# 2) Flash Camera 1 (Face Recognition HW-297) — IO0 grounded during upload
pio run -e cam_face -t upload

# 3) Flash Camera 2 (Auto: QR / Barcode + Plate OCR) — IO0 grounded during upload
pio run -e cam_plate -t upload

# 3b) Optional: Flash Camera 2 defaulting directly to QR Code / Barcode Only mode
pio run -e cam_plate_qr -t upload
```

Monitor any board at **`115200` baud**:
```bash
pio device monitor -b 115200
```

---

## Web Dashboards & Enrollment

1. Power on the **Main Hub** first (`SmartHome-Net` SoftAP starts on channel `1`).
2. Power on **Camera 1** and **Camera 2** (with `IO0` disconnected from `GND`). They automatically join `SmartHome-Net`.
3. Connect your phone or laptop to Wi-Fi **`SmartHome-Net`** (password: **`smarthome123`**):
   - **Main Hub Status**: `http://192.168.4.1`
   - **Camera 1 (Face Recognition)**: `http://192.168.4.101` — Look at the camera (~40–80 cm), enter a name, click **Enroll Next Face** (saved to Flash partition `fr`).
   - **Camera 2 (Car Plate / QR / Barcode)**: `http://192.168.4.102` — Add plate strings manually, or hold a license plate (`~25–55 cm`), QR code (`~15–55 cm`, e.g. containing `ABC1234` or `PLATE:ABC1234`), or Code 128 / Code 39 barcode (`~15–35 cm`, filling most of preview width) in view and click **Use Last Read** → **Add Plate** (saved to NVS).

> **Standalone Bench Setup (Hub Powered Off)**: If a camera cannot find `SmartHome-Net` within `12 s` of boot, it starts a temporary setup hotspot (`SmartHome-FaceCam` at `http://192.168.11.1`, or `SmartHome-PlateCam` at `http://192.168.12.1`, password `smarthome123`) while continuing to scan for the Hub in the background.

---

## Project Structure

```text
.
├── platformio.ini                 # Multi-env build (esp32dev + cam_face + cam_plate + cam_plate_qr)
├── partitions_cam_face.csv        # 4 MB partition table with "fr" face database partition
├── include/
│   ├── smart_home_json.h          # Shared JSON string escaping helper
│   └── smart_home_protocol.h      # Shared Wi-Fi SoftAP & ESP-NOW CamEventPacket definitions
├── src/
│   ├── main.cpp                   # Main Hub firmware (sensors, 3 servos, SoftAP, ESP-NOW, Web UI)
│   ├── cam_common/
│   │   ├── cam_hw.h               # Shared HW-297 LED, OV2640 sensor, and JPEG preview helpers
│   │   ├── cam_hw.cpp
│   │   ├── cam_net.h              # Shared STA + fallback AP + ESP-NOW sender helpers
│   │   └── cam_net.cpp
│   ├── cam_face/
│   │   └── main.cpp               # Camera 1: ESP-DL MSR01/MNP01 + MobileFaceNet S8 + Flash DB
│   └── cam_plate/
│       ├── quirc/                 # PSRAM-backed MIT/ISC QR decoder core
│       ├── plate_code.h           # Camera 2: QR Code (quirc) + Code 128 / Code 39 decoder
│       ├── plate_code.cpp
│       ├── plate_ocr.h            # Camera 2: On-device ALPR localization, CCL & 16x16 NCC OCR
│       ├── plate_ocr.cpp
│       └── main.cpp               # Camera 2 capture loop, mode switcher, NVS plate DB & Web UI
└── README.md                      # Project documentation
```
