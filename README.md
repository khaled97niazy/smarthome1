# Smart Home Automation & Safety System (ESP32)

An ESP32-based smart home safety and access-control firmware built with **PlatformIO** and the **Arduino-ESP32 (core 3.x)** framework. It combines fire, smoke, and intruder motion alarms with an automated ultrasonic dual-servo gate.

---

## Features

1. **Fire Detection**
   - Reads an analog IR flame sensor (`FIRE_SENSOR_PIN = GPIO34`).
   - Uses reversed-logic detection (clean baseline `~4095`, drops toward `0` when flame is present; triggers when `ADC <= 500`).
   - Activates the buzzer siren when fire is detected.

2. **Smoke & Gas Detection**
   - Reads an analog MQ-2 smoke/gas sensor (`MQ2_SENSOR_PIN = GPIO35`).
   - Triggers the buzzer siren when `ADC >= 2000`.
   - Note: The MQ-2 heater requires **~30–60 seconds of warm-up** after power-on before readings stabilize.

3. **PIR Motion Alarm**
   - Reads a digital PIR motion sensor (`PIR_SENSOR_PIN = GPIO27`, active-`HIGH` with internal pulldown).
   - Turns on the buzzer siren whenever motion is detected (`HIGH`), and keeps the alarm active until motion (and any fire/smoke condition) clears.
   - Note: Standard HC-SR501 modules also need **~30–60 seconds** after power-up to calibrate to the ambient IR environment.

4. **Ultrasonic Automatic Dual-Servo Gate**
   - Measures distance using an HC-SR04 ultrasonic sensor (`TRIG = GPIO32`, `ECHO = GPIO33`) every `60 ms`.
   - When an object approaches within `25.0 cm` for `2` consecutive readings (~120 ms glitch filter), both gate servo motors (`GPIO18` and `GPIO19`) open to **`180°`**.
   - The gate remains open while the object stays in range and for **`3 seconds`** (`GATE_HOLD_OPEN_MS = 3000`) after the path clears, then returns to **`0°`** (closed).

5. **Real-Time Serial Diagnostics**
   - Streams live sensor readings, detection states, gate status, and siren frequency over USB Serial at **`115200` baud** every `250 ms`.

---

## Hardware Requirements

- **ESP32 DevKit** (`esp32dev`)
- **Analog Flame Sensor Module** (AO pin)
- **MQ-2 Smoke / Gas Sensor Module** (AO pin)
- **PIR Motion Sensor Module** (e.g., HC-SR501 or AM312)
- **HC-SR04 Ultrasonic Distance Sensor** (or 3.3 V-compatible HC-SR04P)
- **2× Servo Motors** (e.g., MG996R / DS3218 `180°` servos, `500–2500 µs`)
- **Passive Piezo Buzzer** (or transistor-driven buzzer module; active buzzers will beep/click instead of sweeping pitch)
- **External 5 V Power Supply** (recommended for the 2 servo motors and MQ-2 heater) + jumper wires
- **Resistors** (`1 kΩ` + `2 kΩ` or `10 kΩ` + `20 kΩ`) to build a voltage divider for the HC-SR04 `ECHO` pin

---

## Wiring & Pinout

| Component | Module Pin | ESP32 GPIO | Direction / Mode | Notes |
| --- | --- | --- | --- | --- |
| **Flame Sensor** | `AO` | `GPIO34` | Analog Input (`ADC1`) | Input-only pin; triggers when `<= 500` |
| **MQ-2 Smoke Sensor** | `AO` | `GPIO35` | Analog Input (`ADC1`) | Input-only pin; triggers when `>= 2000` |
| **PIR Motion Sensor** | `OUT` | `GPIO27` | Digital Input (`INPUT_PULLDOWN`) | Active `HIGH` (3.3 V logic) |
| **Ultrasonic Sensor** | `TRIG` | `GPIO32` | Digital Output | 10 µs trigger pulse every `60 ms` |
| **Ultrasonic Sensor** | `ECHO` | `GPIO33` | Digital Input | **Use 5 V → 3.3 V divider** if powered at 5 V |
| **Buzzer** | `SIG` / `+` | `GPIO25` | LEDC Tone PWM (8-bit) | Sweeps `700 Hz ↔ 1800 Hz` on alarm |
| **Gate Servo 1 (Left)** | `PWM` | `GPIO18` | `ESP32Servo` 50 Hz PWM (16-bit) | `0°` closed, `180°` open (`500–2500 µs`, smooth ramp) |
| **Gate Servo 2 (Right)** | `PWM` | `GPIO19` | `ESP32Servo` 50 Hz PWM (16-bit) | `0°` closed, `180°` open (`500–2500 µs`, smooth ramp) |

### Important Electrical Notes

- **Ultrasonic `ECHO` Voltage Divider**: Standard HC-SR04 modules powered from 5 V output a 5 V pulse on `ECHO`, while ESP32 GPIOs are 3.3 V tolerant. Step `ECHO` down using a resistor divider (`ECHO → 1 kΩ → GPIO33 → 2 kΩ → GND`) or use a 3.3 V-compatible HC-SR04P module.
- **Servo Power Supply**: Two high-torque servos (`MG996R` / `DS3218`) moving together draw significant peak current. Power the servos (and MQ-2 heater if needed) from an external 5 V supply and connect its **`GND` to the ESP32 `GND`** (common ground) to avoid ESP32 brownout resets.
- **Timer Allocation (`ESP32Servo` + Buzzer LEDC)**: The two gate servos use `ESP32Servo` (`50 Hz`, `16-bit`, `500–2500 µs` pulses, ramped at `2° / 20 ms`) explicitly allocated on LEDC timer `0`, while the buzzer siren (`8-bit`, `700–1800 Hz`) uses native Arduino-ESP32 core 3.x LEDC on a separate timer.

---

## Configuration & Calibration

All tunable parameters are located near the top of [`src/main.cpp`](src/main.cpp):

| Constant | Default | Purpose |
| --- | --- | --- |
| `FIRE_THRESHOLD` | `500` | Flame detected when ADC is at or below this value |
| `SMOKE_THRESHOLD` | `2000` | Smoke/gas detected when ADC is at or above this value |
| `GATE_TRIGGER_DISTANCE_CM` | `25.0f` | Ultrasonic distance threshold (cm) to open the gate |
| `GATE_CONFIRM_READS` | `2` | Consecutive in-range pings required before opening |
| `GATE_HOLD_OPEN_MS` | `3000` | Milliseconds to hold gate open after path clears |
| `SERVO1_CLOSED_ANGLE` / `SERVO1_OPEN_ANGLE` | `0` / `180` | Closed and open angles for Servo 1 |
| `SERVO2_CLOSED_ANGLE` / `SERVO2_OPEN_ANGLE` | `0` / `180` | Closed and open angles for Servo 2 |
| `SIREN_LOW_HZ` / `SIREN_HIGH_HZ` | `700` / `1800` | Buzzer siren sweep frequency range |

> **Tip (Mirrored Gate Arms):** If Servo 2 is mounted facing the opposite direction on a double-leaf gate, swap its angles in [`src/main.cpp`](src/main.cpp) to `SERVO2_CLOSED_ANGLE = 180` and `SERVO2_OPEN_ANGLE = 0`.

---

## Getting Started (PlatformIO)

### Build Firmware
```bash
pio run
```

### Upload to ESP32
```bash
pio run --target upload
```

### Open Serial Monitor (`115200` baud)
```bash
pio device monitor
```

### Example Serial Output
```text
Smart home detection started
Flame GPIO34 (<= 500), Smoke MQ-2 GPIO35 (>= 2000), PIR GPIO27, Buzzer GPIO25
Ultrasonic TRIG GPIO32, ECHO GPIO33 (<= 25.0cm), Gate servos GPIO18 & GPIO19
Flame ADC=4095 fire=NO | Smoke ADC=410 smoke=NO | PIR motion=NO | Dist=--- gate=CLOSED(0deg) | sirenHz=0
Flame ADC=4095 fire=NO | Smoke ADC=415 smoke=NO | PIR motion=YES | Dist=14.6cm gate=OPEN(180deg) | sirenHz=1120
```

---

## Project Structure

```text
.
├── platformio.ini   # PlatformIO ESP32 environment & serial monitor configuration
├── src/
│   └── main.cpp     # Sensor polling, buzzer alarm siren, and dual-servo gate logic
├── include/         # Project header files
├── lib/             # Project-specific libraries
├── test/            # PlatformIO unit tests
└── README.md        # Project documentation
```
