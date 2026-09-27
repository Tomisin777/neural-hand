# MirrorHand: Assistive Hand Rehabilitation & Mirroring Glove

> **Medical Disclaimer**: MirrorHand is a 24-hour hackathon research prototype designed to explore assistive device concepts and mirror therapy for hemiplegic cerebral palsy. It is **NOT** a medical device and is not certified for clinical use.

---

## 🖐️ Project Overview

MirrorHand tracks the user's dominant hand in real-time using a standard laptop webcam and computer vision (MediaPipe Hands). It computes normalized finger curl metrics (0.0 = open, 1.0 = fully closed) for the **Thumb** and **Index** finger and streams them via USB serial to an **Arduino UNO R4 WiFi**. The Arduino actuates two Tower Pro SG92R micro servos mounted on an assistive glove/brace worn on the contralateral (weaker) hand, pulling tendon cords to mirror the user's grasp. Elastic return bands reopen the fingers when tension is released.

---

## ⚡ Hardware & Wiring Specification

### Bill of Materials
- **Microcontroller**: Arduino UNO R4 WiFi (Renesas RA4M1 32-bit ARM Cortex-M4)
- **Servos**: 2× Tower Pro SG92R Micro Servos (Pin 9 = Index, Pin 10 = Thumb)
- **Power**: External 5V Power Supply (e.g., USB 5V power bank or 4×AA battery pack)
- **Safety**: 1× Pushbutton (Momentary NO switch) for emergency kill switch
- **Glove & Tendons**: Lightweight glove/brace, low-stretch fishing line (tendons), elastic rubber bands (return springs), 3D-printed or spool servo horns.

### Arduino UNO R4 WiFi Wiring Table

| Component | Wire Color / Terminal | Arduino UNO R4 Pin | External 5V Power Supply |
| :--- | :--- | :--- | :--- |
| **Index Servo** | Signal (Orange / Yellow) | **Digital Pin 9** (PWM) | — |
| | Power (Red) | — | **+5V (VCC)** |
| | Ground (Brown / Black) | **GND** | **GND (Common)** |
| **Thumb Servo** | Signal (Orange / Yellow) | **Digital Pin 10** (PWM) | — |
| | Power (Red) | — | **+5V (VCC)** |
| | Ground (Brown / Black) | **GND** | **GND (Common)** |
| **Kill Switch** | Terminal A | **Digital Pin 2** (`INPUT_PULLUP`)| — |
| | Terminal B | **GND** | — |
| **Power Ground**| Ground Reference | **GND** (Power Header) | **GND (Common)** |

> ⚠️ **CRITICAL: Common Ground & Power Separation**
> - **NEVER** power the servos directly from the Arduino's 5V or 3.3V header pins. Servo stall/starting currents can exceed 800mA each, causing voltage dips that reset the microcontroller.
> - Connect the **GND of the external 5V supply directly to an Arduino GND pin** so signal PWM levels share a common reference.

---

## 💻 Software Architecture & Protocol

### Data Protocol (Laptop $\to$ Arduino)
- **Medium**: USB Serial (CDC) @ `115200` baud.
- **Update Rate**: 20 Hz (`SEND_HZ = 20`, 50ms interval).
- **Format**: Single newline-terminated ASCII string:
  ```text
  T:<thumb_curl>,I:<index_curl>\n
  ```
  Example: `T:0.42,I:0.80\n`
- **Tolerant Parsing**: Malformed lines are safely ignored without blocking the control loop.

---

## 🚀 Quickstart Installation & Setup

### 1. Arduino IDE Setup (UNO R4 Board Package)
1. Download and install [Arduino IDE 2.x](https://www.arduino.cc/en/software).
2. Open **Boards Manager** (`Tools` $\to$ `Board` $\to$ `Boards Manager...` or `Cmd+Shift+B` on Mac).
3. Search for **"Arduino UNO R4 Boards"** by Arduino and click **Install**.
4. Select your board under **Tools $\to$ Board $\to$ Arduino UNO R4 Boards $\to$ Arduino UNO R4 WiFi**.
5. Select your port under **Tools $\to$ Port** (e.g., `/dev/cu.usbmodem...` on Mac or `COM...` on Windows).
6. The official `Servo` library is bundled directly with the core.

### 2. Upload Firmware to the Arduino
- **Main Firmware**: Open `arduino/mirrorhand/mirrorhand.ino` in Arduino IDE and click **Upload** ($\rightarrow$).
- **Single Servo Diagnostic**: Open `arduino/servo_test/servo_test.ino` and click **Upload** to bench-test a single motor on Pin 9.

### 3. Setup Laptop Python Environment
```bash
# Clone or navigate to repository
cd HackUMBC

# Create & activate Python virtual environment (Python 3.9 - 3.12)
python3 -m venv venv
source venv/bin/activate    # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

---

## 🧪 Four-Stage Verification Test Plan

Follow this 4-stage sequential checklist to ensure safe hardware bring-up and prevent stripping servo gears.

### Stage 1: Vision Only (No Arduino Hardware Needed)
Verify webcam tracking, lighting, and finger curl calculations on screen:
```bash
python laptop/vision.py --no-serial
```
**Checklist:**
1. Stand 2-3 feet from camera with good lighting.
2. Follow the 3-second **OPEN** prompt (spread fingers).
3. Follow the 3-second **CLOSED** prompt (make a fist).
4. Watch the on-screen bars: opening and closing your hand should transition `THUMB` and `INDEX` smoothly from `0%` to `100%`.
5. Press `c` to recalibrate; press `m` to toggle manual override; press `q` to exit.

### Stage 2: Servo Bench Test (Arduino Standalone)
Verify servo motor operation, angle limits, and failsafes before installing on the glove.
1. Upload `arduino/servo_test/servo_test.ino` to the Arduino.
   - Open Serial Monitor at **115200 baud**.
   - Verify the servo on Pin 9 sweeps smoothly between $20^\circ$ and $140^\circ$ without buzzing or hitting hard mechanical stops.
2. Upload `arduino/mirrorhand/mirrorhand.ino` to the Arduino.
   - In Serial Monitor, type `T:0.60,I:0.60` and press Enter.
   - **Timeout Check**: Stop sending commands. Within 1.0 second, the servos should smoothly ramp back to $0.0$ (open position).
   - **Kill Switch Check**: Send `T:1.0,I:1.0` and press the pushbutton on Pin 2. The servos must immediately relax to $0.0$ and ignore serial inputs until released.

### Stage 3: Mechanical Tendon Tuning (CLI Pull Tester)
Tune tendon spool lines, return band tension, and stroke length without camera noise:
```bash
python laptop/pull_test.py
```
**Checklist:**
1. Enter `0` to verify the finger is fully extended.
2. Enter `25`, `50`, `75`, `100` incrementally while observing the glove:
   - Ensure the fishing line winds smoothly onto the servo horn without jumping grooves.
   - Verify the elastic band pulls the finger straight when relaxed to `0`.
   - Check that the servo does not stall or buzz at `100%`. If it stalls, lower `INDEX_MAX_ANGLE` or `THUMB_MAX_ANGLE` in `arduino/mirrorhand/mirrorhand.ino`.
3. Press `q` to quit (relaxes glove to 0%).

### Stage 4: Full End-to-End System
Run complete real-time mirror therapy tracking with camera and glove:
```bash
python laptop/vision.py
```
*(If auto-detection does not pick up your port automatically, specify it explicitly with `--port /dev/cu.usbmodem...` on Mac or `--port COM...` on Windows).*

---

## ⚙️ Configuration & Tunable Parameters

All configurable constants are placed at the top of their respective files:

### `arduino/mirrorhand/mirrorhand.ino`
| Constant | Default | Description |
| :--- | :--- | :--- |
| `NUM_SERVOS` | `2` | Set to `1` if only index finger servo (Pin 9) is fitted; `2` for thumb (Pin 10) + index (Pin 9). |
| `INDEX_MIN_ANGLE` | `20.0f` | Open position angle (deg) for index servo on Pin 9. |
| `INDEX_MAX_ANGLE` | `140.0f` | Closed position angle (deg) for index servo on Pin 9. |
| `INDEX_INVERT` | `false` | Set `true` if rotating in opposite direction pulls tendon. |
| `THUMB_MIN_ANGLE` | `20.0f` | Open position angle (deg) for thumb servo on Pin 10. |
| `THUMB_MAX_ANGLE` | `140.0f` | Closed position angle (deg) for thumb servo on Pin 10. |
| `THUMB_INVERT` | `false` | Set `true` if rotating in opposite direction pulls tendon. |
| `MAX_STEP_PER_TICK` | `1.5f` | Max degrees moved per 20ms tick (~$75^\circ$/sec). Prevents current spikes. |
| `SERIAL_TIMEOUT_MS` | `1000` | Safety timeout: returns servos to open if laptop disconnects. |

### `laptop/vision.py`
| Constant | Default | Description |
| :--- | :--- | :--- |
| `SMOOTHING_ALPHA` | `0.35` | Exponential Moving Average weight. Range `0.1` (heavy smoothing) to `0.9` (instant response). |
| `SEND_HZ` | `20` | Packet transmission rate to the Arduino in Hertz (50ms period). |
| `CALIBRATION_OPEN_SECONDS` | `3.0` | Duration to record user's open hand span. |
| `CALIBRATION_CLOSED_SECONDS`| `3.0` | Duration to record user's closed fist span. |
| `CAMERA_INDEX` | `0` | Default OpenCV camera device index. |
| `ARDUINO_USB_VID` | `0x2341` | Official Arduino USB Vendor ID for auto-detection. |

---

## 🔧 Troubleshooting Guide

| Symptom | Probable Cause | Recommended Fix |
| :--- | :--- | :--- |
| **Arduino resets / brownout when servos move** | Servos pulling power through Arduino 5V pin or power supply voltage sag. | 1. Power servos with external 5V supply, **never** Arduino 5V header.<br>2. Verify common GND between external supply and Arduino GND.<br>3. Decrease `MAX_STEP_PER_TICK` to `1.0f` in `mirrorhand.ino`. |
| **Servo jitter / vibrating at rest** | Tendon over-tension or servo pressed against mechanical hard stop. | 1. Ensure `MIN_ANGLE` and `MAX_ANGLE` do not push against mechanical stops.<br>2. Increase `SMOOTHING_ALPHA` damping (e.g. `0.25`).<br>3. Add slight slack to fishing line at 0% curl. |
| **Thumb tracking noisy / jumpy** | Hand tilted or thumb tracking planar distance instead of opposition. | 1. Keep hand perpendicular to camera plane.<br>2. Perform startup calibration (`c` key) making a distinct opposition pinch.<br>3. Algorithm measures Thumb Tip $\to$ Pinky MCP; keep palm in view. |
| **Serial port not detected on Laptop** | Port occupied by Arduino IDE Serial Monitor or permissions locked. | 1. **Close Arduino IDE Serial Monitor** before running Python scripts.<br>2. On Linux: `sudo usermod -a -G dialout $USER`.<br>3. Pass port explicitly with `--port <PORT>`. |
| **Webcam cannot open** | OS camera privacy permissions blocked. | Check macOS *System Settings $\to$ Privacy & Security $\to$ Camera* and ensure your terminal is permitted. |

---

## 📐 Kinematics & Tracking Notes

### Distance Invariance
All landmark distances are normalized by the **palm scale** ($\text{Distance from Wrist (Landmark 0) to Middle MCP (Landmark 9)}$), allowing the user to move freely without distorting curl readings:

$$\text{Scale} = \|\mathbf{P}_0 - \mathbf{P}_9\|_2$$

### Thumb vs. Index Kinematics
- **Index Finger**: Flexes towards the base of the wrist:
  $$\text{Index Metric} = \frac{\|\mathbf{P}_8 - \mathbf{P}_0\|_2}{\text{Scale}}$$
- **Thumb**: Flexes and opposes *across* the palm towards the base of the pinky (Landmark 17):
  $$\text{Thumb Metric} = \frac{\|\mathbf{P}_4 - \mathbf{P}_{17}\|_2}{\text{Scale}}$$
