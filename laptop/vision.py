#!/usr/bin/env python3
"""
MirrorHand - Laptop Vision & 5-Finger Hand Tracking Node
========================================================
Tracks the user's dominant hand using a webcam and MediaPipe Hands.
Computes normalized finger curl metrics (0.0 = open, 1.0 = closed) for ALL FIVE
fingers (Thumb, Index, Middle, Ring, Pinky), smooths them with an Exponential
Moving Average (EMA), computes a configurable combined average across selected
fingers, renders an on-screen HUD overlay, and streams ASCII data packets
to the microcontroller over USB serial.

Data Protocol:
    T:<combined_curl>,I:<combined_curl>\n
    e.g., "T:0.65,I:0.65\n" (20 updates/sec, values 0.0 to 1.0)
    Both T and I fields receive the combined average so existing 2-servo firmware
    works completely unchanged.
"""

import argparse
import os
import sys
import time
import math
from typing import Dict, List, Optional, Tuple

# Set matplotlib temp config dir to prevent startup cache warnings
os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import cv2
import mediapipe as mp
import serial
import serial.tools.list_ports

# ==============================================================================
# CONFIGURATION & TUNABLE CONSTANTS
# ==============================================================================

# Fingers to include when computing the single combined curl setpoint for the glove
# Options: "thumb", "index", "middle", "ring", "pinky"
AVERAGED_FINGERS = ["thumb", "index", "middle", "ring", "pinky"]

# Serial Communication
SEND_HZ = 20                   # Rate of serial packet updates to Arduino (Hz)
SERIAL_BAUD = 115200           # USB UART Baud rate
ARDUINO_USB_VID = 0x2341       # Official Arduino USB Vendor ID (UNO R4, etc.)

# Smoothing & Filtering
# Alpha for Exponential Moving Average (0.0 < alpha <= 1.0).
# Lower = smoother motion but more latency; Higher = more responsive but more jitter.
SMOOTHING_ALPHA = 0.35

# Calibration Durations (seconds)
CALIBRATION_OPEN_SECONDS = 3.0
CALIBRATION_CLOSED_SECONDS = 3.0

# Camera Settings
CAMERA_INDEX = 0               # Default webcam index (0 is usually built-in camera)
FRAME_WIDTH = 1280             # Capture frame width
FRAME_HEIGHT = 720             # Capture frame height

# Default Fallback Distance Ranges (normalized by palm size: wrist to middle MCP)
# Used prior to calibration or if user bypasses calibration.
# Thumb: [thumb_tip-to-pinky_mcp / palm_scale] (Open ~1.50, Closed ~0.60)
# Index/Middle/Ring/Pinky: [tip-to-wrist / palm_scale]
DEFAULT_FINGER_RANGES = {
    "thumb":  {"open": 1.50, "closed": 0.60},
    "index":  {"open": 1.85, "closed": 0.75},
    "middle": {"open": 2.00, "closed": 0.80},
    "ring":   {"open": 1.90, "closed": 0.75},
    "pinky":  {"open": 1.60, "closed": 0.70},
}

FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]

# ==============================================================================
# SERIAL CONNECTION HELPERS
# ==============================================================================

def find_arduino_port() -> Optional[str]:
    """
    Auto-detect the serial port corresponding to an Arduino (UNO R4 WiFi).
    Checks USB Vendor ID (0x2341) or common Arduino descriptor strings.
    """
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        # Check USB VID for Arduino
        if port.vid == ARDUINO_USB_VID:
            return port.device
        # Check description / manufacturer / hwid strings
        desc = (port.description or "").lower()
        mfg = (port.manufacturer or "").lower()
        if "arduino" in desc or "uno r4" in desc or "arduino" in mfg:
            return port.device

    # Fallback: Check for common USB-Serial device patterns on macOS/Linux
    for port in ports:
        dev = port.device.lower()
        if "usbmodem" in dev or "ttyacm" in dev or "ttyusb" in dev:
            return port.device

    return None


def init_serial(port_override: Optional[str], no_serial: bool) -> Optional[serial.Serial]:
    """
    Initialize and return a serial connection to the Arduino.
    If no_serial is True, returns None.
    """
    if no_serial:
        print("[MirrorHand] Serial communication disabled by --no-serial flag.")
        return None

    port = port_override
    if not port:
        print("[MirrorHand] Auto-detecting Arduino serial port...")
        port = find_arduino_port()

    if not port:
        print("\n" + "=" * 60)
        print("ERROR: Arduino serial port not found!")
        print("Available serial ports:")
        available = list(serial.tools.list_ports.comports())
        if available:
            for p in available:
                print(f"  - {p.device} ({p.description}) [VID:{hex(p.vid) if p.vid else 'None'}]")
        else:
            print("  (No serial devices detected)")
        print("\nTroubleshooting tips:")
        print(" 1. Check the USB cable is connected to the Arduino UNO R4.")
        print(" 2. Specify the port manually using: python laptop/vision.py --port <PORT>")
        print(" 3. Run without hardware using:      python laptop/vision.py --no-serial")
        print("=" * 60 + "\n")
        return None

    try:
        ser = serial.Serial(port=port, baudrate=SERIAL_BAUD, timeout=0.1, write_timeout=0.1)
        # Flush initial buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print(f"[MirrorHand] Successfully connected to Arduino on {port} @ {SERIAL_BAUD} baud.")
        return ser
    except serial.SerialException as e:
        print(f"[MirrorHand] Serial connection failed on {port}: {e}")
        print("Hint: Check permissions or if another program (e.g. Arduino Serial Monitor) is using the port.")
        return None

# ==============================================================================
# KINEMATICS & 5-FINGER CURL CALCULATION
# ==============================================================================

def euclidean_dist(p1, p2) -> float:
    """Compute 2D Euclidean distance between two MediaPipe landmarks."""
    return math.hypot(p1.x - p2.x, p1.y - p2.y)


def compute_raw_finger_metrics(landmarks) -> Tuple[Dict[str, float], float]:
    """
    Computes normalized distance metrics for ALL FIVE fingers.

    --- Normalization Rationale ---
    Camera distance variations cause the absolute pixel distances of fingers
    to change as the user moves forward or backward.
    To make tracking distance-invariant, we normalize all finger measurements
    by the 'palm scale' — the distance from the Wrist (Landmark 0) to the
    Middle Finger Metacarpophalangeal Joint (MCP, Landmark 9).
    The palm distance (0 -> 9) remains constant regardless of finger curling.

    --- Finger Curl Kinematics ---
    1. Thumb: Saddle CMC articulation flexes & opposes ACROSS the palm toward
       the Pinky MCP Joint (Landmark 17).
       Distance: Thumb Tip (4) -> Pinky MCP (17) / palm_scale.
       - Open: Large ratio (~1.50)
       - Closed: Small ratio (~0.60)

    2. Index, Middle, Ring, Pinky:
       Planar flexion towards the palm base/wrist.
       Distance: Fingertip -> Wrist (0) / palm_scale.
       - Index:  Tip (8)  -> Wrist (0)
       - Middle: Tip (12) -> Wrist (0)
       - Ring:   Tip (16) -> Wrist (0)
       - Pinky:  Tip (20) -> Wrist (0)
       - Open: Large ratio (~1.6 - 2.0)
       - Closed: Small ratio (~0.65 - 0.80)
    """
    # Landmark definitions:
    # 0: Wrist, 4: Thumb Tip, 8: Index Tip, 12: Middle Tip, 16: Ring Tip, 20: Pinky Tip
    # 9: Middle MCP (reference), 17: Pinky MCP
    wrist = landmarks[0]
    middle_mcp = landmarks[9]
    pinky_mcp = landmarks[17]

    thumb_tip = landmarks[4]
    index_tip = landmarks[8]
    middle_tip = landmarks[12]
    ring_tip = landmarks[16]
    pinky_tip = landmarks[20]

    # Reference palm scale (distance invariant)
    palm_scale = euclidean_dist(wrist, middle_mcp)
    if palm_scale < 1e-4:
        palm_scale = 1e-4  # Prevent division by zero

    # Raw normalized distance ratios
    raw_metrics = {
        "thumb":  euclidean_dist(thumb_tip, pinky_mcp) / palm_scale,
        "index":  euclidean_dist(index_tip, wrist) / palm_scale,
        "middle": euclidean_dist(middle_tip, wrist) / palm_scale,
        "ring":   euclidean_dist(ring_tip, wrist) / palm_scale,
        "pinky":  euclidean_dist(pinky_tip, wrist) / palm_scale,
    }

    return raw_metrics, palm_scale


def normalize_to_curl(raw_dist: float, open_dist: float, closed_dist: float) -> float:
    """
    Maps a raw distance metric to a curl value between 0.0 (Open) and 1.0 (Closed).
    Clamps the result to [0.0, 1.0].
    """
    if abs(open_dist - closed_dist) < 1e-4:
        return 0.0
    # Higher distance = Open (0.0), Lower distance = Closed (1.0)
    curl = (open_dist - raw_dist) / (open_dist - closed_dist)
    return max(0.0, min(1.0, curl))

# ==============================================================================
# CALIBRATION CONTROLLER
# ==============================================================================

class CalibrationManager:
    """
    Handles two-stage startup calibration for all five fingers:
    Stage 1: Hold hand fully OPEN (~3s) -> records max open distances.
    Stage 2: Hold hand fully CLOSED (~3s) -> records min closed distances.
    """
    STAGE_IDLE = 0
    STAGE_OPEN = 1
    STAGE_CLOSED = 2
    STAGE_DONE = 3

    def __init__(self):
        self.stage = self.STAGE_OPEN
        self.stage_start_time = time.time()

        self.open_samples: Dict[str, List[float]] = {f: [] for f in FINGER_NAMES}
        self.closed_samples: Dict[str, List[float]] = {f: [] for f in FINGER_NAMES}

        self.finger_open: Dict[str, float] = {f: DEFAULT_FINGER_RANGES[f]["open"] for f in FINGER_NAMES}
        self.finger_closed: Dict[str, float] = {f: DEFAULT_FINGER_RANGES[f]["closed"] for f in FINGER_NAMES}

    def reset(self):
        """Restart calibration sequence."""
        self.stage = self.STAGE_OPEN
        self.stage_start_time = time.time()
        for f in FINGER_NAMES:
            self.open_samples[f].clear()
            self.closed_samples[f].clear()
        print("[Calibration] Restarting calibration. Hold hand OPEN...")

    def update(self, raw_metrics: Optional[Dict[str, float]]):
        """Feed current raw metrics into calibration state machine."""
        now = time.time()
        elapsed = now - self.stage_start_time

        if self.stage == self.STAGE_OPEN:
            if raw_metrics is not None:
                for f in FINGER_NAMES:
                    self.open_samples[f].append(raw_metrics[f])

            if elapsed >= CALIBRATION_OPEN_SECONDS:
                for f in FINGER_NAMES:
                    if len(self.open_samples[f]) > 5:
                        self.finger_open[f] = sum(self.open_samples[f]) / len(self.open_samples[f])
                print("[Calibration] Open recorded:",
                      ", ".join(f"{f.capitalize()}={self.finger_open[f]:.2f}" for f in FINGER_NAMES))
                self.stage = self.STAGE_CLOSED
                self.stage_start_time = now
                print("[Calibration] Now hold hand fully CLOSED (tight fist)...")

        elif self.stage == self.STAGE_CLOSED:
            if raw_metrics is not None:
                for f in FINGER_NAMES:
                    self.closed_samples[f].append(raw_metrics[f])

            if elapsed >= CALIBRATION_CLOSED_SECONDS:
                for f in FINGER_NAMES:
                    if len(self.closed_samples[f]) > 5:
                        self.finger_closed[f] = sum(self.closed_samples[f]) / len(self.closed_samples[f])

                    # Sanity check: open distance must be greater than closed distance
                    if self.finger_open[f] <= self.finger_closed[f]:
                        self.finger_open[f] = self.finger_closed[f] + 0.35

                print("[Calibration] Closed recorded:",
                      ", ".join(f"{f.capitalize()}={self.finger_closed[f]:.2f}" for f in FINGER_NAMES))
                print("[Calibration] 5-Finger calibration complete! Tracking active.")
                self.stage = self.STAGE_DONE

    def get_remaining_time(self) -> float:
        now = time.time()
        elapsed = now - self.stage_start_time
        if self.stage == self.STAGE_OPEN:
            return max(0.0, CALIBRATION_OPEN_SECONDS - elapsed)
        elif self.stage == self.STAGE_CLOSED:
            return max(0.0, CALIBRATION_CLOSED_SECONDS - elapsed)
        return 0.0

# ==============================================================================
# UI & HUD RENDERING
# ==============================================================================

# Distinct color coding per finger (BGR format)
FINGER_COLORS = {
    "thumb":  (255, 180, 0),    # Amber / Orange
    "index":  (0, 220, 255),    # Cyan / Yellow-Blue
    "middle": (80, 230, 80),    # Emerald Green
    "ring":   (230, 100, 200),  # Pink / Magenta
    "pinky":  (100, 160, 255),  # Coral / Light Blue
}

def draw_hud(frame, finger_curls: Dict[str, float], combined_curl: float,
             calib: CalibrationManager, is_manual: bool,
             serial_connected: bool, port_name: str):
    """Draw rich HUD overlay with all 5 finger curl bars and combined average."""
    h, w, _ = frame.shape

    card_w = 430
    card_h = 390

    # Semi-transparent background card for telemetry (Top Left)
    overlay = frame.copy()
    cv2.rectangle(overlay, (15, 15), (15 + card_w, 15 + card_h), (20, 24, 30), -1)
    cv2.addWeighted(overlay, 0.78, frame, 0.22, 0, frame)
    cv2.rectangle(frame, (15, 15), (15 + card_w, 15 + card_h), (70, 80, 95), 1)

    # Title
    cv2.putText(frame, "MIRRORHAND 5-FINGER TELEMETRY", (30, 42),
                cv2.FONT_HERSHEY_DUPLEX, 0.58, (0, 220, 255), 1, cv2.LINE_AA)

    # Mode Indicator
    if is_manual:
        mode_text = "MODE: MANUAL OVERRIDE (Keys active)"
        mode_color = (0, 165, 255)  # Orange
    elif calib.stage != CalibrationManager.STAGE_DONE:
        mode_text = "MODE: CALIBRATION"
        mode_color = (0, 255, 255)  # Yellow
    else:
        mode_text = "MODE: 5-FINGER TRACKING"
        mode_color = (0, 255, 0)    # Green
    cv2.putText(frame, mode_text, (30, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.44, mode_color, 1, cv2.LINE_AA)

    # Serial Status
    if serial_connected:
        ser_text = f"SERIAL: CONNECTED ({port_name})"
        ser_color = (0, 255, 100)
    else:
        ser_text = f"SERIAL: {port_name.upper()}"
        ser_color = (120, 120, 255)
    cv2.putText(frame, ser_text, (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.44, ser_color, 1, cv2.LINE_AA)

    # Individual Finger Progress Bars (Thumb, Index, Middle, Ring, Pinky)
    bar_y = 114
    for f in FINGER_NAMES:
        val = finger_curls.get(f, 0.0)
        color = FINGER_COLORS.get(f, (200, 200, 200))
        is_active_in_avg = f in AVERAGED_FINGERS
        tag = "" if is_active_in_avg else " (excl)"
        label = f"{f.upper()}{tag}"
        draw_curl_bar(frame, 30, bar_y, label, val, color, bar_w=240, bar_h=14)
        bar_y += 36

    # Separator Line
    cv2.line(frame, (30, bar_y + 4), (30 + 370, bar_y + 4), (60, 70, 85), 1)
    bar_y += 18

    # Combined Average Progress Bar (Sent to hardware)
    active_names = ",".join(f[0].upper() for f in AVERAGED_FINGERS)
    avg_label = f"COMBINED ({active_names})"
    draw_curl_bar(frame, 30, bar_y, avg_label, combined_curl, (0, 255, 255), bar_w=240, bar_h=16, is_bold=True)

    # Controls footer inside card
    cv2.putText(frame, "[C] Calibrate  [M] Manual  [Q] Quit", (30, 15 + card_h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1, cv2.LINE_AA)

    # Calibration Banner (Center Top)
    if calib.stage == CalibrationManager.STAGE_OPEN:
        draw_banner(frame, f"HOLD HAND OPEN (All 5 Fingers Spread)... {calib.get_remaining_time():.1f}s", (0, 255, 255))
    elif calib.stage == CalibrationManager.STAGE_CLOSED:
        draw_banner(frame, f"HOLD HAND CLOSED (Full Fist / Pinch)... {calib.get_remaining_time():.1f}s", (0, 180, 255))
    elif is_manual:
        draw_banner(frame, "MANUAL MODE: [Up/W]=Close Combined  [Down/S]=Open Combined", (0, 165, 255))


def draw_curl_bar(frame, x: int, y: int, label: str, val: float, color: Tuple[int, int, int],
                  bar_w: int = 240, bar_h: int = 14, is_bold: bool = False):
    """Draw a single finger curl progress bar."""
    text = f"{label:<14} {int(val * 100):3d}% ({val:.2f})"
    font = cv2.FONT_HERSHEY_DUPLEX if is_bold else cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.45 if is_bold else 0.42
    font_color = (255, 255, 255) if is_bold else (220, 220, 220)
    cv2.putText(frame, text, (x, y), font, font_scale, font_color, 1, cv2.LINE_AA)

    # Bar background
    by = y + 6
    cv2.rectangle(frame, (x, by), (x + bar_w, by + bar_h), (40, 45, 55), -1)
    cv2.rectangle(frame, (x, by), (x + bar_w, by + bar_h), (90, 100, 115), 1)

    # Filled portion
    fill_w = int(bar_w * max(0.0, min(1.0, val)))
    if fill_w > 0:
        cv2.rectangle(frame, (x + 1, by + 1), (x + fill_w - 1, by + bar_h - 1), color, -1)


def draw_banner(frame, text: str, color: Tuple[int, int, int]):
    """Draw prominent notification banner at top of window."""
    h, w, _ = frame.shape
    text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_DUPLEX, 0.65, 1)[0]
    bx = (w - text_size[0]) // 2
    by = 45
    cv2.rectangle(frame, (bx - 15, by - 25), (bx + text_size[0] + 15, by + 10), (15, 15, 20), -1)
    cv2.rectangle(frame, (bx - 15, by - 25), (bx + text_size[0] + 15, by + 10), color, 2)
    cv2.putText(frame, text, (bx, by), cv2.FONT_HERSHEY_DUPLEX, 0.65, color, 1, cv2.LINE_AA)

# ==============================================================================
# MAIN VISION LOOP
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(description="MirrorHand Webcam 5-Finger Vision & Tracking Node")
    parser.add_argument("--no-serial", action="store_true", help="Run vision tracking only without serial connection")
    parser.add_argument("--port", type=str, default=None, help="Explicit serial port for Arduino")
    parser.add_argument("--camera", type=int, default=CAMERA_INDEX, help="Webcam device index")
    args = parser.parse_args()

    print("==================================================")
    print("   MIRRORHAND: 5-FINGER DOMINANT HAND TRACKER     ")
    print(f"   Averaged Fingers: {', '.join(AVERAGED_FINGERS)}")
    print("==================================================")

    # Initialize Serial
    ser = init_serial(args.port, args.no_serial)
    port_label = args.port if args.port else ("NO-SERIAL" if args.no_serial else "DISCONNECTED")
    if ser:
        port_label = ser.port

    # Open Camera
    print(f"[MirrorHand] Opening webcam index {args.camera}...")
    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not cap.isOpened():
        print(f"\nERROR: Could not open camera at index {args.camera}!")
        print("Suggestions:")
        print(" 1. Check camera permissions in System Settings.")
        print(" 2. Test a different camera index with: python laptop/vision.py --camera 1")
        if ser:
            ser.close()
        sys.exit(1)

    # Initialize MediaPipe Hands
    # max_num_hands=1 to focus exclusively on the dominant hand
    mp_hands = mp.solutions.hands
    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        min_detection_confidence=0.6,
        min_tracking_confidence=0.6
    )

    # State variables
    calib = CalibrationManager()
    is_manual = False

    # 5-finger curl values (0.0 to 1.0)
    finger_curls: Dict[str, float] = {f: 0.0 for f in FINGER_NAMES}
    manual_curl = 0.0

    last_send_time = 0.0
    send_interval = 1.0 / SEND_HZ

    print("\n[MirrorHand] System Ready. Starting video feed.")
    print("Commands: [C] Calibrate | [M] Manual Override | [Q] Quit\n")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[MirrorHand] Warning: Failed to capture camera frame. Retrying...")
                time.sleep(0.05)
                continue

            # Flip horizontally for natural mirror selfie view
            frame = cv2.flip(frame, 1)
            h, w, _ = frame.shape

            # Convert BGR to RGB for MediaPipe
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb_frame.flags.writeable = False
            results = hands.process(rgb_frame)
            rgb_frame.flags.writeable = True

            raw_metrics = None

            # Process hand landmarks if detected
            if results.multi_hand_landmarks:
                hand_landmarks = results.multi_hand_landmarks[0]

                # Draw landmark mesh
                mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style()
                )

                # Compute normalized distance metrics for all 5 fingers
                raw_metrics, palm_scale = compute_raw_finger_metrics(hand_landmarks.landmark)

                # Update calibration if running
                if calib.stage != CalibrationManager.STAGE_DONE:
                    calib.update(raw_metrics)

                # If tracking normally and not in manual override, compute smoothed curl per finger
                if not is_manual and calib.stage == CalibrationManager.STAGE_DONE:
                    for f in FINGER_NAMES:
                        target = normalize_to_curl(raw_metrics[f], calib.finger_open[f], calib.finger_closed[f])
                        # Exponential Moving Average (EMA) smoothing
                        finger_curls[f] = SMOOTHING_ALPHA * target + (1.0 - SMOOTHING_ALPHA) * finger_curls[f]

            else:
                # Hand lost: if not in manual mode, gently decay curls toward 0.0 (open)
                if not is_manual and calib.stage == CalibrationManager.STAGE_DONE:
                    for f in FINGER_NAMES:
                        finger_curls[f] = max(0.0, finger_curls[f] - 0.05)

            # Compute combined average across configured AVERAGED_FINGERS
            if is_manual:
                combined_val = manual_curl
                # Mirror manual setpoint to all displayed fingers
                display_curls = {f: manual_curl for f in FINGER_NAMES}
            else:
                active_curls = [finger_curls[f] for f in AVERAGED_FINGERS if f in finger_curls]
                combined_val = sum(active_curls) / len(active_curls) if active_curls else 0.0
                display_curls = finger_curls

            # Clamp output safely to [0.0, 1.0]
            out_combined = max(0.0, min(1.0, combined_val))

            # Send serial packet at SEND_HZ
            # Format: T:<val>,I:<val>\n (sending combined average in both fields)
            now = time.time()
            if (now - last_send_time) >= send_interval:
                last_send_time = now
                if ser and ser.is_open:
                    packet = f"T:{out_combined:.2f},I:{out_combined:.2f}\n"
                    try:
                        ser.write(packet.encode("ascii"))
                    except serial.SerialException as e:
                        print(f"[MirrorHand] Serial write error: {e}")

            # Draw HUD
            draw_hud(frame, display_curls, out_combined, calib, is_manual,
                     ser is not None and ser.is_open, port_label)

            cv2.imshow("MirrorHand Tracker", frame)

            # Keyboard Handling
            key = cv2.waitKeyEx(1)
            if key != -1:
                # Support standard keys & common arrow key codes cross-platform
                # 'q' or ESC -> Quit
                if key in (ord('q'), ord('Q'), 27):
                    print("[MirrorHand] Quit requested by user.")
                    break
                # 'c' -> Recalibrate
                elif key in (ord('c'), ord('C')):
                    calib.reset()
                # 'm' -> Toggle Manual Override
                elif key in (ord('m'), ord('M')):
                    is_manual = not is_manual
                    manual_curl = out_combined
                    print(f"[MirrorHand] Manual Override mode: {'ENABLED' if is_manual else 'DISABLED'}")

                # Manual Controls in Override Mode
                if is_manual:
                    STEP = 0.05
                    # Up Arrow / 'w' / 'k' / Right Arrow -> Increase Combined Curl (close)
                    if key in (ord('w'), ord('k'), ord('d'), 65362, 2490368, 65363, 2555904, 0, 3):
                        manual_curl = min(1.0, manual_curl + STEP)
                    # Down Arrow / 's' / 'j' / Left Arrow -> Decrease Combined Curl (open)
                    elif key in (ord('s'), ord('j'), ord('a'), 65364, 2621440, 65361, 2424832, 1, 2):
                        manual_curl = max(0.0, manual_curl - STEP)

    except KeyboardInterrupt:
        print("\n[MirrorHand] Terminated via keyboard interrupt.")

    finally:
        # Failsafe shutdown: open all fingers and release resources
        print("[MirrorHand] Performing clean shutdown...")
        if ser and ser.is_open:
            try:
                # Send explicit full-open packet
                ser.write(b"T:0.00,I:0.00\n")
                ser.flush()
                time.sleep(0.05)
                ser.close()
                print("[MirrorHand] Sent open command (T:0.0,I:0.0) and closed serial port.")
            except Exception as e:
                print(f"[MirrorHand] Error during serial shutdown: {e}")

        cap.release()
        cv2.destroyAllWindows()
        hands.close()
        print("[MirrorHand] Shutdown complete. Goodbye!")

if __name__ == "__main__":
    main()
