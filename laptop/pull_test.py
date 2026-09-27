#!/usr/bin/env python3
"""
MirrorHand - Mechanical Tendon Pull Tester
==========================================
Interactive CLI tool to test glove tendon mechanics, spool winding,
and finger return elastic tension without requiring computer vision.

Streams setpoint packets at 20 Hz to keep the Raspberry Pi Pico's
1.0-second safety watchdog alive while maintaining the desired pull position.

Usage:
    python laptop/pull_test.py
    python laptop/pull_test.py --port /dev/ttyACM0
"""

import argparse
import sys
import threading
import time
from typing import Optional

import serial
import serial.tools.list_ports

# Configuration Constants
SEND_HZ = 20
SERIAL_BAUD = 115200
ARDUINO_USB_VID = 0x2341


def find_arduino_port() -> Optional[str]:
    """Auto-detect Arduino UNO R4 serial port."""
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if port.vid == ARDUINO_USB_VID:
            return port.device
        desc = (port.description or "").lower()
        mfg = (port.manufacturer or "").lower()
        if "arduino" in desc or "uno r4" in desc or "arduino" in mfg:
            return port.device

    for port in ports:
        dev = port.device.lower()
        if "usbmodem" in dev or "ttyacm" in dev or "ttyusb" in dev:
            return port.device
    return None


class TendonPullTester:
    def __init__(self, port: Optional[str]):
        self.port_name = port or find_arduino_port()
        if not self.port_name:
            print("\n" + "=" * 60)
            print("ERROR: Arduino UNO R4 not detected!")
            print("Available ports:")
            for p in serial.tools.list_ports.comports():
                print(f"  - {p.device} ({p.description})")
            print("=" * 60 + "\n")
            sys.exit(1)

        print(f"[PullTester] Connecting to Arduino on {self.port_name} @ {SERIAL_BAUD} baud...")
        self.ser = serial.Serial(self.port_name, SERIAL_BAUD, timeout=0.1, write_timeout=0.1)
        time.sleep(1.5)  # Allow Arduino reset settling

        self.thumb_val = 0.0
        self.index_val = 0.0
        self.running = True
        self.lock = threading.Lock()

        # Background streaming thread (maintains 20 Hz stream so Pico watchdog doesn't expire)
        self.stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.stream_thread.start()

    def _stream_loop(self):
        send_interval = 1.0 / SEND_HZ
        while self.running:
            with self.lock:
                t_val = self.thumb_val
                i_val = self.index_val

            packet = f"T:{t_val:.2f},I:{i_val:.2f}\n"
            try:
                if self.ser.is_open:
                    self.ser.write(packet.encode("ascii"))
            except Exception as e:
                print(f"\n[PullTester] Warning: Serial write error: {e}")
                time.sleep(0.5)

            time.sleep(send_interval)

    def set_values(self, thumb: float, index: float):
        with self.lock:
            self.thumb_val = max(0.0, min(1.0, thumb))
            self.index_val = max(0.0, min(1.0, index))
        print(f"-> Set Target: Thumb = {self.thumb_val * 100:.0f}% ({self.thumb_val:.2f}) | "
              f"Index = {self.index_val * 100:.0f}% ({self.index_val:.2f})")

    def shutdown(self):
        print("\n[PullTester] Stopping test and relaxing servos...")
        self.running = False
        time.sleep(0.1)
        if self.ser and self.ser.is_open:
            try:
                # Send explicit 0.0 open command
                self.ser.write(b"T:0.00,I:0.00\n")
                self.ser.flush()
                time.sleep(0.05)
                self.ser.close()
                print("[PullTester] Sent T:0.0,I:0.0 and closed serial port.")
            except Exception as e:
                print(f"[PullTester] Error during shutdown: {e}")


def main():
    parser = argparse.ArgumentParser(description="MirrorHand Mechanical Tendon Pull Tester")
    parser.add_argument("--port", type=str, default=None, help="Serial port of Raspberry Pi Pico")
    args = parser.parse_args()

    print("==================================================")
    print("      MIRRORHAND: MECHANICAL TENDON TESTER        ")
    print("==================================================")
    print("Commands:")
    print("  <0-100>       : Set both fingers to percent (e.g. '50' for half pull)")
    print("  t <0-100>     : Set thumb only (e.g. 't 75')")
    print("  i <0-100>     : Set index only (e.g. 'i 30')")
    print("  0 or 'open'   : Fully open / relax all tendons")
    print("  q or 'exit'   : Quit and return servos to 0%")
    print("==================================================\n")

    tester = TendonPullTester(args.port)

    try:
        while True:
            prompt = input("MirrorHand [0-100 / 'q']> ").strip().lower()
            if not prompt:
                continue

            if prompt in ("q", "quit", "exit"):
                break
            elif prompt in ("open", "relax", "release"):
                tester.set_values(0.0, 0.0)
            elif prompt.startswith("t "):
                parts = prompt.split()
                try:
                    pct = float(parts[1])
                    tester.set_values(thumb=pct / 100.0, index=tester.index_val)
                except (ValueError, IndexError):
                    print("Invalid format. Use: t <0-100> (e.g. 't 50')")
            elif prompt.startswith("i "):
                parts = prompt.split()
                try:
                    pct = float(parts[1])
                    tester.set_values(thumb=tester.thumb_val, index=pct / 100.0)
                except (ValueError, IndexError):
                    print("Invalid format. Use: i <0-100> (e.g. 'i 50')")
            else:
                try:
                    pct = float(prompt)
                    if 0.0 <= pct <= 100.0:
                        val = pct / 100.0
                        tester.set_values(thumb=val, index=val)
                    else:
                        print("Please enter a value between 0 and 100.")
                except ValueError:
                    print("Invalid input. Enter a number 0-100, 't 50', 'i 50', or 'q'.")

    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        tester.shutdown()


if __name__ == "__main__":
    main()
