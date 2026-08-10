#!/usr/bin/env python3
"""USB serial crash monitor for the grinder ESP32-S3.

Waits for the device to appear, logs all serial output with timestamps,
and auto-reconnects when the port vanishes (device reboot/crash), so
panic backtraces and post-reboot reset reasons are both captured.

Usage: python tools/crash_monitor.py [--log FILE]
"""

import argparse
import sys
import time
from datetime import datetime

import serial
from serial.tools import list_ports

BAUD = 115200
ESPRESSIF_VID = 0x303A  # Native USB-CDC on ESP32-S3


def find_port():
    candidates = list(list_ports.comports())
    for p in candidates:
        if p.vid == ESPRESSIF_VID:
            return p.device
    # Fall back to any serial port if exactly one exists
    if len(candidates) == 1:
        return candidates[0].device
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", default="crash_capture.log")
    args = parser.parse_args()

    log = open(args.log, "a", encoding="utf-8", errors="replace")

    def emit(line):
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        log.write(f"[{stamp}] {line}\n")
        log.flush()

    emit("=== MONITOR STARTED - waiting for device ===")
    print(f"Logging to {args.log}. Waiting for ESP32 on USB...", flush=True)

    connected_before = False
    while True:
        port = find_port()
        if not port:
            time.sleep(0.5)
            continue
        try:
            with serial.Serial(port, BAUD, timeout=1) as ser:
                if connected_before:
                    emit(f"=== RECONNECTED on {port} (device likely rebooted) ===")
                else:
                    emit(f"=== CONNECTED on {port} ===")
                    connected_before = True
                print(f"Connected on {port}", flush=True)
                buffer = b""
                while True:
                    chunk = ser.read(256)
                    if chunk:
                        buffer += chunk
                        while b"\n" in buffer:
                            line, buffer = buffer.split(b"\n", 1)
                            emit(line.decode("utf-8", errors="replace").rstrip("\r"))
        except (serial.SerialException, OSError):
            emit("=== PORT LOST (device resetting?) - waiting to reconnect ===")
            print("Port lost - waiting to reconnect...", flush=True)
            time.sleep(0.5)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
