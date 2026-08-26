#!/usr/bin/env python3
"""Static Alpha.5 checks for finite-operation STOP lifecycle hardening.

This test never opens serial ports. Hardware timing remains the authority for H1.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "firmware/esp32-xiaozhi/main/robot/robot_uart.h"
SOURCE = ROOT / "firmware/esp32-xiaozhi/main/robot/robot_uart.cc"
VERSION = ROOT / "VERSION"

header = HEADER.read_text(encoding="utf-8")
source = SOURCE.read_text(encoding="utf-8")
version = VERSION.read_text(encoding="utf-8").strip()

checks = {
    "alpha5 version": version == "5.0.0-alpha.5",
    "public cancellation-aware stop": "bool Stop(int timeout_ms = 500)" in header,
    "raw stop is private transport": "bool Stop(uint32_t timeout_ms);" in header,
    "stop captures turn waiter": "const bool wake_turn = turn_waiting_;" in header,
    "stop captures distance waiter": "const bool wake_distance = distance_waiting_;" in header,
    "stop blocks new session begin while active": "!stop_in_progress_" in header,
    "turn waiter wake bit": "wake_bits |= kResponseTurnError" in header,
    "distance waiter wake bit": "wake_bits |= kResponseDistanceError" in header,
    "waiters wake only after confirmed stop": (
        header.find("const bool stopped = Stop(bounded_timeout);")
        < header.find("if (stopped) {")
        < header.find("xEventGroupSetBits(response_events_, wake_bits);")
    ),
    "raw stop still waits for DONE STOP": (
        'return SendAndWait("STOP", kResponseStopDone, timeout_ms);' in source
    ),
    "raw stop invalidates correlation before transport": (
        source.find('InvalidateMotionCorrelation("STOP");')
        < source.find('return SendAndWait("STOP", kResponseStopDone, timeout_ms);')
    ),
    "legacy public uint32 default removed": "bool Stop(uint32_t timeout_ms = 500);" not in header,
}

failed = [name for name, ok in checks.items() if not ok]
for name, ok in checks.items():
    print(f"H1_CHECK {name}: {'PASS' if ok else 'FAIL'}")

if failed:
    raise SystemExit("V5_H1_SELFTEST FAIL: " + ", ".join(failed))

print("V5_H1_SELFTEST PASS")
