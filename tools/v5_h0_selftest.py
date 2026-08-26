#!/usr/bin/env python3
"""Static H0 guard for V5 finite-motion correlation and SID observability.

This test intentionally does not open serial ports or move hardware. It protects
only the source-level contract exposed by the Alpha.3 H0 failure:
- finite MOVE/TURN requests require non-zero SID/OP;
- legacy continuous/pulse commands remain outside this finite-operation gate;
- a negotiated ESP32 motion session emits a passive SID log for H0 evidence.
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
STM32 = ROOT / "firmware/stm32/src/communication/robot_link_server.cpp"
ESP32 = ROOT / "firmware/esp32-xiaozhi/main/robot/robot_uart.cc"

errors: list[str] = []

def require(condition: bool, message: str) -> None:
    if not condition:
        errors.append(message)

stm32 = STM32.read_text(errors="replace")
esp32 = ESP32.read_text(errors="replace")

# Correlated finite commands must remain implemented.
for pattern in (
    'MOVE,FWD,%d,%d,SID,%lu,OP,%lu%c',
    'MOVE,BACK,%d,%d,SID,%lu,OP,%lu%c',
    'TURN,REL,LEFT,%d,%d,SID,%lu,OP,%lu%c',
    'TURN,REL,RIGHT,%d,%d,SID,%lu,OP,%lu%c',
    'TURN,ABS,%d,%d,SID,%lu,OP,%lu%c',
):
    require(pattern in stm32, f"missing correlated finite parser: {pattern}")

# The exact legacy finite parsers that caused Alpha.3 H0 to fail must not be
# able to queue motion anymore. Valid legacy CONT and CMD pulse forms are not
# part of this finite-operation correlation requirement.
for pattern in (
    'MOVE,FWD,%d,%d%c',
    'MOVE,BACK,%d,%d%c',
    'TURN,REL,LEFT,%d,%d%c',
    'TURN,REL,RIGHT,%d,%d%c',
    'TURN,ABS,%d,%d%c',
):
    require(pattern not in stm32, f"legacy finite parser still accepted: {pattern}")

require('CORRELATION_REQUIRED' in stm32,
        "STM32 does not expose a clear missing-correlation rejection")
require('correlatedMotion && (sessionId == 0UL || operationId == 0UL)' in stm32,
        "zero SID/OP rejection missing")

# Preserve the existing compatibility scope intentionally.
require('MOVE,FWD,%d,CONT%c' in stm32,
        "legacy continuous compatibility changed unexpectedly")
require('CMD,FWD,%d' in stm32,
        "legacy pulse compatibility changed unexpectedly")

# H0 must be able to record a newly negotiated non-zero SID without issuing a
# motion command.
require('ROBOT_SESSION=READY,SID=%lu' in esp32,
        "passive negotiated SID log missing")
require('motion_session_id_' in esp32 and 'if (motion_session_id_ == 0U)' in esp32,
        "non-zero session generation guard missing")

if errors:
    print("FAIL V5 H0 source guard")
    for error in errors:
        print(f" - {error}")
    sys.exit(1)

print("PASS V5 H0 source guard")
