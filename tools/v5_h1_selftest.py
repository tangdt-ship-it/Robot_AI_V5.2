#!/usr/bin/env python3
"""Static H1 regression checks for finite-operation STOP lifecycle hardening.

This test never opens serial ports. Hardware timing remains the authority for H1.
The H1 behaviour was introduced in Alpha.5 and must remain valid on all later
V5 alpha revisions.
"""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "firmware/esp32-xiaozhi/main/robot/robot_uart.h"
SOURCE = ROOT / "firmware/esp32-xiaozhi/main/robot/robot_uart.cc"
MCP_HEADER = ROOT / "firmware/esp32-xiaozhi/main/mcp_server.h"
MCP_SOURCE = ROOT / "firmware/esp32-xiaozhi/main/mcp_server.cc"
BOARD_SOURCE = ROOT / "firmware/esp32-xiaozhi/main/boards/bread-compact-wifi-s3cam/compact_wifi_board_s3cam.cc"
VERSION = ROOT / "VERSION"

header = HEADER.read_text(encoding="utf-8")
source = SOURCE.read_text(encoding="utf-8")
mcp_header = MCP_HEADER.read_text(encoding="utf-8")
mcp_source = MCP_SOURCE.read_text(encoding="utf-8")
board_source = BOARD_SOURCE.read_text(encoding="utf-8")
version = VERSION.read_text(encoding="utf-8").strip()
version_match = re.fullmatch(r"5\.0\.0-alpha\.(\d+)", version)
alpha_revision = int(version_match.group(1)) if version_match else -1

checks = {
    "alpha5+ version": alpha_revision >= 5,
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
    "stop advances motion cancellation generation": "motion_cancel_generation_.fetch_add(1U)" in header,
    "MCP captures request generation before scheduling": (
        "arguments.SetRequestGeneration(request_generation_provider_(tool_name));" in mcp_source
    ),
    "MCP request generation is retained in callback arguments": (
        "uint32_t RequestGeneration() const" in mcp_header
    ),
    "move callback consumes captured generation": (
        "const uint32_t cancellation_token = properties.RequestGeneration();" in board_source
    ),
    "motion transport checks cancellation before send": (
        "ROBOT_TXN_CANCELLED,BODY=%s,STAGE=BEFORE_SEND" in source
    ),
}

failed = [name for name, ok in checks.items() if not ok]
for name, ok in checks.items():
    print(f"H1_CHECK {name}: {'PASS' if ok else 'FAIL'}")

if failed:
    raise SystemExit("V5_H1_SELFTEST FAIL: " + ", ".join(failed))

print("V5_H1_SELFTEST PASS")
