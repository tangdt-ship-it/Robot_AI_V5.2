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
legacy_version_match = re.fullmatch(r"5\.0\.0-alpha\.(\d+)", version)
current_version_match = re.fullmatch(r"5\.2\.\d+(?:-alpha\.\d+)?", version)
legacy_alpha_revision = (
    int(legacy_version_match.group(1)) if legacy_version_match else -1
)
stop_start = source.index("bool RobotUart::Stop(uint32_t timeout_ms)")
stop_end = source.index("bool RobotUart::GetState(", stop_start)
stop_implementation = source[stop_start:stop_end]

checks = {
    "alpha5+ or current V5.2 version": (
        (legacy_version_match is not None and legacy_alpha_revision >= 5)
        or current_version_match is not None
    ),
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
    # V5.2 STOP is intentionally a direct emergency frame, not an ordinary
    # transaction-mutex request. Preserve the same confirmed-DONE contract.
    "raw stop still waits for DONE STOP": (
        'const bool sent = SendFrame("STOP");' in stop_implementation
        and stop_implementation.index('const bool sent = SendFrame("STOP");')
        < stop_implementation.index("xEventGroupWaitBits(")
        and "kResponseStopDone | kResponseNack" in stop_implementation
    ),
    "raw stop invalidates correlation before transport": (
        stop_implementation.find('InvalidateMotionCorrelation("STOP");')
        < stop_implementation.find('const bool sent = SendFrame("STOP");')
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
