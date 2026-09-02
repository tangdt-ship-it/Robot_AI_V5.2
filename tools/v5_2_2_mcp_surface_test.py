#!/usr/bin/env python3
"""Static guard for Robot AI V5.2.2 MCP motion cleanup.

This test does not move hardware. It verifies that the AI-facing MCP policy keeps
one clear finite MOVE path, one clear relative TURN path, STOP/HOME tools, and
hides/disables the legacy overlapping motion aliases while automatic detour is
disabled.
"""

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
MCP_HEADER = ROOT / "firmware/esp32-xiaozhi/main/mcp_server.h"
MCP_SOURCE = ROOT / "firmware/esp32-xiaozhi/main/mcp_server.cc"
UART_SOURCE = ROOT / "firmware/esp32-xiaozhi/main/robot/robot_uart.cc"
BOARD = ROOT / (
    "firmware/esp32-xiaozhi/main/boards/"
    "bread-compact-wifi-s3cam/compact_wifi_board_s3cam.cc"
)
SDK_DEFAULTS = ROOT / "firmware/esp32-xiaozhi/sdkconfig.robot_ai.defaults"
VERSION = ROOT / "VERSION"
FIRMWARE_ROOT = ROOT / "firmware"


def require(text: str, needle: str, label: str) -> None:
    if needle not in text:
        raise AssertionError(f"missing {label}: {needle}")


def main() -> int:
    header = MCP_HEADER.read_text(encoding="utf-8")
    mcp_source = MCP_SOURCE.read_text(encoding="utf-8")
    uart_source = UART_SOURCE.read_text(encoding="utf-8")
    board = BOARD.read_text(encoding="utf-8")
    defaults = SDK_DEFAULTS.read_text(encoding="utf-8")
    version = VERSION.read_text(encoding="utf-8").strip()

    # Compass was removed from the active firmware contract. Keep this guard
    # broad enough to cover both MCU trees, while excluding generated build
    # output and historical host audit scripts.
    for path in FIRMWARE_ROOT.rglob("*"):
        if (not path.is_file() or ".pio" in path.parts or
                "build" in path.parts or "managed_components" in path.parts):
            continue
        if path.suffix.lower() not in {".cc", ".cpp", ".h", ".ini", ".md"}:
            continue
        active_text = path.read_text(encoding="utf-8", errors="replace")
        if "compass" in active_text.lower():
            raise AssertionError(f"Compass reference remains in firmware: {path}")

    require(version, "5.2.2", "V5.2.2 version")

    # Canonical AI-visible motion tools must still exist in board registration.
    for name in (
        "self.robot.stop",
        "self.robot.move_distance",
        "self.robot.turn_relative",
        "self.robot.turn_revolutions",
        "self.robot.turn_to_heading",
        "self.robot.set_home",
        "self.robot.return_home",
        "self.robot.scan_obstacle",
    ):
        require(board, f'"{name}"', f"registered canonical tool {name}")

    # Legacy aliases remain visible in source history for rollback/audit, but
    # V5.2.2 must both hide them from normal tools/list and replace their
    # callback with a fail-closed response so a stale/hallucinated direct call
    # cannot issue motor commands.
    for name in (
        "self.robot.turn_and_move",
        "self.robot.navigate_autonomously",
        "self.robot.cancel_mission",
        "self.robot.move_forward",
        "self.robot.move_backward",
        "self.robot.turn_left",
        "self.robot.turn_right",
        "self.robot.rotate_continuous",
    ):
        require(header, f'name == "{name}"', f"AI-hidden policy {name}")
        require(board, f'"{name}"', f"legacy source retained {name}")

    require(header, "legacy_motion_tool_disabled", "fail-closed legacy callback")
    require(header, "1 bước=5 cm=50 mm", "voice step conversion policy")
    require(header, "Quay đầu", "180-degree turn language policy")
    require(header, "PHYSICAL MOTION", "body-scan motion warning")
    require(header, "self.robot.turn_revolutions", "revolution-turn language policy")

    # Do not advertise autonomous detour while firmware still rejects it.
    require(defaults, "CONFIG_AUTOMATIC_DETOUR=n", "detour disabled gate")
    if 'ResetHeading' not in uart_source or 'ResetCompass' in uart_source:
        raise AssertionError("Heading reset must not depend on Compass")

    # Finite move/turn cancellation coverage must remain in the board provider.
    for name in (
        "self.robot.move_distance",
        "self.robot.turn_relative",
        "self.robot.turn_revolutions",
        "self.robot.turn_to_heading",
    ):
        require(board, f'tool_name == "{name}"', f"STOP cancellation token {name}")

    require(board, "revolutions * kSegmentsPerRevolution", "bounded 360-degree revolution expansion")
    require(board, "kSegmentDegrees = 90", "timeout-safe revolution segment")
    require(board, "vTaskDelay(pdMS_TO_TICKS(400))", "inter-segment heading settle")
    require(board, "completed_segments", "revolution completion reporting")

    # STOP must not be queued behind a blocking finite-motion callback. The
    # UART implementation therefore has an urgent path that bypasses the
    # ordinary transaction mutex and wakes the finite waiter after physical
    # DONE,STOP confirmation.
    require(mcp_source, 'tool_name == "self.robot.stop"',
            "immediate STOP dispatch")
    require(uart_source, "STOP is an emergency boundary",
            "urgent STOP implementation")
    require(uart_source, "kResponseStopDone | kResponseNack",
            "urgent physical STOP wait")
    require(uart_source, "motion_generation = motion_cancel_generation_.load()",
            "internal motion STOP preservation")

    print("V5.2.2 MCP surface static guard: PASS")
    print("  canonical finite motion: move_distance + turn_relative + turn_revolutions")
    print("  stop/home: retained")
    print("  overlapping aliases: hidden + fail-closed")
    print("  automatic detour: remains disabled")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, OSError) as exc:
        print(f"V5.2.2 MCP surface static guard: FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
