#!/usr/bin/env python3
"""Host-only H2 guards for Robot_AI V5 Alpha.6.

No serial port, actuator, reset or hardware access is performed.  The tests
exercise stale/session/reset semantics and statically verify that the Alpha.6
black-box trace remains telemetry-only.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class Ring:
    def __init__(self, capacity: int = 48) -> None:
        self.capacity = capacity
        self.events: list[tuple[int, str]] = []
        self.sequence = 0

    def record(self, name: str) -> None:
        self.sequence += 1
        self.events.append((self.sequence, name))
        if len(self.events) > self.capacity:
            self.events.pop(0)


class Correlation:
    def __init__(self) -> None:
        self.session = 1
        self.operation = 1
        self.active = True
        self.terminal_seen = False

    def invalidate(self) -> None:
        self.active = False

    def renew(self) -> None:
        self.session += 1
        self.operation = 2
        self.active = True
        self.terminal_seen = False

    def accept(self, sid: int, op: int) -> bool:
        if not self.active or sid == 0 or op == 0:
            return False
        if (sid, op) != (self.session, self.operation):
            return False
        if self.terminal_seen:
            return False
        self.terminal_seen = True
        return True


@dataclass
class ResumeBoundary:
    saved_generation: int
    held: bool = True

    def resume(self, current_generation: int) -> bool:
        return self.held and current_generation == self.saved_generation


class V5H2SelfTest(unittest.TestCase):
    def test_blackbox_ring_is_bounded_and_keeps_latest_order(self) -> None:
        ring = Ring(48)
        for index in range(60):
            ring.record(f"E{index}")
        self.assertEqual(len(ring.events), 48)
        self.assertEqual(ring.events[0][0], 13)
        self.assertEqual(ring.events[-1][0], 60)

    def test_stale_terminal_cannot_complete_new_session(self) -> None:
        correlation = Correlation()
        old_pair = (correlation.session, correlation.operation)
        correlation.invalidate()
        correlation.renew()
        self.assertFalse(correlation.accept(*old_pair))
        self.assertTrue(correlation.accept(correlation.session,
                                           correlation.operation))

    def test_duplicate_terminal_is_rejected(self) -> None:
        correlation = Correlation()
        self.assertTrue(correlation.accept(1, 1))
        self.assertFalse(correlation.accept(1, 1))

    def test_reset_generation_change_blocks_resume(self) -> None:
        boundary = ResumeBoundary(saved_generation=3)
        self.assertTrue(boundary.resume(3))
        self.assertFalse(boundary.resume(4))

    def test_alpha6_blackbox_trace_is_passive(self) -> None:
        source = (ROOT / "firmware/esp32-xiaozhi/main/robot/safety_blackbox.cc").read_text()
        header = (ROOT / "firmware/esp32-xiaozhi/main/robot/safety_blackbox.h").read_text()
        self.assertIn("ROBOT_BLACKBOX SEQ=", source)
        self.assertIn("EventTypeName", source)
        self.assertIn("CopyRecent", source)
        self.assertIn("kCapacity = 48", header)
        self.assertIn("portENTER_CRITICAL", source)
        # SafetyBlackBox must stay a telemetry recorder/classifier, never an
        # actuator or RobotLink command source.
        self.assertNotIn("SendFrame(", source)
        self.assertNotIn("MoveDistance(", source)
        self.assertNotIn("TurnRelative(", source)
        self.assertNotIn("StartContinuous(", source)

    def test_stale_guards_and_reset_boundary_exist_in_embedded_source(self) -> None:
        uart = (ROOT / "firmware/esp32-xiaozhi/main/robot/robot_uart.cc").read_text()
        teach = (ROOT / "firmware/esp32-xiaozhi/main/robot/teach_route.cc").read_text()
        self.assertIn("ACK_STALE", uart)
        self.assertIn("RESULT_STALE", uart)
        self.assertIn("InvalidateMotionCorrelation", uart)
        self.assertIn("STM32_BOOT", uart)
        self.assertIn("RESET_BOUNDARY_UNRESOLVED", teach)

    def test_version_is_alpha6(self) -> None:
        self.assertEqual((ROOT / "VERSION").read_text().strip(),
                         "5.0.0-alpha.6")


if __name__ == "__main__":
    unittest.main(verbosity=2)
