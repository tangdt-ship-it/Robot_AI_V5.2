#!/usr/bin/env python3
"""Host-only H2 guards for Robot_AI V5 Alpha.6+.

No serial port, actuator, reset or hardware access is performed. The tests
exercise stale/session/reset semantics, preserve passive Alpha.6 black-box
observability, guard the Alpha.7 cross-reboot correlation seeding fix, protect
the Alpha.8 STM32-boot-aware Replay/HOLD reset boundary, and verify Alpha.9
RobotLink renegotiation remains non-motion and outside the UART RX task.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re
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
    saved_boot_epoch: int
    held: bool = True

    def resume(self, current_generation: int, current_boot_epoch: int) -> bool:
        return (self.held and current_generation == self.saved_generation and
                current_boot_epoch == self.saved_boot_epoch)


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

    def test_reset_generation_or_stm32_boot_change_blocks_resume(self) -> None:
        boundary = ResumeBoundary(saved_generation=3, saved_boot_epoch=7)
        self.assertTrue(boundary.resume(3, 7))
        self.assertFalse(boundary.resume(4, 7))
        # Alpha.8 regression: a reboot may restart raw RESET_GEN at the same
        # numeric value, but the ESP32-observed STM32 boot epoch must still
        # invalidate the saved Replay/HOLD boundary.
        self.assertFalse(boundary.resume(3, 8))

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

    def test_alpha7_correlation_pair_is_not_fixed_after_reboot(self) -> None:
        header = (ROOT / "firmware/esp32-xiaozhi/main/robot/robot_uart.h").read_text()
        source = (ROOT / "firmware/esp32-xiaozhi/main/robot/robot_uart.cc").read_text()
        self.assertIn("#include <esp_random.h>", header)
        self.assertIn("RandomCorrelationSeed()", header)
        self.assertIn("value = esp_random();", header)
        self.assertIn("value == 0U || value == 0xFFFFFFFFU", header)
        self.assertIn("motion_session_id_ = RandomCorrelationSeed();", header)
        self.assertIn("next_operation_id_ = RandomCorrelationSeed();", header)
        # Preserve the existing successful-negotiation session advance and
        # explicit non-zero wrap guard on top of the per-boot random seed.
        self.assertIn("++motion_session_id_;", source)
        self.assertIn("if (motion_session_id_ == 0U)", source)

    def test_alpha8_stm32_boot_epoch_invalidates_reset_boundary(self) -> None:
        blackbox_h = (ROOT / "firmware/esp32-xiaozhi/main/robot/safety_blackbox.h").read_text()
        blackbox_cc = (ROOT / "firmware/esp32-xiaozhi/main/robot/safety_blackbox.cc").read_text()
        teach_h = (ROOT / "firmware/esp32-xiaozhi/main/robot/teach_route.h").read_text()

        self.assertIn("uint32_t stm32_boot_epoch = 0;", blackbox_h)
        self.assertIn("uint32_t GetStm32BootEpoch();", blackbox_h)
        self.assertIn("g_stm32_boot_epoch", blackbox_cc)
        self.assertIn("AdvanceStm32BootEpoch()", blackbox_cc)
        self.assertIn("type == SafetyEventType::LINK_LOSS", blackbox_cc)
        self.assertIn('std::strcmp(reason, "STM32_BOOT") == 0', blackbox_cc)
        self.assertIn("STM32_EPOCH=%lu", blackbox_cc)

        self.assertIn("struct ResetBoundaryToken", teach_h)
        self.assertIn("stm32_boot_epoch = generation == 0U ? 0U : GetStm32BootEpoch();",
                      teach_h)
        self.assertIn("stm32_boot_epoch != GetStm32BootEpoch()", teach_h)
        self.assertIn("ResetBoundaryToken replay_reset_generation_", teach_h)
        self.assertIn("ResetBoundaryToken resume_reset_generation_", teach_h)

    def test_alpha9_background_protocol_recovery_is_non_motion(self) -> None:
        header = (ROOT / "firmware/esp32-xiaozhi/main/robot/robot_uart.h").read_text()
        self.assertIn("ProtocolRecoveryTaskEntry", header)
        self.assertIn("void ProtocolRecoveryTask()", header)
        self.assertIn("GetStm32BootEpoch()", header)
        self.assertIn("current_epoch != observed_epoch", header)
        self.assertIn("!motion_lease_active_", header)
        self.assertIn("!motion_correlation_active_", header)
        self.assertIn("!turn_waiting_", header)
        self.assertIn("!distance_waiting_", header)
        self.assertIn("CheckProtocol(700)", header)
        self.assertIn("ROBOT_SESSION=RECOVERY,STATE=PASS", header)
        self.assertIn("TaskHandle_t protocol_recovery_task_", header)

        start = header.index("void ProtocolRecoveryTask()")
        end = header.index("void HandleFrame", start)
        recovery = header[start:end]
        # Recovery may only renegotiate HELLO/PING via CheckProtocol. It must
        # never re-arm a mode, acquire a lease or issue any actuator command.
        for forbidden in ("SetMode(", "MoveForward(", "MoveDistance(",
                          "TurnRelative(", "TurnAbsolute(", "Stop(",
                          "StartContinuous(", "StartContinuousRotation("):
            self.assertNotIn(forbidden, recovery)

    def test_version_is_alpha9_or_later(self) -> None:
        version = (ROOT / "VERSION").read_text().strip()
        match = re.fullmatch(r"5\.0\.0-alpha\.(\d+)", version)
        self.assertIsNotNone(match)
        self.assertGreaterEqual(int(match.group(1)), 9)


if __name__ == "__main__":
    unittest.main(verbosity=2)
