#!/usr/bin/env python3
"""Deterministic host tests for the V5 safety state contracts.

This models protocol/state invariants only; it never opens a serial port and
never claims hardware validation.  The embedded implementation remains the
authority at runtime.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
import math
import unittest


class Health(Enum):
    UNKNOWN = auto()
    HEALTHY = auto()
    STALE = auto()
    TIMEOUT = auto()
    INVALID = auto()
    DISCONNECTED_OR_FAULT = auto()


class Zone(Enum):
    UNKNOWN = auto()
    CLEAR = auto()
    BLOCKED = auto()


@dataclass
class Sensor:
    health: Health = Health.UNKNOWN
    zone: Zone = Zone.UNKNOWN
    last_ms: int | None = None

    def sample(self, now: int, distance_cm: float | None) -> None:
        self.last_ms = now
        if distance_cm is None:
            self.health = Health.TIMEOUT
            self.zone = Zone.UNKNOWN
        elif not 2 <= distance_cm <= 400:
            self.health = Health.INVALID
            self.zone = Zone.UNKNOWN
        else:
            self.health = Health.HEALTHY
            self.zone = Zone.BLOCKED if distance_cm <= 16 else Zone.CLEAR

    def refresh(self, now: int, fresh_ms: int = 350) -> None:
        if self.last_ms is not None and now - self.last_ms > fresh_ms:
            self.health = Health.STALE
            self.zone = Zone.UNKNOWN


class StopController:
    def __init__(self) -> None:
        self.motor = (20, 20)
        self.lease = True
        self.operation = "MOVE"
        self.stop_count = 0

    def stop(self, reason: str) -> None:
        self.motor = (0, 0)
        self.lease = False
        self.operation = "SAFE_STOP:" + reason
        self.stop_count += 1


class AiCooldown:
    def __init__(self, cooldown_ms: int = 10_000) -> None:
        self.cooldown_ms = cooldown_ms
        self.active = False
        self.last_zone = ""
        self.last_ms = -cooldown_ms

    def accept(self, zone: str, now: int) -> bool:
        if zone == self.last_zone and now - self.last_ms < self.cooldown_ms:
            return False
        if self.active:
            return False
        self.active = True
        self.last_zone, self.last_ms = zone, now
        return True

    def finish(self) -> None:
        self.active = False


@dataclass
class Segment:
    target: int
    travelled: int = 0
    reset_generation: int = 0
    held: bool = False

    @property
    def remaining(self) -> int:
        return max(0, self.target - self.travelled)

    def hold(self, travelled: int, generation: int) -> None:
        self.travelled = min(self.target, max(self.travelled, travelled))
        self.reset_generation = generation
        self.held = True

    def resume(self, generation: int) -> int | None:
        if not self.held or generation != self.reset_generation:
            return None
        self.held = False
        return self.remaining


def canonical_side(mount: str) -> str:
    # Hardware mapping is normalized exactly once at the STM32 boundary.
    return {"MOUNT_LEFT": "ROBOT_RIGHT", "MOUNT_RIGHT": "ROBOT_LEFT"}[mount]


class ReplayMode(Enum):
    SHORT_SAFETY_TEST = auto()
    FULL_PRODUCTION = auto()


def replay_terminal(mode: ReplayMode, completed_segments: int,
                    total_segments: int) -> str:
    if mode is ReplayMode.SHORT_SAFETY_TEST:
        return "TEST_COMPLETE"
    return "ROUTE_COMPLETE" if completed_segments == total_segments else "HOLD"


def project_to_segment(px: float, py: float, ax: float, ay: float,
                       bx: float, by: float) -> tuple[float, float]:
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq == 0:
        return 0.0, math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    return t, math.hypot(px - (ax + t * dx), py - (ay + t * dy))


class V5SafetySelfTest(unittest.TestCase):
    def test_sensor_timeout_is_not_clear_and_stops_forward_motion(self) -> None:
        left, right = Sensor(), Sensor()
        left.sample(0, None)
        right.sample(0, 100)
        self.assertEqual(left.zone, Zone.UNKNOWN)
        self.assertNotEqual(left.health, Health.HEALTHY)
        stop = StopController()
        if left.health is not Health.HEALTHY or right.health is not Health.HEALTHY:
            stop.stop("SENSOR_FAULT")
        self.assertEqual(stop.motor, (0, 0))
        self.assertFalse(stop.lease)

    def test_stop_is_idempotent_for_move_turn_hold(self) -> None:
        for operation in ("MOVE", "TURN", "HOLD"):
            stop = StopController()
            stop.operation = operation
            stop.stop("USER")
            stop.stop("USER")
            self.assertEqual(stop.motor, (0, 0))
            self.assertEqual(stop.stop_count, 2)
            self.assertFalse(stop.lease)

    def test_cooldown_failure_cleanup_and_next_event(self) -> None:
        ai = AiCooldown()
        self.assertTrue(ai.accept("BLOCKED", 0))
        ai.finish()  # camera/cloud/parse failure still cleans up
        self.assertFalse(ai.active)
        self.assertFalse(ai.accept("BLOCKED", 1_000))
        self.assertTrue(ai.accept("EMERGENCY", 1_000))
        ai.finish()
        self.assertTrue(ai.accept("BLOCKED", 11_001))

    def test_hold_resume_uses_cumulative_distance_and_generation(self) -> None:
        segment = Segment(1_000)
        segment.hold(300, 4)
        segment.hold(500, 4)  # repeated HOLD never resets progress
        self.assertEqual(segment.remaining, 500)
        self.assertIsNone(segment.resume(5))  # unresolved reset boundary
        self.assertEqual(segment.resume(4), 500)

    def test_left_right_mapping_is_single_canonical_mapping(self) -> None:
        self.assertEqual(canonical_side("MOUNT_LEFT"), "ROBOT_RIGHT")
        self.assertEqual(canonical_side("MOUNT_RIGHT"), "ROBOT_LEFT")

    def test_short_test_does_not_claim_route_complete(self) -> None:
        self.assertEqual(replay_terminal(ReplayMode.SHORT_SAFETY_TEST, 1, 4),
                         "TEST_COMPLETE")
        self.assertEqual(replay_terminal(ReplayMode.FULL_PRODUCTION, 4, 4),
                         "ROUTE_COMPLETE")

    def test_rejoin_uses_projection_not_total_detour_distance(self) -> None:
        progress, cross_track = project_to_segment(50, 20, 0, 0, 100, 0)
        self.assertAlmostEqual(progress, 0.5)
        self.assertAlmostEqual(cross_track, 20)


if __name__ == "__main__":
    unittest.main(verbosity=2)
