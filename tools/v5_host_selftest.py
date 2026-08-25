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


class Gate(Enum):
    PASS = auto()
    WARN = auto()
    FAIL = auto()
    UNAVAILABLE = auto()


@dataclass
class PreflightInput:
    link: bool = True
    left_sensor: bool = True
    right_sensor: bool = True
    encoder: bool = True
    odometry: bool = True
    heading: bool = True
    owner: str = "NONE"
    lease: bool = False
    stopped: bool = True
    ps2: bool = False
    route_valid: bool = True
    reset_boundary: bool = False
    needs_turn: bool = False
    camera_available: bool = False


def classify_preflight(value: PreflightInput) -> tuple[bool, str, dict[str, Gate]]:
    gates = {
        "link": Gate.PASS if value.link else Gate.FAIL,
        "sensors": Gate.PASS if value.left_sensor and value.right_sensor else Gate.FAIL,
        "encoder": Gate.PASS if value.encoder else Gate.FAIL,
        "odometry": Gate.PASS if value.odometry else Gate.FAIL,
        "heading": (Gate.PASS if value.heading else Gate.FAIL)
        if value.needs_turn else Gate.UNAVAILABLE,
        "owner": Gate.PASS if value.owner in {"NONE", "MCP"} and not value.ps2 else Gate.FAIL,
        "lease": Gate.FAIL if value.lease else Gate.PASS,
        "stop": Gate.PASS if value.stopped else Gate.FAIL,
        "route": Gate.PASS if value.route_valid else Gate.FAIL,
        "reset": Gate.FAIL if value.reset_boundary else Gate.PASS,
        "camera": Gate.PASS if value.camera_available else Gate.WARN,
    }
    reasons = {
        "link": "ABORTED_LINK_LOSS",
        "sensors": "ABORTED_SENSOR_FAULT",
        "encoder": "ABORTED_POSE_UNRELIABLE",
        "odometry": "ABORTED_POSE_UNRELIABLE",
        "heading": "ABORTED_POSE_UNRELIABLE",
        "owner": "STOPPED",
        "lease": "STOPPED",
        "stop": "STOPPED",
        "route": "INVALID_ROUTE",
        "reset": "ABORTED_RESET_BOUNDARY",
    }
    for name in ("link", "sensors", "encoder", "odometry", "heading",
                 "owner", "lease", "stop", "route", "reset"):
        if gates[name] is Gate.FAIL:
            return False, reasons[name], gates
    return True, "OK", gates


class ReplayModel:
    """Host-only replay state model mirroring the embedded fail-closed contract."""

    def __init__(self, mode: ReplayMode, route: list[tuple[str, int]]) -> None:
        self.mode = mode
        self.route = route
        self.owner = "NONE"
        self.lease = False
        self.terminal: str | None = None
        self.commands: list[tuple[str, int]] = []
        self.operation = 0

    def start(self) -> bool:
        if not self.route or any(kind not in {"MOVE", "TURN"} or value <= 0
                                 for kind, value in self.route):
            self.terminal = "INVALID_ROUTE"
            return False
        self.owner = "MCP"
        return True

    def issue(self, kind: str, value: int, gate: PreflightInput) -> int | None:
        allowed, terminal, _ = classify_preflight(gate)
        if not allowed:
            self.abort(terminal)
            return None
        if self.owner != "MCP" or self.lease:
            self.abort("STOPPED")
            return None
        self.operation += 1
        self.lease = True
        self.commands.append((kind, value))
        return self.operation

    def complete(self, operation: int, success: bool = True) -> bool:
        if operation != self.operation or not self.lease or self.terminal:
            return False
        self.lease = False
        if not success:
            self.abort("STOPPED")
            return False
        return True

    def abort(self, terminal: str) -> None:
        if self.terminal is None:
            self.terminal = terminal
            self.lease = False
            self.owner = "NONE"

    def finish(self) -> None:
        if self.terminal is None:
            self.terminal = ("TEST_COMPLETE" if self.mode is ReplayMode.SHORT_SAFETY_TEST
                             else "ROUTE_COMPLETE")
            self.lease = False
            self.owner = "NONE"


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

    def test_full_route_one_move_finishes_only_after_move(self) -> None:
        replay = ReplayModel(ReplayMode.FULL_PRODUCTION, [("MOVE", 100)])
        self.assertTrue(replay.start())
        operation = replay.issue("MOVE", 100, PreflightInput())
        self.assertIsNotNone(operation)
        self.assertTrue(replay.complete(operation or 0))
        replay.finish()
        self.assertEqual(replay.terminal, "ROUTE_COMPLETE")

    def test_full_route_l_shape_issues_move_turn_move(self) -> None:
        replay = ReplayModel(ReplayMode.FULL_PRODUCTION,
                             [("MOVE", 100), ("TURN", 90), ("MOVE", 100)])
        self.assertTrue(replay.start())
        for kind, value in replay.route:
            operation = replay.issue(kind, value,
                                     PreflightInput(needs_turn=kind == "TURN"))
            self.assertTrue(replay.complete(operation or 0))
        replay.finish()
        self.assertEqual(replay.commands, replay.route)
        self.assertEqual(replay.terminal, "ROUTE_COMPLETE")

    def test_invalid_route_issues_no_motion(self) -> None:
        replay = ReplayModel(ReplayMode.FULL_PRODUCTION, [("BACK", 100)])
        self.assertFalse(replay.start())
        self.assertEqual(replay.commands, [])
        self.assertEqual(replay.terminal, "INVALID_ROUTE")

    def test_stop_and_cancel_cleanup_owner_and_lease(self) -> None:
        replay = ReplayModel(ReplayMode.FULL_PRODUCTION, [("MOVE", 100)])
        replay.start()
        self.assertIsNotNone(replay.issue("MOVE", 100, PreflightInput()))
        replay.abort("STOPPED")
        replay.abort("CANCELLED")  # terminal result is emitted exactly once
        self.assertEqual(replay.terminal, "STOPPED")
        self.assertFalse(replay.lease)
        self.assertEqual(replay.owner, "NONE")

    def test_stale_operation_result_cannot_complete_new_operation(self) -> None:
        replay = ReplayModel(ReplayMode.FULL_PRODUCTION,
                             [("MOVE", 100), ("MOVE", 100)])
        replay.start()
        first = replay.issue("MOVE", 100, PreflightInput())
        self.assertTrue(replay.complete(first or 0))
        second = replay.issue("MOVE", 100, PreflightInput())
        self.assertFalse(replay.complete(first or 0))
        self.assertTrue(replay.lease)
        self.assertTrue(replay.complete(second or 0))

    def test_sensor_fault_blocks_replay_and_auto_resume(self) -> None:
        allowed, terminal, gates = classify_preflight(
            PreflightInput(left_sensor=False))
        self.assertFalse(allowed)
        self.assertEqual(gates["sensors"], Gate.FAIL)
        self.assertEqual(terminal, "ABORTED_SENSOR_FAULT")

    def test_link_loss_blocks_new_motion(self) -> None:
        allowed, terminal, gates = classify_preflight(PreflightInput(link=False))
        self.assertFalse(allowed)
        self.assertEqual(gates["link"], Gate.FAIL)
        self.assertEqual(terminal, "ABORTED_LINK_LOSS")

    def test_encoder_or_odometry_fault_blocks_replay(self) -> None:
        for changes in ({"encoder": False}, {"odometry": False}):
            allowed, terminal, _ = classify_preflight(PreflightInput(**changes))
            self.assertFalse(allowed)
            self.assertEqual(terminal, "ABORTED_POSE_UNRELIABLE")

    def test_heading_only_blocks_turn_not_straight_replay(self) -> None:
        straight, _, straight_gates = classify_preflight(
            PreflightInput(heading=False, needs_turn=False))
        turning, terminal, turn_gates = classify_preflight(
            PreflightInput(heading=False, needs_turn=True))
        self.assertTrue(straight)
        self.assertEqual(straight_gates["heading"], Gate.UNAVAILABLE)
        self.assertFalse(turning)
        self.assertEqual(turn_gates["heading"], Gate.FAIL)
        self.assertEqual(terminal, "ABORTED_POSE_UNRELIABLE")

    def test_reset_boundary_locks_resume(self) -> None:
        allowed, terminal, gates = classify_preflight(
            PreflightInput(reset_boundary=True))
        self.assertFalse(allowed)
        self.assertEqual(gates["reset"], Gate.FAIL)
        self.assertEqual(terminal, "ABORTED_RESET_BOUNDARY")

    def test_ps2_owner_or_active_lease_blocks_replay(self) -> None:
        for changes in ({"ps2": True}, {"owner": "PS2"}, {"lease": True}):
            allowed, terminal, _ = classify_preflight(PreflightInput(**changes))
            self.assertFalse(allowed)
            self.assertEqual(terminal, "STOPPED")

    def test_camera_unavailable_is_warn_for_basic_replay(self) -> None:
        allowed, terminal, gates = classify_preflight(
            PreflightInput(camera_available=False))
        self.assertTrue(allowed)
        self.assertEqual(terminal, "OK")
        self.assertEqual(gates["camera"], Gate.WARN)

    def test_capability_matrix_all_healthy_is_pass_except_optional_camera(self) -> None:
        allowed, terminal, gates = classify_preflight(
            PreflightInput(needs_turn=True, camera_available=False))
        self.assertTrue(allowed)
        self.assertEqual(terminal, "OK")
        self.assertEqual(gates["camera"], Gate.WARN)
        self.assertTrue(all(gate is Gate.PASS for name, gate in gates.items()
                            if name != "camera"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
