"""Host contract tests for bounded Obstacle Avoidance Phase 3.

The model below is intentionally small: it checks the safety/state contract
without pretending to replace the live STM32 HIL.  Source assertions cover the
production ownership, primitive and immutability boundaries.
"""

import re
import unittest
from pathlib import Path


STM32_ROOT = Path(__file__).resolve().parents[1]
CONFIG = (STM32_ROOT / "include" / "robot_config.h").read_text(encoding="utf-8")
MAP = (STM32_ROOT / "src" / "map" / "map_controller.cpp").read_text(encoding="utf-8")
MAP_HEADER = (STM32_ROOT / "include" / "map" / "map_controller.h").read_text(encoding="utf-8")
CLASSIFIER = (STM32_ROOT / "src" / "sensors" / "obstacle_classifier.cpp").read_text(encoding="utf-8")
MAIN = (STM32_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")


def config_number(name):
    match = re.search(r"\b" + re.escape(name) + r"\s*=\s*([0-9]+(?:\.[0-9]+)?)", CONFIG)
    if not match:
        raise AssertionError(f"constant {name} not found")
    return float(match.group(1))


class DetourModel:
    """Deterministic model of the bounded Phase 3 transition contract."""

    def __init__(self, decision="AVOID_RIGHT", stable=True):
        self.phase = "IDLE"
        self.decision = decision
        self.stable = stable
        self.replay_active = True
        self.mode = "REPLAY_HOLD"
        self.hold_reason = "OBSTACLE"
        self.operation = "HOLD"
        self.owner = "NONE"
        self.motors = (0, 0)
        self.current = 1
        self.target = 2
        self.route_generation = 44
        self.attempts = 0
        self.generation = 100
        self.expected_generation = 0
        self.clear_since = None
        self.now = 0
        self.travel_total = 0
        self.turn_total = 0
        self.away_right = None
        self.forward_only = True
        self.route_mutated = False
        self.auto_resumed = False
        self.entry_ok = True
        self.sensors_ready = True
        self.path_clear = False

    def arm(self):
        if (not self.entry_ok or not self.replay_active or
                self.mode != "REPLAY_HOLD" or self.hold_reason != "OBSTACLE" or
                not self.stable or self.decision not in {"AVOID_LEFT", "AVOID_RIGHT"} or
                not self.sensors_ready or self.attempts >= 1):
            return False
        self.attempts = 1
        self.away_right = self.decision == "AVOID_RIGHT"
        self.mode = "REPLAY_RUNNING"
        self.hold_reason = "NONE"
        self.operation = "TURN"
        self.owner = "REPLAY"
        self.motors = (15, -15) if self.away_right else (-15, 15)
        self.phase = "TURN_AWAY"
        self.generation += 1
        self.expected_generation = self.generation
        self.turn_total += 45
        return True

    def turn_result(self, code="DONE", generation=None):
        if generation != self.expected_generation:
            return False
        if self.phase not in {"TURN_AWAY", "TURN_PARALLEL"}:
            return False
        self.operation = "NONE"
        self.owner = "NONE"
        self.motors = (0, 0)
        self.expected_generation = 0
        if code != "DONE":
            self.abort(code)
            return True
        self.clear_since = None
        self.phase = "WAIT_AWAY_CLEAR" if self.phase == "TURN_AWAY" else "WAIT_BYPASS_CLEAR"
        return True

    def tick_clear(self, elapsed_ms, clear=True, valid=True):
        self.now += elapsed_ms
        self.path_clear = clear and valid
        if self.phase not in {"WAIT_AWAY_CLEAR", "WAIT_BYPASS_CLEAR"}:
            return
        if not self.path_clear:
            self.clear_since = None
            return
        if self.clear_since is None:
            self.clear_since = self.now
            return
        if self.now - self.clear_since < config_number("OBSTACLE_DETOUR_CLEAR_STABLE_MS"):
            return
        self.clear_since = None
        self.operation = "MOVE"
        self.owner = "REPLAY"
        self.motors = (15, 15)
        self.generation += 1
        self.expected_generation = self.generation
        if self.phase == "WAIT_AWAY_CLEAR":
            self.phase = "MOVE_AWAY"
            self.travel_total += int(config_number("OBSTACLE_DETOUR_MOVE_AWAY_MM"))
        else:
            self.phase = "MOVE_BYPASS"
            self.travel_total += int(config_number("OBSTACLE_DETOUR_BYPASS_MM"))

    def distance_result(self, code="DONE", generation=None):
        if generation != self.expected_generation:
            return False
        if self.phase not in {"MOVE_AWAY", "MOVE_BYPASS"}:
            return False
        self.operation = "NONE"
        self.owner = "NONE"
        self.motors = (0, 0)
        self.expected_generation = 0
        if code != "DONE":
            self.abort(code)
            return True
        if self.phase == "MOVE_AWAY":
            self.phase = "TURN_PARALLEL"
            self.operation = "TURN"
            self.owner = "REPLAY"
            self.motors = (-15, 15) if self.away_right else (15, -15)
            self.generation += 1
            self.expected_generation = self.generation
            self.turn_total += 45
        else:
            self.phase = "COMPLETE_HOLD"
            self.mode = "REPLAY_HOLD"
            self.hold_reason = "OBSTACLE"
            self.operation = "HOLD"
            self.replay_active = True
            self.auto_resumed = False
        return True

    def abort(self, reason="ERROR"):
        self.phase = "ABORTED"
        self.mode = "REPLAY_HOLD"
        self.hold_reason = "OBSTACLE"
        self.operation = "HOLD"
        self.owner = "NONE"
        self.motors = (0, 0)
        self.expected_generation = 0
        self.replay_active = True
        self.abort_reason = reason

    def start_after_abort(self):
        return self.arm()

    def phase1_resume(self):
        if self.decision == "NONE" and self.stable and self.path_clear:
            self.mode = "REPLAY_RUNNING"
            self.hold_reason = "NONE"
            self.operation = "NONE"
            self.auto_resumed = False
            return True
        return False

    def controller_tick(self):
        """Model the owner guard that runs after an immediate obstacle stop."""
        waiting_for_start = (
            self.mode == "REPLAY_HOLD"
            and self.hold_reason == "OBSTACLE"
            and self.operation == "HOLD"
        )
        if (self.replay_active and self.owner != "REPLAY"
                and self.operation != "NONE" and not waiting_for_start):
            self.abort("EXTERNAL_STOP")

    def complete(self):
        self.route_mutated = False
        self.auto_resumed = False
        return self.distance_result("DONE", self.expected_generation)


class SensorGateModel:
    """Model the Phase 3 post-turn path-clear safety boundary."""

    def __init__(self, strict=True, bounded=False, left_zone="CLEAR",
                 right_zone="CLEAR", overall_zone="CLEAR",
                 requested_speed=15, degraded_limit=8,
                 recent_window=True, timeout_budget=True,
                 prior_far_clear=True, valid_echo=True):
        self.strict = strict
        self.bounded = (bounded and recent_window and timeout_budget and
                        prior_far_clear and valid_echo)
        self.left_zone = left_zone
        self.right_zone = right_zone
        self.overall_zone = overall_zone
        self.requested_speed = requested_speed
        self.degraded_limit = degraded_limit

    def path_clear(self):
        safe_zones = {"CLEAR", "CAUTION"}
        no_hard_block = self.left_zone in safe_zones and self.right_zone in safe_zones
        no_overall_block = self.overall_zone in safe_zones
        return (self.strict and self.overall_zone == "CLEAR") or (
            self.bounded and no_hard_block and no_overall_block
        )

    def permitted_forward(self):
        return (self.requested_speed if self.strict else
                min(self.requested_speed, self.degraded_limit))


class ObstacleDetourHostTests(unittest.TestCase):
    def test_left_obstacle_avoid_right_turns_right(self):
        model = DetourModel("AVOID_RIGHT")
        self.assertTrue(model.arm())
        self.assertTrue(model.away_right)
        self.assertEqual(model.motors, (15, -15))

    def test_right_obstacle_avoid_left_turns_left(self):
        model = DetourModel("AVOID_LEFT")
        self.assertTrue(model.arm())
        self.assertFalse(model.away_right)
        self.assertEqual(model.motors, (-15, 15))

    def test_center_hold_start_does_not_detour(self):
        self.assertFalse(DetourModel("HOLD").arm())

    def test_unknown_hold_does_not_detour(self):
        self.assertFalse(DetourModel("HOLD", stable=False).arm())

    def test_unstable_decision_does_not_detour(self):
        self.assertFalse(DetourModel("AVOID_RIGHT", stable=False).arm())

    def test_clear_start_keeps_phase1_resume_path(self):
        model = DetourModel("NONE")
        model.path_clear = True
        self.assertFalse(model.arm())
        self.assertTrue(model.phase1_resume())
        self.assertEqual(model.phase, "IDLE")

    def test_turn_away_done_enters_wait_away_clear(self):
        model = DetourModel()
        model.arm()
        generation = model.expected_generation
        self.assertTrue(model.turn_result(generation=generation))
        self.assertEqual(model.phase, "WAIT_AWAY_CLEAR")
        self.assertEqual(model.motors, (0, 0))

    def test_one_clear_sample_does_not_move(self):
        model = DetourModel()
        model.arm()
        model.turn_result(generation=model.expected_generation)
        model.tick_clear(0, clear=True)
        model.tick_clear(100, clear=True)
        self.assertEqual(model.phase, "WAIT_AWAY_CLEAR")

    def test_clear_stable_window_starts_move_away(self):
        model = DetourModel()
        model.arm()
        model.turn_result(generation=model.expected_generation)
        model.tick_clear(0, clear=True)
        model.tick_clear(400, clear=True)
        self.assertEqual(model.phase, "MOVE_AWAY")
        self.assertEqual(model.motors, (15, 15))

    def test_invalid_sensor_resets_clear_timer(self):
        model = DetourModel()
        model.arm()
        model.turn_result(generation=model.expected_generation)
        model.tick_clear(0, clear=True)
        model.tick_clear(200, clear=False, valid=False)
        model.tick_clear(200, clear=True)
        self.assertEqual(model.phase, "WAIT_AWAY_CLEAR")

    def test_blocked_before_move_stays_stopped(self):
        model = DetourModel()
        model.arm()
        model.turn_result(generation=model.expected_generation)
        model.tick_clear(0, clear=False)
        model.tick_clear(1000, clear=False)
        self.assertEqual(model.phase, "WAIT_AWAY_CLEAR")
        self.assertEqual(model.motors, (0, 0))

    def test_move_away_done_starts_opposite_parallel_turn(self):
        model = DetourModel()
        model.arm()
        model.turn_result(generation=model.expected_generation)
        model.tick_clear(0, clear=True)
        model.tick_clear(400, clear=True)
        self.assertTrue(model.distance_result(generation=model.expected_generation))
        self.assertEqual(model.phase, "TURN_PARALLEL")
        self.assertEqual(model.motors, (-15, 15))

    def test_parallel_turn_done_enters_bypass_wait(self):
        model = DetourModel()
        model.arm()
        model.turn_result(generation=model.expected_generation)
        model.tick_clear(0, clear=True)
        model.tick_clear(400, clear=True)
        model.distance_result(generation=model.expected_generation)
        self.assertTrue(model.turn_result(generation=model.expected_generation))
        self.assertEqual(model.phase, "WAIT_BYPASS_CLEAR")

    def test_bypass_clear_stable_starts_move(self):
        model = DetourModel()
        model.arm()
        model.turn_result(generation=model.expected_generation)
        model.tick_clear(0, clear=True)
        model.tick_clear(400, clear=True)
        model.distance_result(generation=model.expected_generation)
        model.turn_result(generation=model.expected_generation)
        model.tick_clear(0, clear=True)
        model.tick_clear(400, clear=True)
        self.assertEqual(model.phase, "MOVE_BYPASS")

    def test_bypass_done_enters_complete_hold(self):
        model = DetourModel()
        model.arm()
        model.turn_result(generation=model.expected_generation)
        model.tick_clear(0, clear=True)
        model.tick_clear(400, clear=True)
        model.distance_result(generation=model.expected_generation)
        model.turn_result(generation=model.expected_generation)
        model.tick_clear(0, clear=True)
        model.tick_clear(400, clear=True)
        self.assertTrue(model.distance_result(generation=model.expected_generation))
        self.assertEqual(model.phase, "COMPLETE_HOLD")
        self.assertEqual(model.motors, (0, 0))

    def test_success_does_not_advance_waypoint(self):
        model = DetourModel()
        original = (model.current, model.target)
        self._finish(model)
        self.assertEqual((model.current, model.target), original)

    def test_success_does_not_auto_resume_replay(self):
        model = DetourModel()
        self._finish(model)
        self.assertFalse(model.auto_resumed)
        self.assertEqual(model.mode, "REPLAY_HOLD")

    def test_success_preserves_route_context(self):
        model = DetourModel()
        original = (model.route_generation, model.current, model.target)
        self._finish(model)
        self.assertEqual((model.route_generation, model.current, model.target), original)

    def test_stale_turn_result_is_ignored(self):
        model = DetourModel()
        model.arm()
        original = model.phase
        self.assertFalse(model.turn_result(generation=model.expected_generation - 1))
        self.assertEqual(model.phase, original)

    def test_stale_distance_result_is_ignored(self):
        model = DetourModel()
        model.arm()
        model.turn_result(generation=model.expected_generation)
        model.tick_clear(0, clear=True)
        model.tick_clear(400, clear=True)
        original = model.phase
        self.assertFalse(model.distance_result(generation=model.expected_generation - 1))
        self.assertEqual(model.phase, original)

    def test_turn_timeout_aborts_and_holds(self):
        model = DetourModel()
        model.arm()
        model.abort("TIMEOUT")
        self.assertEqual(model.phase, "ABORTED")
        self.assertEqual(model.motors, (0, 0))

    def test_obstacle_during_move_aborts(self):
        model = DetourModel()
        model.arm()
        model.turn_result(generation=model.expected_generation)
        model.tick_clear(0, clear=True)
        model.tick_clear(400, clear=True)
        model.distance_result("OBSTACLE", model.expected_generation)
        self.assertEqual(model.phase, "ABORTED")

    def test_encoder_fault_aborts(self):
        model = DetourModel()
        model.arm()
        model.turn_result(generation=model.expected_generation)
        model.tick_clear(0, clear=True)
        model.tick_clear(400, clear=True)
        model.distance_result("ENCODER_FAULT", model.expected_generation)
        self.assertEqual(model.phase, "ABORTED")

    def test_manual_stop_aborts_and_cannot_restart(self):
        model = DetourModel()
        model.arm()
        model.abort("USER_STOP")
        self.assertEqual(model.motors, (0, 0))
        self.assertFalse(model.start_after_abort())

    def test_one_attempt_budget(self):
        self.assertEqual(config_number("OBSTACLE_DETOUR_MAX_ATTEMPTS"), 1.0)
        model = DetourModel()
        self.assertTrue(model.arm())
        self.assertEqual(model.attempts, 1)

    def test_second_start_same_event_is_rejected(self):
        model = DetourModel()
        model.arm()
        model.abort("OBSTACLE")
        self.assertFalse(model.start_after_abort())

    def test_no_reverse_command(self):
        model = DetourModel()
        self._finish(model)
        self.assertTrue(model.forward_only)
        self.assertIn("startReplayDistance(true", MAP)

    def test_no_route_mutation(self):
        phase3 = self._phase3_source()
        self.assertNotIn("route_ =", phase3)
        self.assertNotIn("route_.header.generation =", phase3)
        self.assertNotRegex(phase3, r"replayCurrentIndex_\s*=(?!=)")
        self.assertNotRegex(phase3, r"replayTargetIndex_\s*=(?!=)")

    def test_no_flash_write(self):
        self.assertNotIn("store_.save", self._phase3_source())
        self.assertNotIn("EEPROM", self._phase3_source())

    def test_phase1_obstacle_hold_contract_remains(self):
        self.assertIn("enterReplayHold(MapHoldReason::OBSTACLE, true)", MAP)
        self.assertIn("OBSTACLE_CLEAR_STABLE_MS", MAP)
        self.assertIn('debug_.println("MAP,START,ACTION=RESUME")', MAP)

    def test_obstacle_hold_is_not_reclassified_as_external_stop(self):
        model = DetourModel()
        model.controller_tick()
        self.assertEqual(model.hold_reason, "OBSTACLE")
        self.assertEqual(model.operation, "HOLD")
        self.assertEqual(model.motors, (0, 0))
        update_guard = MAP[MAP.index("} else if (robot_.motionOwner()"):
                           MAP.index("} else {\n      updateReplay();", MAP.index("} else if (robot_.motionOwner()"))]
        self.assertIn("holdReason_ == MapHoldReason::OBSTACLE", update_guard)
        self.assertIn("replayOperation_ == MapReplayOperation::HOLD", update_guard)

    def test_strict_healthy_clear_allows_phase3_path(self):
        self.assertTrue(SensorGateModel(strict=True).path_clear())

    def test_short_timeout_after_proven_far_clear_uses_bounded_window(self):
        model = SensorGateModel(strict=False, bounded=True)
        self.assertTrue(model.path_clear())

    def test_bounded_degraded_motion_keeps_production_speed_cap(self):
        model = SensorGateModel(strict=False, bounded=True, requested_speed=15,
                                degraded_limit=8)
        self.assertEqual(model.permitted_forward(), 8)
        self.assertIn("ULTRASONIC_DEGRADED_MAX_FORWARD_COMMAND", CONFIG)
        self.assertIn("limitForwardCommand(forward)",
                      (STM32_ROOT / "src" / "control" / "robot_controller.cpp")
                      .read_text(encoding="utf-8"))

    def test_previous_near_obstacle_timeout_fails_closed(self):
        self.assertFalse(SensorGateModel(strict=False, bounded=True,
                                         left_zone="BLOCKED",
                                         right_zone="CLEAR",
                                         prior_far_clear=False).path_clear())

    def test_startup_without_echo_fails_closed(self):
        self.assertFalse(SensorGateModel(strict=False, bounded=True,
                                         left_zone="UNKNOWN",
                                         right_zone="UNKNOWN",
                                         overall_zone="UNKNOWN",
                                         valid_echo=False).path_clear())

    def test_expired_bounded_window_fails_closed(self):
        self.assertFalse(SensorGateModel(strict=False, bounded=True,
                                         recent_window=False).path_clear())

    def test_exceeded_timeout_budget_fails_closed(self):
        self.assertFalse(SensorGateModel(strict=False, bounded=True,
                                         timeout_budget=False).path_clear())

    def test_blocked_or_emergency_fails_closed(self):
        for zone in ("BLOCKED", "EMERGENCY"):
            self.assertFalse(SensorGateModel(strict=False, bounded=True,
                                             left_zone=zone,
                                             overall_zone=zone).path_clear())

    def test_invalid_electrical_pulse_fails_closed(self):
        self.assertFalse(SensorGateModel(strict=False, bounded=True,
                                         valid_echo=False).path_clear())

    def test_path_gate_reuses_bounded_sensor_policy_without_lcd_state(self):
        start = MAP.index("bool MapController::obstacleDetourPathClear")
        end = MAP.index("bool MapController::obstacleDetourEntryGates", start)
        path_gate = MAP[start:end]
        self.assertIn("hasRecentClearWindow(millis())", path_gate)
        self.assertIn("ObstacleZone::UNKNOWN", path_gate)
        self.assertIn("ObstacleZone::BLOCKED", path_gate)
        self.assertIn("ObstacleZone::EMERGENCY", path_gate)
        self.assertNotIn("displayFar", path_gate)
        self.assertNotIn("displayNoEchoFar", path_gate)

    def test_phase2_classifier_remains_decision_only(self):
        for forbidden in ("startReplay", "startTurn", "startDistance", "MotorController", "route_"):
            self.assertNotIn(forbidden, CLASSIFIER)
        self.assertIn("obstacleClassifier.update();", MAIN)
        self.assertIn("obstacleClassifier_.decision()", MAP)

    def test_static_safety_contract(self):
        phase3 = self._phase3_source()
        for forbidden in ("analogWrite", "digitalWrite", "PWM_PIN", "DIR_PIN",
                          "MotorController", "advanceReplayAfterTarget", "reverse=true"):
            self.assertNotIn(forbidden, phase3)
        self.assertNotIn("startNextReplaySegment", phase3)
        self.assertIn("OBSTACLE_DETOUR_MAX_TOTAL_DISTANCE_MM", phase3)
        self.assertIn("OBSTACLE_DETOUR_MAX_TOTAL_TURN_DEG", phase3)

    def test_constants_and_budgets(self):
        self.assertEqual(config_number("OBSTACLE_DETOUR_TURN_AWAY_DEG"), 45.0)
        self.assertEqual(config_number("OBSTACLE_DETOUR_MOVE_AWAY_MM"), 250.0)
        self.assertEqual(config_number("OBSTACLE_DETOUR_BYPASS_MM"), 450.0)
        self.assertEqual(config_number("OBSTACLE_DETOUR_MAX_TOTAL_DISTANCE_MM"), 700.0)
        self.assertEqual(config_number("OBSTACLE_DETOUR_MAX_TOTAL_TURN_DEG"), 90.0)

    def _phase3_source(self):
        start = MAP.index("bool MapController::obstacleDetourContextActive")
        end = MAP.index("MapController::Pose MapController::routePointWorld", start)
        return MAP[start:end]

    def _finish(self, model):
        self.assertTrue(model.arm())
        model.turn_result(generation=model.expected_generation)
        model.tick_clear(0, clear=True)
        model.tick_clear(400, clear=True)
        model.distance_result(generation=model.expected_generation)
        model.turn_result(generation=model.expected_generation)
        model.tick_clear(0, clear=True)
        model.tick_clear(400, clear=True)
        self.assertTrue(model.complete())


if __name__ == "__main__":
    unittest.main(verbosity=2)
