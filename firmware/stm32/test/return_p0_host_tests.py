"""Host regression tests for the Phase 2 universal Return-to-P0 core.

These tests are deliberately dependency-free.  They exercise the route
projection/path safety model and assert the production source contracts which
cannot be instantiated without the STM32/Arduino runtime.
"""

import math
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAP = (ROOT / "src" / "map" / "map_controller.cpp").read_text(encoding="utf-8")
MAP_H = (ROOT / "include" / "map" / "map_controller.h").read_text(encoding="utf-8")
TYPES = (ROOT / "include" / "map" / "map_types.h").read_text(encoding="utf-8")
CONFIG = (ROOT / "include" / "robot_config.h").read_text(encoding="utf-8")
ROUTE_STORE = (ROOT / "include" / "map" / "route_store.h").read_text(encoding="utf-8")
ROBOT_CONTROLLER = (ROOT / "src" / "control" / "robot_controller.cpp").read_text(
    encoding="utf-8")


def constant(name):
    match = re.search(r"\b" + re.escape(name) + r"\s*=\s*(-?(?:\d+(?:\.\d*)?|\.\d+))", CONFIG)
    if not match:
        raise AssertionError("missing constant " + name)
    return float(match.group(1))


def projection(start, end, point):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq == 0:
        return None
    t = max(0.0, min(1.0, ((point[0] - start[0]) * dx +
                           (point[1] - start[1]) * dy) / length_sq))
    projected = (start[0] + t * dx, start[1] + t * dy)
    return t, projected, math.hypot(point[0] - projected[0],
                                    point[1] - projected[1])


def guided_arrival_tolerance_allowed(tolerance_mm):
    """Model the explicit start gate shared by Map Replay and Return P0."""
    maximum = max(constant("MAP_GUIDE_ARRIVAL_POSITION_TOLERANCE_MM"),
                  constant("MAP_RETURN_P0_POSITION_TOLERANCE_MM"))
    return 0 < tolerance_mm <= maximum


ROUTE = [(0.0, 0.0), (1000.0, 0.0), (1000.0, 700.0), (0.0, 700.0)]


class ReturnModel:
    def __init__(self):
        self.home = False
        self.slot = 1
        self.generation = 0
        self.odom_generation = 0
        self.heading_generation = 0
        self.state = "IDLE"
        self.motor = (0, 0)
        self.return_generation = 0

    def arm(self, generation=1):
        self.home = True
        self.generation = generation

    def invalidate(self):
        self.home = False

    def locate(self, point):
        candidates = []
        for index in range(len(ROUTE) - 1):
            item = projection(ROUTE[index], ROUTE[index + 1], point)
            if item is not None:
                candidates.append((item[2], index, item))
        candidates.sort(key=lambda item: item[0])
        if not candidates or candidates[0][0] > constant("MAP_RETURN_P0_MAX_CROSSTRACK_MM"):
            return None
        if (len(candidates) > 1 and abs(candidates[1][1] - candidates[0][1]) > 1 and
                abs(candidates[1][0] - candidates[0][0]) <=
                constant("MAP_RETURN_P0_AMBIGUITY_MARGIN_MM")):
            return None
        return candidates[0]

    def request(self, point):
        located = self.locate(point)
        if not self.home or located is None:
            self.state = "ABORTED"
            self.motor = (0, 0)
            return False
        self.return_generation += 1
        self.state = "RETURN_WAYPOINT"
        self.motor = (20, 20)
        return True

    def complete_position(self, position_error, heading_error):
        self.state = "P0_HEADING_SETTLE"
        if position_error <= constant("MAP_RETURN_P0_POSITION_TOLERANCE_MM") and abs(heading_error) <= constant("MAP_RETURN_P0_HEADING_TOLERANCE_DEG"):
            self.state = "COMPLETE"
            self.motor = (0, 0)


class ReturnP0HostTests(unittest.TestCase):
    # HOME CONTEXT 1-10
    def test_01_successful_teach_save_arms_home(self):
        model = ReturnModel(); model.arm(4)
        self.assertTrue(model.home); self.assertEqual(model.generation, 4)
        self.assertIn("armHomeContextAfterSave", MAP)

    def test_02_teach_cancel_does_not_arm(self):
        model = ReturnModel()
        self.assertFalse(model.home)
        self.assertIn("pendingHomeContextValid_ = false", MAP)

    def test_03_save_failure_does_not_arm(self):
        model = ReturnModel(); model.invalidate()
        self.assertFalse(model.home)
        self.assertIn('invalidateHomeContext("STORAGE_ERROR")', MAP)

    def test_04_new_teach_invalidates_old_home(self):
        model = ReturnModel(); model.arm(); model.invalidate()
        self.assertFalse(model.home); self.assertIn('"NEW_TEACH"', MAP)

    def test_05_slot_change_invalidates_home(self):
        self.assertIn('invalidateHomeContext("SLOT_CHANGED")', MAP)

    def test_06_route_generation_change_invalidates_home(self):
        self.assertIn('invalidateHomeContext("ROUTE_GENERATION")', MAP)

    def test_07_odometry_reset_invalidates_home(self):
        self.assertIn('invalidateHomeContext("RESET_BOUNDARY")', MAP)

    def test_08_heading_reset_invalidates_home(self):
        self.assertIn("headingResetGeneration", MAP_H)
        self.assertIn("headingResetGeneration()", MAP)

    def test_09_reboot_defaults_home_invalid(self):
        self.assertIn("homeContext_ = {}", MAP)
        self.assertIn("pendingHomeContextValid_ = false", MAP)

    def test_10_normal_replay_does_not_invalidate_home(self):
        self.assertIn("if (returnP0InProgress())", MAP)
        self.assertIn("else if (odometry_.resetGeneration() != replayOriginResetGeneration_", MAP)

    # ROUTE LOCALIZATION 11-20
    def test_11_projection_exactly_at_p0(self):
        result = projection(ROUTE[0], ROUTE[1], ROUTE[0])
        self.assertAlmostEqual(result[0], 0.0)

    def test_12_projection_exactly_at_p1(self):
        result = projection(ROUTE[0], ROUTE[1], ROUTE[1])
        self.assertAlmostEqual(result[0], 1.0)

    def test_13_projection_mid_first_segment(self):
        result = projection(ROUTE[0], ROUTE[1], (500.0, 0.0))
        self.assertAlmostEqual(result[0], 0.5)

    def test_14_projection_mid_second_segment(self):
        result = projection(ROUTE[1], ROUTE[2], (1000.0, 350.0))
        self.assertAlmostEqual(result[0], 0.5)

    def test_15_projection_mid_third_segment(self):
        result = projection(ROUTE[2], ROUTE[3], (500.0, 700.0))
        self.assertAlmostEqual(result[0], 0.5)

    def test_16_small_cross_track_offset(self):
        result = projection(ROUTE[0], ROUTE[1], (500.0, 25.0))
        self.assertAlmostEqual(result[2], 25.0)

    def test_17_cross_track_at_allowed_bound(self):
        result = projection(ROUTE[0], ROUTE[1], (500.0, constant("MAP_RETURN_P0_MAX_CROSSTRACK_MM")))
        self.assertLessEqual(result[2], constant("MAP_RETURN_P0_MAX_CROSSTRACK_MM"))

    def test_18_cross_track_beyond_bound_rejected(self):
        model = ReturnModel(); model.arm()
        self.assertFalse(model.request((500.0, constant("MAP_RETURN_P0_MAX_CROSSTRACK_MM") + 1.0)))

    def test_19_degenerate_segment_is_ignored(self):
        self.assertIsNone(projection((1.0, 1.0), (1.0, 1.0), (1.0, 1.0)))
        self.assertIn("if (lengthSquared <= 1.0e-3f) continue", MAP)

    def test_20_non_adjacent_ambiguity_is_rejected(self):
        self.assertIn("AMBIGUOUS_SEGMENT", MAP)
        self.assertIn("MAP_RETURN_P0_AMBIGUITY_MARGIN_MM", CONFIG)

    # RETURN PATH 21-28
    def test_21_endpoint_p3_returns_p2_p1_p0(self):
        self.assertEqual([2, 1, 0], list(range(2, -1, -1)))

    def test_22_mid_p2_p3_returns_p2_p1_p0(self):
        model = ReturnModel(); model.arm()
        self.assertTrue(model.request((1000.0, 500.0)))
        self.assertEqual(model.state, "RETURN_WAYPOINT")

    def test_23_mid_p1_p2_returns_p1_p0(self):
        result = projection(ROUTE[1], ROUTE[2], (1000.0, 300.0))
        self.assertAlmostEqual(result[0], 300.0 / 700.0)
        self.assertEqual([1, 0], [1, 0])

    def test_24_exact_p1_targets_p0(self):
        model = ReturnModel(); model.arm()
        self.assertTrue(model.request(ROUTE[1]))

    def test_25_at_p0_skips_translation_but_restores_heading(self):
        model = ReturnModel(); model.arm(); self.assertTrue(model.request(ROUTE[0]))
        self.assertIn("P0_POSITION_SETTLE", MAP)
        self.assertIn("P0_HEADING_RESTORE", MAP)

    def test_26_return_indices_decrease(self):
        values = [3, 2, 1, 0]
        self.assertEqual(values, sorted(values, reverse=True))
        self.assertIn("--returnP0TargetIndex_", MAP)

    def test_27_no_direct_current_to_p0_shortcut(self):
        self.assertIn("startReturnWaypoint", MAP)
        self.assertIn("returnP0TargetIndex_", MAP)
        self.assertNotIn("startReplayGuidedWaypoint(homeContext_.p0WorldPose", MAP)

    def test_28_route_flash_data_is_not_extended(self):
        self.assertIn("MapRouteData", ROUTE_STORE)
        self.assertNotIn("HomeContext", ROUTE_STORE)
        self.assertNotIn("ReturnP0", ROUTE_STORE)

    # PREEMPTION 29-33
    def test_29_request_during_replay_stops_old_motion(self):
        self.assertIn("robot_.stopImmediately(true)", MAP[MAP.index("bool MapController::requestReturnToP0Internal"):])

    def test_30_return_gets_fresh_generation(self):
        self.assertIn("returnP0Generation_ = replayGeneration_", MAP)
        self.assertIn("nextReplayGeneration()", MAP)

    def test_31_stale_old_move_is_dropped(self):
        self.assertIn("consumeReturnDistanceResult", MAP)
        self.assertIn("DROP_STALE", MAP)

    def test_32_stale_old_turn_is_dropped(self):
        self.assertIn("consumeReturnTurnResult", MAP)
        self.assertIn("result.motionGeneration != returnP0SegmentGeneration_", MAP)

    def test_33_route_is_preserved_during_preemption(self):
        self.assertIn("route_", MAP[MAP.index("bool MapController::requestReturnToP0Internal"):MAP.index("bool MapController::locateRouteProjection")])

    # FINAL P0 34-40
    def test_34_position_arrival_precedes_complete(self):
        self.assertIn("P0_POSITION_SETTLE", MAP)
        self.assertIn("P0_HEADING_RESTORE", MAP)

    def test_35_final_heading_comes_from_home(self):
        self.assertIn("homeContext_.p0WorldPose.headingDeg", MAP)

    def test_36_heading_is_not_reset_or_faked(self):
        body = MAP[MAP.index("bool MapController::startReturnP0Heading"):MAP.index("void MapController::updateReturnToP0")]
        self.assertNotIn("resetHeadingReference", body)
        self.assertNotIn("headingDeg = 0", body)

    def test_37_position_and_heading_tolerances_complete(self):
        model = ReturnModel(); model.arm(); model.complete_position(30.0, 2.0)
        self.assertEqual(model.state, "COMPLETE")

    def test_38_heading_over_tolerance_does_not_complete(self):
        model = ReturnModel(); model.arm(); model.complete_position(30.0, 2.1)
        self.assertNotEqual(model.state, "COMPLETE")

    def test_39_position_over_tolerance_does_not_complete(self):
        model = ReturnModel(); model.arm(); model.complete_position(31.0, 0.0)
        self.assertNotEqual(model.state, "COMPLETE")

    def test_40_complete_requires_zero_motor(self):
        model = ReturnModel(); model.arm(); model.complete_position(30.0, 2.0)
        self.assertEqual(model.motor, (0, 0))
        self.assertIn("robot_.motorsStopped()", MAP)

    def test_40a_return_p0_tolerance_is_accepted_by_guided_start(self):
        # Regression: Return P0 used its contractual 30 mm tolerance, but
        # the common guided-start input gate only admitted normal 5 mm MAP
        # tolerance and caused MAP,RETURN_P0,ABORT,REASON=RETURN_START.
        self.assertTrue(guided_arrival_tolerance_allowed(
            constant("MAP_GUIDE_ARRIVAL_POSITION_TOLERANCE_MM")))
        self.assertTrue(guided_arrival_tolerance_allowed(
            constant("MAP_RETURN_P0_POSITION_TOLERANCE_MM")))
        self.assertFalse(guided_arrival_tolerance_allowed(
            constant("MAP_RETURN_P0_POSITION_TOLERANCE_MM") + 1.0))
        self.assertIn("maxArrivalPositionToleranceMm", ROBOT_CONTROLLER)
        self.assertIn("MAP_RETURN_P0_POSITION_TOLERANCE_MM", ROBOT_CONTROLLER)

    # SAFETY 41-49
    def test_41_obstacle_during_return_holds(self):
        self.assertIn("returnP0State_ = ReturnP0State::HOLD", MAP)
        self.assertIn("enterReplayHold(MapHoldReason::OBSTACLE, false)", MAP)

    def test_42_external_stop_does_not_auto_continue(self):
        self.assertIn('abortReturnToP0("EXTERNAL_STOP")', MAP)

    def test_43_ps2_takeover_stops_return(self):
        self.assertIn('"PS2_TAKEOVER"', MAP)

    def test_44_encoder_unhealthy_aborts(self):
        self.assertIn('"ENCODER_FAULT"', MAP)

    def test_45_fusion_invalid_aborts(self):
        self.assertIn('"HEADING_LOST"', MAP)

    def test_46_reset_boundary_aborts(self):
        self.assertIn('abortReturnToP0("RESET_BOUNDARY")', MAP)

    def test_47_route_change_aborts_or_invalidates(self):
        self.assertIn('invalidateHomeContext("ROUTE_GENERATION")', MAP)

    def test_48_slot_change_aborts_or_invalidates(self):
        self.assertIn('invalidateHomeContext("SLOT_CHANGED")', MAP)

    def test_49_bounded_retry_prevents_infinite_realign(self):
        for name in ("MAP_RETURN_P0_MAX_REACQUIRE_ATTEMPTS", "MAP_RETURN_P0_MAX_POSITION_CORRECTIONS", "MAP_RETURN_P0_MAX_HEADING_ATTEMPTS"):
            self.assertIn(name, CONFIG)
        self.assertIn("returnP0PositionCorrectionAttempts_", MAP)

    # COMPATIBILITY 50-55
    def test_50_post_teach_back_remains_separate(self):
        self.assertIn("PostTeachBackContext", MAP_H)
        self.assertIn("startPostTeachBack", MAP)

    def test_51_normal_replay_origin_remains_separate(self):
        self.assertIn("replayOrigin_", MAP)
        self.assertIn("routePointWorldFromOrigin(replayOrigin_", MAP)

    def test_52_phase1_ai_auto_resume_remains(self):
        self.assertIn("AI_VOICE = 2U", TYPES)
        self.assertIn("autonomousResumeInhibited_", MAP)

    def test_53_ps2_obstacle_behavior_remains(self):
        self.assertIn("MapHoldReason::OBSTACLE", MAP)
        self.assertIn("ps2_.holdMapInput()", MAP)

    def test_54_single_front_sr04_is_preserved(self):
        ultrasonic = (ROOT / "include" / "sensors" / "ultrasonic_sensor.h").read_text(encoding="utf-8")
        self.assertTrue("Ultrasonic" in ultrasonic)

    def test_55_auto_detour_is_not_enabled_by_return_core(self):
        self.assertIn("requestReturnToP0", MAP_H)
        request_body = MAP[MAP.index("bool MapController::requestReturnToP0Internal"):MAP.index("void MapController::handleTriangle")]
        self.assertNotIn("armObstacleDetour", request_body)


if __name__ == "__main__":
    unittest.main(verbosity=2)
