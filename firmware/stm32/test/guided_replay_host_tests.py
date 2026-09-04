"""Host checks for the MAP coarse-turn and guided-waypoint replay path.

The production controller depends on Arduino and STM32 peripherals, so the
tests below combine source-contract checks with a small geometry oracle.  The
oracle intentionally mirrors only the bounded guidance math; it does not
replace the real motor/safety controller.
"""

import math
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = (ROOT / "include" / "robot_config.h").read_text(encoding="utf-8")
MAP_TYPES = (ROOT / "include" / "map" / "map_types.h").read_text(
    encoding="utf-8"
)
CTRL = (ROOT / "src" / "control" / "robot_controller.cpp").read_text(
    encoding="utf-8"
)
CTRL_HEADER = (ROOT / "include" / "control" / "robot_controller.h").read_text(
    encoding="utf-8"
)
MAP = (ROOT / "src" / "map" / "map_controller.cpp").read_text(
    encoding="utf-8"
)
MAP_HEADER = (ROOT / "include" / "map" / "map_controller.h").read_text(
    encoding="utf-8"
)
MAIN = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")


def config_number(name):
    match = re.search(
        r"\b" + re.escape(name) + r"\s*=\s*(-?(?:\d+(?:\.\d*)?|\.\d+))",
        CONFIG,
    )
    if not match:
        raise AssertionError(f"constant {name} not found")
    return float(match.group(1))


def shortest_delta(target, current):
    delta = target - current
    while delta > 180.0:
        delta -= 360.0
    while delta <= -180.0:
        delta += 360.0
    return delta


def bearing(current, target):
    return math.degrees(
        math.atan2(target[1] - current[1], target[0] - current[0])
    )


def cross_track(start, target, current):
    dx = target[0] - start[0]
    dy = target[1] - start[1]
    length = math.hypot(dx, dy)
    if length <= 1.0:
        return 0.0
    value = (dx * (current[1] - start[1]) -
             dy * (current[0] - start[0])) / length
    limit = config_number("MAP_GUIDE_MAX_CROSSTRACK_MM")
    return max(-limit, min(limit, value))


def steering(heading_error, cross_track_error):
    value = (
        config_number("MAP_GUIDE_HEADING_GAIN") * heading_error -
        config_number("MAP_GUIDE_CROSSTRACK_GAIN") * cross_track_error
    )
    limit = config_number("MAP_GUIDE_MAX_STEER_COMMAND")
    return max(-limit, min(limit, round(value)))


def normal_decel_speed(remaining_mm, requested_speed):
    arrival = config_number("MAP_GUIDE_ARRIVAL_POSITION_TOLERANCE_MM")
    slow_distance = config_number("MAP_GUIDE_SLOW_DISTANCE_MM")
    ratio = max(0.0, min(1.0, (remaining_mm - arrival) /
                              (slow_distance - arrival)))
    value = config_number("MAP_GUIDE_MIN_SPEED") + (
        requested_speed - config_number("MAP_GUIDE_MIN_SPEED")
    ) * ratio
    return math.floor(value + 0.5)


def without_comments(text):
    text = re.sub(r"//[^\n]*", "", text)
    return re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)


def arrival_action(distance_mm, incoming_heading_deg, current_heading_deg):
    """Mirror the position + incoming-segment-heading arrival gate."""
    position_tolerance = config_number("MAP_GUIDE_ARRIVAL_POSITION_TOLERANCE_MM")
    heading_tolerance = config_number("MAP_GUIDE_ARRIVAL_HEADING_TOLERANCE_DEG")
    error = shortest_delta(incoming_heading_deg, current_heading_deg)
    if distance_mm <= position_tolerance:
        return "ADVANCE" if abs(error) <= heading_tolerance else "ARRIVAL_REALIGN"
    return "PATH_GUIDANCE"


def realign_action(reason, distance_mm, incoming_heading_deg,
                   target_bearing_deg, current_heading_deg):
    """Mirror the MAP sequencer's next action for a realign reason."""
    position_tolerance = config_number("MAP_GUIDE_ARRIVAL_POSITION_TOLERANCE_MM")
    preturn_tolerance = config_number("MAP_REPLAY_PRETURN_TOLERANCE_DEG")
    if reason == "ARRIVAL":
        if distance_mm > position_tolerance:
            return "GUIDED"
        error = shortest_delta(incoming_heading_deg, current_heading_deg)
        return "ADVANCE" if abs(error) <= config_number(
            "MAP_GUIDE_ARRIVAL_HEADING_TOLERANCE_DEG"
        ) else "ARRIVAL_COARSE_TURN"
    error = shortest_delta(target_bearing_deg, current_heading_deg)
    if distance_mm <= position_tolerance:
        arrival_error = shortest_delta(incoming_heading_deg, current_heading_deg)
        return "ADVANCE" if abs(arrival_error) <= config_number(
            "MAP_GUIDE_ARRIVAL_HEADING_TOLERANCE_DEG"
        ) else "ARRIVAL_COARSE_TURN"
    return "PATH_COARSE_TURN" if abs(error) > preturn_tolerance else "GUIDED"


class GuidedReplayHostTests(unittest.TestCase):
    def test_named_map_profile_constants(self):
        self.assertEqual(config_number("MAP_REPLAY_PRETURN_TOLERANCE_DEG"), 5.0)
        self.assertGreaterEqual(config_number("MAP_REPLAY_PRETURN_SETTLE_MS"), 80.0)
        self.assertLessEqual(config_number("MAP_REPLAY_PRETURN_SETTLE_MS"), 120.0)
        self.assertEqual(config_number("MAP_GUIDE_ARRIVAL_HEADING_TOLERANCE_DEG"), 6.0)
        self.assertEqual(config_number("MAP_GUIDE_REALIGN_THRESHOLD_DEG"), 15.0)
        self.assertEqual(config_number("MAP_GUIDE_SLOW_DISTANCE_MM"), 250.0)

    def test_guided_primitive_is_replay_only_and_internal(self):
        self.assertIn("GUIDED_WAYPOINT", CTRL_HEADER)
        self.assertIn("startReplayGuidedWaypoint", CTRL_HEADER)
        self.assertIn("float arrivalBearingDeg", CTRL_HEADER)
        self.assertIn("guidedArrivalBearingDeg_", CTRL_HEADER)
        self.assertIn(
            "guidedArrivalBearingDeg_ = HeadingFusion::normalize(arrivalBearingDeg)",
            CTRL,
        )
        self.assertIn(
            "const float incomingBearing = guidedArrivalBearingDeg_", CTRL,
        )
        self.assertIn("motionOwner_ = MotionOwner::REPLAY", CTRL)
        self.assertNotIn("startReplayGuidedWaypoint", MAIN)
        self.assertNotIn("GUIDED_WAYPOINT", (ROOT / "src" / "communication" /
                                               "robot_link_server.cpp").read_text(
                                                   encoding="utf-8"
                                               ))

    def test_preturn_skip_at_three_degrees(self):
        self.assertLessEqual(3.0, config_number("MAP_REPLAY_PRETURN_TOLERANCE_DEG"))
        self.assertIn("bool coarsePreturn = false", MAP)
        self.assertIn("!startupNoiseTurn", MAP)
        self.assertIn("startReplayGuidedWaypoint", MAP)
        self.assertIn('logGuidePreturn(desiredHeadingError, "SKIP")', MAP)

    def test_preturn_coarse_at_seventy_degrees(self):
        self.assertGreater(70.0, config_number("MAP_REPLAY_PRETURN_TOLERANCE_DEG"))
        self.assertIn("AiTurnProfile::MAP_COARSE", MAP)
        self.assertIn("MAP_REPLAY_PRETURN_TOLERANCE_DEG", CTRL)
        self.assertIn("aiTurnToleranceDeg_", CTRL)
        self.assertIn("aiTurnSettleMs_", CTRL)

    def test_dynamic_bearing_does_not_hold_start_heading(self):
        target_bearing = bearing((400.0, 80.0), (1000.0, 0.0))
        self.assertAlmostEqual(target_bearing, -math.degrees(math.atan2(80, 600)), 5)
        self.assertNotEqual(round(target_bearing, 3), 0.0)
        self.assertIn("const float targetBearing = atan2f(dy, dx)", CTRL)
        self.assertIn("guidedHeadingErrorDeg_ = HeadingFusion::shortestDelta", CTRL)

    def test_cross_track_sign_points_back_to_line(self):
        left_of_line = cross_track((0.0, 0.0), (1000.0, 0.0), (400.0, 100.0))
        right_of_line = cross_track((0.0, 0.0), (1000.0, 0.0), (400.0, -100.0))
        self.assertGreater(left_of_line, 0.0)
        self.assertLess(right_of_line, 0.0)
        self.assertLess(steering(-5.0, left_of_line), 0.0)
        self.assertGreater(steering(5.0, right_of_line), 0.0)
        self.assertIn("so its correction is subtracted", CTRL)

    def test_heading_correction_signs(self):
        self.assertGreater(steering(10.0, 0.0), 0.0)
        self.assertLess(steering(-10.0, 0.0), 0.0)
        self.assertIn("const int16_t rawLeft = guidedBaseSpeed_ - steering", CTRL)
        self.assertIn("const int16_t rawRight = guidedBaseSpeed_ + steering", CTRL)

    def test_steering_is_bounded_and_forward(self):
        limit = config_number("MAP_GUIDE_MAX_STEER_COMMAND")
        self.assertLessEqual(abs(steering(14.9, 1000.0)), limit)
        self.assertLessEqual(abs(steering(-14.9, -1000.0)), limit)
        self.assertIn("maximumSafeSteer", CTRL)
        self.assertIn("OBSTACLE_MIN_FORWARD_COMMAND", CTRL)
        self.assertIn("const int16_t rawLeft = guidedBaseSpeed_ - steering", CTRL)
        self.assertIn("const int16_t rawRight = guidedBaseSpeed_ + steering", CTRL)

    def test_map_wheel_absolute_cap_50(self):
        self.assertIn("MAP_REPLAY_SPEED_MAX = 50", MAP_TYPES)
        self.assertIn("const int16_t rawMaximum = max(rawLeft, rawRight)", CTRL)
        self.assertIn("const float wheelScale", CTRL)
        self.assertIn("MAP_REPLAY_SPEED_MAX", CTRL)
        for left, right in ((44.0, 56.0), (56.0, 44.0), (20.0, 26.0)):
            scale = min(1.0, 50.0 / max(left, right))
            self.assertLessEqual(left * scale, 50.0)
            self.assertLessEqual(right * scale, 50.0)

    def test_map_global_ramp_off(self):
        apply_start = CTRL.index("void RobotController::applyMotorCommand")
        apply_end = CTRL.index("void RobotController::stopImmediately", apply_start)
        apply = CTRL[apply_start:apply_end]
        self.assertIn("const bool replayGuided = motionOwner_ == MotionOwner::REPLAY", apply)
        self.assertIn("if (!replayGuided) updateRamp();", apply)
        self.assertIn("if (replayGuided) {", apply)
        self.assertIn("left = constrain(targetLeft_, 0, MAP_REPLAY_SPEED_MAX)", apply)

    def test_map_global_ramp_on_same_profile(self):
        self.assertIn("bool rampEnabled_ = false", CTRL_HEADER)
        self.assertIn("if (!rampEnabled_)", CTRL)
        self.assertIn("currentLeft_ = rampToward(currentLeft_, targetLeft_)", CTRL)
        self.assertIn("currentRight_ = rampToward(currentRight_, targetRight_)", CTRL)
        self.assertIn("if (!replayGuided) updateRamp();", CTRL)

    def test_robot_global_ramp_unchanged(self):
        ramp_start = CTRL.index("void RobotController::updateRamp()")
        ramp_end = CTRL.index("void RobotController::applyMotorCommand", ramp_start)
        ramp = CTRL[ramp_start:ramp_end]
        self.assertIn("if (targetLeft_ == 0 && targetRight_ == 0)", ramp)
        self.assertIn("if (!rampEnabled_)", ramp)
        self.assertIn("currentLeft_ = rampToward(currentLeft_, targetLeft_)", ramp)
        self.assertIn("currentRight_ = rampToward(currentRight_, targetRight_)", ramp)

    def test_near_target_slowdown_keeps_commissioned_minimum(self):
        slow = config_number("MAP_GUIDE_SLOW_DISTANCE_MM")
        minimum = config_number("MAP_GUIDE_MIN_SPEED")
        requested = 20.0
        remaining = 100.0
        ratio = max(0.0, min(1.0, remaining / slow))
        base = round(minimum + (requested - minimum) * ratio)
        self.assertLess(base, requested)
        self.assertGreaterEqual(base, minimum)
        self.assertIn("guidedRequestedSpeed_", CTRL)
        self.assertIn("MAP_GUIDE_MIN_SPEED", CTRL)

    def test_TEST_DECEL_250_EQUALS_CONFIG_SPEED(self):
        self.assertEqual(normal_decel_speed(300.0, 50.0), 50)
        self.assertEqual(normal_decel_speed(250.0, 50.0), 50)
        self.assertIn("slowNumerator", CTRL)
        self.assertIn("slowDenominator", CTRL)

    def test_TEST_DECEL_60_EQUALS_MIN_SPEED(self):
        self.assertEqual(normal_decel_speed(60.0, 50.0), 15)
        self.assertEqual(normal_decel_speed(60.0, 20.0), 15)
        self.assertIn("MAP_GUIDE_ARRIVAL_POSITION_TOLERANCE_MM", CTRL)

    def test_TEST_DECEL_MONOTONIC(self):
        values = [normal_decel_speed(distance, 50.0)
                  for distance in (250.0, 200.0, 150.0, 100.0, 60.0)]
        self.assertEqual(values, sorted(values, reverse=True))

    def test_TEST_DECEL_SPEED15_CONSTANT(self):
        for distance in (300.0, 250.0, 155.0, 100.0, 60.0):
            self.assertEqual(normal_decel_speed(distance, 15.0), 15)

    def test_TEST_SAFETY_STOP_IMMEDIATE_UNCHANGED(self):
        self.assertIn("void RobotController::stopImmediately", CTRL)
        self.assertIn("enterReplayHold(MapHoldReason::OBSTACLE, true)",
                      MAP)
        self.assertIn("finishAiDistance(AiDistanceResultCode::ENCODER_FAULT)",
                      CTRL)
        self.assertIn("finishAiDistance(AiDistanceResultCode::HEADING_LOST)",
                      CTRL)

    def test_position_arrival_and_moderate_heading_are_explicit(self):
        self.assertLessEqual(50.0, config_number(
            "MAP_GUIDE_ARRIVAL_POSITION_TOLERANCE_MM"
        ))
        self.assertEqual(config_number("MAP_GUIDE_ARRIVAL_HEADING_TOLERANCE_DEG"), 6.0)
        self.assertIn("remaining <= MAP_GUIDE_ARRIVAL_POSITION_TOLERANCE_MM", CTRL)
        self.assertIn("MAP_GUIDE_ARRIVAL_HEADING_TOLERANCE_DEG", CTRL)
        self.assertIn(
            "fabsf(arrivalHeadingError) > MAP_GUIDE_ARRIVAL_HEADING_TOLERANCE_DEG",
            CTRL,
        )
        self.assertIn("MAP,GUIDE,ARRIVAL", CTRL)
        self.assertNotIn("startAiTurnAbsolute", CTRL[CTRL.find("void RobotController::updateAiGuidedWaypoint"):])

    def test_realign_preserves_same_waypoint(self):
        self.assertIn("MAP_GUIDE_REALIGN_THRESHOLD_DEG", CTRL)
        self.assertIn("AiDistanceResultCode::REALIGN_REQUIRED", CTRL)
        self.assertIn("result.code == AiDistanceResultCode::REALIGN_REQUIRED", MAP)
        self.assertIn("ReplayRealignReason", MAP_HEADER)
        self.assertIn("replayRealignReason_", MAP)
        self.assertIn("logGuideRealign(replayRealignReason_", MAP)
        self.assertIn("startReplayGuidedWaypoint", MAP)
        realign_start = MAP.find(
            "if (result.code == AiDistanceResultCode::REALIGN_REQUIRED)"
        )
        realign_block = MAP[realign_start:MAP.find(
            "} else if (result.code == AiDistanceResultCode::DONE",
            realign_start,
        )]
        self.assertNotIn("advanceReplayAfterTarget", realign_block)

    def test_arrival_side_offset_uses_incoming_bearing(self):
        incoming = bearing((0.0, 0.0), (1000.0, 0.0))
        current_to_target = bearing((1000.0, 40.0), (1000.0, 0.0))
        arrival_error = shortest_delta(incoming, 20.0)
        self.assertAlmostEqual(incoming, 0.0, places=4)
        self.assertAlmostEqual(current_to_target, -90.0, places=4)
        self.assertAlmostEqual(arrival_error, -20.0, places=4)
        self.assertEqual(arrival_action(40.0, incoming, 20.0), "ARRIVAL_REALIGN")
        self.assertIn("replayIncomingBearing(replayCurrentIndex_, replayTargetIndex_)", MAP)
        self.assertIn("desiredBearing = incomingBearing", MAP)
        self.assertIn("ReplayRealignReason::ARRIVAL", MAP)
        self.assertIn("AiTurnProfile::MAP_COARSE", MAP)

    def test_arrival_40mm_4deg_advances_exactly_once(self):
        self.assertEqual(arrival_action(40.0, 0.0, 4.0), "ADVANCE")
        self.assertIn(
            "fabsf(arrivalHeadingError) <=\n        MAP_GUIDE_ARRIVAL_HEADING_TOLERANCE_DEG",
            MAP,
        )
        self.assertIn("advanceReplayAfterTarget();", MAP)

    def test_arrival_40mm_10deg_requires_arrival_realign(self):
        self.assertEqual(arrival_action(40.0, 0.0, 10.0), "ARRIVAL_REALIGN")
        self.assertIn("replayRealignReason_ = actionReason", MAP)
        self.assertIn("logGuideRealign(actionReason, replayTargetIndex_", MAP)

    def test_arrival_40mm_20deg_does_not_use_current_target_bearing(self):
        self.assertEqual(arrival_action(40.0, 0.0, 20.0), "ARRIVAL_REALIGN")
        self.assertNotIn(
            "replayRealignReason_ = ReplayRealignReason::PATH;\n      logGuideRealign",
            MAP,
        )
        self.assertIn("desiredHeadingError = arrivalHeadingError", MAP)
        self.assertIn("logGuideRealign(replayRealignReason_, to, targetDistance, desiredBearing", MAP)

    def test_arrival_turn_complete_advances_once(self):
        self.assertEqual(realign_action("ARRIVAL", 42.0, 0.0, 0.0, 4.0), "ADVANCE")
        self.assertIn("logGuideRealignDone(actionReason, replayTargetIndex_", MAP)
        self.assertIn('"ADVANCE"', MAP)
        self.assertIn("replayRealignReason_ = ReplayRealignReason::NONE", MAP)

    def test_arrival_turn_drift_resumes_guided_same_target(self):
        self.assertEqual(realign_action("ARRIVAL", 75.0, 0.0, 0.0, 4.0), "GUIDED")
        self.assertIn("if (!inArrivalZone)", MAP)
        self.assertIn('"GUIDED"', MAP)
        self.assertIn("startReplayGuidedWaypoint", MAP)
        self.assertIn("replayTarget_.xMm, replayTarget_.yMm", MAP)

    def test_path_realign_uses_live_target_bearing(self):
        incoming = bearing((0.0, 0.0), (1000.0, 0.0))
        target_bearing = bearing((500.0, 100.0), (1000.0, 0.0))
        self.assertEqual(
            realign_action("PATH", math.hypot(500.0, 100.0), incoming,
                           target_bearing, 20.0),
            "PATH_COARSE_TURN",
        )
        self.assertIn("replayRealignReason_ = inArrivalZone ? ReplayRealignReason::ARRIVAL", MAP)
        self.assertIn("shortestDeltaDeg(targetBearing, current.headingDeg)", MAP)
        self.assertIn("PATH realign uses the live current-pose-to-target bearing", MAP)

    def test_arrival_6deg_boundary_is_exact(self):
        self.assertEqual(arrival_action(50.0, 0.0, 6.0), "ADVANCE")
        self.assertEqual(arrival_action(50.0, 0.0, 6.5), "ARRIVAL_REALIGN")
        self.assertIn(
            "fabsf(arrivalHeadingError) <=\n               MAP_GUIDE_ARRIVAL_HEADING_TOLERANCE_DEG",
            MAP,
        )

    def test_arbitrary_incoming_angles_are_not_snapped(self):
        for angle in (60.0, 70.0, 130.0, -65.0, 170.0, -170.0):
            target = (math.cos(math.radians(angle)), math.sin(math.radians(angle)))
            self.assertAlmostEqual(bearing((0.0, 0.0), target), angle, places=4)
        incoming_source = MAP[MAP.find("float MapController::replayIncomingBearing"):
                              MAP.find("bool MapController::startNextReplaySegment")]
        self.assertIn("atan2f(target.yMm - segmentStart.yMm", incoming_source)
        self.assertNotIn("90.0f", incoming_source)
        self.assertNotIn("headingDeg", incoming_source)

    def test_hold_and_cancel_during_arrival_realign_keep_safety_semantics(self):
        hold_start = MAP.find("void MapController::enterReplayHold")
        hold_end = MAP.find("void MapController::abortReplay", hold_start)
        hold_block = MAP[hold_start:hold_end]
        self.assertIn("stopImmediately", hold_block)
        self.assertNotIn("clearReplayResumeContext", hold_block)
        self.assertNotIn("replayRealignReason_ =", hold_block)
        cancel_start = MAP.find("void MapController::cancelReplay")
        cancel_end = MAP.find("void MapController::clearReplayResumeContext", cancel_start)
        cancel_block = MAP[cancel_start:cancel_end]
        self.assertIn("stopImmediately", cancel_block)
        self.assertIn("clearReplayResumeContext", cancel_block)
        clear_start = MAP.find("void MapController::clearReplayResumeContext")
        clear_end = MAP.find("bool MapController::canResumeReplay", clear_start)
        self.assertIn("replayRealignReason_ = ReplayRealignReason::NONE;", MAP[clear_start:clear_end])

    def test_stale_turn_result_after_realign_is_dropped(self):
        turn_start = MAP.find("bool MapController::consumeReplayTurnResult")
        turn_done = MAP.find("const uint16_t from", turn_start)
        validation = MAP[turn_start:turn_done]
        self.assertIn("result.motionGeneration != replaySegmentGeneration_", validation)
        self.assertIn("MAP,SEGMENT_DROP_STALE", validation)
        self.assertNotIn("replayRealignReason_ =", validation)

    def test_obstacle_during_arrival_realign_preserves_hold(self):
        turn_start = MAP.find("bool MapController::consumeReplayTurnResult")
        turn_end = MAP.find("bool MapController::consumeReplayDistanceResult", turn_start)
        turn_block = MAP[turn_start:turn_end]
        self.assertIn("AiTurnResultCode::OBSTACLE", turn_block)
        self.assertIn("enterReplayHold(MapHoldReason::OBSTACLE, true)", turn_block)
        self.assertNotIn("advanceReplayAfterTarget", turn_block)

    def test_done_telemetry_uses_live_bearing(self):
        done_start = MAP.find(
            "} else if (result.code == AiDistanceResultCode::DONE)"
        )
        obstacle_start = MAP.find(
            "} else if (result.code == AiDistanceResultCode::OBSTACLE)",
            done_start,
        )
        done_block = MAP[done_start:obstacle_start]
        self.assertIn("const float arrivalBearing", done_block)
        self.assertIn("replayIncomingBearing(replayCurrentIndex_, replayTargetIndex_)", done_block)
        self.assertIn("logGuideDone(to, positionError, arrivalBearing, current.headingDeg", done_block)
        self.assertNotIn("replayGuideBearingDeg_", done_block)
        self.assertIn("ARRIVAL_BEARING", MAP)
        self.assertIn("ARRIVAL_HDG_ERR", MAP)

    def test_mcp_precise_turn_contract_is_unchanged(self):
        self.assertEqual(config_number("TURN_TOLERANCE_DEG"), 0.5)
        self.assertIn("AiTurnProfile::PRECISE", CTRL_HEADER)
        self.assertIn("profile == AiTurnProfile::MAP_COARSE", CTRL)
        self.assertIn(": TURN_TOLERANCE_DEG", CTRL)
        self.assertIn("startTurnSession(MotionOwner::MCP", CTRL)
        self.assertNotIn("AiTurnProfile::MAP_COARSE", CTRL[CTRL.find("bool RobotController::startAiTurnRelative"):CTRL.find("void RobotController::finishAiTurn")])

    def test_hold_resume_cancel_and_safety_fences_remain(self):
        for token in (
            "cancelAiMotionForManual",
            "finishAiDistance(AiDistanceResultCode::CANCELLED)",
            "MotionOwner::REPLAY",
            "motionGeneration",
            "obstacleLimited_",
            "motors_.brake()",
            "headingAvailable()",
            "odometry_.healthy()",
            "stopImmediately",
        ):
            self.assertIn(token, CTRL)
        for token in (
            "MapHoldReason::OBSTACLE",
            "startNextReplaySegment",
            "replaySegmentGeneration_",
            "clearReplayResumeContext",
        ):
            self.assertIn(token, MAP)

    def test_no_dynamic_allocation_in_guided_sources(self):
        allocation_call = r"\b(?:malloc|calloc|realloc|free)\s*\(|\bnew\s+[A-Za-z_]"
        self.assertNotRegex(without_comments(CTRL), allocation_call)
        self.assertNotRegex(without_comments(MAP), allocation_call)

    def test_guidance_telemetry_is_rate_limited(self):
        self.assertIn("MAP_GUIDE_TELEMETRY_MS", CONFIG)
        self.assertIn("lastGuidanceLogMs_", MAP_HEADER)
        self.assertIn("now - lastGuidanceLogMs_ >= MAP_GUIDE_TELEMETRY_MS", MAP)
        self.assertIn("MAP,GUIDE,UPDATE", MAP)

    def test_generic_waypoint_and_direction_paths_are_retained(self):
        self.assertIn("replayTargetIndex_", MAP)
        self.assertIn("replayDirection_", MAP)
        self.assertIn("replayLapCounter_", MAP)
        self.assertIn("routeType_ == MapRouteType::CLOSED", MAP)
        self.assertIn("routeMode_ == MapReplayMode::RETURN", MAP)
        self.assertIn("MapReplayMode::PING_PONG", MAP)
        self.assertIn("replayTarget_.xMm, replayTarget_.yMm", MAP)


if __name__ == "__main__":
    unittest.main()
