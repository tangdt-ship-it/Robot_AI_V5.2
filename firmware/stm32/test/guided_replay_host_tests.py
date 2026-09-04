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


def without_comments(text):
    text = re.sub(r"//[^\n]*", "", text)
    return re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)


def sequencer_action(distance_mm, heading_error_deg, realign_pending):
    """Return the required next action for the near-target state machine."""
    position_tolerance = config_number("MAP_GUIDE_ARRIVAL_POSITION_TOLERANCE_MM")
    preturn_tolerance = config_number("MAP_REPLAY_PRETURN_TOLERANCE_DEG")
    if not realign_pending and distance_mm <= position_tolerance:
        return "ADVANCE"
    if realign_pending:
        if abs(heading_error_deg) > preturn_tolerance:
            return "COARSE_TURN"
        return "ADVANCE" if distance_mm <= position_tolerance else "GUIDED"
    return "COARSE_TURN" if abs(heading_error_deg) > preturn_tolerance else "GUIDED"


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
        self.assertIn("motionOwner_ = MotionOwner::REPLAY", CTRL)
        self.assertNotIn("startReplayGuidedWaypoint", MAIN)
        self.assertNotIn("GUIDED_WAYPOINT", (ROOT / "src" / "communication" /
                                               "robot_link_server.cpp").read_text(
                                                   encoding="utf-8"
                                               ))

    def test_preturn_skip_at_three_degrees(self):
        self.assertLessEqual(3.0, config_number("MAP_REPLAY_PRETURN_TOLERANCE_DEG"))
        self.assertIn("const bool skipPreturn", MAP)
        self.assertIn("startReplayGuidedWaypoint", MAP)
        self.assertIn('logGuidePreturn(turnDelta, "SKIP")', MAP)

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
        self.assertIn("targetLeft_ = constrain(guidedBaseSpeed_ - steering", CTRL)
        self.assertIn("targetRight_ = constrain(guidedBaseSpeed_ + steering", CTRL)

    def test_steering_is_bounded_and_forward(self):
        limit = config_number("MAP_GUIDE_MAX_STEER_COMMAND")
        self.assertLessEqual(abs(steering(14.9, 1000.0)), limit)
        self.assertLessEqual(abs(steering(-14.9, -1000.0)), limit)
        self.assertIn("maximumSafeSteer", CTRL)
        self.assertIn("OBSTACLE_MIN_FORWARD_COMMAND", CTRL)
        self.assertIn("constrain(guidedBaseSpeed_ - steering, 0, 255)", CTRL)
        self.assertIn("constrain(guidedBaseSpeed_ + steering, 0, 255)", CTRL)

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

    def test_position_arrival_and_moderate_heading_are_explicit(self):
        self.assertLessEqual(50.0, config_number(
            "MAP_GUIDE_ARRIVAL_POSITION_TOLERANCE_MM"
        ))
        self.assertLessEqual(4.5, config_number(
            "MAP_GUIDE_ARRIVAL_HEADING_TOLERANCE_DEG"
        ))
        self.assertIn("remaining <= MAP_GUIDE_ARRIVAL_POSITION_TOLERANCE_MM", CTRL)
        self.assertIn("MAP_GUIDE_ARRIVAL_HEADING_TOLERANCE_DEG", CTRL)
        self.assertIn("MAP,GUIDE,ARRIVAL", CTRL)
        self.assertNotIn("startAiTurnAbsolute", CTRL[CTRL.find("void RobotController::updateAiGuidedWaypoint"):])

    def test_realign_preserves_same_waypoint(self):
        self.assertIn("MAP_GUIDE_REALIGN_THRESHOLD_DEG", CTRL)
        self.assertIn("AiDistanceResultCode::REALIGN_REQUIRED", CTRL)
        self.assertIn("result.code == AiDistanceResultCode::REALIGN_REQUIRED", MAP)
        self.assertIn("logGuideRealign(to, headingError)", MAP)
        self.assertIn("startReplayGuidedWaypoint", MAP)
        # REALIGN clears only the current operation; the sequencer's target
        # index remains unchanged until DONE calls advanceReplayAfterTarget.
        realign_start = MAP.find(
            "if (result.code == AiDistanceResultCode::REALIGN_REQUIRED)"
        )
        realign_block = MAP[realign_start:MAP.find(
            "} else if (result.code == AiDistanceResultCode::DONE",
            realign_start,
        )]
        self.assertNotIn("advanceReplayAfterTarget", realign_block)

    def test_40mm_20deg_realign_forces_coarse_turn_without_advance(self):
        self.assertEqual(sequencer_action(40.0, 20.0, True), "COARSE_TURN")
        self.assertIn("bool replayRealignPending_ = false;", MAP_HEADER)
        self.assertIn("const bool realignPending = replayRealignPending_;", MAP)
        self.assertIn(
            "if (!realignPending && targetDistance <= kWaypointToleranceMm)",
            MAP,
        )
        self.assertIn("replayRealignPending_ = true;", MAP)
        pending_cycle = MAP[MAP.find("const bool realignPending"):
                            MAP.find("void MapController::logSegmentStart")]
        self.assertIn("AiTurnProfile::MAP_COARSE", pending_cycle)
        self.assertNotIn(
            "advanceReplayAfterTarget();\n      return true;",
            pending_cycle[pending_cycle.find("if (realignPending)"):],
        )

    def test_coarse_done_at_40mm_5deg_advances_once(self):
        self.assertEqual(sequencer_action(40.0, 5.0, True), "ADVANCE")
        self.assertIn("replayRealignPending_ = false;", MAP)
        self.assertIn("MAP,GUIDE,REALIGN,COARSE_DONE=1", MAP)
        self.assertIn("advanceReplayAfterTarget();", MAP)

    def test_realign_at_100mm_resumes_guided_same_target(self):
        self.assertEqual(sequencer_action(100.0, 20.0, True), "COARSE_TURN")
        self.assertEqual(sequencer_action(100.0, 5.0, True), "GUIDED")
        self.assertIn("startReplayGuidedWaypoint", MAP)
        self.assertIn("replayTarget_.xMm, replayTarget_.yMm", MAP)

    def test_stale_result_after_realign_is_dropped(self):
        self.assertIn("result.motionGeneration != replaySegmentGeneration_", MAP)
        self.assertIn("MAP,SEGMENT_DROP_STALE", MAP)
        self.assertIn("replayRealignPending_ = true;", MAP)

    def test_hold_and_cancel_during_realign_keep_safety_semantics(self):
        hold_start = MAP.find("void MapController::enterReplayHold")
        hold_end = MAP.find("void MapController::abortReplay", hold_start)
        hold_block = MAP[hold_start:hold_end]
        self.assertIn("stopImmediately", hold_block)
        self.assertNotIn("clearReplayResumeContext", hold_block)
        cancel_start = MAP.find("void MapController::cancelReplay")
        cancel_end = MAP.find("void MapController::clearReplayResumeContext", cancel_start)
        cancel_block = MAP[cancel_start:cancel_end]
        self.assertIn("stopImmediately", cancel_block)
        self.assertIn("clearReplayResumeContext", cancel_block)
        clear_start = MAP.find("void MapController::clearReplayResumeContext")
        clear_end = MAP.find("bool MapController::canResumeReplay", clear_start)
        self.assertIn("replayRealignPending_ = false;", MAP[clear_start:clear_end])

    def test_done_telemetry_uses_live_bearing(self):
        done_start = MAP.find(
            "} else if (result.code == AiDistanceResultCode::DONE)"
        )
        obstacle_start = MAP.find(
            "} else if (result.code == AiDistanceResultCode::OBSTACLE)",
            done_start,
        )
        done_block = MAP[done_start:obstacle_start]
        self.assertIn("const float liveBearing", done_block)
        self.assertIn("atan2f(replayTarget_.yMm - current.yMm", done_block)
        self.assertNotIn("replayGuideBearingDeg_", done_block)

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
