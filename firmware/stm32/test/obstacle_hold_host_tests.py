"""Host tests for fail-closed MAP obstacle hold and safe resume."""

import re
import unittest
from pathlib import Path


STM32_ROOT = Path(__file__).resolve().parents[1]
CONFIG = (STM32_ROOT / "include" / "robot_config.h").read_text(encoding="utf-8")
MAP = (STM32_ROOT / "src" / "map" / "map_controller.cpp").read_text(encoding="utf-8")
MAP_HEADER = (STM32_ROOT / "include" / "map" / "map_controller.h").read_text(encoding="utf-8")
MAP_TYPES = (STM32_ROOT / "include" / "map" / "map_types.h").read_text(encoding="utf-8")
CTRL = (STM32_ROOT / "src" / "control" / "robot_controller.cpp").read_text(encoding="utf-8")


def config_number(name):
    match = re.search(r"\b" + re.escape(name) + r"\s*=\s*(\d+)", CONFIG)
    if not match:
        raise AssertionError(f"constant {name} not found")
    return int(match.group(1))


class ObstacleHoldHostTests(unittest.TestCase):
    def test_stable_clear_window_is_bounded_and_longer_than_one_sensor_sample(self):
        stable_ms = config_number("OBSTACLE_CLEAR_STABLE_MS")
        sample_ms = config_number("ULTRASONIC_SAMPLE_PERIOD_MS")
        self.assertGreaterEqual(stable_ms, 300)
        self.assertLessEqual(stable_ms, 500)
        self.assertGreater(stable_ms, sample_ms)

    def test_obstacle_hold_is_a_map_state_not_a_detour(self):
        self.assertIn("OBSTACLE = 2U", MAP_TYPES)
        self.assertIn("MapHoldReason", MAP_HEADER)
        self.assertIn("enterReplayHold(MapHoldReason::OBSTACLE, true)", MAP)
        self.assertNotIn("DETOUR", MAP.upper())
        self.assertNotIn("SIDESTEP", MAP.upper())
        self.assertNotIn("avoidance", MAP.lower())

    def test_hold_preserves_route_and_waypoint(self):
        hold_start = MAP.index("void MapController::enterReplayHold")
        hold_end = MAP.index("void MapController::abortReplay", hold_start)
        hold = MAP[hold_start:hold_end]
        self.assertIn("replayTargetIndex_", hold)
        self.assertIn("replayCurrentIndex_", MAP_HEADER)
        self.assertIn("routeMode_", MAP_HEADER)
        self.assertIn("replayLapCounter_", MAP_HEADER)
        self.assertNotIn("clearReplayResumeContext", hold)

    def test_invalid_or_stale_sensor_cannot_resume_obstacle_hold(self):
        resume_start = MAP.index("bool MapController::canResumeReplay")
        resume_end = MAP.index("bool MapController::consumeReplayTurnResult", resume_start)
        resume = MAP[resume_start:resume_end]
        self.assertIn("ultrasonic_.isFresh()", resume)
        self.assertIn("ultrasonic_.healthy()", resume)
        self.assertIn("ObstacleZone::CLEAR", resume)
        self.assertIn('rejectReason = "OBSTACLE_NOT_CLEAR"', resume)
        self.assertIn("obstacleClearSinceMs_ = 0U", resume)
        self.assertNotIn("displayFar", resume)

    def test_one_clear_sample_does_not_resume_and_start_is_required(self):
        self.assertIn("serviceObstacleHold();", MAP)
        self.assertIn("if ((now - obstacleClearSinceMs_) < OBSTACLE_CLEAR_STABLE_MS)", MAP)
        start = MAP.index("void MapController::handleStart")
        resume = MAP.index("if (mode_ == MapControllerMode::REPLAY_HOLD)", start)
        self.assertIn("if (canResumeReplay(rejectReason))", MAP[resume:])
        self.assertNotIn("replayActive_ = true", MAP[MAP.index("void MapController::serviceObstacleHold"):resume])

    def test_resume_starts_a_new_operation_without_advancing_waypoint(self):
        start = MAP.index("if (canResumeReplay(rejectReason))")
        end = MAP.index("} else {", start)
        accepted = MAP[start:end]
        self.assertIn("replayOperation_ = MapReplayOperation::NONE", accepted)
        self.assertIn("replayTargetIndex_", accepted)
        self.assertNotIn("advanceReplayAfterTarget", accepted)
        self.assertIn("nextReplayGeneration()", accepted)

    def test_duplicate_obstacle_results_are_not_mcp_or_completion(self):
        self.assertIn("result.owner != MotionOwner::REPLAY", MAP)
        self.assertIn("enterReplayHold(MapHoldReason::OBSTACLE, true)", MAP)
        self.assertIn("MAP,SEGMENT_DROP_STALE", MAP)
        self.assertNotIn("reportDistanceResult", MAP)
        self.assertNotIn("reportTurnResult", MAP)
        self.assertNotIn('replayReason_ = "DONE"', MAP[MAP.index("enterReplayHold"):MAP.index("void MapController::abortReplay")])

    def test_robot_actuator_path_stops_forward_motion_fail_closed(self):
        self.assertIn("motionOwner_ != MotionOwner::REPLAY", CTRL)
        self.assertIn("ultrasonic_.isFresh() && ultrasonic_.healthy()", CTRL)
        self.assertIn("ultrasonic_.overallZone() != ObstacleZone::UNKNOWN", CTRL)
        self.assertIn("ultrasonic_.limitForwardCommand(forward)", CTRL)
        self.assertIn("if (limitedForward == 0", CTRL)
        self.assertIn("aiMotionMode_ = AiMotionMode::NONE", CTRL)
        self.assertIn("motors_.brake()", CTRL)
        self.assertIn("AiDistanceResultCode::OBSTACLE", CTRL)
        self.assertIn("AiTurnResultCode::OBSTACLE", CTRL)

    def test_map_turn_is_fail_closed_when_sensor_is_not_fully_valid(self):
        turn_start = CTRL.index("void RobotController::updateAiTurn")
        turn_end = CTRL.index("void RobotController::updateFast", turn_start)
        turn = CTRL[turn_start:turn_end]
        self.assertIn("const bool mapSensorClear", turn)
        self.assertIn("const bool oneSectorClear = mapTurnProfile", turn)
        self.assertIn("const bool recentClearWindow = !mapTurnProfile", turn)


if __name__ == "__main__":
    unittest.main(verbosity=2)
