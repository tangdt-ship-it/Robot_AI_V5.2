"""Host regression contracts for PS2 takeover of AI Return-to-P0."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
CTRL = (ROOT / "firmware/stm32/src/control/robot_controller.cpp").read_text(
    encoding="utf-8"
)
CTRL_HEADER = (ROOT / "firmware/stm32/include/control/robot_controller.h").read_text(
    encoding="utf-8"
)
MAP = (ROOT / "firmware/stm32/src/map/map_controller.cpp").read_text(
    encoding="utf-8"
)
MAIN = (ROOT / "firmware/stm32/src/main.cpp").read_text(encoding="utf-8")


def block(source: str, start: str, end: str) -> str:
    begin = source.index(start)
    finish = source.index(end, begin)
    return source[begin:finish]


def consume_model(primitive: str, *, generation_matches: bool = True,
                  ps2_cancel: bool = False, owner: str = "NONE",
                  ps2_motion_active: bool = False, r3: bool = False,
                  return_source: str = "AI_VOICE") -> dict[str, object]:
    """Model only the result arbitration and its motor-stop side effect."""
    if not generation_matches:
        return {"event": "DROP_STALE", "reason": "", "stops": 0,
                "owner": owner, "return_active": True}
    if (ps2_cancel and return_source == "AI_VOICE" and owner == "PS2" and
            ps2_motion_active and not r3):
        return {"event": "PS2_TAKEOVER", "reason": "PS2_TAKEOVER",
                "stops": 0, "owner": owner, "return_active": False}
    reason = "MOVE_ERROR" if primitive == "MOVE" else "TURN_ERROR"
    return {"event": "ABORT", "reason": reason, "stops": 1,
            "owner": "NONE", "return_active": False}


class ReturnP0Ps2TakeoverTests(unittest.TestCase):
    def test_E1_move_fresh_ps2_takes_over_without_second_stop(self) -> None:
        result = consume_model("MOVE", ps2_cancel=True, owner="PS2",
                               ps2_motion_active=True)
        self.assertEqual(result["event"], "PS2_TAKEOVER")
        self.assertEqual(result["reason"], "PS2_TAKEOVER")
        self.assertEqual(result["stops"], 0)
        self.assertEqual(result["owner"], "PS2")
        consumer = block(MAP, "bool MapController::consumeReturnDistanceResult(",
                         "MapController::Pose MapController::routePointWorld(")
        self.assertIn("result.cancelledByPs2Motion", consumer)
        self.assertIn("terminateReturnToP0ForPs2Takeover()", consumer)
        self.assertNotIn('abortReturnToP0("MOVE_ERROR")', consumer.split(
            "terminateReturnToP0ForPs2Takeover();", 1)[0])

    def test_E2_turn_fresh_ps2_takes_over_without_second_stop(self) -> None:
        result = consume_model("TURN", ps2_cancel=True, owner="PS2",
                               ps2_motion_active=True)
        self.assertEqual(result["event"], "PS2_TAKEOVER")
        self.assertEqual(result["reason"], "PS2_TAKEOVER")
        self.assertEqual(result["stops"], 0)
        self.assertEqual(result["owner"], "PS2")
        consumer = block(MAP, "bool MapController::consumeReturnTurnResult(",
                         "bool MapController::consumeReturnDistanceResult(")
        self.assertIn("result.cancelledByPs2Motion", consumer)
        self.assertIn("terminateReturnToP0ForPs2Takeover()", consumer)
        self.assertNotIn('abortReturnToP0("TURN_ERROR")', consumer.split(
            "terminateReturnToP0ForPs2Takeover();", 1)[0])

    def test_E3_cancelled_without_ps2_is_not_takeover(self) -> None:
        result = consume_model("MOVE", ps2_cancel=False)
        self.assertEqual(result["event"], "ABORT")
        self.assertEqual(result["reason"], "MOVE_ERROR")
        self.assertEqual(result["stops"], 1)
        self.assertEqual(result["owner"], "NONE")

    def test_E4_R3_remains_fail_closed(self) -> None:
        result = consume_model("MOVE", ps2_cancel=True, owner="PS2",
                               ps2_motion_active=True, r3=True)
        self.assertEqual(result["event"], "ABORT")
        self.assertEqual(result["stops"], 1)
        update = block(MAP, "void MapController::update()",
                       "void MapController::handleEvent(")
        self.assertIn('abortReturnToP0(ps2_.state().r3 ? "R3"', update)
        abort = block(MAP, "void MapController::abortReturnToP0(",
                      "void MapController::completeReturnToP0(")
        self.assertIn("robot_.stopImmediately(true)", abort)

    def test_E5_ps2_timeout_does_not_cancel_ai(self) -> None:
        update_fast = block(CTRL, "void RobotController::updateFast()",
                            "void RobotController::updateControl()")
        self.assertIn("const bool ps2MotionFresh = ps2FrameFresh && ps2Usable", update_fast)
        self.assertIn("if (ps2MotionFresh && aiMotionMode_ != AiMotionMode::NONE)",
                      update_fast)
        self.assertIn("if (!ps2_.state().frameFresh)", update_fast)
        self.assertNotIn("cancelAiMotionForManual();", update_fast.split(
            "if (!ps2_.state().frameFresh)", 1)[1])

    def test_E6_neutral_ps2_does_not_cancel_ai(self) -> None:
        update_fast = block(CTRL, "void RobotController::updateFast()",
                            "void RobotController::updateControl()")
        self.assertIn("!ps2_.motionCommandActive()", update_fast)
        self.assertIn("if (ps2MotionFresh && aiMotionMode_ != AiMotionMode::NONE)",
                      update_fast)

    def test_E7_stale_cancelled_generation_is_dropped_before_takeover(self) -> None:
        result = consume_model("MOVE", generation_matches=False,
                               ps2_cancel=True, owner="PS2",
                               ps2_motion_active=True)
        self.assertEqual(result["event"], "DROP_STALE")
        self.assertEqual(result["owner"], "PS2")
        consumer = block(MAP, "bool MapController::consumeReturnDistanceResult(",
                         "MapController::Pose MapController::routePointWorld(")
        self.assertLess(consumer.index("result.motionGeneration != returnP0SegmentGeneration_"),
                        consumer.index("terminateReturnToP0ForPs2Takeover()"))
        cleanup = block(MAP, "void MapController::terminateReturnToP0ForPs2Takeover(",
                        "void MapController::completeReturnToP0(")
        self.assertIn("nextReplayGeneration()", cleanup)
        self.assertIn("replaySegmentGeneration_ = 0U", cleanup)
        self.assertIn("replayOperation_ = MapReplayOperation::NONE", cleanup)
        self.assertIn("replayActive_ = false", cleanup)
        self.assertIn('replayReason_ = "PS2_TAKEOVER"', cleanup)
        self.assertIn("MAP,RETURN_P0,ABORT,REASON=PS2_TAKEOVER", cleanup)
        self.assertNotIn("stopImmediately", cleanup)

    def test_takeover_cause_is_latched_only_by_actual_manual_cancellation(self) -> None:
        self.assertIn("bool cancelledByPs2Motion = false", CTRL_HEADER)
        cancel = block(CTRL, "void RobotController::cancelAiMotionForManual()",
                       "bool RobotController::takeAiDistanceResult")
        self.assertIn("AiDistanceResultCode::CANCELLED, true", cancel)
        self.assertIn("AiTurnResultCode::CANCELLED, true", cancel)
        distance_finish = block(CTRL, "void RobotController::finishAiDistance(",
                                "void RobotController::cancelAiMotionForManual()")
        turn_finish = block(CTRL, "void RobotController::finishAiTurn(",
                            "bool RobotController::takeAiTurnResult")
        self.assertIn("cancelledByPs2Motion", distance_finish)
        self.assertIn("cancelledByPs2Motion", turn_finish)

    def test_takeover_gate_is_limited_to_voice_return_and_live_ps2_owner(self) -> None:
        for consumer in (
            block(MAP, "bool MapController::consumeReturnTurnResult(",
                  "bool MapController::consumeReturnDistanceResult("),
            block(MAP, "bool MapController::consumeReturnDistanceResult(",
                  "MapController::Pose MapController::routePointWorld("),
        ):
            self.assertIn("returnP0Source_ == ReturnP0Source::AI_VOICE", consumer)
            self.assertIn("robot_.motionOwner() == MotionOwner::PS2", consumer)
            self.assertIn("ps2_.motionCommandActive() && !ps2_.state().r3", consumer)

    def test_control_order_applies_manual_command_before_map_result_cleanup(self) -> None:
        update_fast = block(CTRL, "void RobotController::updateFast()",
                            "void RobotController::emitDiagnostics(")
        cancel_index = update_fast.index("cancelAiMotionForManual();")
        manual_index = update_fast.index("handleManualControl();", cancel_index)
        apply_index = update_fast.index("applyMotorCommand();", manual_index)
        self.assertLess(cancel_index, manual_index)
        self.assertLess(manual_index, apply_index)

        update_fast_index = MAIN.index("robot.updateFast();")
        turn_dispatch_index = MAIN.index("robot.takeAiTurnResult(", update_fast_index)
        distance_dispatch_index = MAIN.index("robot.takeAiDistanceResult(", update_fast_index)
        map_update_index = MAIN.index("mapController.update();", update_fast_index)
        self.assertLess(update_fast_index, turn_dispatch_index)
        self.assertLess(turn_dispatch_index, map_update_index)
        self.assertLess(distance_dispatch_index, map_update_index)


if __name__ == "__main__":
    unittest.main(verbosity=2)
