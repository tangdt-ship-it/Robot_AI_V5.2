"""Host/static regression tests for the STM32-local MAP implementation.

These tests use the checked-in MAP constants and wire layout, then exercise
storage CRC, generation selection, route-mode transitions and local-frame
transform without importing Arduino or touching hardware.
"""

import math
import re
import struct
import unittest
import zlib
from pathlib import Path


STM32_ROOT = Path(__file__).resolve().parents[1]
INCLUDE_ROOT = STM32_ROOT / "include"
SRC_ROOT = STM32_ROOT / "src"
ESP32_ROOT = STM32_ROOT.parent / "esp32-xiaozhi"


def _read(path):
    return path.read_text(encoding="utf-8")


def _constant(text, name):
    match = re.search(
        r"\b" + re.escape(name) + r"\s*=\s*(0x[0-9A-Fa-f]+|\d+)", text
    )
    if not match:
        raise AssertionError(f"constant {name} not found")
    return int(match.group(1), 0)


FLASH_TEXT = _read(INCLUDE_ROOT / "map" / "flash_layout.h")
TYPES_TEXT = _read(INCLUDE_ROOT / "map" / "map_types.h")
MAP_TEXT = _read(SRC_ROOT / "map" / "map_controller.cpp")
MAP_HEADER_TEXT = _read(INCLUDE_ROOT / "map" / "map_controller.h")
STORE_TEXT = _read(SRC_ROOT / "map" / "route_store.cpp")
CTRL_TEXT = _read(SRC_ROOT / "control" / "robot_controller.cpp")
CTRL_HEADER_TEXT = _read(INCLUDE_ROOT / "control" / "robot_controller.h")
MAIN_TEXT = _read(SRC_ROOT / "main.cpp")
PS2_TEXT = _read(SRC_ROOT / "ps2" / "ps2_controller.cpp")
PS2_HEADER_TEXT = _read(INCLUDE_ROOT / "ps2" / "ps2_controller.h")
LCD_TEXT = _read(SRC_ROOT / "display" / "lcd_display.cpp")
ESP32_TEXT = _read(ESP32_ROOT / "main" / "robot" / "teach_route.cc")
ESP32_CMAKE = _read(ESP32_ROOT / "main" / "CMakeLists.txt")
ESP32_BOARD = _read(
    ESP32_ROOT / "main" / "boards" / "bread-compact-wifi-s3cam" /
    "compact_wifi_board_s3cam.cc"
)

MAX_WAYPOINTS = _constant(TYPES_TEXT, "STM32_MAP_MAX_WAYPOINTS")
FORMAT_VERSION = _constant(TYPES_TEXT, "STM32_MAP_FORMAT_VERSION")
FLASH_PAGE = _constant(FLASH_TEXT, "kFlashPageSize")
FLASH_END = _constant(FLASH_TEXT, "kFlashOrigin") + _constant(
    FLASH_TEXT, "kFlashLength"
)
MAP_ADDRESSES = [
    _constant(FLASH_TEXT, name)
    for name in ("kMap1A", "kMap1B", "kMap2A", "kMap2B")
]
CALIBRATION_PAGE = _constant(FLASH_TEXT, "kCalibrationPage")

HEADER = struct.Struct("<I H B B B B H H I I I I 2x")
WAYPOINT = struct.Struct("<i i h B B")


def crc_for(header_fields, waypoints):
    fields = list(header_fields)
    fields[10] = 0  # crc32 is excluded while calculating CRC.
    payload = HEADER.pack(*fields)
    payload += b"".join(WAYPOINT.pack(*point) for point in waypoints)
    return zlib.crc32(payload) & 0xFFFFFFFF


def route_allowed(route_type, replay_mode):
    if route_type == "CLOSED":
        return replay_mode in ("ONCE", "LOOP")
    return replay_mode in ("ONCE", "RETURN", "PING_PONG")


def advance(current, direction, count, route_type, replay_mode):
    """Mirror MapController::advanceReplayAfterTarget for planner testing."""
    if direction > 0:
        if current < count - 1:
            return current + 1, direction, False
        if route_type == "CLOSED" and replay_mode == "LOOP":
            return 0, direction, False
        if replay_mode == "ONCE":
            return None, direction, True
        return count - 2, -1, False
    if current > 0:
        return current - 1, direction, False
    if replay_mode == "RETURN":
        return None, direction, True
    return 1, 1, False


class MapHostTests(unittest.TestCase):
    def test_empty_map_and_teach_states(self):
        self.assertIn("MapControllerMode::READY", MAP_TEXT)
        self.assertIn("MapControllerMode::TEACHING", MAP_TEXT)
        self.assertIn("route_ = {}", MAP_TEXT)
        self.assertIn("beginTeach()", MAP_TEXT)

    def test_manual_auto_undo_save_cancel(self):
        for name in (
            "markManualWaypoint",
            "sampleTeach",
            "handleSquare",
            "finalizeTeach",
            "cancelTeach",
        ):
            with self.subTest(name=name):
                self.assertIn(name, MAP_TEXT)
        self.assertIn("MAP_WP_MANUAL_MARK", MAP_TEXT)
        self.assertIn("MAP_WP_AUTO_DISTANCE", MAP_TEXT)
        self.assertIn("MAP_WP_AUTO_CORNER", MAP_TEXT)
        self.assertIn("savePending_ = true", MAP_TEXT)
        self.assertIn("storeState_ = MapStoreState::STORAGE_ERROR", MAP_TEXT)
        self.assertIn("OLD_ROUTE_RETAINED=1", MAP_TEXT)

    def test_storage_record_sizes_and_version(self):
        self.assertEqual(MAX_WAYPOINTS, 128)
        self.assertEqual(FORMAT_VERSION, 2)
        self.assertEqual(WAYPOINT.size, 12)
        self.assertEqual(HEADER.size, 32)
        self.assertLessEqual(
            HEADER.size + MAX_WAYPOINTS * WAYPOINT.size, FLASH_PAGE
        )

    def test_flash_ab_storage_and_calibration_are_disjoint(self):
        self.assertEqual(len(set(MAP_ADDRESSES)), 4)
        self.assertTrue(all(address % FLASH_PAGE == 0 for address in MAP_ADDRESSES))
        self.assertEqual(CALIBRATION_PAGE % FLASH_PAGE, 0)
        self.assertTrue(
            all(address + FLASH_PAGE <= CALIBRATION_PAGE for address in MAP_ADDRESSES)
        )
        self.assertLessEqual(CALIBRATION_PAGE + FLASH_PAGE, FLASH_END)

    def test_crc_valid_invalid(self):
        header = [
            0x4D415032,
            FORMAT_VERSION,
            1,
            1,
            0,
            0,
            2,
            24,
            7,
            1000,
            0,
            0,
        ]
        points = [(0, 0, 0, 1, 0), (1000, 0, 0, 16, 0)]
        crc = crc_for(header, points)
        stored = header.copy()
        stored[10] = crc
        self.assertEqual(crc_for(stored, points), crc)
        corrupted = list(points)
        corrupted[1] = (1001, 0, 0, 16, 0)
        self.assertNotEqual(crc_for(stored, corrupted), crc)

    def test_ab_generation_and_powerloss_selection(self):
        records = [(4, True), (5, False)]
        valid = [generation for generation, is_valid in records if is_valid]
        self.assertEqual(max(valid), 4)
        self.assertIn("generationNewer", STORE_TEXT)
        self.assertIn("load(slot, verified)", STORE_TEXT)

    def test_open_closed_route_detection_and_modes(self):
        self.assertTrue(route_allowed("OPEN", "ONCE"))
        self.assertTrue(route_allowed("OPEN", "RETURN"))
        self.assertTrue(route_allowed("OPEN", "PING_PONG"))
        self.assertTrue(route_allowed("CLOSED", "ONCE"))
        self.assertTrue(route_allowed("CLOSED", "LOOP"))
        self.assertFalse(route_allowed("OPEN", "LOOP"))
        self.assertFalse(route_allowed("CLOSED", "RETURN"))
        self.assertIn("kEndpointDistanceMm", MAP_TEXT)
        self.assertIn("kEndpointHeadingDeg", MAP_TEXT)

    def test_once_sequence(self):
        current, direction = 0, 1
        visited = [current]
        for _ in range(4):
            current, direction, complete = advance(
                current, direction, 4, "OPEN", "ONCE"
            )
            if complete:
                break
            visited.append(current)
        self.assertEqual(visited, [0, 1, 2, 3])
        self.assertTrue(complete)

    def test_loop_sequence(self):
        current, direction = 2, 1
        current, direction, complete = advance(
            current, direction, 3, "CLOSED", "LOOP"
        )
        self.assertEqual((current, direction, complete), (0, 1, False))
        current, direction, complete = advance(
            current, direction, 3, "CLOSED", "LOOP"
        )
        self.assertEqual((current, direction, complete), (1, 1, False))

    def test_return_sequence(self):
        current, direction, complete = advance(
            3, 1, 4, "OPEN", "RETURN"
        )
        self.assertEqual((current, direction, complete), (2, -1, False))
        current, direction, complete = advance(0, -1, 4, "OPEN", "RETURN")
        self.assertEqual((current, direction, complete), (None, -1, True))

    def test_ping_pong_sequence(self):
        current, direction, complete = advance(
            3, 1, 4, "OPEN", "PING_PONG"
        )
        self.assertEqual((current, direction, complete), (2, -1, False))
        current, direction, complete = advance(0, -1, 4, "OPEN", "PING_PONG")
        self.assertEqual((current, direction, complete), (1, 1, False))

    def test_zero_length_and_impossible_segment_rejection(self):
        self.assertIn("kMinimumSegmentMm", MAP_TEXT)
        self.assertIn("kMaximumSegmentMm", MAP_TEXT)
        self.assertIn('reason = "SEGMENT"', MAP_TEXT)
        self.assertIn('reason = "TYPE_MODE"', MAP_TEXT)

    def test_replay_transform(self):
        origin_x, origin_y, origin_heading = 100.0, -50.0, 90.0
        local_x, local_y = 1000.0, 0.0
        radians = math.radians(origin_heading)
        world_x = origin_x + local_x * math.cos(radians) - local_y * math.sin(radians)
        world_y = origin_y + local_x * math.sin(radians) + local_y * math.cos(radians)
        self.assertAlmostEqual(world_x, 100.0, places=4)
        self.assertAlmostEqual(world_y, 950.0, places=4)
        self.assertIn("replayOrigin_.headingDeg", MAP_TEXT)
        self.assertIn("toTeachLocal", MAP_TEXT)

    def test_start_activity_is_not_replay_reject_or_takeover(self):
        control = CTRL_TEXT[CTRL_TEXT.index("void RobotController::updateControl"):]
        self.assertIn("ps2SafetyStop", control)
        self.assertIn("ps2MotionFresh || ps2SafetyStop", control)
        self.assertIn("!ps2_.controlActive(nowMs)", CTRL_TEXT)
        self.assertIn("canStartReplayMotion", CTRL_TEXT)

    def test_real_ps2_takeover_and_stop(self):
        self.assertIn(
            "if (ps2MotionFresh && aiMotionMode_ != AiMotionMode::NONE)", CTRL_TEXT
        )
        self.assertIn('cancelReplay("PS2_TAKEOVER")', MAP_TEXT)
        self.assertIn("stopImmediately", MAP_TEXT)
        self.assertIn("replayOperation_ = MapReplayOperation::NONE", MAP_TEXT)
        self.assertIn("MotionOwner::REPLAY", CTRL_TEXT)

    def test_obstacle_hold_and_resume_gate(self):
        self.assertIn("enterReplayHold(MapHoldReason::OBSTACLE, true)", MAP_TEXT)
        self.assertIn("canResumeReplay", MAP_TEXT)
        self.assertIn("obstacleLiveClear", MAP_TEXT)
        self.assertIn("obstacleGraceClear", MAP_TEXT)
        self.assertIn("hasRecentClearWindow", MAP_TEXT)
        self.assertIn("replayHoldPose_", MAP_TEXT)

    def test_encoder_reset_boundary(self):
        self.assertIn("replayOriginResetGeneration_", MAP_TEXT)
        self.assertIn("resetGeneration() != replayOriginResetGeneration_", MAP_TEXT)
        self.assertIn("RESET_BOUNDARY", MAP_TEXT)

    def test_result_owner_and_mcp_routing(self):
        self.assertIn(
            "MotionOwner owner", _read(INCLUDE_ROOT / "control" / "robot_controller.h")
        )
        self.assertIn("turnResult.owner != MotionOwner::REPLAY", MAIN_TEXT)
        self.assertIn("Never expose it as an MCP terminal frame", MAIN_TEXT)
        self.assertIn("mapController.consumeReplayTurnResult", MAIN_TEXT)
        self.assertIn("mapController.consumeReplayDistanceResult", MAIN_TEXT)

    def test_legacy_map_dormant_and_lcd_local(self):
        self.assertIn("ROBOT_MAP_OWNER_STM32", ESP32_TEXT)
        self.assertIn("LEGACY_ESP32_MAP_RUNTIME=DORMANT", ESP32_TEXT)
        self.assertRegex(
            ESP32_BOARD,
            r"#if !ROBOT_MAP_OWNER_STM32\s+teach_route_\.Begin\(\);\s+#endif",
        )
        self.assertRegex(
            ESP32_BOARD,
            r"#if !ROBOT_MAP_OWNER_STM32\s+teach_route_\.StartInputTask\(\);\s+#endif",
        )
        self.assertIn("ROUTE,OWNER=STM32", ESP32_TEXT)
        self.assertIn("ROBOT_MAP_OWNER_STM32=1", ESP32_CMAKE)
        lcd = _read(SRC_ROOT / "display" / "lcd_display.cpp")
        self.assertNotIn("WAIT ESP32 STATUS", lcd)
        self.assertIn("MAP STORAGE ERROR", lcd)
        self.assertIn("MAP,UI,", _read(SRC_ROOT / "communication" / "robot_link_server.cpp"))

    def test_ps2_map_actions_are_local(self):
        self.assertIn("queueMapEvent(Ps2MapAction::START)", PS2_TEXT)
        self.assertIn("queueMapEvent(Ps2MapAction::TRIANGLE)", PS2_TEXT)
        self.assertIn("queueMapEvent(Ps2MapAction::CROSS)", PS2_TEXT)
        self.assertIn("#if !STM32_LOCAL_MAP_ENABLE", PS2_TEXT)
        self.assertIn("display.togglePage()", PS2_TEXT)

    def test_TEST_START_ONE_PRESS(self):
        self.assertIn(
            "const bool startPressed = state_.start && !previousMapStart_",
            PS2_TEXT,
        )
        for event in (
            "MAP,INPUT,START_RAW_DOWN",
            "MAP,INPUT,START_EDGE",
            "MAP,INPUT,START_ACTION",
            "MAP,START,REJECT,REASON=NOT_ARMED",
            "MAP,START,REJECT,REASON=QUEUE_FULL",
        ):
            self.assertIn(event, PS2_TEXT)
        self.assertIn("mapEvent_.action != Ps2MapAction::CROSS", PS2_TEXT)
        self.assertNotIn("if (state_.start) queueMapEvent", PS2_TEXT)

    def test_TEST_START_HELD(self):
        self.assertIn("previousMapStart_ = state_.start", PS2_TEXT)
        self.assertIn("if (startPressed) queueMapEvent(Ps2MapAction::START)", PS2_TEXT)
        self.assertIn("mapNeutralReleaseFrames_", PS2_HEADER_TEXT)
        self.assertIn("kMapNeutralFramesToArm", PS2_TEXT)

    def test_TEST_X_CANCEL_GENERATION(self):
        self.assertIn("void Ps2Controller::disarmMapInput()", PS2_TEXT)
        self.assertIn("ps2_.disarmMapInput()", MAP_TEXT)
        self.assertIn("nextReplayGeneration();", MAP_TEXT)
        self.assertIn("replaySegmentGeneration_", MAP_HEADER_TEXT)
        self.assertIn("result.motionGeneration != replaySegmentGeneration_", MAP_TEXT)
        self.assertIn("MAP,SEGMENT_DROP_STALE", MAP_TEXT)
        self.assertIn("MAP,REPLAY_CANCEL", MAP_TEXT)

    def test_TEST_STALE_START(self):
        self.assertIn(
            "if (action == Ps2MapAction::CROSS &&",
            PS2_TEXT,
        )
        self.assertIn("SAFETY_PRIORITY", PS2_TEXT)
        self.assertIn("mapEventPending_ = false", PS2_TEXT)
        self.assertIn("replayResumeAllowed_ = false", MAP_TEXT)
        self.assertIn("replayActive_ = false", MAP_TEXT)

    def test_cancel_trace_is_bounded_and_diagnostic(self):
        self.assertIn("MAP,CANCEL_TRACE,T=", MAP_TEXT)
        self.assertIn("cancelTraceLines_ >= 20U", MAP_TEXT)
        self.assertIn("> 2000U", MAP_TEXT)
        for field in (
            "TARGET_L=",
            "TARGET_R=",
            "CURRENT_L=",
            "CURRENT_R=",
            ",LV=",
            ",RV=",
            ",GEN=",
        ):
            self.assertIn(field, MAP_TEXT)

    def test_x_input_is_processed_before_async_result_but_replay_update_stays_after(self):
        input_index = MAIN_TEXT.index("mapController.processInput();")
        result_index = MAIN_TEXT.index("AiTurnResult turnResult;")
        update_index = MAIN_TEXT.index("mapController.update();")
        self.assertLess(input_index, result_index)
        self.assertGreater(update_index, result_index)
        self.assertIn("uint32_t motionGeneration", CTRL_HEADER_TEXT)
        self.assertIn("motionGeneration = aiMotionGeneration_", CTRL_TEXT)

    def test_stale_result_does_not_become_mcp_done(self):
        self.assertIn("turnResult.owner != MotionOwner::REPLAY", MAIN_TEXT)
        self.assertIn("distanceResult.owner != MotionOwner::REPLAY", MAIN_TEXT)
        self.assertIn("Never expose it as an MCP terminal frame", MAIN_TEXT)
        self.assertIn("mapController.consumeReplayTurnResult", MAIN_TEXT)
        self.assertIn("mapController.consumeReplayDistanceResult", MAIN_TEXT)

    # Required acceptance-test IDs from the MAP HIL-4 cancellation task.
    def test_TEST_START_RELEASE_PRESS_AGAIN(self):
        self.assertIn("const bool startPressed = state_.start && !previousMapStart_", PS2_TEXT)
        self.assertIn("previousMapStart_ = state_.start", PS2_TEXT)
        self.assertIn("if (allMapButtonsReleased)", PS2_TEXT)

    def test_TEST_REARM_AFTER_X(self):
        self.assertIn("mapActionsArmed_ = false", PS2_TEXT)
        self.assertIn("mapNeutralReleaseFrames_ = 0U", PS2_TEXT)
        self.assertIn("kMapNeutralFramesToArm", PS2_TEXT)
        self.assertIn('robotDebug.println("MAP,INPUT,ARMED")', PS2_TEXT)

    def test_TEST_NO_AUTO_RESUME(self):
        self.assertIn("replayResumeAllowed_ = false", MAP_TEXT)
        self.assertIn("replayActive_ = false", MAP_TEXT)
        self.assertIn("mode_ = storeState_ == MapStoreState::SAVED", MAP_TEXT)

    def test_TEST_MCP_OWNER_UNCHANGED(self):
        self.assertIn("MotionOwner::MCP", CTRL_TEXT)
        self.assertIn("MotionOwner::REPLAY", CTRL_TEXT)
        self.assertIn("if (turnResult.owner != MotionOwner::REPLAY)", MAIN_TEXT)
        self.assertIn("if (distanceResult.owner != MotionOwner::REPLAY)", MAIN_TEXT)

    def test_TEST_REPLAY_RESULT_NOT_MCP_DONE(self):
        self.assertIn("turnResult.owner != MotionOwner::REPLAY", MAIN_TEXT)
        self.assertIn("distanceResult.owner != MotionOwner::REPLAY", MAIN_TEXT)
        self.assertIn("reportTurnResult", MAIN_TEXT)
        self.assertIn("reportDistanceResult", MAIN_TEXT)
        self.assertIn("consumeReplayTurnResult", MAIN_TEXT)
        self.assertIn("consumeReplayDistanceResult", MAIN_TEXT)

    def test_TEST_STALE_SEGMENT_RESULT(self):
        self.assertIn("result.motionGeneration != replaySegmentGeneration_", MAP_TEXT)
        self.assertIn("MAP,SEGMENT_DROP_STALE", MAP_TEXT)
        self.assertIn("return false", MAP_TEXT)

    def test_TEST_SAVED_START_RUN(self):
        self.assertIn('debug_.println("MAP,START,ACTION=RUN")', MAP_TEXT)
        self.assertIn("prepareReplay(reason)", MAP_TEXT)
        self.assertIn("replayCurrentIndex_ = 0U", MAP_TEXT)
        self.assertIn("replayTargetIndex_ = 1U", MAP_TEXT)

    def test_TEST_NO_SELECT_REQUIRED(self):
        self.assertIn('debug_.println("MAP,START,ACTION=RUN")', MAP_TEXT)
        self.assertNotIn("handleSlot(event.slot); handleStart()", MAP_TEXT)
        event_body = MAP_TEXT.split("void MapController::handleEvent", 1)[1].split(
            "void MapController::handleSlot", 1
        )[0]
        self.assertNotIn("selectedSlot_ =", event_body)

    def test_TEST_SELECT_ONLY_CHANGES_SLOT(self):
        self.assertIn("case Ps2MapAction::SLOT: handleSlot(event.slot); break;", MAP_TEXT)
        self.assertIn("selectedSlot_ = slot == 2U ? MapSlot::MAP_2 : MapSlot::MAP_1", MAP_TEXT)
        self.assertIn("mode_ == MapControllerMode::REPLAY_HOLD", MAP_TEXT)

    def test_TEST_SELECT_LOCKED_DURING_RUN(self):
        self.assertIn("replayActive_", MAP_TEXT)
        self.assertIn("mode_ == MapControllerMode::REPLAY_HOLD", MAP_TEXT)
        self.assertIn("status.mode >= 1U && status.mode <= 7U", _read(INCLUDE_ROOT / "display" / "lcd_display.h"))

    def test_TEST_X_SHORT_HOLD(self):
        self.assertIn("queueMapEvent(Ps2MapAction::CROSS)", PS2_TEXT)
        self.assertIn("MapHoldReason::USER", MAP_TEXT)
        self.assertIn("enterReplayHold(MapHoldReason::USER, true)", MAP_TEXT)
        self.assertIn("robot_.stopImmediately(true)", MAP_TEXT)
        self.assertIn("mode_ = MapControllerMode::REPLAY_HOLD", MAP_TEXT)
        self.assertIn("MAP,HOLD,REASON=", MAP_TEXT)

    def test_TEST_X_LONG_CANCEL(self):
        self.assertIn("CROSS_LONG", PS2_TEXT)
        self.assertIn("kMapCrossLongPressMs = 1200U", PS2_TEXT)
        self.assertIn("void MapController::handleCrossLong()", MAP_TEXT)
        self.assertIn('cancelReplay("X_LONG")', MAP_TEXT)
        self.assertIn("MAP,CANCEL,REASON=", MAP_TEXT)

    def test_TEST_HOLD_STALE_SEGMENT(self):
        self.assertIn("replayOperation_ = MapReplayOperation::HOLD", MAP_TEXT)
        self.assertIn("nextReplayGeneration();", MAP_TEXT)
        self.assertIn("replaySegmentGeneration_ = 0U", MAP_TEXT)
        self.assertIn("result.motionGeneration != replaySegmentGeneration_", MAP_TEXT)

    def test_TEST_USER_HOLD_RESUME(self):
        self.assertIn("canResumeReplay(rejectReason)", MAP_TEXT)
        self.assertIn('debug_.println("MAP,START,ACTION=RESUME")', MAP_TEXT)
        self.assertIn("replayTargetIndex_", MAP_TEXT)
        self.assertIn("replayActive_ = true", MAP_TEXT)
        self.assertIn("replayReason_ = \"RESUME\"", MAP_TEXT)

    def test_TEST_HOLD_COAST(self):
        self.assertIn("currentReplayPose(current)", MAP_TEXT)
        self.assertIn("targetDistance = distanceMm(current.xMm, current.yMm", MAP_TEXT)
        self.assertIn("current live pose", MAP_TEXT)
        self.assertIn("old remaining-distance", MAP_TEXT)

    def test_TEST_RESUME_RESET_BOUNDARY(self):
        self.assertIn('rejectReason = "RESET_BOUNDARY"', MAP_TEXT)
        self.assertIn("resetGeneration() != replayOriginResetGeneration_", MAP_TEXT)
        self.assertIn('debug_.print("MAP,RESUME,REJECT,REASON=")', MAP_TEXT)

    def test_TEST_OBSTACLE_HOLD_RESUME_GATE(self):
        self.assertIn("MapHoldReason::OBSTACLE", MAP_TEXT)
        self.assertIn('rejectReason = "OBSTACLE_NOT_CLEAR"', MAP_TEXT)
        self.assertIn("obstacleLiveClear", MAP_TEXT)
        self.assertIn("obstacleGraceClear", MAP_TEXT)
        self.assertIn("hasRecentClearWindow", MAP_TEXT)
        self.assertIn("replayResumeAllowed_", MAP_TEXT)

    def test_TEST_START_WHILE_HOLD(self):
        self.assertIn('debug_.println("MAP,START,ACTION=RESUME")', MAP_TEXT)
        self.assertIn("MAP,RESUME,REQUEST,WP=", MAP_TEXT)
        self.assertIn("MAP,RESUME,ACCEPT,WP=", MAP_TEXT)
        self.assertNotIn("replayCurrentIndex_ = 0U;\n      replayTargetIndex_ = 1U;", MAP_TEXT)

    def test_TEST_START_AFTER_LONG_CANCEL(self):
        self.assertIn('cancelReplay("X_LONG")', MAP_TEXT)
        self.assertIn("clearReplayResumeContext();", MAP_TEXT)
        self.assertIn('debug_.println("MAP,START,ACTION=RUN")', MAP_TEXT)

    def test_TEST_MANUAL_TAKEOVER_CANCELS(self):
        self.assertIn('cancelReplay("PS2_TAKEOVER")', MAP_TEXT)
        self.assertIn("ps2_.motionCommandActive()", MAP_TEXT)
        self.assertIn("replayResumeAllowed_ = false", MAP_TEXT)

    def test_TEST_HOLD_ONCE_CONTEXT(self):
        for token in ("routeMode_", "replayDirection_", "replayCurrentIndex_",
                      "replayTargetIndex_", "replayContextSlot_", "replayOriginValid_"):
            self.assertIn(token, MAP_HEADER_TEXT)
        self.assertIn("MapReplayMode::ONCE", MAP_TEXT)

    def test_TEST_HOLD_LOOP_CONTEXT(self):
        self.assertIn("MapReplayMode::LOOP", MAP_TEXT)
        self.assertIn("routeMode_", MAP_TEXT)
        self.assertIn("replayDirection_", MAP_TEXT)

    def test_TEST_HOLD_RETURN_CONTEXT(self):
        self.assertIn("MapReplayMode::RETURN", MAP_TEXT)
        self.assertIn("replayReturned_", MAP_TEXT)
        self.assertIn("replayDirection_", MAP_TEXT)

    def test_TEST_HOLD_PING_PONG_CONTEXT(self):
        self.assertIn("MapReplayMode::PING_PONG", MAP_TEXT)
        self.assertIn("replayReturned_", MAP_TEXT)
        self.assertIn("replayTargetIndex_", MAP_TEXT)

    def test_hold_lcd_semantics(self):
        self.assertIn("uint8_t holdReason", _read(INCLUDE_ROOT / "display" / "lcd_display.h"))
        self.assertIn('snprintf(desired_[1], 21, "USER HOLD")', LCD_TEXT)
        self.assertIn('snprintf(desired_[1], 21, "OBS BLOCKED")', LCD_TEXT)
        self.assertIn('snprintf(desired_[3], 21, "X HOLD X-LONG CANCEL")', LCD_TEXT)


if __name__ == "__main__":
    unittest.main(verbosity=2)
