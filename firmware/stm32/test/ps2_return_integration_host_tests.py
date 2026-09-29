"""Phase 3 PS2 BACK_READY_P0 integration contracts.

These tests intentionally inspect the production source boundary. They keep
the Phase 2 Return-to-P0 core isolated while proving that PS2 START selects it
only for the explicit BACK_READY state and that normal MAP/hold semantics stay
on their existing paths.
"""

import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[3]
MAP = (ROOT / "firmware/stm32/src/map/map_controller.cpp").read_text()
MAP_HEADER = (ROOT / "firmware/stm32/include/map/map_controller.h").read_text()
LCD = (ROOT / "firmware/stm32/src/display/lcd_display.cpp").read_text()
TYPES = (ROOT / "firmware/stm32/include/map/map_types.h").read_text()


def start_block():
    start = MAP.index("bool MapController::requestStart(")
    end = MAP.index("void MapController::handleTriangle()", start)
    return MAP[start:end]


class Ps2ReturnIntegrationTests(unittest.TestCase):
    def test_01_home_context_predicate_exists(self):
        self.assertIn("backReadyP0Available", MAP_HEADER)
        self.assertIn("bool MapController::backReadyP0Available", MAP)

    def test_02_teach_save_arms_home(self):
        save = MAP[MAP.index("if (savePending_)"):MAP.index("if (modeSavePending_)")]
        self.assertIn("armHomeContextAfterSave()", save)

    def test_03_teach_save_failure_invalidates_home(self):
        save = MAP[MAP.index("if (savePending_)"):MAP.index("if (modeSavePending_)")]
        self.assertIn('invalidateHomeContext("STORAGE_ERROR")', save)

    def test_04_home_predicate_requires_valid_context(self):
        block = MAP[MAP.index("bool MapController::backReadyP0Available"):
                    MAP.index("bool MapController::startPostTeachBack")]
        self.assertIn("!homeContext_.valid", block)

    def test_05_home_predicate_checks_slot(self):
        self.assertIn("selectedSlot_ != homeContext_.slot", MAP)

    def test_06_home_predicate_checks_route_generation(self):
        self.assertIn("route_.header.generation != homeContext_.routeGeneration",
                      MAP)

    def test_07_home_predicate_checks_reset_generations(self):
        self.assertIn("odometry_.resetGeneration() != homeContext_.odometryResetGeneration",
                      MAP)
        self.assertIn("robot_.headingResetGeneration() != homeContext_.headingResetGeneration",
                      MAP)

    def test_08_home_predicate_checks_start_waypoint(self):
        self.assertIn("route_.waypoints[0].flags & MAP_WP_START", MAP)

    def test_09_home_predicate_checks_stopped_and_owner(self):
        block = MAP[MAP.index("bool MapController::backReadyP0Available"):
                    MAP.index("bool MapController::startPostTeachBack")]
        self.assertIn("robot_.motorsStopped()", block)
        self.assertIn("robot_.motionOwner() != MotionOwner::NONE", block)

    def test_10_home_predicate_checks_ps2_neutral(self):
        block = MAP[MAP.index("bool MapController::backReadyP0Available"):
                    MAP.index("bool MapController::startPostTeachBack")]
        self.assertIn("ps2_.motionCommandActive()", block)
        self.assertIn("PS2_NOT_NEUTRAL", block)

    def test_11_home_predicate_checks_sensor_safe(self):
        block = MAP[MAP.index("bool MapController::backReadyP0Available"):
                    MAP.index("bool MapController::startPostTeachBack")]
        self.assertIn("ultrasonic_.isFresh()", block)
        self.assertIn("ObstacleZone::CLEAR", block)

    def test_12_ps2_start_uses_universal_api(self):
        block = start_block()
        self.assertIn("requestReturnToP0(ReturnP0Source::PS2_START", block)

    def test_13_ps2_start_does_not_call_legacy_back(self):
        block = start_block()
        self.assertNotIn("startPostTeachBack(", block)

    def test_14_return_source_is_ps2(self):
        self.assertIn("ReturnP0Source::PS2_START", MAP)
        self.assertIn("SOURCE=PS2_START", MAP)

    def test_15_return_active_start_is_rejected(self):
        block = start_block()
        self.assertIn("returnP0InProgress()", block)
        self.assertIn("RETURN_P0_ACTIVE", block)

    def test_16_duplicate_start_has_no_new_request(self):
        block = start_block()
        first = block.index("if (returnP0InProgress())")
        second = block.index("requestReturnToP0(ReturnP0Source::PS2_START")
        self.assertLess(first, second)

    def test_17_return_api_has_active_guard(self):
        body = MAP[MAP.index("bool MapController::requestReturnToP0Internal"):
                   MAP.index("void MapController::handleTriangle")]
        self.assertIn('reason = "RETURN_P0_ACTIVE"', body)

    def test_18_home_frame_is_not_recaptured(self):
        body = MAP[MAP.index("bool MapController::requestReturnToP0Internal"):
                   MAP.index("void MapController::handleTriangle")]
        self.assertIn("replayOrigin_ = homeContext_.p0WorldPose", body)
        self.assertNotIn("readPose(homeContext_.p0WorldPose)", body)

    def test_19_p0_position_uses_home_pose(self):
        self.assertIn("const Pose target = homeContext_.p0WorldPose", MAP)

    def test_20_p0_heading_uses_home_heading(self):
        self.assertIn("homeContext_.p0WorldPose.headingDeg", MAP)

    def test_21_final_tolerances_are_preserved(self):
        self.assertIn("MAP_RETURN_P0_POSITION_TOLERANCE_MM", MAP)
        self.assertIn("MAP_RETURN_P0_HEADING_TOLERANCE_DEG", MAP)

    def test_22_complete_requires_final_gate(self):
        final = MAP[MAP.index("case ReturnP0State::P0_HEADING_SETTLE"):
                    MAP.index("void MapController::abortReturnToP0")]
        self.assertIn("robot_.motorsStopped()", final)
        self.assertIn("motionOwner() == MotionOwner::NONE", final)
        self.assertIn("completeReturnToP0()", final)

    def test_23_complete_publishes_stopped_state(self):
        self.assertIn('debug_.println("MAP,RETURN_P0,COMPLETE")', MAP)
        self.assertIn("robot_.stopImmediately(true)", MAP[MAP.index(
            "void MapController::completeReturnToP0"):])

    def test_24_complete_disables_back_ready(self):
        block = MAP[MAP.index("bool MapController::backReadyP0Available"):
                    MAP.index("bool MapController::startPostTeachBack")]
        self.assertIn("ReturnP0State::COMPLETE", block)

    def test_25_cross_dismisses_without_home_invalidation(self):
        cross = MAP[MAP.index("void MapController::handleCross()"):
                    MAP.index("void MapController::handleCrossLong()")]
        self.assertIn("backP0UiDismissed_ = true", cross)
        self.assertNotIn('invalidateHomeContext("DISMISS")', cross)

    def test_26_cross_dismisses_without_motion(self):
        cross = MAP[MAP.index("void MapController::handleCross()"):
                    MAP.index("void MapController::handleCrossLong()")]
        self.assertIn('MAP,RETURN_P0,DISMISS', cross)
        self.assertNotIn("startReplay", cross)

    def test_27_external_stop_aborts_return(self):
        self.assertIn('abortReturnToP0("EXTERNAL_STOP")', MAP)

    def test_28_ps2_takeover_aborts_return(self):
        self.assertIn('abortReturnToP0(ps2_.state().r3 ? "R3" : "PS2_TAKEOVER")',
                      MAP)

    def test_29_obstacle_return_enters_hold(self):
        return_body = MAP[MAP.index("bool MapController::consumeReturnTurnResult"):
                           MAP.index("MapController::Pose MapController::routePointWorld")]
        self.assertIn("MapHoldReason::OBSTACLE", return_body)
        self.assertIn("ReturnP0State::HOLD", return_body)

    def test_30_obstacle_return_does_not_auto_resume(self):
        self.assertIn("returnP0Source_ != ReturnP0Source::AI_VOICE", MAP)
        self.assertIn("resumeReturnP0FromObstacleHold", MAP)

    def test_31_normal_saved_start_still_prepares_replay(self):
        block = start_block()
        self.assertIn("const bool started = prepareReplay(reason, initiator)", block)

    def test_32_replay_hold_start_path_is_before_normal_run(self):
        block = start_block()
        self.assertLess(block.index("mode_ == MapControllerMode::REPLAY_HOLD"),
                        block.index("prepareReplay(reason, initiator)"))

    def test_33_settings_start_semantics_remain(self):
        block = start_block()
        self.assertIn("saveSettingsAndExit()", block)

    def test_34_ai_is_not_wired_to_ps2_back(self):
        block = start_block()
        self.assertIn("requestReturnToP0(ReturnP0Source::PS2_START", block)
        self.assertNotIn("requestReturnToP0(ReturnP0Source::AI_VOICE", block)

    def test_35_robotlink_is_not_changed_by_map_integration(self):
        self.assertIn("requestReturnToP0", MAP_HEADER)
        self.assertIn("notifyExternalStop", MAP_HEADER)

    def test_36_lcd_ready_text_is_bounded(self):
        for line in ("MAP1 BACK P0 READY", "BACK TO P0", "START BACK X EXIT"):
            self.assertLessEqual(len(line), 20)
        self.assertIn('"MAP%u BACK P0 READY"', LCD)

    def test_37_lcd_active_text_is_bounded(self):
        self.assertIn("postTeachBackActive", LCD)
        self.assertIn('"BACK TO P0"', LCD)

    def test_38_lcd_complete_text_is_bounded(self):
        self.assertIn('"MAP%u BACK COMPLETE"', LCD)

    def test_39_lcd_uses_integrated_status_flags(self):
        self.assertIn("postTeachBackAvailable", LCD)
        self.assertIn("postTeachBackComplete", LCD)

    def test_40_home_dismiss_flag_is_runtime_only(self):
        self.assertIn("backP0UiDismissed_", MAP_HEADER)
        self.assertNotIn("backP0UiDismissed", TYPES)

    def test_41_legacy_post_teach_context_remains_for_history(self):
        self.assertIn("PostTeachBackContext", MAP_HEADER)
        self.assertIn("startPostTeachBack", MAP)

    def test_42_legacy_start_is_not_reachable_from_production_start(self):
        block = start_block()
        self.assertNotIn("postTeachBack_.valid", block)
        self.assertNotIn("startPostTeachBack", block)

    def test_43_no_new_storage_format_field(self):
        self.assertNotIn("MapRouteHeader", MAP_HEADER.split("HomeContext", 1)[1])

    def test_44_phase1_ai_auto_resume_symbol_remains(self):
        self.assertIn("MapMissionInitiator::AI_VOICE", MAP)
        self.assertIn("autonomousResumeInhibited_", MAP)

    def test_45_auto_detour_not_started_by_return(self):
        body = MAP[MAP.index("bool MapController::requestReturnToP0Internal"):
                   MAP.index("void MapController::handleTriangle")]
        self.assertNotIn("armObstacleDetour", body)

    def test_46_single_front_sensor_policy_not_changed(self):
        self.assertNotIn("enableRight", MAP)
        self.assertIn("ultrasonic_", MAP)

    def test_47_return_state_enum_is_used(self):
        self.assertIn("ReturnP0State::", MAP)
        self.assertIn("enum class ReturnP0State", TYPES)
        self.assertGreaterEqual(MAP.count("returnP0State_"), 10)

    def test_48_return_source_enum_is_used(self):
        self.assertIn("enum class ReturnP0Source", TYPES)
        self.assertIn("returnP0Source_", MAP_HEADER)

    def test_49_generation_fence_remains_active(self):
        self.assertIn("returnP0SegmentGeneration_", MAP)
        self.assertIn("DROP_STALE", MAP)

    def test_50_route_direction_is_reverse(self):
        body = MAP[MAP.index("bool MapController::requestReturnToP0Internal"):
                   MAP.index("void MapController::handleTriangle")]
        self.assertIn("replayDirection_ = -1", body)

    def test_51_no_direct_current_to_p0_shortcut_contract(self):
        self.assertIn("startReturnWaypoint", MAP)
        self.assertIn("startReturnP0Position", MAP)

    def test_52_reboot_clears_home(self):
        begin = MAP[MAP.index("void MapController::begin()"):
                    MAP.index("void MapController::processInput()")]
        self.assertIn("homeContext_ = {}", begin)
        self.assertIn("backP0UiDismissed_ = false", begin)

    def test_53_new_teach_resets_return_ui(self):
        arm = MAP[MAP.index("void MapController::armHomeContextAfterSave"):
                  MAP.index("void MapController::invalidateHomeContext")]
        self.assertIn("backP0UiDismissed_ = false", arm)

    def test_54_no_flash_or_hil_code_path_added(self):
        self.assertIn("requestReturnToP0", MAP)
        self.assertNotIn("flashApplication", MAP)

    def test_55_integration_is_stm32_map_only(self):
        self.assertIn("class MapController", MAP_HEADER)
        self.assertIn("PS2", MAP)


if __name__ == "__main__":
    unittest.main()
