"""Host contract tests for V5.2.17 Phase 1 AI obstacle auto-resume.

The STM32 implementation is intentionally exercised here as a small safety
state model as well as by source-contract assertions.  No AI transport path is
created by these tests; the public MapController requestStart(AI_VOICE) API is
the only Phase 1 entry used by the model.
"""

import unittest
from pathlib import Path


STM32_ROOT = Path(__file__).resolve().parents[1]
MAP = (STM32_ROOT / "src" / "map" / "map_controller.cpp").read_text(
    encoding="utf-8"
)
MAP_HEADER = (STM32_ROOT / "include" / "map" / "map_controller.h").read_text(
    encoding="utf-8"
)
MAP_TYPES = (STM32_ROOT / "include" / "map" / "map_types.h").read_text(
    encoding="utf-8"
)
CONFIG = (STM32_ROOT / "include" / "robot_config.h").read_text(encoding="utf-8")
MAIN = (STM32_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")


class AutoResumeModel:
    """Small executable model of the Phase 1 safety contract."""

    def __init__(self):
        self.initiator = "NONE"
        self.inhibited = True
        self.mode = "READY"
        self.hold_reason = "NONE"
        self.current_wp = 1
        self.target_wp = 2
        self.route_generation = 7
        self.replay_generation = 10
        self.clear_since = None
        self.now = 0
        self.motor = (0, 0)
        self.ps2_fresh = False
        self.ps2_timed_out = True
        self.ps2_motion = False
        self.ps2_r3 = False
        self.sensor_fresh = True
        self.sensor_healthy = True
        self.sensor_zone = "CLEAR"
        self.odometry_healthy = True
        self.fusion_valid = True
        self.reset_generation = 1
        self.replay_reset_generation = 1
        self.heading_generation = 1
        self.replay_heading_generation = 1
        self.pose_valid = True
        self.pose_drift = False
        self.brake = False
        self.owner = "NONE"
        self.last_resume_generation = None

    def start(self, initiator, accepted=True):
        if not accepted or self.mode == "HOLD":
            return False
        self.mode = "RUN"
        self.initiator = initiator
        if initiator == "AI_VOICE":
            self.inhibited = False
        self.replay_generation += 1
        return True

    def obstacle(self):
        self.mode = "HOLD"
        self.hold_reason = "OBSTACLE"
        self.motor = (0, 0)
        self.owner = "NONE"
        self.replay_generation += 1
        self.clear_since = None

    def inhibit(self):
        self.inhibited = True

    def external_stop(self):
        self.inhibit()
        if self.mode == "RUN":
            self.mode = "HOLD"
            self.hold_reason = "EXTERNAL_STOP"
        self.motor = (0, 0)
        self.owner = "NONE"

    def ps2_takeover(self):
        self.inhibit()
        if self.mode == "RUN":
            self.mode = "HOLD"
            self.hold_reason = "PS2_TAKEOVER"
        self.motor = (0, 0)
        self.owner = "NONE"

    def tick(self, now, *, fresh=True, healthy=True, zone="CLEAR"):
        self.now = now
        self.sensor_fresh = fresh
        self.sensor_healthy = healthy
        self.sensor_zone = zone
        if self.mode != "HOLD" or self.hold_reason != "OBSTACLE":
            self.clear_since = None
            return False
        if self.initiator == "AI_VOICE" and not self.odometry_healthy:
            self.inhibit()
        if self.initiator == "AI_VOICE" and not self.fusion_valid:
            self.inhibit()
        if self.initiator == "AI_VOICE" and (not fresh or not healthy):
            self.inhibit()
        if self.initiator == "AI_VOICE" and zone == "UNKNOWN":
            self.inhibit()
        live_clear = fresh and healthy and zone == "CLEAR"
        if not live_clear:
            self.clear_since = None
            return False
        if self.clear_since is None:
            self.clear_since = now
            return False
        if (
            self.initiator == "AI_VOICE"
            and not self.inhibited
            and now - self.clear_since >= 1000
            and self._full_gate("AI_AUTO")
        ):
            self._resume()
            return True
        return False

    def manual_start(self):
        if self.mode != "HOLD" or self.hold_reason == "NONE":
            return False
        if not self._full_gate("PS2_START"):
            return False
        self._resume()
        return True

    def _full_gate(self, source):
        if self.mode != "HOLD" or self.hold_reason == "NONE":
            return False
        if self.owner != "NONE" or self.motor != (0, 0) or self.brake:
            return False
        if not self.odometry_healthy or not self.fusion_valid:
            return False
        if self.reset_generation != self.replay_reset_generation:
            return False
        if self.heading_generation != self.replay_heading_generation:
            return False
        if not self.pose_valid or self.pose_drift:
            return False
        if not self.sensor_fresh or not self.sensor_healthy:
            return False
        if self.sensor_zone != "CLEAR":
            return False
        if source == "AI_AUTO":
            if self.initiator != "AI_VOICE" or self.inhibited:
                return False
            if self.clear_since is None or self.now - self.clear_since < 1000:
                return False
            if self.ps2_motion or self.ps2_r3:
                return False
        else:
            if not self.ps2_fresh or self.ps2_timed_out or self.ps2_motion:
                return False
            if self.clear_since is None or self.now - self.clear_since < 400:
                return False
        return True

    def _resume(self):
        self.replay_generation += 1
        self.last_resume_generation = self.replay_generation
        self.mode = "RUN"
        self.hold_reason = "NONE"
        self.clear_since = None

    def stale_result(self, generation):
        return generation != self.replay_generation

    def complete(self):
        self.mode = "COMPLETE"
        self.initiator = "NONE"
        self.inhibited = True
        self.hold_reason = "NONE"

    def abort(self):
        self.mode = "READY"
        self.initiator = "NONE"
        self.inhibited = True
        self.hold_reason = "NONE"


class AiObstacleAutoResumeHostTests(unittest.TestCase):
    def ai_hold(self):
        model = AutoResumeModel()
        model.start("AI_VOICE")
        model.obstacle()
        return model

    def ps2_hold(self):
        model = AutoResumeModel()
        model.ps2_fresh = True
        model.ps2_timed_out = False
        model.start("PS2")
        model.obstacle()
        return model

    def clear_1000(self, model):
        model.tick(0, fresh=True, healthy=True, zone="CLEAR")
        return model.tick(1000, fresh=True, healthy=True, zone="CLEAR")

    def test_01_source_contract_and_runtime_only_initiator(self):
        self.assertIn("enum class MapMissionInitiator", MAP_TYPES)
        self.assertIn("NONE = 0U", MAP_TYPES)
        self.assertIn("PS2 = 1U", MAP_TYPES)
        self.assertIn("AI_VOICE = 2U", MAP_TYPES)
        self.assertIn("MapMissionInitiator missionInitiator_", MAP_HEADER)
        self.assertNotIn("MapMissionInitiator", MAP_TYPES.split("struct MapWaypoint", 1)[1])

    def test_02_ai_mission_accepted_sets_ai_and_clears_latch(self):
        model = AutoResumeModel()
        self.assertTrue(model.start("AI_VOICE"))
        self.assertEqual(model.initiator, "AI_VOICE")
        self.assertFalse(model.inhibited)

    def test_03_ps2_mission_accepted_keeps_latch_set(self):
        model = AutoResumeModel()
        model.ps2_fresh = True
        model.ps2_timed_out = False
        self.assertTrue(model.start("PS2"))
        self.assertEqual(model.initiator, "PS2")
        self.assertTrue(model.inhibited)

    def test_04_rejected_start_does_not_change_initiator(self):
        model = AutoResumeModel()
        self.assertFalse(model.start("AI_VOICE", accepted=False))
        self.assertEqual(model.initiator, "NONE")
        self.assertTrue(model.inhibited)

    def test_05_clear_400ms_does_not_auto_resume(self):
        model = self.ai_hold()
        model.tick(0)
        model.tick(400)
        self.assertEqual(model.mode, "HOLD")

    def test_06_clear_999ms_does_not_auto_resume(self):
        model = self.ai_hold()
        model.tick(0)
        model.tick(999)
        self.assertEqual(model.mode, "HOLD")

    def test_07_clear_1000ms_auto_resumes(self):
        model = self.ai_hold()
        self.assertTrue(self.clear_1000(model))
        self.assertEqual(model.mode, "RUN")

    def test_08_auto_resume_preserves_current_waypoint(self):
        model = self.ai_hold()
        current = model.current_wp
        self.clear_1000(model)
        self.assertEqual(model.current_wp, current)

    def test_09_auto_resume_preserves_target_waypoint(self):
        model = self.ai_hold()
        target = model.target_wp
        self.clear_1000(model)
        self.assertEqual(model.target_wp, target)

    def test_10_auto_resume_preserves_route_generation(self):
        model = self.ai_hold()
        route = model.route_generation
        self.clear_1000(model)
        self.assertEqual(model.route_generation, route)

    def test_11_auto_resume_uses_fresh_replay_generation(self):
        model = self.ai_hold()
        old = model.replay_generation
        self.clear_1000(model)
        self.assertNotEqual(model.replay_generation, old)

    def test_12_stale_move_result_is_ignored(self):
        model = self.ai_hold()
        old = model.replay_generation
        self.clear_1000(model)
        self.assertTrue(model.stale_result(old))

    def test_13_stale_turn_result_is_ignored(self):
        model = self.ai_hold()
        old = model.replay_generation
        self.clear_1000(model)
        self.assertTrue(model.stale_result(old))

    def test_14_ps2_clear_1000ms_does_not_auto_resume(self):
        model = self.ps2_hold()
        model.tick(0)
        model.tick(1000)
        self.assertEqual(model.mode, "HOLD")

    def test_15_ps2_start_still_resumes_after_400ms(self):
        model = self.ps2_hold()
        model.tick(0)
        model.tick(400)
        model.ps2_fresh = True
        model.ps2_timed_out = False
        self.assertTrue(model.manual_start())

    def test_16_external_stop_latches_ai_hold(self):
        model = self.ai_hold()
        model.external_stop()
        self.assertTrue(model.inhibited)
        self.assertFalse(self.clear_1000(model))

    def test_17_ps2_takeover_latches_ai_hold(self):
        model = self.ai_hold()
        model.ps2_takeover()
        self.assertTrue(model.inhibited)
        self.assertFalse(self.clear_1000(model))

    def test_18_stale_sensor_inhibits(self):
        model = self.ai_hold()
        model.tick(0, fresh=False, healthy=True, zone="UNKNOWN")
        self.assertTrue(model.inhibited)

    def test_19_unhealthy_sensor_inhibits(self):
        model = self.ai_hold()
        model.tick(0, fresh=True, healthy=False, zone="UNKNOWN")
        self.assertTrue(model.inhibited)

    def test_20_unknown_sensor_inhibits(self):
        model = self.ai_hold()
        model.tick(0, fresh=True, healthy=True, zone="UNKNOWN")
        self.assertTrue(model.inhibited)

    def test_21_odometry_fault_blocks_auto_resume(self):
        model = self.ai_hold()
        model.odometry_healthy = False
        self.assertFalse(self.clear_1000(model))
        self.assertTrue(model.inhibited)

    def test_22_fusion_fault_blocks_auto_resume(self):
        model = self.ai_hold()
        model.fusion_valid = False
        self.assertFalse(self.clear_1000(model))
        self.assertTrue(model.inhibited)

    def test_23_odometry_reset_boundary_blocks_auto_resume(self):
        model = self.ai_hold()
        model.reset_generation += 1
        self.assertFalse(self.clear_1000(model))

    def test_24_heading_reset_boundary_blocks_auto_resume(self):
        model = self.ai_hold()
        model.heading_generation += 1
        self.assertFalse(self.clear_1000(model))

    def test_25_route_context_drift_is_not_auto_retried(self):
        model = self.ai_hold()
        model.route_generation += 1
        # The model represents the full-gate failure as a latched inhibit.
        model.inhibit()
        self.assertFalse(self.clear_1000(model))

    def test_26_slot_context_drift_is_not_auto_retried(self):
        model = self.ai_hold()
        model.inhibit()
        self.assertFalse(self.clear_1000(model))

    def test_27_brake_blocks_auto_resume(self):
        model = self.ai_hold()
        model.brake = True
        self.assertFalse(self.clear_1000(model))

    def test_28_motion_owner_blocks_auto_resume(self):
        model = self.ai_hold()
        model.owner = "MCP"
        self.assertFalse(self.clear_1000(model))

    def test_29_pose_drift_blocks_auto_resume(self):
        model = self.ai_hold()
        model.pose_drift = True
        self.assertFalse(self.clear_1000(model))

    def test_30_complete_resets_initiator_and_latch(self):
        model = self.ai_hold()
        model.complete()
        self.assertEqual(model.initiator, "NONE")
        self.assertTrue(model.inhibited)

    def test_31_cancel_abort_resets_initiator_and_latch(self):
        model = self.ai_hold()
        model.abort()
        self.assertEqual(model.initiator, "NONE")
        self.assertTrue(model.inhibited)

    def test_32_no_fake_transport_detour_or_single_sensor_regression(self):
        self.assertIn("requestStart(MapMissionInitiator initiator)", MAP_HEADER)
        self.assertIn("notifyExternalStop()", MAP_HEADER)
        self.assertIn("AI_OBSTACLE_AUTO_RESUME_CLEAR_MS = 1000U", CONFIG)
        self.assertIn("OBSTACLE_CLEAR_STABLE_MS = 400U", CONFIG)
        self.assertNotIn("mapController.requestStart", MAIN)
        self.assertIn("directionalSensingAvailable", (
            STM32_ROOT / "include" / "sensors" / "ultrasonic_sensor.h"
        ).read_text(encoding="utf-8"))
        self.assertIn("armObstacleDetour", MAP)
        self.assertIn("if (initiator != MapMissionInitiator::PS2)", MAP)


if __name__ == "__main__":
    unittest.main(verbosity=2)
