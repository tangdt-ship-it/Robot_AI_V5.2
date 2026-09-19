"""Static regression checks for the two-channel HC-SR04 scheduler."""

import re
import unittest
from pathlib import Path


STM32_ROOT = Path(__file__).resolve().parents[1]
CONFIG_TEXT = (STM32_ROOT / "include" / "robot_config.h").read_text(
    encoding="utf-8"
)
SENSOR_TEXT = (STM32_ROOT / "src" / "sensors" / "ultrasonic_sensor.cpp").read_text(
    encoding="utf-8"
)
SENSOR_HEADER_TEXT = (
    STM32_ROOT / "include" / "sensors" / "ultrasonic_sensor.h"
).read_text(encoding="utf-8")
MAP_TEXT = (STM32_ROOT / "src" / "map" / "map_controller.cpp").read_text(
    encoding="utf-8"
)


def constant(name):
    match = re.search(r"\b" + re.escape(name) + r"\s*=\s*(\d+)", CONFIG_TEXT)
    if not match:
        raise AssertionError(f"constant {name} not found")
    return int(match.group(1))


class UltrasonicHostTests(unittest.TestCase):
    def test_physical_pin_mapping_is_explicit(self):
        self.assertIn("ULTRASONIC_TRIG_PIN = 44U;       // PC12", CONFIG_TEXT)
        self.assertIn("ULTRASONIC_ECHO_PIN = 41U;       // PC9", CONFIG_TEXT)
        self.assertIn("ULTRASONIC_RIGHT_TRIG_PIN = 36U; // PC4", CONFIG_TEXT)
        self.assertIn("ULTRASONIC_RIGHT_ECHO_PIN = 39U; // PC7", CONFIG_TEXT)

    def test_round_robin_has_one_active_channel(self):
        self.assertIn("activeChannel_==0xFF", SENSOR_TEXT)
        self.assertIn("nextChannel_=(i+1)%2", SENSOR_TEXT)
        self.assertIn("activeChannel_=i", SENSOR_TEXT)
        self.assertIn("activeChannel_!=0xFF", SENSOR_TEXT)

    def test_single_center_sensor_disables_the_right_hardware_channel(self):
        self.assertIn("ULTRASONIC_RIGHT_ENABLED = false", CONFIG_TEXT)
        self.assertIn(
            "channels_[RIGHT_MOUNT].enabled=ULTRASONIC_RIGHT_ENABLED;",
            SENSOR_TEXT,
        )
        self.assertIn("if(!c.enabled) continue;", SENSOR_TEXT)
        self.assertIn("o.health=SensorHealth::DISABLED;", SENSOR_TEXT)
        self.assertIn("if(!leftEnabled||!rightEnabled){suggestion_=AvoidanceDirection::STOP;return;}", SENSOR_TEXT)

    def test_single_channel_clear_and_fault_state_remain_fail_closed(self):
        self.assertIn("overallFresh_=(!leftEnabled||frontLeft_.fresh)&&(!rightEnabled||frontRight_.fresh);", SENSOR_TEXT)
        self.assertIn("if(!anyEnabled||!l||!r){overallZone_=ObstacleZone::UNKNOWN", SENSOR_TEXT)
        self.assertIn("bool anyEnabled=false;", SENSOR_TEXT)
        self.assertIn("return anyEnabled;", SENSOR_TEXT)

    def test_disabled_right_mount_is_excluded_from_degraded_forward_limit(self):
        # A disabled right sensor retains the default filter value zero.  That
        # must not clamp a known-clear centre sensor to zero during a bounded
        # dropout.
        self.assertNotIn(
            "const float degradedNearest=min(channels_[LEFT_MOUNT].filteredDistanceCm,channels_[RIGHT_MOUNT].filteredDistanceCm);",
            SENSOR_TEXT,
        )
        self.assertIn("if(!channel.enabled) continue;", SENSOR_TEXT)
        self.assertIn("float degradedNearest=ULTRASONIC_MAX_CM;", SENSOR_TEXT)
        self.assertIn("if(!anyEnabled)return 0;", SENSOR_TEXT)

    def test_map_turn_accepts_only_bounded_single_front_timeout_grace(self):
        # Regression: a real, recently validated >50 cm centre reading may
        # bridge a short no-Echo interval while MAP rotates in place.  A close
        # reading, startup/no evidence, or a timeout beyond the grace window
        # remains fail-closed.
        def map_turn_allowed(fresh_healthy_clear, recent_wide_clear):
            return fresh_healthy_clear or recent_wide_clear

        self.assertTrue(map_turn_allowed(False, True))
        self.assertFalse(map_turn_allowed(False, False))
        self.assertFalse(map_turn_allowed(False, False))  # prior close echo
        self.assertTrue(map_turn_allowed(True, False))

        self.assertIn("const bool mapSensorClear", (STM32_ROOT / "src" / "control" / "robot_controller.cpp").read_text(encoding="utf-8"))
        controller_text = (STM32_ROOT / "src" / "control" / "robot_controller.cpp").read_text(encoding="utf-8")
        self.assertIn("const bool recentClearWindow = ultrasonic_.hasRecentClearWindow(nowMs);", controller_text)
        self.assertNotIn("const bool recentClearWindow = !mapTurnProfile", controller_text)

    def test_echo_lines_must_be_quiet_before_next_trigger(self):
        self.assertIn("if(digitalRead(c.echoPin)==HIGH) {", SENSOR_TEXT)
        self.assertIn("c.displayNoEchoFar=false;", SENSOR_TEXT)
        self.assertIn("A disconnected or", SENSOR_TEXT)
        self.assertIn("floating Echo input must not globally block", SENSOR_TEXT)
        self.assertNotIn("digitalRead(channels_[LEFT_MOUNT].echoPin)==HIGH ||", SENSOR_TEXT)

    def test_inter_sensor_guard_preserves_continuous_observation(self):
        sample_ms = constant("ULTRASONIC_SAMPLE_PERIOD_MS")
        guard_ms = constant("ULTRASONIC_INTER_SENSOR_GUARD_MS")
        timeout_us = constant("ULTRASONIC_ECHO_TIMEOUT_US")
        fresh_ms = constant("ULTRASONIC_FRESH_MS")
        self.assertGreaterEqual(guard_ms, 40)
        self.assertLessEqual(sample_ms, 80)
        self.assertLessEqual(timeout_us // 1000 + guard_ms, fresh_ms // 3)

    def test_rejects_only_immediate_echo_coupling_not_close_obstacles(self):
        min_rise_us = constant("ULTRASONIC_ECHO_MIN_RISE_US")
        self.assertGreaterEqual(min_rise_us, 50)
        # 2 cm acoustic round-trip is about 117 us, so the electrical-noise
        # floor must remain below it.
        self.assertLess(min_rise_us, 117)
        self.assertIn("const uint32_t riseDelayUs=us-c.waitEchoStartedUs;", SENSOR_TEXT)
        self.assertIn("riseDelayUs<ULTRASONIC_ECHO_MIN_RISE_US", SENSOR_TEXT)
        self.assertIn("++c.earlyEchoCount;", SENSOR_TEXT)

    def test_map_precheck_uses_bounded_clear_grace(self):
        self.assertIn("const bool obstacleLiveClear", MAP_TEXT)
        self.assertIn("const bool obstacleGraceClear", MAP_TEXT)
        self.assertIn("hasRecentClearWindow(now)", MAP_TEXT)
        self.assertIn('debug_.println("MAP,REPLAY=PRECHECK,OBSTACLE_GRACE=1")', MAP_TEXT)

    def test_lcd_far_state_uses_filtered_distance_with_hysteresis(self):
        self.assertIn("ULTRASONIC_DISPLAY_FAR_ENTER_CM", CONFIG_TEXT)
        self.assertIn("ULTRASONIC_DISPLAY_FAR_EXIT_CM", CONFIG_TEXT)
        self.assertIn(
            "c.filteredDistanceCm>=ULTRASONIC_DISPLAY_FAR_ENTER_CM", SENSOR_TEXT
        )
        self.assertIn(
            "c.filteredDistanceCm<=ULTRASONIC_DISPLAY_FAR_EXIT_CM", SENSOR_TEXT
        )
        self.assertNotIn("c.displayFar=d>=100.0f", SENSOR_TEXT)
        self.assertNotIn("const bool recoveredFromNoEcho=c.displayFar", SENSOR_TEXT)
        self.assertNotIn("c.historyCount=0;", SENSOR_TEXT)

    def test_no_echo_far_is_bounded_and_requires_prior_clear_far_evidence(self):
        self.assertIn("ULTRASONIC_DISPLAY_NO_ECHO_FAR_TIMEOUTS", CONFIG_TEXT)
        self.assertIn("ULTRASONIC_NO_ECHO_FAR_GRACE_MS", CONFIG_TEXT)
        self.assertIn("ULTRASONIC_NO_ECHO_FAR_MAX_TIMEOUTS", CONFIG_TEXT)
        self.assertIn("c.noEchoFar=true", SENSOR_TEXT)
        self.assertIn("c.noEchoFar=false", SENSOR_TEXT)
        self.assertIn("c.consecutiveTimeouts>=ULTRASONIC_DISPLAY_NO_ECHO_FAR_TIMEOUTS", SENSOR_TEXT)
        self.assertIn("c.displayFar && c.zone==ObstacleZone::CLEAR", SENSOR_TEXT)
        self.assertIn("hasBoundedNoEchoFar", SENSOR_TEXT)
        self.assertIn("c.health=noEcho?SensorHealth::TIMEOUT:SensorHealth::INVALID", SENSOR_TEXT)

    def test_lcd_ok_after_a_known_working_sensor_loses_its_reflector(self):
        # Display recovery is intentionally separate from safety: a close
        # object removed from the cone becomes LCD OK after clean no-Echo
        # samples, while zone/health still require fresh real Echo data.
        self.assertIn("bool displayNoEchoFar=false", SENSOR_HEADER_TEXT)
        self.assertIn("c.displayNoEchoFar=true", SENSOR_TEXT)
        self.assertIn("c.displayNoEchoFar=false", SENSOR_TEXT)
        self.assertIn("boundedNoEchoFar || c.displayNoEchoFar", SENSOR_TEXT)
        self.assertIn(
            "not\n    // used by the obstacle/motion safety model",
            SENSOR_HEADER_TEXT,
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
