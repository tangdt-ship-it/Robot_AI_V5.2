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

    def test_echo_lines_must_be_quiet_before_next_trigger(self):
        self.assertIn("if(digitalRead(c.echoPin)==HIGH) continue;", SENSOR_TEXT)
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

    def test_lcd_can_present_no_echo_as_far_without_changing_safety_health(self):
        self.assertIn("ULTRASONIC_DISPLAY_NO_ECHO_FAR_TIMEOUTS", CONFIG_TEXT)
        self.assertIn("c.noEchoFar=true", SENSOR_TEXT)
        self.assertIn("c.noEchoFar=false", SENSOR_TEXT)
        self.assertIn("c.consecutiveTimeouts>=ULTRASONIC_DISPLAY_NO_ECHO_FAR_TIMEOUTS", SENSOR_TEXT)
        self.assertRegex(SENSOR_TEXT, r"\|\|\s+c\.noEchoFar")
        self.assertIn("c.health=noEcho?SensorHealth::TIMEOUT:SensorHealth::INVALID", SENSOR_TEXT)


if __name__ == "__main__":
    unittest.main(verbosity=2)
