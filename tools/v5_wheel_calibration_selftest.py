#!/usr/bin/env python3
"""Host checks for the guided wheel-geometry calibration contract."""

from pathlib import Path
import math
import unittest


ROOT = Path(__file__).resolve().parents[1]
ODOMETRY_CC = ROOT / "firmware/stm32/src/encoders/wheel_odometry.cpp"
ROBOT_LINK_CC = ROOT / "firmware/stm32/src/communication/robot_link_server.cpp"
ULTRASONIC_H = ROOT / "firmware/stm32/include/sensors/ultrasonic_sensor.h"
ULTRASONIC_CC = ROOT / "firmware/stm32/src/sensors/ultrasonic_sensor.cpp"
ROBOT_CONFIG_H = ROOT / "firmware/stm32/include/robot_config.h"


def straight_mm_per_tick(reference_mm: float, ticks: int) -> float:
    if reference_mm < 100.0 or reference_mm > 2000.0 or abs(ticks) < 50:
        raise ValueError("invalid straight calibration sample")
    return reference_mm / abs(ticks)


def track_width(left_ticks: int, right_ticks: int, left_mpt: float,
                right_mpt: float, reference_deg: float) -> float:
    if abs(left_ticks) < 50 or abs(right_ticks) < 50:
        raise ValueError("sample too small")
    if (left_ticks > 0) == (right_ticks > 0):
        raise ValueError("turn must drive wheels in opposite directions")
    if reference_deg < 45.0 or reference_deg > 720.0:
        raise ValueError("invalid turn reference")
    return abs(right_ticks * right_mpt - left_ticks * left_mpt) / math.radians(reference_deg)


class V5WheelCalibrationSelfTest(unittest.TestCase):
    def test_straight_scale_is_per_wheel(self):
        self.assertAlmostEqual(straight_mm_per_tick(1000.0, 17000),
                               1000.0 / 17000.0)
        self.assertAlmostEqual(straight_mm_per_tick(1000.0, -18000),
                               1000.0 / 18000.0)

    def test_turn_scale_uses_signed_wheel_difference(self):
        width = track_width(-5000, 5000, 0.06, 0.055, 180.0)
        self.assertAlmostEqual(width, 575.0 / math.pi, places=5)

    def test_invalid_samples_are_rejected(self):
        with self.assertRaises(ValueError):
            straight_mm_per_tick(99.0, 1000)
        with self.assertRaises(ValueError):
            straight_mm_per_tick(500.0, 10)
        with self.assertRaises(ValueError):
            track_width(5000, 5000, 0.06, 0.055, 180.0)
        with self.assertRaises(ValueError):
            track_width(-5000, 5000, 0.06, 0.055, 30.0)

    def test_runtime_kinematics_and_protocol_are_present(self):
        odometry = ODOMETRY_CC.read_text(encoding="utf-8")
        link = ROBOT_LINK_CC.read_text(encoding="utf-8")
        self.assertIn("leftMmPerTick_ * scale", odometry)
        self.assertIn("rightMmPerTick_ * scale", odometry)
        self.assertIn("/ trackMm_", odometry)
        self.assertIn("HAL_FLASHEx_Erase", odometry)
        self.assertIn('"CAL,BEGIN,STRAIGHT"', link)
        # STM32/newlib builds do not reliably parse float arguments through
        # sscanf; the production parser deliberately uses strtof with an
        # exact end-of-frame check.
        self.assertIn("ParseCalibrationReference", link)
        self.assertIn("strtof(valueStart, &valueEnd)", link)
        self.assertIn('"CAL,END,TURN,"', link)

    def test_ultrasonic_timeout_grace_is_bounded_and_fail_closed(self):
        config = ROBOT_CONFIG_H.read_text(encoding="utf-8")
        header = ULTRASONIC_H.read_text(encoding="utf-8")
        source = ULTRASONIC_CC.read_text(encoding="utf-8")
        self.assertIn("ULTRASONIC_DEGRADED_GRACE_MS", config)
        self.assertIn("ULTRASONIC_DEGRADED_MAX_TIMEOUTS", config)
        self.assertIn("ULTRASONIC_DEGRADED_CLEAR_CM", config)
        self.assertIn("degradedClearWindow", header)
        self.assertIn("c.lastValidEchoMs", source)
        self.assertIn("c.consecutiveTimeouts>ULTRASONIC_DEGRADED_MAX_TIMEOUTS", source)
        self.assertIn("if(!degradedClearWindow(millis()))return 0", source)
        self.assertIn("return min(cmd,ULTRASONIC_DEGRADED_MAX_FORWARD_COMMAND)", source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
