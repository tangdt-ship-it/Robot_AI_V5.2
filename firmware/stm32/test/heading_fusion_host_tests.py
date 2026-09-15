"""Static and arithmetic regression checks for calibrated MPU6050 yaw rate."""

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = (ROOT / "include" / "robot_config.h").read_text(encoding="utf-8")
IMU = (ROOT / "src" / "imu" / "mpu6050.cpp").read_text(encoding="utf-8")
FUSION = (ROOT / "src" / "localization" / "heading_fusion.cpp").read_text(
    encoding="utf-8"
)


def number(name):
    match = re.search(
        r"\b" + re.escape(name) + r"\s*=\s*(\d+(?:\.\d*)?)f", CONFIG
    )
    if not match:
        raise AssertionError(f"missing constant: {name}")
    return float(match.group(1))


class HeadingFusionHostTests(unittest.TestCase):
    def test_yaw_scale_matches_fixed_pointer_commissioning(self):
        # At 1.019 the fixed-pointer runs were consistently one degree short
        # at both 90 and 180 degrees; the fitted correction is 1.012.
        self.assertAlmostEqual(number("IMU_GYRO_Z_YAW_SCALE"), 1.012, places=6)

    def test_scale_is_applied_after_bias_removal(self):
        bias = 10.0
        raw_rate = 100.0
        corrected = (raw_rate - bias) * number("IMU_GYRO_Z_YAW_SCALE")
        self.assertAlmostEqual(corrected, 91.08, places=5)
        # A stationary raw sample equal to the learned bias must remain zero.
        self.assertEqual((bias - bias) * number("IMU_GYRO_Z_YAW_SCALE"), 0.0)
        bias_line = "data_.gyroZDps -= gyroBiasZDegS_;"
        scale_line = "data_.gyroZDps *= IMU_GYRO_Z_YAW_SCALE;"
        self.assertIn(bias_line, IMU)
        self.assertIn(scale_line, IMU)
        self.assertLess(IMU.index(bias_line), IMU.index(scale_line))

    def test_heading_fusion_consumes_the_corrected_rate_without_gain_change(self):
        self.assertIn("const float gyroDeltaDeg = gyroZDegS * dt;", FUSION)
        self.assertIn("FUSION_ENCODER_DELTA_WEIGHT = 0.10f", CONFIG)
        self.assertNotIn("IMU_GYRO_Z_YAW_SCALE", FUSION)


if __name__ == "__main__":
    unittest.main()
