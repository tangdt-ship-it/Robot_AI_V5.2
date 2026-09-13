"""Static checks for the production external motor-driver cascade."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
MOTOR_H = (ROOT / "firmware/stm32/include/motor/motor_controller.h").read_text()
MOTOR_CC = (ROOT / "firmware/stm32/src/motor/motor_controller.cpp").read_text()
CONFIG = (ROOT / "firmware/stm32/include/robot_config.h").read_text()
PIO = (ROOT / "firmware/stm32/platformio.ini").read_text()
MAIN = (ROOT / "firmware/stm32/src/main.cpp").read_text()
ROBOT_CC = (ROOT / "firmware/stm32/src/control/robot_controller.cpp").read_text()
ODOMETRY_H = (ROOT / "firmware/stm32/include/encoders/wheel_odometry.h").read_text()
ODOMETRY_CC = (ROOT / "firmware/stm32/src/encoders/wheel_odometry.cpp").read_text()


class ExternalDriverCascadeSelfTest(unittest.TestCase):
    def test_stm32_has_no_inner_speed_controller(self) -> None:
        self.assertNotIn("PID", MOTOR_H + MOTOR_CC)
        control_dir = ROOT / "firmware/stm32/include/control"
        source_dir = ROOT / "firmware/stm32/src/control"
        self.assertFalse(any("pid" in p.stem.casefold()
                             for p in control_dir.iterdir()))
        self.assertFalse(any("pid" in p.stem.casefold()
                             for p in source_dir.iterdir()))

    def test_external_driver_is_documented_and_only_production_profile_remains(self) -> None:
        architecture = (ROOT / "docs/ARCHITECTURE.md").read_text(encoding="utf-8")
        control = (ROOT / "docs/CONTROL.md").read_text(encoding="utf-8")
        self.assertIn("Driver Motor DC PID V1.0", architecture)
        self.assertIn("Driver Motor DC PID V1.0", control)
        self.assertIn("default_envs = stm32_robot_v4_2", PIO)
        self.assertFalse(any("pid" in line.casefold()
                             for line in PIO.splitlines()))

    def test_direct_command_pwm_dir_and_stop_contract_are_preserved(self) -> None:
        self.assertIn("digitalWrite(dirPin, speed > 0 ? HIGH : LOW)", MOTOR_CC)
        self.assertIn("const uint8_t pwm = static_cast<uint8_t>(255 - abs(speed));", MOTOR_CC)
        self.assertIn("analogWrite(pwmPin, PWM_STOP)", MOTOR_CC)
        self.assertIn("BRAKE_PWM_LOCK", MOTOR_CC)
        self.assertIn("speed = constrain(speed, -255, 255)", MOTOR_CC)
        self.assertNotIn("motors.update", MAIN)

    def test_encoder_odometry_and_calibration_paths_remain(self) -> None:
        for token in (
            "leftMmPerTick",
            "rightMmPerTick",
            "trackMm",
            "startCalibrationStraight",
            "finishCalibrationStraight",
            "startCalibrationTurn",
            "finishCalibrationTurn",
            "commitCalibration",
            "abortCalibration",
            "calibrationStatus",
        ):
            self.assertIn(token, ODOMETRY_H + ODOMETRY_CC)


if __name__ == "__main__":
    unittest.main(verbosity=2)
