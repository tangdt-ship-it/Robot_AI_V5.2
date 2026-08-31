"""Static contract checks for the STM32 wheel-speed PID candidate.

This test does not open serial ports, flash either MCU, or drive motors. It
protects the split between requested commands, PID-applied PWM commands and
the existing safety stop paths.
"""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
MOTOR_H = (ROOT / "firmware/stm32/include/motor/motor_controller.h").read_text()
MOTOR_CC = (ROOT / "firmware/stm32/src/motor/motor_controller.cpp").read_text()
PID_H = (ROOT / "firmware/stm32/include/control/wheel_speed_pid.h").read_text()
PID_CC = (ROOT / "firmware/stm32/src/control/wheel_speed_pid.cpp").read_text()
CONFIG = (ROOT / "firmware/stm32/include/robot_config.h").read_text()
PIO = (ROOT / "firmware/stm32/platformio.ini").read_text()
MAIN = (ROOT / "firmware/stm32/src/main.cpp").read_text()
ROBOT_CC = (ROOT / "firmware/stm32/src/control/robot_controller.cpp").read_text()


class V5WheelSpeedPidSelfTest(unittest.TestCase):
    def test_pid_is_stm32_local_and_allocation_free(self) -> None:
        self.assertIn("class WheelSpeedPid", PID_H)
        self.assertNotIn("malloc", PID_H + PID_CC)
        self.assertNotIn("new WheelSpeedPid", PID_H + PID_CC)
        self.assertIn("WheelSpeedPid leftPid_", MOTOR_H)
        self.assertIn("WheelSpeedPid rightPid_", MOTOR_H)

    def test_stable_profile_keeps_pid_disabled_by_default(self) -> None:
        self.assertIn("#define ROBOT_WHEEL_SPEED_PID_ENABLE 0", CONFIG)
        self.assertIn("[env:stm32_robot_v4_2_wheel_pid]", PIO)
        self.assertIn("-D ROBOT_WHEEL_SPEED_PID_ENABLE=1", PIO)

    def test_feedback_uses_encoder_velocity_and_preserves_requested_command(self) -> None:
        self.assertIn("updateSpeedPid(float leftVelocityMmS,", MOTOR_H)
        self.assertIn("leftPid_.update(leftSpeed_, leftVelocityMmS", MOTOR_CC)
        self.assertIn("rightPid_.update(rightSpeed_, rightVelocityMmS", MOTOR_CC)
        self.assertIn("motors.updateSpeedPid(wheelOdometry.data().leftVelocityMmS", MAIN)
        self.assertIn("int16_t leftAppliedCommand_ = 0", MOTOR_H)

    def test_stop_brake_zero_and_direction_reset_pid_state(self) -> None:
        self.assertIn("leftPid_.reset();", MOTOR_CC)
        self.assertIn("rightPid_.reset();", MOTOR_CC)
        self.assertIn("if (brakeActive_ || (leftSpeed_ == 0 && rightSpeed_ == 0))", MOTOR_CC)
        self.assertIn("direction change is a new control session", PID_CC)
        self.assertIn("integral_ = 0.0f", PID_CC)

    def test_saturation_and_windup_are_bounded(self) -> None:
        self.assertIn("kIntegralLimit", PID_CC)
        self.assertIn("kCommandMin", PID_CC)
        self.assertIn("kCommandMax", PID_CC)
        self.assertIn("outputMin = command > 0 ? 0.0f : kCommandMin", PID_CC)
        self.assertIn("outputMax = command > 0 ? kCommandMax : 0.0f", PID_CC)
        self.assertIn("saturatingHigh", PID_CC)
        self.assertIn("saturatingLow", PID_CC)
        self.assertIn("roundf(output)", PID_CC)

    def test_safety_paths_remain_outside_pid(self) -> None:
        self.assertIn("if (brakeEnabled_)", ROBOT_CC)
        self.assertIn("obstacleBrakeActive_", ROBOT_CC)
        self.assertIn("motors_.brake();", ROBOT_CC)


if __name__ == "__main__":
    unittest.main(verbosity=2)
