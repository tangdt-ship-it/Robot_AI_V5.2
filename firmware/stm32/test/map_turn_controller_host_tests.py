"""Deterministic contract tests for the STM32 MAP coarse-turn PD profile."""

import math
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = (ROOT / "include" / "robot_config.h").read_text(encoding="utf-8")
CTRL = (ROOT / "src" / "control" / "robot_controller.cpp").read_text(
    encoding="utf-8"
)
CTRL_HEADER = (ROOT / "include" / "control" / "robot_controller.h").read_text(
    encoding="utf-8"
)


def number(name):
    match = re.search(
        r"\b" + re.escape(name) +
        r"\s*=\s*(-?(?:\d+(?:\.\d*)?|\.\d+))",
        CONFIG,
    )
    if not match:
        raise AssertionError(f"missing constant: {name}")
    return float(match.group(1))


def pd_command(error, rate, maximum=30):
    relation = error * rate
    damping = number("MAP_TURN_PD_KD") * abs(rate)
    if relation < 0:
        damping = -damping
    elif relation == 0:
        damping = 0.0
    command = number("MAP_TURN_PD_KP") * abs(error) - damping
    configured_upper = min(maximum, number("TURN_MAX_SPEED"))
    moving = abs(rate) >= number("MAP_TURN_PD_MOVING_RATE_DEG_S")
    lower = (number("MAP_TURN_PD_MOVING_MIN_COMMAND") if moving
             else number("TURN_MIN_SPEED"))
    ratio = max(0.0, min(1.0,
                (abs(error) - number("MAP_TURN_PD_PULSE_ZONE_DEG")) /
                (number("MAP_TURN_PD_SLOW_ZONE_DEG") -
                 number("MAP_TURN_PD_PULSE_ZONE_DEG"))))
    scheduled_upper = (number("MAP_TURN_PD_MOVING_MIN_COMMAND") +
                       ratio * (configured_upper -
                                number("MAP_TURN_PD_MOVING_MIN_COMMAND")))
    upper = max(lower, scheduled_upper)
    return round(max(lower, min(command, upper)))


def shortest_delta(target, current):
    delta = target - current
    while delta > 180.0:
        delta -= 360.0
    while delta <= -180.0:
        delta += 360.0
    return delta


def peak_after_first_cross(errors, crossing_index):
    if crossing_index is None:
        return 0.0
    peak = 0.0
    for error in errors[crossing_index:]:
        peak = max(peak, abs(error))
    return peak


class MapTurnControllerHostTests(unittest.TestCase):
    def test_profile_isolated_from_precise_mcp(self):
        self.assertIn("AiTurnProfile::MAP_COARSE", CTRL)
        self.assertIn("mapTurnPdCommand", CTRL)
        precise_start = CTRL.split(
            "bool RobotController::startAiTurnRelative", 1
        )[1].split("bool RobotController::startAiTurnAbsolute", 1)[0]
        self.assertNotIn("AiTurnProfile::MAP_COARSE", precise_start)
        self.assertIn("profile == AiTurnProfile::MAP_COARSE", CTRL)
        self.assertIn("TURN_TOLERANCE_DEG", CTRL)

    def test_pd_constants_preserve_map_contract_and_no_integral(self):
        self.assertEqual(number("TURN_TOLERANCE_DEG"), 0.5)
        self.assertEqual(number("MAP_REPLAY_PRETURN_TOLERANCE_DEG"), 2.0)
        self.assertEqual(number("MAP_GUIDE_ARRIVAL_HEADING_TOLERANCE_DEG"), 2.0)
        self.assertEqual(number("MAP_GUIDE_REALIGN_THRESHOLD_DEG"), 15.0)
        self.assertEqual(number("TURN_MIN_SPEED"), 15.0)
        self.assertEqual(number("TURN_MAX_SPEED"), 30.0)
        self.assertEqual(number("MAP_TURN_PD_KI"), 0.0)
        self.assertEqual(number("MAP_TURN_PD_MOVING_MIN_COMMAND"), 8.0)
        self.assertEqual(number("MAP_TURN_PD_MOVING_RATE_DEG_S"), 5.0)
        self.assertEqual(number("MAP_TURN_PD_PULSE_ZONE_DEG"), 2.0)
        self.assertEqual(number("MAP_TURN_PD_CORRECTION_COMMAND"), 15.0)
        self.assertEqual(number("MAP_TURN_PD_PULSE_NEAR_MS"), 20.0)
        self.assertEqual(number("MAP_TURN_PD_CORRECTION_COAST_MS"), 100.0)
        self.assertEqual(number("MAP_TURN_PD_OVERSHOOT_COAST_MS"), 160.0)
        self.assertNotIn("mapTurnIntegral", CTRL_HEADER + CTRL)

    def test_pd_response_has_bounded_symmetric_shape(self):
        for error in (30.0, 45.0, 90.0, 135.0, 180.0):
            self.assertGreaterEqual(pd_command(error, 0.0),
                                    number("TURN_MIN_SPEED"))
            self.assertLessEqual(pd_command(error, 0.0),
                                 number("TURN_MAX_SPEED"))
            self.assertEqual(pd_command(error, 0.0),
                             pd_command(-error, 0.0))
        self.assertGreaterEqual(pd_command(20.0, 0.0),
                                pd_command(10.0, 0.0))
        self.assertLess(pd_command(15.0, 20.0), pd_command(15.0, 0.0))

    def test_toward_rate_damps_and_away_rate_corrects(self):
        still = pd_command(15.0, 0.0)
        toward = pd_command(15.0, 25.0)
        fast_toward = pd_command(15.0, 50.0)
        away = pd_command(15.0, -25.0)
        self.assertLess(toward, still)
        self.assertLessEqual(fast_toward, toward)
        self.assertGreaterEqual(away, still)
        self.assertGreaterEqual(away, number("TURN_MIN_SPEED"))

    def test_wrap_and_signed_direction_contracts_are_present(self):
        self.assertIn("HeadingFusion::shortestDelta", CTRL)
        self.assertIn("aiTurnErrorDeg_ * controlYawRateDegS", CTRL)
        self.assertIn("MAP_TURN_PD_PREDICT_TIME_S", CTRL)
        self.assertIn("MAP,TURN,PD,ERR=", CTRL)

    def test_moving_turn_can_decelerate_below_static_start_command(self):
        command = pd_command(8.0, 20.0, 20.0)
        self.assertGreaterEqual(command,
                                number("MAP_TURN_PD_MOVING_MIN_COMMAND"))
        self.assertLess(command, number("TURN_MIN_SPEED"))
        self.assertIn("const bool moving =", CTRL)
        self.assertIn("const float scheduledUpper", CTRL)

    def test_map_braking_uses_conservative_fused_or_encoder_rate(self):
        self.assertIn("encoderYawRateDegS", CTRL)
        self.assertIn("controlYawRateDegS", CTRL)
        self.assertIn(
            "fabsf(encoderYawRateDegS) > fabsf(controlYawRateDegS)", CTRL)
        self.assertIn(",FRATE=", CTRL)
        self.assertIn(",ERATE=", CTRL)
        self.assertIn("MAP,TURN,DONE,TARGET=", CTRL)
        self.assertAlmostEqual(shortest_delta(179.0, -179.0), -2.0)
        self.assertAlmostEqual(shortest_delta(-179.0, 179.0), 2.0)

    def test_overshoot_model_releases_before_opposite_correction(self):
        errors = (20.0, 12.0, 6.0, 2.0, -1.0)
        commands = [pd_command(error, 10.0) for error in errors[:-1]]
        self.assertGreaterEqual(commands[0], commands[1])
        self.assertGreaterEqual(commands[1], commands[2])
        self.assertIn("overshot", CTRL)
        self.assertIn("aiTurnCoastUntilMs_", CTRL)
        self.assertIn("stopTurnDrive()", CTRL)

    def test_settle_model_requires_heading_and_rate(self):
        tolerance = number("MAP_REPLAY_PRETURN_TOLERANCE_DEG")
        rate_limit = number("MAP_TURN_PD_SETTLE_RATE_DEG_S")

        def settled(error, rate):
            return abs(error) <= tolerance and abs(rate) <= rate_limit

        self.assertTrue(settled(0.5, 1.0))
        self.assertFalse(settled(0.5, 3.0))
        self.assertFalse(settled(6.0, 0.0))
        self.assertIn("turnRateSettled", CTRL)
        self.assertIn("aiTurnSettleStartMs_", CTRL)

    def test_overshoot_and_settle_metrics_are_ram_only(self):
        for field in (
            "mapTurnStartHeading_", "mapTurnMaxAbsYawRate_",
            "mapTurnMaxOvershootDeg_", "mapTurnFirstTargetCrossMs_",
            "mapTurnSettleDurationMs_", "mapTurnFinalError_",
        ):
            self.assertIn(field, CTRL_HEADER)
            self.assertIn(field, CTRL)
        self.assertIn("MAP_TURN_PD_TELEMETRY_MS", CTRL)
        self.assertEqual(number("MAP_TURN_PD_TELEMETRY_MS"), 300.0)
        self.assertIn("MAP_TURN_PD_KI = 0.0f", CONFIG)

    def test_peak_overshoot_tracks_every_sample_after_first_cross(self):
        cases = (
            ((10.0, 4.0, 1.0, -1.0, -3.0, -4.0, -2.0), 3, 4.0),
            ((-10.0, -4.0, -1.0, 1.0, 3.0, 5.0, 2.0), 3, 5.0),
            ((10.0, 5.0, 3.0, 1.0, 0.5), None, 0.0),
            ((3.0, -1.0, -4.0, -2.0, 1.0, 2.0, 0.5), 1, 4.0),
        )
        for errors, crossing_index, expected in cases:
            self.assertEqual(peak_after_first_cross(errors, crossing_index),
                             expected)

    def test_peak_tracking_is_after_first_cross_and_never_resets(self):
        fresh_sample = CTRL.split(
            "if (headingSampleUpdated) {", 1
        )[1].split("if (!headingSampleUpdated", 1)[0]
        self.assertIn("if (mapTurnProfile && mapTurnFirstTargetCrossMs_ != 0U)",
                      fresh_sample)
        self.assertIn("mapTurnMaxOvershootDeg_ =", fresh_sample)
        self.assertEqual(CTRL.count("mapTurnFirstTargetCrossMs_ = nowMs;"), 1)
        self.assertIn("mapTurnMaxOvershootDeg_ = 0.0f;", CTRL)

    def test_map_heading_tolerance_boundaries(self):
        tolerance = number("MAP_REPLAY_PRETURN_TOLERANCE_DEG")
        self.assertEqual(tolerance, 2.0)

        arrival_tolerance = number("MAP_GUIDE_ARRIVAL_HEADING_TOLERANCE_DEG")
        self.assertEqual(arrival_tolerance, 2.0)

    def test_map_final_pulses_are_isolated_from_replay_speed(self):
        self.assertIn(
            "mapTurnProfile\n        ? constrain(MAP_TURN_PD_CORRECTION_COMMAND",
            CTRL,
        )
        self.assertIn("MAP_TURN_PD_PULSE_NEAR_MS", CTRL)
        self.assertIn("MAP_TURN_PD_CORRECTION_COAST_MS", CTRL)
        self.assertIn("maxWheelSpeed > TURN_SETTLE_WHEEL_SPEED_MM_S", CTRL)
        self.assertEqual(number("TURN_CORRECTION_SPEED"), 15.0)


if __name__ == "__main__":
    unittest.main()
