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
    upper = min(maximum, number("TURN_MAX_SPEED"))
    return round(max(number("TURN_MIN_SPEED"),
                     min(command, upper)))


def shortest_delta(target, current):
    delta = target - current
    while delta > 180.0:
        delta -= 360.0
    while delta <= -180.0:
        delta += 360.0
    return delta


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
        self.assertEqual(number("MAP_REPLAY_PRETURN_TOLERANCE_DEG"), 5.0)
        self.assertEqual(number("MAP_GUIDE_ARRIVAL_HEADING_TOLERANCE_DEG"), 6.0)
        self.assertEqual(number("MAP_GUIDE_REALIGN_THRESHOLD_DEG"), 15.0)
        self.assertEqual(number("TURN_MIN_SPEED"), 15.0)
        self.assertEqual(number("TURN_MAX_SPEED"), 30.0)
        self.assertEqual(number("MAP_TURN_PD_KI"), 0.0)
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
        self.assertLess(pd_command(20.0, 20.0), pd_command(20.0, 0.0))

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
        self.assertIn("aiTurnErrorDeg_ * aiTurnYawRateDegS_", CTRL)
        self.assertIn("MAP_TURN_PD_PREDICT_TIME_S", CTRL)
        self.assertIn("MAP,TURN,PD,ERR=", CTRL)
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
        self.assertIn("MAP_TURN_PD_KI = 0.0f", CONFIG)


if __name__ == "__main__":
    unittest.main()
