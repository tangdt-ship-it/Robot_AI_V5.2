"""Host tests for the Phase 2 two-sector obstacle classifier.

The production classifier is deliberately decision-only.  This file mirrors
its pure classification rules and checks the source-level no-motion contract.
"""

import math
import re
import unittest
from dataclasses import dataclass
from pathlib import Path


STM32_ROOT = Path(__file__).resolve().parents[1]
CONFIG = (STM32_ROOT / "include" / "robot_config.h").read_text(encoding="utf-8")
CLASSIFIER_H = (STM32_ROOT / "include" / "sensors" / "obstacle_classifier.h").read_text(encoding="utf-8")
CLASSIFIER_CC = (STM32_ROOT / "src" / "sensors" / "obstacle_classifier.cpp").read_text(encoding="utf-8")
MAIN = (STM32_ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
MAP = (STM32_ROOT / "src" / "map" / "map_controller.cpp").read_text(encoding="utf-8")
MAP_HEADER = (STM32_ROOT / "include" / "map" / "map_controller.h").read_text(encoding="utf-8")
CTRL = (STM32_ROOT / "src" / "control" / "robot_controller.cpp").read_text(encoding="utf-8")


def config_number(name):
    match = re.search(r"\b" + re.escape(name) + r"\s*=\s*([0-9.]+)", CONFIG)
    if not match:
        raise AssertionError(f"constant {name} not found")
    return float(match.group(1))


@dataclass
class Sample:
    fresh: bool = True
    healthy: bool = True
    valid: bool = True
    distance_cm: float = 100.0
    zone: str = "CLEAR"


def classify(left: Sample, right: Sample):
    valid_zones = {"CLEAR", "CAUTION", "BLOCKED", "EMERGENCY"}
    valid = all(
        (s.fresh and s.healthy and s.valid and math.isfinite(s.distance_cm)
         and 2.0 <= s.distance_cm <= 500.0 and s.zone in valid_zones)
        for s in (left, right)
    )
    if not valid:
        return "UNKNOWN", "HOLD"
    left_blocked = left.zone != "CLEAR"
    right_blocked = right.zone != "CLEAR"
    if not left_blocked and not right_blocked:
        return "CLEAR", "NONE"
    if left_blocked and not right_blocked:
        return "LEFT", "AVOID_RIGHT"
    if right_blocked and not left_blocked:
        return "RIGHT", "AVOID_LEFT"
    cls = "CENTER" if abs(left.distance_cm - right.distance_cm) * 10 <= config_number(
        "OBSTACLE_CLASS_CENTER_BALANCE_MM") else "BOTH_BLOCKED"
    margin = config_number("OBSTACLE_CLASS_SIDE_CLEARANCE_MARGIN_MM")
    if ((right.distance_cm - left.distance_cm) * 10 >= margin
            and right.zone not in {"BLOCKED", "EMERGENCY"}):
        decision = "AVOID_RIGHT"
    elif ((left.distance_cm - right.distance_cm) * 10 >= margin
          and left.zone not in {"BLOCKED", "EMERGENCY"}):
        decision = "AVOID_LEFT"
    else:
        decision = "HOLD"
    return cls, decision


class StabilityModel:
    def __init__(self):
        self.candidate = None
        self.since = 0
        self.cls = "UNKNOWN"
        self.decision = "HOLD"
        self.stable = False

    def update(self, now, left, right):
        cls, decision = classify(left, right)
        if cls == "UNKNOWN":
            self.candidate = None
            self.cls, self.decision, self.stable = "UNKNOWN", "HOLD", False
            return self.cls, self.decision, self.stable
        if self.candidate != (cls, decision):
            self.candidate = (cls, decision)
            self.since = now
            self.stable = False
        self.cls = cls
        if now - self.since >= config_number("OBSTACLE_CLASS_STABLE_MS"):
            self.decision, self.stable = decision, True
        else:
            self.decision, self.stable = "HOLD", False
        return self.cls, self.decision, self.stable


class ObstacleClassifierHostTests(unittest.TestCase):
    def test_constants_are_conservative_and_separate_from_stop_policy(self):
        self.assertEqual(config_number("OBSTACLE_CLASS_CENTER_BALANCE_MM"), 100.0)
        self.assertEqual(config_number("OBSTACLE_CLASS_SIDE_CLEARANCE_MARGIN_MM"), 80.0)
        self.assertGreaterEqual(config_number("OBSTACLE_CLASS_STABLE_MS"), 180.0)
        self.assertLessEqual(config_number("OBSTACLE_CLASS_STABLE_MS"), 300.0)
        self.assertIn("OBSTACLE_BLOCKED_CM", CONFIG)
        self.assertIn("OBSTACLE_CLASS_CENTER_BALANCE_MM", CLASSIFIER_CC)

    def test_both_clear(self):
        self.assertEqual(classify(Sample(), Sample()), ("CLEAR", "NONE"))

    def test_left_blocked_right_clear(self):
        self.assertEqual(classify(Sample(distance_cm=20, zone="BLOCKED"),
                                  Sample(distance_cm=100)),
                         ("LEFT", "AVOID_RIGHT"))

    def test_right_blocked_left_clear(self):
        self.assertEqual(classify(Sample(distance_cm=100),
                                  Sample(distance_cm=20, zone="BLOCKED")),
                         ("RIGHT", "AVOID_LEFT"))

    def test_both_blocked_balanced_is_center_hold(self):
        self.assertEqual(classify(Sample(distance_cm=20, zone="BLOCKED"),
                                  Sample(distance_cm=25, zone="BLOCKED")),
                         ("CENTER", "HOLD"))

    def test_both_blocked_unbalanced_is_both_blocked_hold_when_both_stop(self):
        self.assertEqual(classify(Sample(distance_cm=10, zone="EMERGENCY"),
                                  Sample(distance_cm=30, zone="BLOCKED")),
                         ("BOTH_BLOCKED", "HOLD"))

    def test_center_can_recommend_only_with_clearance_evidence(self):
        self.assertEqual(classify(Sample(distance_cm=20, zone="CAUTION"),
                                  Sample(distance_cm=29, zone="CAUTION")),
                         ("CENTER", "AVOID_RIGHT"))

    def test_invalid_left_is_unknown_hold(self):
        self.assertEqual(classify(Sample(fresh=False), Sample()), ("UNKNOWN", "HOLD"))

    def test_invalid_right_is_unknown_hold(self):
        self.assertEqual(classify(Sample(), Sample(healthy=False)), ("UNKNOWN", "HOLD"))

    def test_both_invalid_are_unknown_hold(self):
        self.assertEqual(classify(Sample(valid=False), Sample(valid=False)),
                         ("UNKNOWN", "HOLD"))

    def test_stale_during_stable_timer_is_immediate_unknown_hold(self):
        model = StabilityModel()
        model.update(0, Sample(distance_cm=20, zone="BLOCKED"), Sample())
        self.assertEqual(model.update(100, Sample(fresh=False), Sample()),
                         ("UNKNOWN", "HOLD", False))

    def test_one_noisy_opposite_sample_does_not_flip_direction(self):
        model = StabilityModel()
        left = Sample(distance_cm=20, zone="BLOCKED")
        right = Sample()
        self.assertEqual(model.update(0, left, right), ("LEFT", "HOLD", False))
        self.assertEqual(model.update(240, left, right), ("LEFT", "AVOID_RIGHT", True))
        self.assertEqual(model.update(300, Sample(),
                                      Sample(distance_cm=20, zone="BLOCKED")),
                         ("RIGHT", "HOLD", False))

    def test_new_direction_becomes_stable_after_gate(self):
        model = StabilityModel()
        model.update(0, Sample(), Sample(distance_cm=20, zone="BLOCKED"))
        self.assertEqual(model.update(240, Sample(),
                                      Sample(distance_cm=20, zone="BLOCKED")),
                         ("RIGHT", "AVOID_LEFT", True))

    def test_class_stability_timer_resets_on_change(self):
        model = StabilityModel()
        model.update(0, Sample(distance_cm=20, zone="BLOCKED"), Sample())
        model.update(200, Sample(distance_cm=20, zone="BLOCKED"), Sample())
        self.assertEqual(model.update(201, Sample(),
                                      Sample(distance_cm=20, zone="BLOCKED")),
                         ("RIGHT", "HOLD", False))
        self.assertEqual(model.update(440, Sample(),
                                      Sample(distance_cm=20, zone="BLOCKED")),
                         ("RIGHT", "HOLD", False))
        self.assertEqual(model.update(441, Sample(),
                                      Sample(distance_cm=20, zone="BLOCKED")),
                         ("RIGHT", "AVOID_LEFT", True))

    def test_phase1_hold_and_explicit_start_are_unchanged(self):
        self.assertIn("enterReplayHold(MapHoldReason::OBSTACLE, true)", MAP)
        self.assertIn("OBSTACLE_CLEAR_STABLE_MS", MAP)
        self.assertIn("MAP,START,ACTION=RESUME", MAP)
        self.assertIn("replayTargetIndex_", MAP_HEADER)

    def test_classifier_has_no_motion_map_or_route_side_effects(self):
        for forbidden in (
            "startTurn", "startDistance", "startReplay", "motor", "Motor",
            "waypoint", "replayTargetIndex", "route_", "advanceReplay",
        ):
            self.assertNotIn(forbidden, CLASSIFIER_CC)
        self.assertIn("obstacleClassifier.update();", MAIN)
        self.assertNotIn("obstacleClassifier.update();", CTRL)

    def test_decision_only_contract_and_state_change_logging(self):
        self.assertIn("OBS,CLASS,", CLASSIFIER_CC)
        self.assertIn("OBS,DECIDE,", CLASSIFIER_CC)
        self.assertIn("OBSTACLE_CLASS_STABLE_MS", CLASSIFIER_CC)
        self.assertIn("candidateSinceMs_", CLASSIFIER_H)
        self.assertIn("if (!result.valid)", CLASSIFIER_CC)


if __name__ == "__main__":
    unittest.main(verbosity=2)
