#!/usr/bin/env python3
"""Host checks for the guided wheel-geometry calibration contract."""

from pathlib import Path
import math
import struct
import unittest
import zlib


ROOT = Path(__file__).resolve().parents[1]
ODOMETRY_CC = ROOT / "firmware/stm32/src/encoders/wheel_odometry.cpp"
ODOMETRY_H = ROOT / "firmware/stm32/include/encoders/wheel_odometry.h"
FLASH_LAYOUT_H = ROOT / "firmware/stm32/include/map/flash_layout.h"
MAP_HOST_TESTS = ROOT / "firmware/stm32/test/map_host_tests.py"
ROBOT_LINK_CC = ROOT / "firmware/stm32/src/communication/robot_link_server.cpp"
ULTRASONIC_H = ROOT / "firmware/stm32/include/sensors/ultrasonic_sensor.h"
ULTRASONIC_CC = ROOT / "firmware/stm32/src/sensors/ultrasonic_sensor.cpp"
ROBOT_CONFIG_H = ROOT / "firmware/stm32/include/robot_config.h"

CALIBRATION_HEADER = struct.Struct("<I H H I f f f H H")
CALIBRATION_RECORD = struct.Struct("<I H H I f f f H H I")
CALIBRATION_MAX_SAMPLES = 8
CALIBRATION_A = 0x0803F000
CALIBRATION_B = 0x0803F800
FLASH_END = 0x08040000
FLASH_PAGE = 0x800


def calibration_crc(fields):
    payload = CALIBRATION_HEADER.pack(*fields[:-1])
    return zlib.crc32(payload) & 0xFFFFFFFF


def make_record(generation, left, right, track, straight, turn,
                crc_ok=True, geometry_ok=True):
    if not geometry_ok:
        left = float("nan")
    fields = (0x5743414C, 2, CALIBRATION_RECORD.size, generation,
              left, right, track, straight, turn, 0)
    crc = calibration_crc(fields)
    if not crc_ok:
        crc ^= 1
    return fields[:-1] + (crc,)


def record_geometry_valid(record):
    _, version, size, _, left, right, track, straight, turn, crc = record
    return (version == 2 and size == CALIBRATION_RECORD.size and
            math.isfinite(left) and math.isfinite(right) and
            math.isfinite(track) and 0.02 <= left <= 0.20 and
            0.02 <= right <= 0.20 and 100.0 <= track <= 500.0 and
            straight <= CALIBRATION_MAX_SAMPLES and
            turn <= CALIBRATION_MAX_SAMPLES and
            crc == calibration_crc(record))


def generation_newer(candidate, current):
    delta = (candidate - current) & 0xFFFFFFFF
    return candidate != current and delta < 0x80000000


def select_record(record_a, record_b):
    valid_a = record_a is not None and record_geometry_valid(record_a)
    valid_b = record_b is not None and record_geometry_valid(record_b)
    if not valid_a and not valid_b:
        return None, "NOMINAL"
    if valid_a and valid_b:
        return ((record_b, "B") if generation_newer(record_b[3], record_a[3])
                else (record_a, "A"))
    return (record_a, "A") if valid_a else (record_b, "B")


class CalibrationSessionModel:
    """Small host model for the safety/persistence contract, not the formula."""

    def __init__(self, active=(0.06, 0.06, 250.0), persisted=False,
                 source="NOMINAL", generation=0, committed=(0, 0)):
        self.active = active
        self.candidate = active
        self.persisted = persisted
        self.source = source
        self.generation = generation
        self.committed = committed
        self.session_straight = 0
        self.session_turn = 0
        self.flash_writes = 0

    def straight_sample(self, left, right):
        if self.session_straight >= CALIBRATION_MAX_SAMPLES:
            return False
        if self.session_straight == 0:
            self.candidate = (left, right, self.candidate[2])
        else:
            count = self.session_straight
            self.candidate = (
                (self.candidate[0] * count + left) / (count + 1),
                (self.candidate[1] * count + right) / (count + 1),
                self.candidate[2],
            )
        self.session_straight += 1
        return True

    def turn_sample(self, track):
        if self.session_turn >= CALIBRATION_MAX_SAMPLES:
            return False
        if self.session_turn == 0:
            self.candidate = (self.candidate[0], self.candidate[1], track)
        else:
            count = self.session_turn
            self.candidate = (
                self.candidate[0], self.candidate[1],
                (self.candidate[2] * count + track) / (count + 1),
            )
        self.session_turn += 1
        return True

    def abort(self):
        self.candidate = self.active
        self.session_straight = 0
        self.session_turn = 0

    def commit(self, records, write_ok=True, readback_ok=True):
        if self.session_straight == 0:
            return False, "NO_STRAIGHT_THIS_SESSION"
        if self.session_turn == 0:
            return False, "NO_TURN_THIS_SESSION"
        active_record, source = select_record(*records)
        previous_generation = active_record[3] if active_record else 0
        target = "B" if source == "A" else "A"
        if active_record is None:
            target = "A"
        next_generation = (previous_generation + 1) & 0xFFFFFFFF
        if not write_ok or not readback_ok:
            return False, "FLASH"
        self.active = self.candidate
        self.candidate = self.active
        self.committed = (
            min(CALIBRATION_MAX_SAMPLES,
                self.committed[0] + self.session_straight),
            min(CALIBRATION_MAX_SAMPLES,
                self.committed[1] + self.session_turn),
        )
        self.session_straight = 0
        self.session_turn = 0
        self.persisted = True
        self.source = target
        self.generation = next_generation
        self.flash_writes += 1
        return True, target


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

    def test_boot_persisted_record_starts_a_fresh_session(self):
        model = CalibrationSessionModel(persisted=True, source="A",
                                         generation=7, committed=(8, 6))
        self.assertEqual((model.session_straight, model.session_turn), (0, 0))
        self.assertEqual(model.committed, (8, 6))

    def test_commit_requires_a_new_straight_sample(self):
        model = CalibrationSessionModel(persisted=True, committed=(8, 8))
        model.turn_sample(250.0)
        ok, error = model.commit((None, None))
        self.assertFalse(ok)
        self.assertEqual(error, "NO_STRAIGHT_THIS_SESSION")

    def test_commit_requires_a_new_turn_sample(self):
        model = CalibrationSessionModel(persisted=True, committed=(8, 8))
        model.straight_sample(0.061, 0.062)
        ok, error = model.commit((None, None))
        self.assertFalse(ok)
        self.assertEqual(error, "NO_TURN_THIS_SESSION")

    def test_commit_accepts_one_new_sample_of_each_kind(self):
        model = CalibrationSessionModel(persisted=True, committed=(8, 8))
        model.straight_sample(0.061, 0.062)
        model.turn_sample(251.0)
        ok, _ = model.commit((None, None))
        self.assertTrue(ok)
        self.assertEqual(model.committed, (8, 8))
        self.assertEqual((model.session_straight, model.session_turn), (0, 0))

    def test_first_session_sample_does_not_average_persisted_geometry(self):
        model = CalibrationSessionModel(active=(0.05, 0.05, 220.0),
                                        persisted=True, committed=(8, 8))
        model.straight_sample(0.075, 0.085)
        self.assertEqual(model.candidate[:2], (0.075, 0.085))
        model.turn_sample(275.0)
        self.assertEqual(model.candidate[2], 275.0)

    def test_second_session_sample_averages_only_session_samples(self):
        model = CalibrationSessionModel(active=(0.05, 0.05, 220.0),
                                        persisted=True, committed=(8, 8))
        model.straight_sample(0.075, 0.085)
        model.straight_sample(0.085, 0.095)
        model.turn_sample(275.0)
        model.turn_sample(285.0)
        self.assertAlmostEqual(model.candidate[0], 0.080)
        self.assertAlmostEqual(model.candidate[1], 0.090)
        self.assertAlmostEqual(model.candidate[2], 280.0)

    def test_session_sample_limit_is_eight(self):
        model = CalibrationSessionModel()
        for _ in range(CALIBRATION_MAX_SAMPLES):
            self.assertTrue(model.straight_sample(0.06, 0.06))
            self.assertTrue(model.turn_sample(250.0))
        self.assertFalse(model.straight_sample(0.06, 0.06))
        self.assertFalse(model.turn_sample(250.0))

    def test_abort_discards_candidate_and_session_only(self):
        model = CalibrationSessionModel(active=(0.061, 0.062, 251.0),
                                        persisted=True, source="B",
                                        generation=9, committed=(4, 4))
        model.straight_sample(0.075, 0.085)
        model.turn_sample(280.0)
        model.abort()
        self.assertEqual(model.candidate, model.active)
        self.assertEqual((model.session_straight, model.session_turn), (0, 0))
        self.assertEqual((model.source, model.generation, model.committed),
                         ("B", 9, (4, 4)))
        self.assertEqual(model.flash_writes, 0)

    def test_ab_both_invalid_uses_nominal_fallback(self):
        selected, source = select_record(None, None)
        self.assertIsNone(selected)
        self.assertEqual(source, "NOMINAL")

    def test_ab_selects_the_only_valid_record(self):
        record_a = make_record(3, 0.061, 0.062, 251.0, 4, 4)
        record_b = make_record(4, 0.061, 0.062, 252.0, 5, 5,
                               crc_ok=False)
        selected, source = select_record(record_a, record_b)
        self.assertEqual(selected, record_a)
        self.assertEqual(source, "A")
        selected, source = select_record(record_b, record_a)
        self.assertEqual(selected, record_a)
        self.assertEqual(source, "B")

    def test_ab_selects_newest_generation(self):
        record_a = make_record(10, 0.061, 0.062, 251.0, 4, 4)
        record_b = make_record(11, 0.061, 0.062, 252.0, 5, 5)
        selected, source = select_record(record_a, record_b)
        self.assertEqual(selected, record_b)
        self.assertEqual(source, "B")

    def test_ab_generation_compare_is_wrap_safe(self):
        record_a = make_record(0xFFFFFFFF, 0.061, 0.062, 251.0, 4, 4)
        record_b = make_record(0, 0.061, 0.062, 252.0, 5, 5)
        selected, source = select_record(record_a, record_b)
        self.assertEqual(selected, record_b)
        self.assertEqual(source, "B")

    def test_crc_covers_generation_and_record_fields(self):
        record = make_record(7, 0.061, 0.062, 251.0, 4, 4)
        self.assertTrue(record_geometry_valid(record))
        changed = list(record)
        changed[3] = 8
        changed[-1] = record[-1]
        self.assertFalse(record_geometry_valid(tuple(changed)))

    def test_commit_targets_inactive_page_and_increments_generation(self):
        model = CalibrationSessionModel(persisted=True, source="A",
                                         generation=12, committed=(4, 4))
        model.straight_sample(0.061, 0.062)
        model.turn_sample(251.0)
        ok, target = model.commit((make_record(12, 0.06, 0.06, 250.0, 4, 4),
                                   None))
        self.assertTrue(ok)
        self.assertEqual(target, "B")
        self.assertEqual(model.generation, 13)

    def test_commit_with_no_records_starts_at_a_generation_one(self):
        model = CalibrationSessionModel()
        model.straight_sample(0.061, 0.062)
        model.turn_sample(251.0)
        ok, target = model.commit((None, None))
        self.assertTrue(ok)
        self.assertEqual((target, model.generation), ("A", 1))

    def test_failed_write_keeps_active_state_selectable(self):
        model = CalibrationSessionModel(active=(0.061, 0.062, 251.0),
                                         persisted=True, source="A",
                                         generation=12, committed=(4, 4))
        original = (model.active, model.candidate, model.generation,
                    model.source, model.committed)
        model.straight_sample(0.07, 0.071)
        model.turn_sample(260.0)
        ok, error = model.commit((make_record(12, 0.061, 0.062, 251.0, 4, 4),
                                  None), write_ok=False)
        self.assertFalse(ok)
        self.assertEqual(error, "FLASH")
        self.assertEqual((model.active, model.generation, model.source,
                          model.committed),
                         (original[0], original[2], original[3], original[4]))

    def test_readback_failure_keeps_active_state_selectable(self):
        model = CalibrationSessionModel(persisted=True, source="B",
                                         generation=20, committed=(4, 4))
        model.straight_sample(0.07, 0.071)
        model.turn_sample(260.0)
        ok, error = model.commit((None,
                                  make_record(20, 0.061, 0.062, 251.0, 4, 4)),
                                 readback_ok=False)
        self.assertFalse(ok)
        self.assertEqual(error, "FLASH")
        self.assertEqual((model.source, model.generation), ("B", 20))

    def test_layout_and_production_source_contract(self):
        layout = FLASH_LAYOUT_H.read_text(encoding="utf-8")
        odometry = ODOMETRY_CC.read_text(encoding="utf-8")
        header = ODOMETRY_H.read_text(encoding="utf-8")
        map_tests = MAP_HOST_TESTS.read_text(encoding="utf-8")
        self.assertIn("kCalibrationA = 0x0803F000UL", layout)
        self.assertIn("kCalibrationB = 0x0803F800UL", layout)
        self.assertIn("kCalibrationB + kFlashPageSize == kFlashEnd", layout)
        self.assertIn("kCalibrationVersion = 2U", odometry)
        self.assertIn("uint32_t generation", odometry)
        self.assertIn("readback", odometry)
        self.assertIn("calibrationValuesValid(readback.leftMmPerTick", odometry)
        self.assertIn("NO_STRAIGHT_THIS_SESSION", odometry)
        self.assertIn("NO_TURN_THIS_SESSION", odometry)
        self.assertIn("committedStraightSamples", header)
        self.assertIn("kCalibrationA", map_tests)
        self.assertNotIn("kCalibrationPage", layout + odometry)

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
        self.assertIn("c.consecutiveTimeouts<=ULTRASONIC_DEGRADED_MAX_TIMEOUTS", source)
        self.assertIn("if(!degradedClearWindow(millis()))return 0", source)
        self.assertIn("return min(cmd,ULTRASONIC_DEGRADED_MAX_FORWARD_COMMAND)", source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
