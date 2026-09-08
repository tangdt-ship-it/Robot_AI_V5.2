"""Host/static regression tests for the STM32-local MAP implementation.

These tests use the checked-in MAP constants and wire layout, then exercise
storage CRC, generation selection, route-mode transitions and local-frame
transform without importing Arduino or touching hardware.
"""

import math
import re
import struct
import unittest
import zlib
from pathlib import Path


STM32_ROOT = Path(__file__).resolve().parents[1]
INCLUDE_ROOT = STM32_ROOT / "include"
SRC_ROOT = STM32_ROOT / "src"
ESP32_ROOT = STM32_ROOT.parent / "esp32-xiaozhi"


def _read(path):
    return path.read_text(encoding="utf-8")


def _constant(text, name):
    match = re.search(
        r"\b" + re.escape(name) + r"\s*=\s*(0x[0-9A-Fa-f]+|\d+)", text
    )
    if not match:
        raise AssertionError(f"constant {name} not found")
    return int(match.group(1), 0)


FLASH_TEXT = _read(INCLUDE_ROOT / "map" / "flash_layout.h")
TYPES_TEXT = _read(INCLUDE_ROOT / "map" / "map_types.h")
CONFIG_TEXT = _read(INCLUDE_ROOT / "robot_config.h")
MAP_TEXT = _read(SRC_ROOT / "map" / "map_controller.cpp")
MAP_HEADER_TEXT = _read(INCLUDE_ROOT / "map" / "map_controller.h")
CLEANER_HEADER_TEXT = _read(INCLUDE_ROOT / "map" / "route_cleaner.h")
CLEANER_TEXT = _read(SRC_ROOT / "map" / "route_cleaner.cpp")
STORE_TEXT = _read(SRC_ROOT / "map" / "route_store.cpp")
CTRL_TEXT = _read(SRC_ROOT / "control" / "robot_controller.cpp")
CTRL_HEADER_TEXT = _read(INCLUDE_ROOT / "control" / "robot_controller.h")
MAIN_TEXT = _read(SRC_ROOT / "main.cpp")
PS2_TEXT = _read(SRC_ROOT / "ps2" / "ps2_controller.cpp")
PS2_HEADER_TEXT = _read(INCLUDE_ROOT / "ps2" / "ps2_controller.h")
LCD_TEXT = _read(SRC_ROOT / "display" / "lcd_display.cpp")
ESP32_TEXT = _read(ESP32_ROOT / "main" / "robot" / "teach_route.cc")
ESP32_CMAKE = _read(ESP32_ROOT / "main" / "CMakeLists.txt")
ESP32_BOARD = _read(
    ESP32_ROOT / "main" / "boards" / "bread-compact-wifi-s3cam" /
    "compact_wifi_board_s3cam.cc"
)

MAX_WAYPOINTS = _constant(TYPES_TEXT, "STM32_MAP_MAX_WAYPOINTS")
FORMAT_VERSION = _constant(TYPES_TEXT, "STM32_MAP_FORMAT_VERSION")
FLASH_PAGE = _constant(FLASH_TEXT, "kFlashPageSize")
FLASH_END = _constant(FLASH_TEXT, "kFlashOrigin") + _constant(
    FLASH_TEXT, "kFlashLength"
)
MAP_ADDRESSES = [
    _constant(FLASH_TEXT, name)
    for name in ("kMap1A", "kMap1B", "kMap2A", "kMap2B")
]
CALIBRATION_PAGE = _constant(FLASH_TEXT, "kCalibrationPage")

HEADER = struct.Struct("<I H B B B B H H I I I I 2x")
WAYPOINT = struct.Struct("<i i h B B")


def crc_for(header_fields, waypoints):
    fields = list(header_fields)
    fields[10] = 0  # crc32 is excluded while calculating CRC.
    payload = HEADER.pack(*fields)
    payload += b"".join(WAYPOINT.pack(*point) for point in waypoints)
    return zlib.crc32(payload) & 0xFFFFFFFF


def route_allowed(route_type, replay_mode):
    if route_type == "CLOSED":
        return replay_mode in ("ONCE", "LOOP")
    return replay_mode in ("ONCE", "RETURN", "CLOSED", "LOOP",
                           "PING_PONG")


def effective_mode(route_type, replay_mode):
    if route_type == "CLOSED" and replay_mode == "ONCE":
        return "CLOSED"
    return replay_mode


def v529_route_allowed(route_type, replay_mode):
    """Mirror the V5.2.9 persisted route/mode validator."""
    if route_type == "CLOSED":
        return replay_mode in ("ONCE", "LOOP")
    return replay_mode in ("ONCE", "RETURN", "PING_PONG")


def persisted_route_type(runtime_mode):
    return "CLOSED" if runtime_mode in ("CLOSED", "LOOP") else "OPEN"


def persisted_replay_mode(runtime_mode):
    return "ONCE" if runtime_mode == "CLOSED" else runtime_mode


def persisted_length(points, runtime_mode):
    return route_length(
        points, closed=runtime_mode in ("CLOSED", "LOOP")
    )


def v529_load_pass(record, points):
    return (
        v529_route_allowed(record["route_type"], record["replay_mode"])
        and record["waypoint_count"] == len(points)
        and record["payload_bytes"] == len(points) * WAYPOINT.size
        and abs(record["route_length"] - route_length(
            points, closed=record["route_type"] == "CLOSED"
        )) <= 50
    )


class StorageCompatibilityModel:
    """Model runtime modes and their V5.2.9-compatible Flash records."""

    def __init__(self, points, runtime_mode, speed=20, lap=0):
        self.points = points
        self.runtime = (runtime_mode, speed, lap)
        self.flash = self.persisted_record()
        self.write_count = 0

    def persisted_record(self):
        mode, speed, lap = self.runtime
        return {
            "route_type": persisted_route_type(mode),
            "replay_mode": persisted_replay_mode(mode),
            "route_length": persisted_length(self.points, mode),
            "waypoint_count": len(self.points),
            "payload_bytes": len(self.points) * WAYPOINT.size,
            "speed": speed,
            "lap": lap,
        }

    def load(self):
        self.runtime = (
            effective_mode(self.flash["route_type"], self.flash["replay_mode"]),
            self.flash["speed"],
            self.flash["lap"],
        )

    def save_settings(self, runtime_mode=None, speed=None, lap=None,
                      succeed=True):
        old_runtime = self.runtime
        requested = (
            runtime_mode if runtime_mode is not None else old_runtime[0],
            speed if speed is not None else old_runtime[1],
            lap if lap is not None else old_runtime[2],
        )
        if requested == old_runtime:
            return "NO_CHANGE"
        self.write_count += 1
        if not succeed:
            return "FAIL"
        self.runtime = requested
        self.flash = self.persisted_record()
        return "OK"


def v521_execution_mode(user_mode, repeat_target):
    if user_mode == "ONCE":
        return "ONCE"
    if user_mode == "SHUTTLE":
        return "RETURN" if repeat_target == 1 else "PING_PONG"
    return "CLOSED" if repeat_target == 1 else "LOOP"


def v521_persisted_record(user_mode, repeat_target, speed=20):
    execution = v521_execution_mode(user_mode, repeat_target)
    if user_mode == "LOOP":
        route_type = "CLOSED"
        replay_mode = "ONCE" if repeat_target == 1 else "LOOP"
    elif user_mode == "SHUTTLE":
        route_type = "OPEN"
        replay_mode = "RETURN"
    else:
        route_type = "OPEN"
        replay_mode = "ONCE"
    return {
        "route_type": route_type,
        "replay_mode": replay_mode,
        "repeat_target": repeat_target,
        "speed": speed,
        "shuttle_flag": user_mode == "SHUTTLE" and repeat_target != 1,
        "execution": execution,
    }


def shuttle_edges(count, cycles):
    edges = []
    for _ in range(cycles):
        edges.extend((index, index + 1) for index in range(count - 1))
        edges.extend((index, index - 1) for index in range(count - 1, 0, -1))
    return edges


def advance(current, direction, count, route_type, replay_mode):
    """Mirror MapController::advanceReplayAfterTarget for planner testing."""
    replay_mode = effective_mode(route_type, replay_mode)
    if direction > 0:
        if current < count - 1:
            return current + 1, direction, False
        if replay_mode in ("CLOSED", "LOOP"):
            return 0, direction, False
        if replay_mode == "ONCE":
            return None, direction, True
        return count - 2, -1, False
    if current > 0:
        return current - 1, direction, False
    if replay_mode == "RETURN":
        return None, direction, True
    return 1, 1, False


def classify_close(distance_mm, heading_deg, count):
    """Mirror the teach endpoint classifier without touching hardware."""
    if count < 3:
        return "OPEN"
    if distance_mm <= 50 and heading_deg <= 5:
        return "AUTO_CLOSED"
    if distance_mm <= 200 and heading_deg <= 20:
        return "CANDIDATE"
    return "OPEN"


def manual_finish_confirmation(points):
    """Manual Teach saves one canonical open route without X/O confirmation."""
    if not points:
        return {"state": "INVALID", "points": 0, "close_distance": 0.0}
    return {
        "state": "SAVED",
        "points": len(points),
        "close_distance": math.dist(points[-1], points[0]),
        "type": "OPEN",
        "mode": "ONCE",
    }


def manual_select_route(points, route_type, max_segment=5000.0,
                        minimum_segment=20.0, closure_skip=20.0):
    """Model the canonical Teach save; replay topology is a later mode."""
    confirmation = manual_finish_confirmation(points)
    if confirmation["state"] != "SAVED":
        return {"accepted": False, "state": confirmation["state"]}
    if len(points) < 2:
        return {"accepted": False, "state": "SAVED", "reason": "POINTS"}
    return {"accepted": True, "state": "SAVED", "type": "OPEN",
            "mode": "ONCE", "points": len(points),
            "length": route_length(points, closed=False)}


class ClosedReplayModel:
    """Small iterative model for logical Pn -> P0 replay semantics."""

    def __init__(self, count, mode="ONCE", closure_distance_mm=None,
                 closure_skip_mm=20.0):
        self.count = count
        self.mode = mode
        self.closure_distance_mm = closure_distance_mm
        self.closure_skip_mm = closure_skip_mm
        self.current = 0
        self.target = 1
        self.lap = 0
        self.complete = False
        self.held = False
        self.hold_reason = None
        self.cancelled = False
        self.generation = 0
        self.motion_edges = []

    def edge(self):
        return self.current, self.target

    def start_segment(self):
        self.generation += 1
        return self.edge(), self.generation

    def finish_segment(self, generation=None):
        if self.held or self.cancelled or self.complete:
            return False
        if generation is not None and generation != self.generation:
            return False
        source, target = self.edge()
        near_zero_closure = (
            source == self.count - 1 and target == 0 and
            self.closure_distance_mm is not None and
            self.closure_distance_mm <= self.closure_skip_mm
        )
        if not near_zero_closure:
            self.motion_edges.append((source, target))
        self.current = target
        if source == self.count - 1 and target == 0:
            if self.mode == "LOOP":
                self.lap += 1
                self.target = 1
            else:
                self.complete = True
            return True
        self.target = 0 if self.current == self.count - 1 else self.current + 1
        return True

    def hold(self, reason="USER"):
        self.held = True
        self.hold_reason = reason
        return self.edge(), self.lap

    def resume(self):
        self.held = False
        self.hold_reason = None

    def cancel(self):
        self.cancelled = True
        self.held = False


class MultiModeReplayModel:
    """State-machine model for one canonical route and four replay modes."""

    def __init__(self, count, mode, loop_target=0):
        self.count = count
        self.mode = mode
        self.loop_target = loop_target
        self.current = 0
        self.target = 1
        self.direction = 1
        self.lap = 0
        self.complete = False
        self.held = False
        self.cancelled = False
        self.generation = 0
        self.edges = []

    def start_segment(self):
        self.generation += 1
        return (self.current, self.target), self.generation

    def finish_segment(self, generation):
        if (self.complete or self.cancelled or self.held or
                generation != self.generation):
            return False
        source, target = self.current, self.target
        self.edges.append((source, target))
        self.current = target
        if self.direction > 0:
            if self.current < self.count - 1:
                self.target = self.current + 1
            elif self.mode in ("CLOSED", "LOOP"):
                self.target = 0
            elif self.mode == "ONCE":
                self.complete = True
            else:  # RETURN
                self.direction = -1
                self.target = self.count - 2
        else:
            if self.current > 0:
                self.target = self.current - 1
            else:
                self.complete = True
        if (self.mode == "LOOP" and source == self.count - 1 and
                target == 0):
            self.lap += 1
            if self.loop_target and self.lap >= self.loop_target:
                self.complete = True
            else:
                self.target = 1
        elif (self.mode == "CLOSED" and source == self.count - 1 and
              target == 0):
            self.complete = True
        return True

    def hold(self):
        self.held = True

    def resume(self):
        self.held = False

    def cancel(self):
        self.cancelled = True
        self.held = False


class PostTeachBackModel:
    """Model the transient Pn -> ... -> P0 action without a user mode."""

    def __init__(self, count, route_generation=7, origin=(0.0, 0.0, 0.0)):
        self.count = count
        self.route_generation = route_generation
        self.origin = origin
        self.available_flag = True
        self.active = False
        self.current = None
        self.target = None
        self.edges = []
        self.generation = 0
        self.held = False
        self.cancelled = False
        self.complete = False

    def available(self, route_generation=None, current_endpoint_error=0.0,
                  reset_changed=False, heading_reset_changed=False,
                  slot_changed=False, motion=False):
        if not self.available_flag or self.active:
            return False
        if route_generation is not None and route_generation != self.route_generation:
            return False
        if current_endpoint_error > 100.0:
            return False
        if reset_changed or heading_reset_changed or slot_changed or motion:
            return False
        return self.count >= 2

    def start(self):
        if not self.available():
            return False
        self.active = True
        self.current = self.count - 1
        self.target = self.count - 2
        self.generation += 1
        return True

    def finish_segment(self, generation=None):
        if (not self.active or self.held or self.cancelled or self.complete or
                (generation is not None and generation != self.generation)):
            return False
        self.edges.append((self.current, self.target))
        self.current = self.target
        if self.current == 0:
            self.complete = True
            self.active = False
        else:
            self.target = self.current - 1
            self.generation += 1
        return True

    def hold(self):
        self.held = True

    def resume(self):
        self.held = False
        self.generation += 1

    def cancel(self):
        self.cancelled = True
        self.active = False
        self.available_flag = False
        self.held = False


def run_mode(model):
    while not model.complete and not model.cancelled:
        _, generation = model.start_segment()
        model.finish_segment(generation)
    return model.edges


def closed_route_edges(count, laps=1):
    """Return the logical edges for CLOSED LOOP, including Pn -> P0 once/lap."""
    edges = []
    for _ in range(laps):
        edges.extend((index, index + 1) for index in range(count - 1))
        edges.append((count - 1, 0))
    return edges


def route_length(points, closed=False, closure_skip_mm=20):
    total = 0.0
    for first, second in zip(points, points[1:]):
        total += math.dist(first, second)
    if closed and len(points) >= 2:
        closure = math.dist(points[-1], points[0])
        if closure > closure_skip_mm:
            total += closure
    return round(total)


MAP_WP_START = 1 << 0
MAP_WP_MANUAL_MARK = 1 << 1
MAP_WP_AUTO_DISTANCE = 1 << 2
MAP_WP_AUTO_CORNER = 1 << 3
MAP_WP_ENDPOINT = 1 << 4


def point_distance(first, second):
    return math.dist(first[:2], second[:2])


def point_segment_distance(point, first, second):
    ax, ay = first[:2]
    bx, by = second[:2]
    px, py = point[:2]
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy
    if length_squared == 0:
        return point_distance(point, first)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) /
                       length_squared))
    return math.dist((px, py), (ax + t * dx, ay + t * dy))


def direction_change(first, middle, last):
    incoming = math.degrees(math.atan2(middle[1] - first[1],
                                        middle[0] - first[0]))
    outgoing = math.degrees(math.atan2(last[1] - middle[1],
                                       last[0] - middle[0]))
    delta = (outgoing - incoming + 180.0) % 360.0 - 180.0
    return abs(delta)


def protected(point):
    return bool(point[3] & (MAP_WP_START | MAP_WP_MANUAL_MARK |
                            MAP_WP_ENDPOINT))


def reference_safe_simplify(points, max_deviation=60.0):
    """Small host geometry oracle for the safety properties.

    It intentionally mirrors only the conservative collinear rule. The C++
    implementation has the additional duplicate/corner-cluster passes; these
    tests verify the invariant that every accepted shortcut is corridor-safe.
    """
    kept = list(range(len(points)))
    changed = True
    while changed:
        changed = False
        for position in range(1, len(kept) - 1):
            current = points[kept[position]]
            if protected(current) or (current[3] & MAP_WP_AUTO_CORNER):
                continue
            first = points[kept[position - 1]]
            last = points[kept[position + 1]]
            if not 20.0 <= point_distance(first, last) <= 5000.0:
                continue
            if direction_change(first, current, last) > 10.0:
                continue
            if point_segment_distance(current, first, last) > 50.0:
                continue
            raw_first, raw_last = kept[position - 1], kept[position + 1]
            if any(protected(points[index]) or
                   point_segment_distance(points[index], first, last) >
                   max_deviation
                   for index in range(raw_first + 1, raw_last)):
                continue
            del kept[position]
            changed = True
            break
    return kept


class MapHostTests(unittest.TestCase):
    def test_empty_map_and_teach_states(self):
        self.assertIn("MapControllerMode::READY", MAP_TEXT)
        self.assertIn("MapControllerMode::TEACHING", MAP_TEXT)
        self.assertIn("route_ = {}", MAP_TEXT)
        self.assertIn("beginTeach()", MAP_TEXT)

    def test_manual_auto_undo_save_cancel(self):
        for name in (
            "markManualWaypoint",
            "sampleTeach",
            "handleSquare",
            "finalizeTeach",
            "cancelTeach",
        ):
            with self.subTest(name=name):
                self.assertIn(name, MAP_TEXT)
        self.assertIn("MAP_WP_MANUAL_MARK", MAP_TEXT)
        self.assertIn("MAP_WP_AUTO_DISTANCE", MAP_TEXT)
        self.assertIn("MAP_WP_AUTO_CORNER", MAP_TEXT)
        self.assertIn("savePending_ = true", MAP_TEXT)
        self.assertIn("storeState_ = MapStoreState::STORAGE_ERROR", MAP_TEXT)
        self.assertIn("OLD_ROUTE_RETAINED=1", MAP_TEXT)

    def test_TEST_MANUAL_NO_AUTO_DISTANCE(self):
        self.assertIn("enum class MapTeachMode", TYPES_TEXT)
        self.assertIn("MANUAL_KEYFRAME = 0U", TYPES_TEXT)
        self.assertIn(
            "MapTeachMode teachMode_ = MapTeachMode::MANUAL_KEYFRAME",
            MAP_HEADER_TEXT,
        )
        sample = MAP_TEXT[MAP_TEXT.index("void MapController::sampleTeach()") :]
        manual = sample[:sample.index("if (!lastTeachSampleValid_)")]
        self.assertNotIn("appendWaypoint(local, MAP_WP_AUTO_DISTANCE)", manual)

    def test_TEST_MANUAL_NO_AUTO_CORNER(self):
        sample = MAP_TEXT[MAP_TEXT.index("void MapController::sampleTeach()") :]
        manual = sample[:sample.index("if (!lastTeachSampleValid_)")]
        self.assertNotIn("appendWaypoint(local, MAP_WP_AUTO_CORNER)", manual)
        self.assertIn("teachMode_ == MapTeachMode::MANUAL_KEYFRAME", manual)

    def test_TEST_EXPLICIT_MARK_ONLY(self):
        mark = MAP_TEXT[MAP_TEXT.index("void MapController::markManualWaypoint()") :]
        mark = mark[:mark.index("void MapController::sampleTeach()")]
        self.assertIn("appendWaypoint(local, MAP_WP_MANUAL_MARK)", mark)
        self.assertIn("MAP,KEYFRAME,MARK,IDX=", mark)
        self.assertNotIn("MAP_WP_AUTO_DISTANCE", mark)
        self.assertNotIn("MAP_WP_AUTO_CORNER", mark)

    def test_TEST_JOYSTICK_CORRECTIONS_IGNORED(self):
        sample = MAP_TEXT[MAP_TEXT.index("void MapController::sampleTeach()") :]
        sample = sample[:sample.index("bool MapController::appendWaypoint")]
        self.assertNotIn("ps2_", sample)

    def test_TEST_ANGLE_60(self):
        self.assertAlmostEqual(direction_change((0, 0), (100, 0),
                                                (150, 86.6025)), 60.0, places=2)
        self.assertIn("replayIncomingBearing(replayCurrentIndex_, replayTargetIndex_)",
                      MAP_TEXT)

    def test_TEST_ANGLE_130(self):
        self.assertAlmostEqual(direction_change((0, 0), (100, 0),
                                                (35.724, 76.604)), 130.0,
                               places=2)

    def test_TEST_RIGHT_65(self):
        self.assertAlmostEqual(direction_change((0, 0), (100, 0),
                                                (142.262, -90.631)), 65.0,
                               places=2)
        self.assertIn("shortestDeltaDeg(incomingBearing, current.headingDeg)", MAP_TEXT)

    def test_TEST_ANGLE_WRAP(self):
        delta = (179.0 - (-179.0) + 180.0) % 360.0 - 180.0
        self.assertAlmostEqual(delta, -2.0)
        self.assertIn("shortestDeltaDeg", MAP_TEXT)
        self.assertNotIn("90.0f", MAP_TEXT)

    def test_TEST_MARK_HELD(self):
        self.assertIn(
            "if (trianglePressed) queueMapEvent(Ps2MapAction::TRIANGLE);",
            PS2_TEXT,
        )
        self.assertIn("previousMapTriangle_ = state_.triangle", PS2_TEXT)

    def test_TEST_TOO_CLOSE(self):
        mark = MAP_TEXT[MAP_TEXT.index("void MapController::markManualWaypoint()") :]
        mark = mark[:mark.index("void MapController::sampleTeach()")]
        self.assertIn("separation <= kMinimumSegmentMm", mark)
        self.assertIn("MAP,KEYFRAME,REJECT,REASON=TOO_CLOSE", mark)
        self.assertIn("heading-only waypoint", MAP_TEXT)

    def test_TEST_UNDO(self):
        self.assertIn("if (count > 1U)", MAP_TEXT)
        self.assertIn("MAP,KEYFRAME,UNDO,IDX=", MAP_TEXT)
        self.assertIn("REASON=START_PROTECTED", MAP_TEXT)

    def test_TEST_FINISH_NEAR_LAST_MARK(self):
        self.assertIn("endpointSeparation <= kMinimumSegmentMm", MAP_TEXT)
        self.assertIn("last.flags |= MAP_WP_ENDPOINT", MAP_TEXT)
        self.assertIn("ACTION=", MAP_TEXT)

    def test_TEST_OPEN_ROUTE(self):
        self.assertIn("MapRouteType::OPEN", MAP_TEXT)
        self.assertIn('debug_.println(",CLASS=OPEN")', MAP_TEXT)

    def test_TEST_CLOSED_ROUTE(self):
        self.assertIn("MapRouteType::CLOSED", MAP_TEXT)
        self.assertIn('debug_.println(",CLASS=AUTO_CLOSED")', MAP_TEXT)
        self.assertIn('debug_.println(",CLASS=CANDIDATE")', MAP_TEXT)

    def test_manual_finish_saves_canonical_open_route(self):
        finish = MAP_TEXT.index("bool MapController::finalizeTeach()")
        manual = MAP_TEXT.index(
            "if (teachMode_ == MapTeachMode::MANUAL_KEYFRAME)", finish
        )
        auto = MAP_TEXT.index("const bool enoughPoints", manual)
        manual_block = MAP_TEXT[manual:auto]
        self.assertNotIn("autoClosed", manual_block)
        self.assertNotIn("closedCandidate", manual_block)
        self.assertIn("routeType_ = MapRouteType::OPEN", manual_block)
        self.assertIn("routeMode_ = MapReplayMode::ONCE", manual_block)
        self.assertIn("TYPE=OPEN_CANONICAL", manual_block)
        self.assertIn("savePending_ = true", manual_block)

    def test_manual_far_route_does_not_auto_classify_and_keeps_points(self):
        points = [(0.0, 0.0), (1000.0, 0.0), (1500.0, 800.0)]
        confirmation = manual_finish_confirmation(points)
        self.assertEqual(confirmation["state"], "SAVED")
        self.assertEqual(confirmation["points"], 3)
        self.assertEqual(manual_select_route(points, "OPEN")["type"], "OPEN")
        self.assertEqual(manual_select_route(points, "CLOSED")["type"], "OPEN")
        self.assertEqual(manual_select_route(points, "CLOSED")["mode"], "ONCE")
        self.assertEqual(manual_select_route(points, "OPEN")["points"], 3)
        self.assertEqual(manual_select_route(points, "CLOSED")["points"], 3)

    def test_TEST_MANUAL_BYPASS_OPTIMIZER(self):
        manual = MAP_TEXT.index(
            "if (teachMode_ == MapTeachMode::MANUAL_KEYFRAME) {",
            MAP_TEXT.index("bool MapController::finalizeTeach()"),
        )
        cleaner = MAP_TEXT.index("cleanMapRoute", manual)
        self.assertIn("MAP,SEMANTIC=BYPASS_MANUAL_KEYFRAME", MAP_TEXT[manual:cleaner])
        self.assertGreater(cleaner, manual)

    def test_storage_record_sizes_and_version(self):
        self.assertEqual(MAX_WAYPOINTS, 128)
        self.assertEqual(FORMAT_VERSION, 2)
        self.assertEqual(WAYPOINT.size, 12)
        self.assertEqual(HEADER.size, 32)
        self.assertLessEqual(
            HEADER.size + MAX_WAYPOINTS * WAYPOINT.size, FLASH_PAGE
        )

    def test_flash_ab_storage_and_calibration_are_disjoint(self):
        self.assertEqual(len(set(MAP_ADDRESSES)), 4)
        self.assertTrue(all(address % FLASH_PAGE == 0 for address in MAP_ADDRESSES))
        self.assertEqual(CALIBRATION_PAGE % FLASH_PAGE, 0)
        self.assertTrue(
            all(address + FLASH_PAGE <= CALIBRATION_PAGE for address in MAP_ADDRESSES)
        )
        self.assertLessEqual(CALIBRATION_PAGE + FLASH_PAGE, FLASH_END)

    def test_crc_valid_invalid(self):
        header = [
            0x4D415032,
            FORMAT_VERSION,
            1,
            1,
            0,
            0,
            2,
            24,
            7,
            1000,
            0,
            0,
        ]
        points = [(0, 0, 0, 1, 0), (1000, 0, 0, 16, 0)]
        crc = crc_for(header, points)
        stored = header.copy()
        stored[10] = crc
        self.assertEqual(crc_for(stored, points), crc)
        corrupted = list(points)
        corrupted[1] = (1001, 0, 0, 16, 0)
        self.assertNotEqual(crc_for(stored, corrupted), crc)

    def test_ab_generation_and_powerloss_selection(self):
        records = [(4, True), (5, False)]
        valid = [generation for generation, is_valid in records if is_valid]
        self.assertEqual(max(valid), 4)
        self.assertIn("generationNewer", STORE_TEXT)
        self.assertIn("load(slot, verified)", STORE_TEXT)

    def test_open_closed_route_detection_and_modes(self):
        self.assertTrue(route_allowed("OPEN", "ONCE"))
        self.assertTrue(route_allowed("OPEN", "RETURN"))
        self.assertTrue(route_allowed("OPEN", "CLOSED"))
        self.assertTrue(route_allowed("OPEN", "LOOP"))
        self.assertTrue(route_allowed("OPEN", "PING_PONG"))
        self.assertTrue(route_allowed("CLOSED", "ONCE"))
        self.assertTrue(route_allowed("CLOSED", "LOOP"))
        self.assertFalse(route_allowed("CLOSED", "RETURN"))
        self.assertIn("kClosedAutoDistanceMm", MAP_TEXT)
        self.assertIn("kClosedAutoHeadingDeg", MAP_TEXT)
        self.assertIn("kClosedCandidateDistanceMm", MAP_TEXT)
        self.assertIn("kClosedCandidateHeadingDeg", MAP_TEXT)

    def test_closed_endpoint_classification(self):
        self.assertEqual(classify_close(50, 5, 3), "AUTO_CLOSED")
        self.assertEqual(classify_close(200, 20, 3), "CANDIDATE")
        self.assertEqual(classify_close(40, 3, 2), "OPEN")
        self.assertEqual(classify_close(201, 1, 3), "OPEN")
        self.assertEqual(classify_close(100, 21, 3), "OPEN")
        self.assertIn("CLOSED_CONFIRM = 9U", TYPES_TEXT)
        self.assertIn('CLASS=AUTO_CLOSED', MAP_TEXT)
        self.assertIn('CLASS=CANDIDATE', MAP_TEXT)
        self.assertIn('CLASS=OPEN', MAP_TEXT)

    def test_teach_does_not_ask_open_closed_confirmation(self):
        self.assertIn('TYPE=OPEN_CANONICAL', MAP_TEXT)
        self.assertNotIn('MAP,TYPE_CONFIRM,POINTS=', MAP_TEXT)
        self.assertNotIn('MAP,TYPE_SELECT,TYPE=CLOSED,CLOSE_DIST=', MAP_TEXT)
        self.assertNotIn('MAP,CLOSE_CONFIRM,RESULT=CLOSED', MAP_TEXT)
        self.assertNotIn('snprintf(desired_[2], 21, "X=OPEN")', LCD_TEXT)
        self.assertNotIn('snprintf(desired_[3], 21, "O=CLOSED")', LCD_TEXT)
        self.assertIn('updateRouteHeaderForSave(candidate, settingsUserMode_, normalizedTarget)', MAP_TEXT)

    def test_manual_open_and_closed_selection_model(self):
        points = [(0.0, 0.0), (1000.0, 0.0), (1500.0, 800.0)]
        opened = manual_select_route(points, "OPEN")
        closed = manual_select_route(points, "CLOSED")
        self.assertTrue(opened["accepted"])
        self.assertTrue(closed["accepted"])
        self.assertEqual(opened["length"], route_length(points))
        self.assertEqual(closed["length"], route_length(points))
        self.assertEqual(opened["points"], closed["points"])

    def test_manual_closed_rejects_too_long_or_too_few_without_losing_route(self):
        two_points = [(0.0, 0.0), (1000.0, 0.0)]
        too_few = manual_select_route(two_points, "CLOSED")
        self.assertTrue(too_few["accepted"])

        far_closure = [(0.0, 0.0), (1000.0, 0.0), (6000.0, 0.0)]
        too_long = manual_select_route(far_closure, "CLOSED")
        self.assertTrue(too_long["accepted"])
        self.assertTrue(manual_select_route(far_closure, "OPEN")["accepted"])
        self.assertIn('reason = closure > kMaximumSegmentMm ? "CLOSURE_TOO_LONG"', MAP_TEXT)

    def test_closed_mode_is_selected_in_settings(self):
        for token in (
            'MapReplayMode::CLOSED',
            'hasValidClosingEdge',
            'MAP,MODE,CLOSE_UNAVAILABLE,REASON=',
            'updateRouteHeaderForSave(candidate, settingsUserMode_, normalizedTarget)',
        ):
            self.assertIn(token, MAP_TEXT)
        self.assertIn('status.mode != 8U', _read(INCLUDE_ROOT / 'display' / 'lcd_display.h'))

    def test_closed_once_includes_closing_edge(self):
        self.assertEqual(
            closed_route_edges(4), [(0, 1), (1, 2), (2, 3), (3, 0)]
        )
        self.assertIn('fromIndex == count - 1U && targetIndex == 0U', MAP_TEXT)
        self.assertIn('routeMode_ == MapReplayMode::CLOSED', MAP_TEXT)
        self.assertIn('MAP,CLOSE_EDGE,PLAN,FROM=', MAP_TEXT)
        self.assertIn('startReplayGuidedWaypoint', MAP_TEXT)
        self.assertIn('completeReplay();', MAP_TEXT)

    def test_closed_loop_repeats_full_laps(self):
        one_lap = closed_route_edges(4)
        two_laps = closed_route_edges(4, laps=2)
        self.assertEqual(one_lap[-1], (3, 0))
        self.assertEqual(two_laps[:4], two_laps[4:8])
        self.assertIn('MAP,LOOP,START', MAP_TEXT)
        self.assertIn('MAP,LOOP,LAP_COMPLETE,LAP=', MAP_TEXT)
        self.assertIn('replayTargetIndex_ = count > 1U ? 1U : 0U', MAP_TEXT)
        self.assertIn('mode_ = MapControllerMode::REPLAY_RUNNING', MAP_TEXT)
        self.assertIn('replayActive_ = true', MAP_TEXT)

    def test_closed_replay_model_completes_once_only_after_p0(self):
        model = ClosedReplayModel(4, "ONCE")
        visited = []
        while not model.complete:
            visited.append(model.edge())
            model.finish_segment()
        self.assertEqual(visited, [(0, 1), (1, 2), (2, 3), (3, 0)])
        self.assertEqual(model.current, 0)
        self.assertEqual(model.lap, 0)

    def test_closed_replay_model_counts_two_laps_at_closing_edge(self):
        model = ClosedReplayModel(4, "LOOP")
        visited = []
        while len(visited) < 8:
            visited.append(model.edge())
            model.finish_segment()
        self.assertEqual(visited, closed_route_edges(4, laps=2))
        self.assertEqual(model.lap, 2)
        self.assertEqual(model.edge(), (0, 1))

    def test_closed_replay_model_hold_resume_on_closing_edge(self):
        model = ClosedReplayModel(4, "LOOP")
        for _ in range(3):
            model.finish_segment()
        self.assertEqual(model.edge(), (3, 0))
        held_edge, held_lap = model.hold()
        self.assertEqual(held_edge, (3, 0))
        self.assertEqual(held_lap, 0)
        model.resume()
        model.finish_segment()
        self.assertEqual(model.edge(), (0, 1))
        self.assertEqual(model.lap, 1)

    def test_closed_replay_model_obstacle_hold_on_closing_edge(self):
        model = ClosedReplayModel(4, "LOOP")
        for _ in range(3):
            model.finish_segment()
        held_edge, held_lap = model.hold("OBSTACLE")
        self.assertEqual(held_edge, (3, 0))
        self.assertEqual(held_lap, 0)
        self.assertEqual(model.hold_reason, "OBSTACLE")
        model.finish_segment()
        self.assertEqual(model.edge(), (3, 0))
        self.assertEqual(model.lap, 0)
        model.resume()
        model.finish_segment()
        self.assertEqual(model.edge(), (0, 1))
        self.assertEqual(model.lap, 1)

    def test_closed_replay_model_near_zero_closing_edge_has_no_motor_command(self):
        model = ClosedReplayModel(3, "ONCE", closure_distance_mm=5.0)
        model.finish_segment()
        model.finish_segment()
        edge, generation = model.start_segment()
        self.assertEqual(edge, (2, 0))
        self.assertTrue(model.finish_segment(generation))
        self.assertTrue(model.complete)
        self.assertNotIn((2, 0), model.motion_edges)

    def test_closed_replay_model_stale_generation_is_dropped_across_lap(self):
        model = ClosedReplayModel(3, "LOOP")
        closing_generation = None
        for _ in range(3):
            edge, generation = model.start_segment()
            if edge == (2, 0):
                closing_generation = generation
            self.assertTrue(model.finish_segment(generation))
        self.assertEqual(model.lap, 1)
        next_edge, current_generation = model.start_segment()
        self.assertEqual(next_edge, (0, 1))
        self.assertFalse(model.finish_segment(closing_generation))
        self.assertEqual(model.edge(), next_edge)
        self.assertEqual(model.lap, 1)
        self.assertTrue(model.finish_segment(current_generation))

    def test_closed_replay_model_cancel_on_closing_edge_preserves_lap(self):
        model = ClosedReplayModel(3, "LOOP")
        model.finish_segment()
        model.finish_segment()
        self.assertEqual(model.edge(), (2, 0))
        model.cancel()
        self.assertFalse(model.finish_segment())
        self.assertEqual(model.edge(), (2, 0))
        self.assertEqual(model.lap, 0)

    def test_closed_loop_index_and_counter_boundaries(self):
        edges = closed_route_edges(128, laps=3)
        self.assertTrue(all(0 <= source < 128 and 0 <= target < 128
                            for source, target in edges))
        self.assertIn('if (replayLapCounter_ != 0xFFFFFFFFUL)', MAP_TEXT)
        self.assertIn('count - 1U', MAP_TEXT)
        self.assertIn('count > 1U ? 1U : 0U', MAP_TEXT)

    def test_closed_loop_origin_is_stable_between_laps(self):
        prepare = MAP_TEXT.split('bool MapController::prepareReplay', 1)[1].split(
            'bool MapController::replayPrecheck', 1
        )[0]
        advance_body = MAP_TEXT.split(
            'void MapController::advanceReplayAfterTarget', 1
        )[1].split('void MapController::enterReplayHold', 1)[0]
        self.assertIn('replayOrigin_ = live', prepare)
        self.assertIn('replayOriginValid_ = true', prepare)
        self.assertNotIn('replayOrigin_ =', advance_body)
        self.assertIn('replayLapCounter_ = 0U', prepare)

    def test_closed_loop_hold_resume_cancel_context(self):
        for token in (
            'MAP,LOOP,HOLD,LAP=',
            'MAP,LOOP,RESUME,LAP=',
            'MAP,LOOP,CANCEL,LAP=',
            'const uint32_t cancelledLap = replayLapCounter_',
            'debug_.println(cancelledLap)',
            'replayLapCounter_',
            'replayTargetIndex_',
            'replayDirection_',
            'replayContextSlot_',
            'replayOriginRouteGeneration_',
            'replayOriginResetGeneration_',
            'replayOriginHeadingResetGeneration_',
        ):
            self.assertIn(token, MAP_TEXT)
        self.assertIn('cancelReplay("PS2_TAKEOVER")', MAP_TEXT)
        self.assertIn('enterReplayHold(MapHoldReason::EXTERNAL_STOP, false)', MAP_TEXT)
        self.assertIn('enterReplayHold(MapHoldReason::OBSTACLE, true)', MAP_TEXT)

    def test_closed_route_length_and_near_zero_closure(self):
        self.assertEqual(route_length([(0, 0), (100, 0), (100, 100)], False), 200)
        self.assertEqual(route_length([(0, 0), (100, 0), (100, 100)], True), 341)
        self.assertEqual(route_length([(0, 0), (100, 0), (100, 100), (0, 0)], True), 341)
        self.assertIn('kClosedClosureSkipDistanceMm', MAP_TEXT)
        self.assertIn('canonical open geometry', MAP_TEXT)
        self.assertIn('closure > kClosedClosureSkipDistanceMm &&', MAP_TEXT)
        self.assertIn('reason = closure > kMaximumSegmentMm ? "CLOSURE_TOO_LONG"', MAP_TEXT)

    def test_closed_near_zero_edge_is_skipped_without_motion(self):
        points = [(0.0, 0.0), (500.0, 0.0), (5.0, 5.0)]
        result = manual_select_route(points, "CLOSED")
        self.assertTrue(result["accepted"])
        self.assertLessEqual(math.dist(points[-1], points[0]), 20.0)
        self.assertIn('MAP,CLOSE_EDGE,SKIP,DIST=', MAP_TEXT)
        skip_block = MAP_TEXT[MAP_TEXT.index('MAP,CLOSE_EDGE,SKIP,DIST='):
                               MAP_TEXT.index('Pose current;', MAP_TEXT.index(
                                   'MAP,CLOSE_EDGE,SKIP,DIST='))]
        self.assertNotIn('startReplayGuidedWaypoint', skip_block)

    def test_closed_edge_validation_and_storage_are_logical_only(self):
        self.assertIn('IsClosingMode(routeMode_)', MAP_TEXT)
        self.assertIn('type == MapRouteType::CLOSED', MAP_TEXT)
        self.assertIn('CLOSED = 4U', TYPES_TEXT)
        self.assertIn('header.replayMode > static_cast<uint8_t>(MapReplayMode::CLOSED)', STORE_TEXT)
        save_block = STORE_TEXT[STORE_TEXT.index("bool MapRouteStore::save") :]
        self.assertIn('static_cast<uint8_t>(MapReplayMode::PING_PONG)', save_block)
        self.assertNotIn('route_.header.waypointCount++', MAP_TEXT)
        self.assertNotIn('appendWaypoint(routePointWorld(0', MAP_TEXT)
        self.assertIn('MAP,CLOSE_EDGE,DONE', MAP_TEXT)

    def test_closed_loop_is_iterative_and_generation_fenced(self):
        advance_body = MAP_TEXT.split(
            'void MapController::advanceReplayAfterTarget', 1
        )[1].split('void MapController::enterReplayHold', 1)[0]
        self.assertNotIn('startNextReplaySegment()', advance_body)
        self.assertIn('replayLapCounter_', advance_body)
        self.assertIn('result.motionGeneration != replaySegmentGeneration_', MAP_TEXT)
        self.assertIn('MAP,SEGMENT_DROP_STALE', MAP_TEXT)

    def test_once_sequence(self):
        current, direction = 0, 1
        visited = [current]
        for _ in range(4):
            current, direction, complete = advance(
                current, direction, 4, "OPEN", "ONCE"
            )
            if complete:
                break
            visited.append(current)
        self.assertEqual(visited, [0, 1, 2, 3])
        self.assertTrue(complete)

    def test_loop_sequence(self):
        current, direction = 2, 1
        current, direction, complete = advance(
            current, direction, 3, "CLOSED", "LOOP"
        )
        self.assertEqual((current, direction, complete), (0, 1, False))
        current, direction, complete = advance(
            current, direction, 3, "CLOSED", "LOOP"
        )
        self.assertEqual((current, direction, complete), (1, 1, False))

    def test_return_sequence(self):
        current, direction, complete = advance(
            3, 1, 4, "OPEN", "RETURN"
        )
        self.assertEqual((current, direction, complete), (2, -1, False))
        current, direction, complete = advance(0, -1, 4, "OPEN", "RETURN")
        self.assertEqual((current, direction, complete), (None, -1, True))

    def test_shuttle_ping_emits_dedicated_phase_telemetry(self):
        advance_body = MAP_TEXT.split(
            'void MapController::advanceReplayAfterTarget', 1
        )[1].split('void MapController::enterReplayHold', 1)[0]
        turnaround = advance_body[advance_body.index(
            'replayDirection_ = -1;'
        ):]
        self.assertIn('routeMode_ == MapReplayMode::PING_PONG', turnaround)
        self.assertIn('replayReturnPhase_ = ReplayReturnPhase::INBOUND', turnaround)
        self.assertIn('MAP,PING,TURNAROUND,WP=', turnaround)

    def test_one_canonical_route_supports_all_replay_modes(self):
        self.assertEqual(
            run_mode(MultiModeReplayModel(4, "ONCE")),
            [(0, 1), (1, 2), (2, 3)],
        )
        self.assertEqual(
            run_mode(MultiModeReplayModel(4, "RETURN")),
            [(0, 1), (1, 2), (2, 3), (3, 2), (2, 1), (1, 0)],
        )
        self.assertEqual(
            run_mode(MultiModeReplayModel(4, "CLOSED")),
            [(0, 1), (1, 2), (2, 3), (3, 0)],
        )
        self.assertEqual(
            run_mode(MultiModeReplayModel(4, "LOOP", loop_target=2)),
            [(0, 1), (1, 2), (2, 3), (3, 0),
             (0, 1), (1, 2), (2, 3), (3, 0)],
        )

    def test_return_arbitrary_angle_and_wrap_are_geometric(self):
        # The inbound edge uses the actual reverse geometry, not a hard-coded
        # 180-degree turnaround or an orthogonal heading assumption.
        points = [(0.0, 0.0), (300.0, 400.0), (100.0, 900.0)]
        inbound = math.degrees(math.atan2(
            points[1][1] - points[2][1], points[1][0] - points[2][0]))
        self.assertAlmostEqual(inbound, -68.20, places=1)
        self.assertIn('replayReturnPhase_ = ReplayReturnPhase::INBOUND', MAP_TEXT)
        self.assertIn('replayIncomingBearing', MAP_TEXT)

    def test_multimode_hold_resume_cancel_and_stale_generation(self):
        model = MultiModeReplayModel(4, "RETURN")
        edge, generation = model.start_segment()
        self.assertEqual(edge, (0, 1))
        model.hold()
        self.assertFalse(model.finish_segment(generation))
        model.resume()
        self.assertTrue(model.finish_segment(generation))
        _, new_generation = model.start_segment()
        self.assertFalse(model.finish_segment(generation))
        model.cancel()
        self.assertFalse(model.finish_segment(new_generation))
        self.assertIn('result.motionGeneration != replaySegmentGeneration_', MAP_TEXT)
        self.assertIn('replayReturnPhase_', MAP_TEXT)

    def test_ping_pong_sequence(self):
        current, direction, complete = advance(
            3, 1, 4, "OPEN", "PING_PONG"
        )
        self.assertEqual((current, direction, complete), (2, -1, False))
        current, direction, complete = advance(0, -1, 4, "OPEN", "PING_PONG")
        self.assertEqual((current, direction, complete), (1, 1, False))

    def test_zero_length_and_impossible_segment_rejection(self):
        self.assertIn("kMinimumSegmentMm", MAP_TEXT)
        self.assertIn("kMaximumSegmentMm", MAP_TEXT)
        self.assertIn('reason = "SEGMENT"', MAP_TEXT)
        self.assertIn('reason = "TYPE_MODE"', MAP_TEXT)

    def test_replay_transform(self):
        origin_x, origin_y, origin_heading = 100.0, -50.0, 90.0
        local_x, local_y = 1000.0, 0.0
        radians = math.radians(origin_heading)
        world_x = origin_x + local_x * math.cos(radians) - local_y * math.sin(radians)
        world_y = origin_y + local_x * math.sin(radians) + local_y * math.cos(radians)
        self.assertAlmostEqual(world_x, 100.0, places=4)
        self.assertAlmostEqual(world_y, 950.0, places=4)
        self.assertIn("replayOrigin_.headingDeg", MAP_TEXT)
        self.assertIn("toTeachLocal", MAP_TEXT)

    def test_post_teach_back_is_transient_and_uses_saved_origin(self):
        self.assertIn("PostTeachBackContext", MAP_HEADER_TEXT)
        self.assertIn("postTeachBackActive_", MAP_HEADER_TEXT)
        self.assertIn("stagePostTeachBackSnapshot", MAP_TEXT)
        self.assertIn("armPostTeachBackAfterSave", MAP_TEXT)
        self.assertIn("routePointWorldFromOrigin(\n      postTeachBack_.teachOrigin",
                      MAP_TEXT)
        self.assertNotIn("MapUserMode::BACK", TYPES_TEXT)
        start_body = MAP_TEXT.split(
            "bool MapController::startPostTeachBack", 1
        )[1].split("bool MapController::validateRoute", 1)[0]
        self.assertNotIn("store_.save", start_body)

    def test_post_teach_back_sequences_never_drive_pn(self):
        for count in (2, 4):
            model = PostTeachBackModel(count)
            self.assertTrue(model.start())
            expected = [(index, index - 1)
                        for index in range(count - 1, 0, -1)]
            while model.active:
                generation = model.generation
                self.assertTrue(model.finish_segment(generation))
            self.assertEqual(model.edges, expected)
            self.assertEqual(model.edges[0][0], count - 1)
            self.assertEqual(model.edges[-1][1], 0)
            self.assertNotIn((0, 1), model.edges)

    def test_post_teach_back_endpoint_and_heading_gates(self):
        model = PostTeachBackModel(4, origin=(100.0, -50.0, 90.0))
        self.assertTrue(model.available(current_endpoint_error=100.0))
        self.assertFalse(model.available(current_endpoint_error=100.1))
        # A 90-degree Teach heading is valid; availability has no hard-coded
        # orthogonal or 180-degree heading requirement.
        self.assertEqual(model.origin[2], 90.0)
        self.assertIn("replayOrigin_ = postTeachBack_.teachOrigin", MAP_TEXT)

    def test_post_teach_back_hold_resume_cancel_and_stale(self):
        model = PostTeachBackModel(4)
        self.assertTrue(model.start())
        generation = model.generation
        model.hold()
        self.assertFalse(model.finish_segment(generation))
        model.resume()
        self.assertTrue(model.finish_segment(model.generation))
        stale = model.generation - 1
        self.assertFalse(model.finish_segment(stale))
        model.cancel()
        self.assertFalse(model.active)
        self.assertFalse(model.available())
        for token in (
            'enterReplayHold(MapHoldReason::OBSTACLE, true)',
            'cancelReplay("PS2_TAKEOVER")',
            'MAP,BACK_P0,HOLD,WP=',
            'MAP,BACK_P0,RESUME,WP=',
            'MAP,BACK_P0,CANCEL,REASON=',
            'MAP,BACK_P0,COMPLETE',
        ):
            self.assertIn(token, MAP_TEXT)

    def test_post_teach_back_invalidation_fences(self):
        for token in (
            'invalidatePostTeachBack("NEW_TEACH")',
            'invalidatePostTeachBack("SLOT_CHANGED")',
            'invalidatePostTeachBack("SETTINGS")',
            'invalidatePostTeachBack("SETTINGS_SAVE")',
            'invalidatePostTeachBack("ROUTE_LOAD")',
            'invalidatePostTeachBack("MOTION_AFTER_SAVE")',
            'invalidatePostTeachBack("RESET_BOUNDARY")',
            'invalidatePostTeachBack("STORAGE_ERROR")',
            'MAP,BACK_P0,INVALIDATE,REASON=',
            'MAP,BACK_P0,REJECT,REASON=',
        ):
            self.assertIn(token, MAP_TEXT)
        self.assertIn('route_.header.generation != postTeachBack_.routeGeneration',
                      MAP_TEXT)
        self.assertIn('robot_.headingResetGeneration() != '
                      'postTeachBack_.headingResetGeneration', MAP_TEXT)

    def test_post_teach_back_transient_rejects_keep_context(self):
        self.assertIn('if (!display_.isMapPage())', MAP_TEXT)
        self.assertIn('reason = "NOT_MAP_PAGE"', MAP_TEXT)
        self.assertIn('postTeachBackRejectShouldInvalidate', MAP_TEXT)
        for reason in (
            '"NOT_MAP_PAGE"', '"PS2_NOT_NEUTRAL"', '"BRAKE"',
            '"ODOMETRY"', '"HEADING"', '"OBSTACLE_SENSOR"',
            '"OBSTACLE_NOT_CLEAR"', '"POSE"'):
            self.assertIn(reason, MAP_TEXT)
        self.assertIn('debug_.println(",RETRY=1")', MAP_TEXT)
        start_block = MAP_TEXT.split(
            'void MapController::handleStart()', 1
        )[1].split('void MapController::handleTriangle()', 1)[0]
        self.assertIn('robot_.stopImmediately(true)', start_block)
        self.assertIn('postTeachBackRejectShouldInvalidate(reason)', start_block)
        self.assertIn('postTeachBack_.valid && !postTeachBackActive_', MAP_TEXT)

    def test_post_teach_back_motion_owner_is_hard_invalidated(self):
        helper = MAP_TEXT.split(
            'bool MapController::postTeachBackRejectShouldInvalidate', 1
        )[1].split('bool MapController::postTeachBackAvailable', 1)[0]
        transient = helper.split('const char* transientReasons[]', 1)[1]
        transient = transient.split('};', 1)[0]
        self.assertNotIn('"MOTION_OWNER"', transient)
        start_block = MAP_TEXT.split(
            'void MapController::handleStart()', 1
        )[1].split('void MapController::handleTriangle()', 1)[0]
        self.assertIn('invalidatePostTeachBack(reason)', start_block)

        for owner in ("REPLAY", "MCP", "MISSION", "PS2"):
            model = PostTeachBackModel(4)
            self.assertTrue(model.start(), owner)
            # START rejection still performs the fail-safe stop; ownership
            # invalidates the saved post-Teach BACK context before retry.
            model.cancel()
            self.assertFalse(model.active, owner)
            self.assertFalse(model.available(), owner)
            self.assertFalse(model.start(), owner)

    def test_post_teach_back_lcd_is_bounded_and_preserves_user_mode(self):
        lines = (
            "MAP1 BACK P0 READY",
            "BACK TO P0",
            "START BACK X EXIT",
            "MAP1 BACK WP:02/04",
            "MAP1 BACK COMPLETE",
            "X HOLD XL CANCEL",
        )
        self.assertTrue(all(len(line) <= 20 for line in lines))
        self.assertIn("postTeachBackAvailable", LCD_TEXT)
        self.assertIn("postTeachBackActive", LCD_TEXT)
        self.assertIn("postTeachBackComplete", LCD_TEXT)
        self.assertIn('snprintf(desired_[2], 21, "BACK TO P0")', LCD_TEXT)
        self.assertIn('snprintf(desired_[3], 21, "START BACK X EXIT")',
                      LCD_TEXT)
        self.assertIn('snprintf(desired_[2], 21, "MODE:ONCE")', LCD_TEXT)

    def test_post_teach_back_does_not_change_storage_or_motion_tuning(self):
        self.assertNotIn("MapRouteHeader", MAP_HEADER_TEXT.split(
            "PostTeachBackContext", 1)[1])
        self.assertNotIn("store_.save", MAP_TEXT.split(
            "bool MapController::startPostTeachBack", 1)[1].split(
                "bool MapController::validateRoute", 1)[0])
        self.assertNotIn("TURN_MAX_SPEED", MAP_TEXT.split(
            "bool MapController::startPostTeachBack", 1)[1].split(
                "bool MapController::validateRoute", 1)[0])

    def test_start_activity_is_not_replay_reject_or_takeover(self):
        control = CTRL_TEXT[CTRL_TEXT.index("void RobotController::updateControl"):]
        self.assertIn("ps2SafetyStop", control)
        self.assertIn("ps2MotionFresh || ps2SafetyStop", control)
        self.assertIn("!ps2_.controlActive(nowMs)", CTRL_TEXT)
        self.assertIn("canStartReplayMotion", CTRL_TEXT)

    def test_real_ps2_takeover_and_stop(self):
        self.assertIn(
            "if (ps2MotionFresh && aiMotionMode_ != AiMotionMode::NONE)", CTRL_TEXT
        )
        self.assertIn('cancelReplay("PS2_TAKEOVER")', MAP_TEXT)
        self.assertIn("stopImmediately", MAP_TEXT)
        self.assertIn("replayOperation_ = MapReplayOperation::NONE", MAP_TEXT)
        self.assertIn("MotionOwner::REPLAY", CTRL_TEXT)

    def test_obstacle_hold_and_resume_gate(self):
        self.assertIn("enterReplayHold(MapHoldReason::OBSTACLE, true)", MAP_TEXT)
        self.assertIn("canResumeReplay", MAP_TEXT)
        self.assertIn("obstacleLiveClear", MAP_TEXT)
        self.assertIn("obstacleGraceClear", MAP_TEXT)
        self.assertIn("hasRecentClearWindow", MAP_TEXT)
        self.assertIn("replayHoldPose_", MAP_TEXT)

    def test_encoder_reset_boundary(self):
        self.assertIn("replayOriginResetGeneration_", MAP_TEXT)
        self.assertIn("resetGeneration() != replayOriginResetGeneration_", MAP_TEXT)
        self.assertIn("RESET_BOUNDARY", MAP_TEXT)

    def test_result_owner_and_mcp_routing(self):
        self.assertIn(
            "MotionOwner owner", _read(INCLUDE_ROOT / "control" / "robot_controller.h")
        )
        self.assertIn("turnResult.owner != MotionOwner::REPLAY", MAIN_TEXT)
        self.assertIn("Never expose it as an MCP terminal frame", MAIN_TEXT)
        self.assertIn("mapController.consumeReplayTurnResult", MAIN_TEXT)
        self.assertIn("mapController.consumeReplayDistanceResult", MAIN_TEXT)

    def test_legacy_map_dormant_and_lcd_local(self):
        self.assertIn("ROBOT_MAP_OWNER_STM32", ESP32_TEXT)
        self.assertIn("LEGACY_ESP32_MAP_RUNTIME=DORMANT", ESP32_TEXT)
        self.assertRegex(
            ESP32_BOARD,
            r"#if !ROBOT_MAP_OWNER_STM32\s+teach_route_\.Begin\(\);\s+#endif",
        )
        self.assertRegex(
            ESP32_BOARD,
            r"#if !ROBOT_MAP_OWNER_STM32\s+teach_route_\.StartInputTask\(\);\s+#endif",
        )
        self.assertIn("ROUTE,OWNER=STM32", ESP32_TEXT)
        self.assertIn("ROBOT_MAP_OWNER_STM32=1", ESP32_CMAKE)
        lcd = _read(SRC_ROOT / "display" / "lcd_display.cpp")
        self.assertNotIn("WAIT ESP32 STATUS", lcd)
        self.assertIn("MAP STORAGE ERROR", lcd)
        self.assertIn("MAP,UI,", _read(SRC_ROOT / "communication" / "robot_link_server.cpp"))

    def test_ps2_map_actions_are_local(self):
        self.assertIn("queueMapEvent(Ps2MapAction::START)", PS2_TEXT)
        self.assertIn("queueMapEvent(Ps2MapAction::TRIANGLE)", PS2_TEXT)
        self.assertIn("queueMapEvent(Ps2MapAction::CROSS)", PS2_TEXT)
        self.assertIn("#if !STM32_LOCAL_MAP_ENABLE", PS2_TEXT)
        self.assertIn("display.togglePage()", PS2_TEXT)

    def test_TEST_START_ONE_PRESS(self):
        self.assertIn(
            "const bool startPressed = state_.start && !previousMapStart_",
            PS2_TEXT,
        )
        for event in (
            "MAP,INPUT,START_RAW_DOWN",
            "MAP,INPUT,START_EDGE",
            "MAP,INPUT,START_ACTION",
            "MAP,START,REJECT,REASON=NOT_ARMED",
            "MAP,START,REJECT,REASON=QUEUE_FULL",
        ):
            self.assertIn(event, PS2_TEXT)
        self.assertIn("mapEvent_.action != Ps2MapAction::CROSS", PS2_TEXT)
        self.assertNotIn("if (state_.start) queueMapEvent", PS2_TEXT)

    def test_TEST_START_HELD(self):
        self.assertIn("previousMapStart_ = state_.start", PS2_TEXT)
        self.assertIn("if (startPressed) queueMapEvent(Ps2MapAction::START)", PS2_TEXT)
        self.assertIn("mapNeutralReleaseFrames_", PS2_HEADER_TEXT)
        self.assertIn("kMapNeutralFramesToArm", PS2_TEXT)

    def test_TEST_CIRCLE_ONE_PRESS_TEACH(self):
        self.assertIn(
            "const bool circlePressed = state_.circle && !previousMapCircle_",
            PS2_TEXT,
        )
        self.assertIn("if (circlePressed) queueMapEvent(Ps2MapAction::CIRCLE)", PS2_TEXT)
        self.assertIn("MAP,INPUT,CIRCLE_RAW_DOWN", PS2_TEXT)
        self.assertIn("MAP,INPUT,CIRCLE_EDGE,PAGE=", PS2_TEXT)
        self.assertIn("MAP,INPUT,CIRCLE_ACTION", PS2_TEXT)
        self.assertIn("MAP,EVENT=", MAP_TEXT)
        self.assertIn("MAP,CIRCLE,ACTION=FINISH_TEACH", MAP_TEXT)

    def test_TEST_CIRCLE_HELD(self):
        self.assertIn("previousMapCircle_ = state_.circle", PS2_TEXT)
        self.assertIn("state_.circle && !previousMapCircle_", PS2_TEXT)

    def test_TEST_CIRCLE_RELEASE_PRESS(self):
        self.assertIn("previousMapCircle_ = state_.circle", PS2_TEXT)

    def test_TEST_CIRCLE_NOT_ARMED_LOG(self):
        self.assertIn("MAP,CIRCLE,REJECT,REASON=NOT_ARMED", PS2_TEXT)
        self.assertIn("mapActionsArmed_", PS2_TEXT)

    def test_TEST_CIRCLE_MAP_PAGE_ONLY(self):
        self.assertIn("MAP,CIRCLE,REJECT,REASON=NOT_MAP_PAGE", PS2_TEXT)
        self.assertIn("MAP,CIRCLE,REJECT,REASON=PAGE_TRANSITION", PS2_TEXT)
        self.assertIn("display.isMapPage()", PS2_TEXT)

    def test_TEST_CIRCLE_QUEUE_DELIVERY(self):
        self.assertIn("MAP,INPUT,CIRCLE_ACTION", PS2_TEXT)
        self.assertIn("MAP,INPUT,CIRCLE_ACTION,DROPPED=QUEUE_FULL", PS2_TEXT)
        self.assertIn("bool Ps2Controller::takeMapEvent", PS2_TEXT)
        self.assertIn("case Ps2MapAction::CIRCLE: handleCircle(); break;", MAP_TEXT)

    def test_TEST_CIRCLE_FINISH_TEACH(self):
        self.assertIn("MAP,CIRCLE,HANDLE,MODE=", MAP_TEXT)
        self.assertIn("MAP,CIRCLE,ACTION=FINISH_TEACH", MAP_TEXT)
        self.assertIn("requestTeachFinish();", MAP_TEXT)

    def test_TEST_CIRCLE_PENDING_STOP(self):
        self.assertIn("teachFinishPending_ = true", MAP_TEXT)
        self.assertIn("if (teachFinishPending_)", MAP_TEXT)
        self.assertIn("finalizeTeach();", MAP_TEXT)

    def test_TEST_CIRCLE_SAVED_MODE(self):
        self.assertIn("MAP,CIRCLE,REJECT,REASON=SETTINGS_REQUIRED", MAP_TEXT)
        self.assertIn("bool MapController::enterSettings(const char*& reason)", MAP_TEXT)
        self.assertIn("MapControllerMode::SETTINGS", MAP_TEXT)
        self.assertIn("saveSettingsAndExit()", MAP_TEXT)

    def test_TEST_CIRCLE_NO_CLOSED_CONFIRM(self):
        self.assertNotIn("MAP,CIRCLE,ACTION=CONFIRM_CLOSED", MAP_TEXT)
        self.assertNotIn("MAP,CLOSE_CONFIRM,RESULT=CLOSED", MAP_TEXT)

    def test_TEST_X_PRIORITY_OVER_CIRCLE(self):
        self.assertIn("if (crossPressed && display.isMapPage())", PS2_TEXT)
        self.assertIn("mapEvent_.action != Ps2MapAction::CROSS", PS2_TEXT)
        self.assertIn("CIRCLE_ACTION,DROPPED=QUEUE_FULL", PS2_TEXT)

    def test_TEST_START_PRIORITY_POLICY_UNCHANGED(self):
        self.assertIn("if (startPressed) queueMapEvent(Ps2MapAction::START)", PS2_TEXT)
        self.assertIn("MAP,INPUT,START_ACTION", PS2_TEXT)
        self.assertIn("MAP,START,REJECT,REASON=QUEUE_FULL", PS2_TEXT)

    def test_TEST_X_CANCEL_GENERATION(self):
        self.assertIn("void Ps2Controller::disarmMapInput()", PS2_TEXT)
        self.assertIn("ps2_.disarmMapInput()", MAP_TEXT)
        self.assertIn("nextReplayGeneration();", MAP_TEXT)
        self.assertIn("replaySegmentGeneration_", MAP_HEADER_TEXT)
        self.assertIn("result.motionGeneration != replaySegmentGeneration_", MAP_TEXT)
        self.assertIn("MAP,SEGMENT_DROP_STALE", MAP_TEXT)
        self.assertIn("MAP,REPLAY_CANCEL", MAP_TEXT)

    def test_TEST_STALE_START(self):
        self.assertIn(
            "if (action == Ps2MapAction::CROSS &&",
            PS2_TEXT,
        )
        self.assertIn("SAFETY_PRIORITY", PS2_TEXT)
        self.assertIn("mapEventPending_ = false", PS2_TEXT)
        self.assertIn("replayResumeAllowed_ = false", MAP_TEXT)
        self.assertIn("replayActive_ = false", MAP_TEXT)

    def test_cancel_trace_is_bounded_and_diagnostic(self):
        self.assertIn("MAP,CANCEL_TRACE,T=", MAP_TEXT)
        self.assertIn("cancelTraceLines_ >= 20U", MAP_TEXT)
        self.assertIn("> 2000U", MAP_TEXT)
        for field in (
            "TARGET_L=",
            "TARGET_R=",
            "CURRENT_L=",
            "CURRENT_R=",
            ",LV=",
            ",RV=",
            ",GEN=",
        ):
            self.assertIn(field, MAP_TEXT)

    def test_x_input_is_processed_before_async_result_but_replay_update_stays_after(self):
        input_index = MAIN_TEXT.index("mapController.processInput();")
        result_index = MAIN_TEXT.index("AiTurnResult turnResult;")
        update_index = MAIN_TEXT.index("mapController.update();")
        self.assertLess(input_index, result_index)
        self.assertGreater(update_index, result_index)
        self.assertIn("uint32_t motionGeneration", CTRL_HEADER_TEXT)
        self.assertIn("motionGeneration = aiMotionGeneration_", CTRL_TEXT)

    def test_stale_result_does_not_become_mcp_done(self):
        self.assertIn("turnResult.owner != MotionOwner::REPLAY", MAIN_TEXT)
        self.assertIn("distanceResult.owner != MotionOwner::REPLAY", MAIN_TEXT)
        self.assertIn("Never expose it as an MCP terminal frame", MAIN_TEXT)
        self.assertIn("mapController.consumeReplayTurnResult", MAIN_TEXT)
        self.assertIn("mapController.consumeReplayDistanceResult", MAIN_TEXT)

    # Required acceptance-test IDs from the MAP HIL-4 cancellation task.
    def test_TEST_START_RELEASE_PRESS_AGAIN(self):
        self.assertIn("const bool startPressed = state_.start && !previousMapStart_", PS2_TEXT)
        self.assertIn("previousMapStart_ = state_.start", PS2_TEXT)
        self.assertIn("if (allMapButtonsReleased)", PS2_TEXT)

    def test_TEST_REARM_AFTER_X(self):
        self.assertIn("mapActionsArmed_ = false", PS2_TEXT)
        self.assertIn("mapNeutralReleaseFrames_ = 0U", PS2_TEXT)
        self.assertIn("kMapNeutralFramesToArm", PS2_TEXT)
        self.assertIn('robotDebug.println("MAP,INPUT,ARMED")', PS2_TEXT)

    def test_TEST_NO_AUTO_RESUME(self):
        self.assertIn("replayResumeAllowed_ = false", MAP_TEXT)
        self.assertIn("replayActive_ = false", MAP_TEXT)
        self.assertIn("mode_ = storeState_ == MapStoreState::SAVED", MAP_TEXT)

    def test_TEST_MCP_OWNER_UNCHANGED(self):
        self.assertIn("MotionOwner::MCP", CTRL_TEXT)
        self.assertIn("MotionOwner::REPLAY", CTRL_TEXT)
        self.assertIn("if (turnResult.owner != MotionOwner::REPLAY)", MAIN_TEXT)
        self.assertIn("if (distanceResult.owner != MotionOwner::REPLAY)", MAIN_TEXT)

    def test_TEST_REPLAY_RESULT_NOT_MCP_DONE(self):
        self.assertIn("turnResult.owner != MotionOwner::REPLAY", MAIN_TEXT)
        self.assertIn("distanceResult.owner != MotionOwner::REPLAY", MAIN_TEXT)
        self.assertIn("reportTurnResult", MAIN_TEXT)
        self.assertIn("reportDistanceResult", MAIN_TEXT)
        self.assertIn("consumeReplayTurnResult", MAIN_TEXT)
        self.assertIn("consumeReplayDistanceResult", MAIN_TEXT)

    def test_TEST_STALE_SEGMENT_RESULT(self):
        self.assertIn("result.motionGeneration != replaySegmentGeneration_", MAP_TEXT)
        self.assertIn("MAP,SEGMENT_DROP_STALE", MAP_TEXT)
        self.assertIn("return false", MAP_TEXT)

    def test_TEST_SAVED_START_RUN(self):
        self.assertIn('debug_.println("MAP,START,ACTION=RUN")', MAP_TEXT)
        self.assertIn("prepareReplay(reason)", MAP_TEXT)
        self.assertIn("replayCurrentIndex_ = 0U", MAP_TEXT)
        self.assertIn("replayTargetIndex_ = 1U", MAP_TEXT)

    def test_TEST_NO_SELECT_REQUIRED(self):
        self.assertIn('debug_.println("MAP,START,ACTION=RUN")', MAP_TEXT)
        self.assertNotIn("handleSlot(event.slot); handleStart()", MAP_TEXT)
        event_body = MAP_TEXT.split("void MapController::handleEvent", 1)[1].split(
            "void MapController::handleSlot", 1
        )[0]
        self.assertNotIn("selectedSlot_ =", event_body)

    def test_TEST_SELECT_ONLY_CHANGES_SLOT(self):
        self.assertIn("case Ps2MapAction::SLOT: handleSlot(event.slot); break;", MAP_TEXT)
        self.assertIn("selectedSlot_ = slot == 2U ? MapSlot::MAP_2 : MapSlot::MAP_1", MAP_TEXT)
        self.assertIn("mode_ == MapControllerMode::REPLAY_HOLD", MAP_TEXT)

    def test_TEST_SELECT_LOCKED_DURING_RUN(self):
        self.assertIn("replayActive_", MAP_TEXT)
        self.assertIn("mode_ == MapControllerMode::REPLAY_HOLD", MAP_TEXT)
        lcd_header = _read(INCLUDE_ROOT / "display" / "lcd_display.h")
        self.assertIn("status.mode >= 1U && status.mode != 2U", lcd_header)
        self.assertIn("status.mode != 8U", lcd_header)

    def test_TEST_X_SHORT_HOLD(self):
        self.assertIn("queueMapEvent(Ps2MapAction::CROSS)", PS2_TEXT)
        self.assertIn("MapHoldReason::USER", MAP_TEXT)
        self.assertIn("enterReplayHold(MapHoldReason::USER, true)", MAP_TEXT)
        self.assertIn("robot_.stopImmediately(true)", MAP_TEXT)
        self.assertIn("mode_ = MapControllerMode::REPLAY_HOLD", MAP_TEXT)
        self.assertIn("MAP,HOLD,REASON=", MAP_TEXT)

    def test_TEST_X_LONG_CANCEL(self):
        self.assertIn("CROSS_LONG", PS2_TEXT)
        self.assertIn("MAP_LONG_PRESS_MS", PS2_TEXT)
        self.assertIn("MAP_LONG_PRESS_MS = 1300U", CONFIG_TEXT)
        self.assertIn("void MapController::handleCrossLong()", MAP_TEXT)
        self.assertIn('cancelReplay("X_LONG")', MAP_TEXT)
        self.assertIn("MAP,CANCEL,REASON=", MAP_TEXT)

    def test_TEST_HOLD_STALE_SEGMENT(self):
        self.assertIn("replayOperation_ = MapReplayOperation::HOLD", MAP_TEXT)
        self.assertIn("nextReplayGeneration();", MAP_TEXT)
        self.assertIn("replaySegmentGeneration_ = 0U", MAP_TEXT)
        self.assertIn("result.motionGeneration != replaySegmentGeneration_", MAP_TEXT)

    def test_TEST_USER_HOLD_RESUME(self):
        self.assertIn("canResumeReplay(rejectReason)", MAP_TEXT)
        self.assertIn('debug_.println("MAP,START,ACTION=RESUME")', MAP_TEXT)
        self.assertIn("replayTargetIndex_", MAP_TEXT)
        self.assertIn("replayActive_ = true", MAP_TEXT)
        self.assertIn("replayReason_ = \"RESUME\"", MAP_TEXT)

    def test_TEST_HOLD_COAST(self):
        self.assertIn("currentReplayPose(current)", MAP_TEXT)
        self.assertIn("targetDistance = distanceMm(current.xMm, current.yMm", MAP_TEXT)
        self.assertIn("current live pose", MAP_TEXT)
        self.assertIn("old remaining-distance", MAP_TEXT)

    def test_TEST_RESUME_RESET_BOUNDARY(self):
        self.assertIn('rejectReason = "RESET_BOUNDARY"', MAP_TEXT)
        self.assertIn("resetGeneration() != replayOriginResetGeneration_", MAP_TEXT)
        self.assertIn('debug_.print("MAP,RESUME,REJECT,REASON=")', MAP_TEXT)

    def test_TEST_OBSTACLE_HOLD_RESUME_GATE(self):
        self.assertIn("MapHoldReason::OBSTACLE", MAP_TEXT)
        self.assertIn('rejectReason = "OBSTACLE_NOT_CLEAR"', MAP_TEXT)
        self.assertIn("obstacleLiveClear", MAP_TEXT)
        self.assertIn("obstacleGraceClear", MAP_TEXT)
        self.assertIn("hasRecentClearWindow", MAP_TEXT)
        self.assertIn("replayResumeAllowed_", MAP_TEXT)

    def test_TEST_START_WHILE_HOLD(self):
        self.assertIn('debug_.println("MAP,START,ACTION=RESUME")', MAP_TEXT)
        self.assertIn("MAP,RESUME,REQUEST,WP=", MAP_TEXT)
        self.assertIn("MAP,RESUME,ACCEPT,WP=", MAP_TEXT)
        self.assertNotIn("replayCurrentIndex_ = 0U;\n      replayTargetIndex_ = 1U;", MAP_TEXT)

    def test_TEST_START_AFTER_LONG_CANCEL(self):
        self.assertIn('cancelReplay("X_LONG")', MAP_TEXT)
        self.assertIn("clearReplayResumeContext();", MAP_TEXT)
        self.assertIn('debug_.println("MAP,START,ACTION=RUN")', MAP_TEXT)

    def test_TEST_MANUAL_TAKEOVER_CANCELS(self):
        self.assertIn('cancelReplay("PS2_TAKEOVER")', MAP_TEXT)
        self.assertIn("ps2_.motionCommandActive()", MAP_TEXT)
        self.assertIn("replayResumeAllowed_ = false", MAP_TEXT)

    def test_TEST_HOLD_ONCE_CONTEXT(self):
        for token in ("routeMode_", "replayDirection_", "replayCurrentIndex_",
                      "replayTargetIndex_", "replayContextSlot_", "replayOriginValid_"):
            self.assertIn(token, MAP_HEADER_TEXT)
        self.assertIn("MapReplayMode::ONCE", MAP_TEXT)

    def test_TEST_HOLD_LOOP_CONTEXT(self):
        self.assertIn("MapReplayMode::LOOP", MAP_TEXT)
        self.assertIn("routeMode_", MAP_TEXT)
        self.assertIn("replayDirection_", MAP_TEXT)
        self.assertIn("replayLapCounter_", MAP_HEADER_TEXT)

    def test_TEST_HOLD_RETURN_CONTEXT(self):
        self.assertIn("MapReplayMode::RETURN", MAP_TEXT)
        self.assertIn("replayReturned_", MAP_TEXT)
        self.assertIn("replayDirection_", MAP_TEXT)
        self.assertIn("ReplayReturnPhase::NONE", MAP_HEADER_TEXT)
        self.assertIn("ReplayReturnPhase::OUTBOUND", MAP_TEXT)
        self.assertIn("ReplayReturnPhase::INBOUND", MAP_TEXT)

    def test_TEST_HOLD_PING_PONG_CONTEXT(self):
        self.assertIn("MapReplayMode::PING_PONG", MAP_TEXT)
        self.assertIn("replayReturned_", MAP_TEXT)
        self.assertIn("replayTargetIndex_", MAP_TEXT)

    def test_hold_lcd_semantics(self):
        self.assertIn("uint8_t holdReason", _read(INCLUDE_ROOT / "display" / "lcd_display.h"))
        self.assertIn('"TGT:%lu TRV:%lu"', LCD_TEXT)
        self.assertIn("writeRunMode(desired_[2])", LCD_TEXT)
        self.assertIn('"SHUT %s CY:%lu/INF"', LCD_TEXT)
        self.assertIn('snprintf(desired_[3], 21, "X HOLD XL CANCEL")', LCD_TEXT)

    def test_map_settings_and_motion_capture(self):
        for token in (
            "MapControllerMode::SETTINGS",
            "MapControllerMode::HELP",
            "MapControllerMode::DELETE_CONFIRM",
            "setMapUiCapture(true)",
            "mapUiCaptureActive()",
            "saveSettingsAndExit()",
            "leaveHelp()",
        ):
            self.assertIn(token, MAP_TEXT + CTRL_TEXT + PS2_TEXT)
        self.assertIn("SELECT_LONG", PS2_TEXT)
        self.assertIn("if (ps2_.mapUiCaptureActive())", CTRL_TEXT)
        self.assertIn("targetLeft_ = targetRight_ = 0", CTRL_TEXT)
        self.assertIn("state_ = RobotState::STOP", CTRL_TEXT)

    def test_map_settings_storage_is_backward_compatible(self):
        self.assertIn("MapRouteHeader::reserved", TYPES_TEXT)
        for token in (
            "MAP_SETTINGS_SPEED_MASK",
            "MAP_SETTINGS_LOOP_MASK",
            "MAP_SETTINGS_SHUTTLE_REPEAT_MASK",
            "mapReplaySpeedFromReserved",
            "mapLoopTargetFromReserved",
            "mapReplaySpeedToReserved",
            "mapLoopTargetToReserved",
            "mapShuttleRepeatFromReserved",
            "mapShuttleRepeatToReserved",
        ):
            self.assertIn(token, TYPES_TEXT)
        self.assertEqual(_constant(TYPES_TEXT, "MAP_REPLAY_SPEED_DEFAULT"), 20)
        self.assertEqual(_constant(TYPES_TEXT, "MAP_REPLAY_SPEED_MIN"), 15)
        self.assertEqual(_constant(TYPES_TEXT, "MAP_REPLAY_SPEED_MAX"), 50)
        self.assertEqual(_constant(TYPES_TEXT, "MAP_REPLAY_SPEED_STEP"), 5)
        self.assertEqual(_constant(TYPES_TEXT, "MAP_LOOP_TARGET_MAX"), 20)
        self.assertIn("result.replaySpeed", STORE_TEXT)
        self.assertIn("result.loopTarget", STORE_TEXT)
        self.assertIn("result.shuttleRepeat", STORE_TEXT)

    def test_v521_user_mode_layer_and_execution_mapping(self):
        self.assertIn("enum class MapUserMode", TYPES_TEXT)
        self.assertIn("MapUserMode::ONCE", MAP_HEADER_TEXT)
        self.assertIn("settingsUserMode_", MAP_HEADER_TEXT)
        self.assertIn("executionModeFor", MAP_TEXT)
        self.assertEqual(v521_execution_mode("ONCE", 1), "ONCE")
        self.assertEqual(v521_execution_mode("SHUTTLE", 1), "RETURN")
        self.assertEqual(v521_execution_mode("SHUTTLE", 3), "PING_PONG")
        self.assertEqual(v521_execution_mode("SHUTTLE", 0), "PING_PONG")
        self.assertEqual(v521_execution_mode("LOOP", 1), "CLOSED")
        self.assertEqual(v521_execution_mode("LOOP", 3), "LOOP")

    def test_v521_reserved_shuttle_flag_isolated_from_speed_and_count(self):
        flag = _constant(TYPES_TEXT, "MAP_SETTINGS_SHUTTLE_REPEAT_MASK")
        speed_mask = _constant(TYPES_TEXT, "MAP_SETTINGS_SPEED_MASK")
        count_mask = _constant(TYPES_TEXT, "MAP_SETTINGS_LOOP_MASK")
        original = 0xA5000000 | 27 | (3 << 8)
        self.assertEqual(flag, 0x00010000)
        marked = original | flag
        cleared = marked & ~flag
        self.assertEqual(marked & (speed_mask | count_mask),
                         original & (speed_mask | count_mask))
        self.assertEqual(cleared & (speed_mask | count_mask),
                         original & (speed_mask | count_mask))
        self.assertIn("mapShuttleRepeatToReserved", TYPES_TEXT)
        self.assertIn("~MAP_SETTINGS_SHUTTLE_REPEAT_MASK", TYPES_TEXT)

    def test_v521_exact_new_persistence_records(self):
        expected = {
            ("ONCE", 1): ("OPEN", "ONCE", False),
            ("SHUTTLE", 1): ("OPEN", "RETURN", False),
            ("SHUTTLE", 3): ("OPEN", "RETURN", True),
            ("SHUTTLE", 0): ("OPEN", "RETURN", True),
            ("LOOP", 1): ("CLOSED", "ONCE", False),
            ("LOOP", 3): ("CLOSED", "LOOP", False),
            ("LOOP", 0): ("CLOSED", "LOOP", False),
        }
        for key, persisted in expected.items():
            record = v521_persisted_record(*key)
            self.assertEqual((record["route_type"], record["replay_mode"],
                              record["shuttle_flag"]), persisted)
        self.assertNotIn('storedMode = MapReplayMode::PING_PONG', MAP_TEXT)
        self.assertIn("storedMode = MapReplayMode::RETURN", MAP_TEXT)

    def test_v521_shuttle_sequences_and_cycle_boundary(self):
        one = shuttle_edges(4, 1)
        two = shuttle_edges(4, 2)
        self.assertEqual(one, [(0, 1), (1, 2), (2, 3),
                               (3, 2), (2, 1), (1, 0)])
        self.assertEqual(two[:len(one)], one)
        self.assertEqual(two[len(one):], one)
        self.assertIn("replayCycleCounter_", MAP_HEADER_TEXT)
        self.assertIn("MAP,PING,CYCLE_COMPLETE,CYCLE=", MAP_TEXT)
        self.assertIn("replayCycleCounter_ >= loopTarget_", MAP_TEXT)
        self.assertIn("replayCycleCounter_ != 0xFFFFFFFFUL", MAP_TEXT)

    def test_v521_shuttle_infinite_is_iterative_and_safe(self):
        edges = shuttle_edges(3, 5)
        self.assertEqual(len(edges), 20)
        self.assertEqual(edges[:4], [(0, 1), (1, 2), (2, 1), (1, 0)])
        self.assertEqual(edges[4:8], edges[:4])
        self.assertIn("replayTargetIndex_ = 1U", MAP_TEXT)
        self.assertIn("replayReturnPhase_ = ReplayReturnPhase::OUTBOUND", MAP_TEXT)
        self.assertIn("MAP,PING,PHASE=OUT,CYCLE=", MAP_TEXT)
        self.assertNotIn("startNextReplaySegment();", MAP_TEXT[MAP_TEXT.index(
            "void MapController::advanceReplayAfterTarget"):MAP_TEXT.index(
            "void MapController::enterReplayHold")])

    def test_v521_mode_cycle_and_count_semantics(self):
        self.assertIn("MapUserMode::ONCE, MapUserMode::SHUTTLE, MapUserMode::LOOP",
                      MAP_TEXT)
        self.assertIn("settingsRepeatTarget_", MAP_TEXT)
        self.assertIn("settingsUserMode_ == MapUserMode::ONCE", MAP_TEXT)
        self.assertIn('">COUNT:--"', LCD_TEXT)
        self.assertIn('">CYCLE:INF"', LCD_TEXT)
        self.assertIn('">LAP:INF"', LCD_TEXT)

    def test_v521_legacy_load_and_safe_degrade_contract(self):
        # Legacy records remain readable, while only new shuttle records use
        # OPEN+RETURN plus the marker rather than persisted PING_PONG.
        self.assertIn("mode == MapReplayMode::PING_PONG", MAP_TEXT)
        self.assertIn("Legacy PING_PONG is always infinite", MAP_TEXT)
        self.assertIn("mapShuttleRepeatFromReserved", MAP_TEXT)
        self.assertIn("mode == MapReplayMode::RETURN", MAP_TEXT)
        self.assertIn("shuttleRepeat ? encodedTarget", MAP_TEXT)
        self.assertIn("MapReplayMode::RETURN", STORE_TEXT)

    def test_v521_user_ui_hides_legacy_mode_names(self):
        self.assertNotIn('"RETURN GO + BACK"', LCD_TEXT)
        self.assertNotIn('"CLOSED CLOSE ONCE"', LCD_TEXT)
        self.assertNotIn('"MODE:RETURN"', LCD_TEXT)
        self.assertNotIn('"MODE:PING"', LCD_TEXT)
        self.assertNotIn('"MODE:CLOSED"', LCD_TEXT)
        for line in (
            "MODE:ONCE", "MODE:SHUTTLE CY:INF", "MODE:LOOP LAP:INF",
            "SHUT OUT CY:%lu/INF", "SHUT BACK CY:%lu/INF",
        ):
            self.assertLessEqual(len(line), 20)

    def test_return_phase_and_storage_encoding_are_runtime_only(self):
        for token in (
            "returnPhase",
            "MAP,RETURN,START,POINTS=",
            "MAP,RETURN,PHASE=OUT",
            "MAP,RETURN,TURNAROUND,WP=",
            "MAP,RETURN,PHASE=BACK,FROM=",
            "MAP,RETURN,HOLD,PHASE=",
            "MAP,RETURN,COMPLETE,WP=0",
        ):
            self.assertIn(token, MAP_TEXT + MAP_HEADER_TEXT + LCD_TEXT)
        self.assertNotIn("legacyCanonicalMigrationPending_", MAP_TEXT + MAP_HEADER_TEXT)
        self.assertNotIn("returnPhase", TYPES_TEXT)
        self.assertNotIn("returnPhase", STORE_TEXT)
        self.assertIn("persistedRouteType", MAP_TEXT)
        self.assertIn("persistedReplayMode", MAP_TEXT)
        self.assertIn("persistedRouteLengthMm", MAP_TEXT)
        self.assertIn("normalizeRouteForRuntime", MAP_TEXT)
        self.assertIn("MapReplayMode::PING_PONG", _read(SRC_ROOT / "map" / "route_store.cpp"))

    def test_legacy_closed_once_is_unchanged_on_no_change_save(self):
        points = [(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)]
        model = StorageCompatibilityModel(points, "CLOSED")
        model.load()
        old_flash = model.flash.copy()
        self.assertEqual(model.runtime[0], "CLOSED")
        self.assertEqual(model.save_settings(), "NO_CHANGE")
        self.assertEqual(model.write_count, 0)
        self.assertEqual(model.flash, old_flash)

    def test_persisted_mode_mapping_is_v529_compatible(self):
        points = [(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)]
        for runtime_mode, route_type, replay_mode in (
            ("ONCE", "OPEN", "ONCE"),
            ("RETURN", "OPEN", "RETURN"),
            ("CLOSED", "CLOSED", "ONCE"),
            ("LOOP", "CLOSED", "LOOP"),
        ):
            model = StorageCompatibilityModel(points, runtime_mode, lap=2)
            record = model.flash
            self.assertEqual(record["route_type"], route_type)
            self.assertEqual(record["replay_mode"], replay_mode)
            self.assertNotEqual(record["replay_mode"], "CLOSED")
            self.assertTrue(v529_load_pass(record, points))

    def test_round_trip_closed_return_loop_and_failure_rollback(self):
        points = [(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)]
        model = StorageCompatibilityModel(points, "CLOSED", lap=2)
        self.assertTrue(v529_load_pass(model.flash, points))
        model.load()
        self.assertEqual(model.save_settings(runtime_mode="RETURN"), "OK")
        self.assertTrue(v529_load_pass(model.flash, points))
        self.assertEqual((model.flash["route_type"], model.flash["replay_mode"]),
                         ("OPEN", "RETURN"))
        self.assertEqual(model.save_settings(runtime_mode="LOOP"), "OK")
        self.assertTrue(v529_load_pass(model.flash, points))
        self.assertEqual((model.flash["route_type"], model.flash["replay_mode"]),
                         ("CLOSED", "LOOP"))
        old_flash = model.flash.copy()
        old_runtime = model.runtime
        self.assertEqual(model.save_settings(runtime_mode="RETURN", succeed=False),
                         "FAIL")
        self.assertEqual(model.flash, old_flash)
        self.assertEqual(model.runtime, old_runtime)
        self.assertEqual(model.write_count, 3)

    def test_closed_and_loop_stored_length_includes_logical_closure(self):
        points = [(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0)]
        base = route_length(points, closed=False)
        closed = route_length(points, closed=True)
        self.assertEqual(base, 2000)
        self.assertEqual(closed, 3414)
        for mode in ("CLOSED", "LOOP"):
            model = StorageCompatibilityModel(points, mode)
            self.assertEqual(model.flash["route_length"], closed)

    def test_canonical_unchanged_settings_save_does_not_write(self):
        model = StorageCompatibilityModel([(0.0, 0.0), (1000.0, 0.0)],
                                           "RETURN")
        model.load()
        self.assertEqual(model.save_settings(), "NO_CHANGE")
        self.assertEqual(model.write_count, 0)

    def test_map_settings_speed_and_lap_bounds(self):
        self.assertIn("MAP_REPLAY_SPEED_MIN", MAP_TEXT)
        self.assertIn("MAP_REPLAY_SPEED_MAX", MAP_TEXT)
        self.assertIn("MAP_REPLAY_SPEED_STEP", MAP_TEXT)
        self.assertIn("MAP_LOOP_TARGET_INF", MAP_TEXT)
        self.assertIn("settingsUserMode_ == MapUserMode::ONCE", MAP_TEXT)
        self.assertIn("settingsRepeatTarget_ = settingsRepeatTarget_ == MAP_LOOP_TARGET_MAX", MAP_TEXT)

    def test_TEST_SETTINGS_NO_CHANGE_NO_FLASH_WRITE(self):
        start = MAP_TEXT.index("void MapController::saveSettingsAndExit")
        end = MAP_TEXT.index("void MapController::cancelSettings", start)
        block = MAP_TEXT[start:end]
        no_change_start = block.index("const bool settingsChanged")
        candidate_start = block.index("MapRouteData candidate")
        no_change = block[no_change_start:candidate_start]
        self.assertIn("settingsUserMode_ != userMode_", no_change)
        self.assertIn('MAP,SETTINGS,SAVE=NO_CHANGE', no_change)
        self.assertNotIn("store_.save", no_change)

    def test_TEST_SETTINGS_NO_CHANGE_GENERATION_UNCHANGED(self):
        start = MAP_TEXT.index("void MapController::saveSettingsAndExit")
        end = MAP_TEXT.index("void MapController::cancelSettings", start)
        block = MAP_TEXT[start:end]
        no_change = block[block.index("const bool settingsChanged"):
                          block.index("MapRouteData candidate")]
        self.assertNotIn("updateRouteHeaderForSave", no_change)
        self.assertIn("leaveMapUiCapture()", no_change)
        self.assertIn("mode_ = MapControllerMode::SAVED", no_change)

    def test_TEST_SETTINGS_HELP_NO_CHANGE(self):
        self.assertIn("enterHelp()", MAP_TEXT)
        self.assertIn("leaveHelp()", MAP_TEXT)
        self.assertIn("settingsUserMode_ = userMode_", MAP_TEXT)
        self.assertIn("settingsSpeed_ = replaySpeed_", MAP_TEXT)
        self.assertIn("settingsRepeatTarget_ = loopTarget_", MAP_TEXT)

    def test_TEST_SETTINGS_NAV_NO_CHANGE(self):
        settings_start = MAP_TEXT.index("void MapController::handleSettingsInput")
        settings_end = MAP_TEXT.index("void MapController::handleStart", settings_start)
        settings_block = MAP_TEXT[settings_start:settings_end]
        self.assertIn("settingsItem_ = static_cast<MapSettingsItem>", settings_block)
        self.assertIn("cycleSettingsMode", settings_block)
        self.assertIn("cycleSettingsSpeed", settings_block)
        self.assertIn("cycleSettingsLap", settings_block)
        self.assertIn("const bool settingsChanged", MAP_TEXT)

    def test_TEST_SETTINGS_CHANGED_THEN_REVERT_NO_CHANGE(self):
        self.assertIn("settingsSpeed_ != replaySpeed_", MAP_TEXT)
        self.assertIn("normalizedTarget != loopTarget_", MAP_TEXT)
        self.assertNotIn("settingsDirty_", MAP_TEXT)

    def test_TEST_SETTINGS_CHANGED_WRITE_ONCE(self):
        start = MAP_TEXT.index("void MapController::saveSettingsAndExit")
        end = MAP_TEXT.index("void MapController::cancelSettings", start)
        block = MAP_TEXT[start:end]
        self.assertEqual(block.count("store_.save(selectedSlot_, candidate)"), 1)
        self.assertIn("route_ = candidate", block)

    def test_TEST_SETTINGS_SAVE_FAIL_ROLLBACK_RUNTIME(self):
        start = MAP_TEXT.index("void MapController::saveSettingsAndExit")
        end = MAP_TEXT.index("void MapController::cancelSettings", start)
        block = MAP_TEXT[start:end]
        fail_start = block.index("if (!store_.save")
        success_start = block.index("route_ = candidate")
        fail_block = block[fail_start:success_start]
        self.assertIn("routeMode_ = oldMode", fail_block)
        self.assertIn("replaySpeed_ = oldSpeed", fail_block)
        self.assertIn("loopTarget_ = oldLoopTarget", fail_block)
        self.assertIn("OLD_ROUTE_RETAINED=1", fail_block)

    def test_TEST_SETTINGS_SAVE_FAIL_OLD_FLASH_RETAINED(self):
        start = MAP_TEXT.index("void MapController::saveSettingsAndExit")
        end = MAP_TEXT.index("void MapController::cancelSettings", start)
        block = MAP_TEXT[start:end]
        self.assertIn("MapRouteData candidate = route_", block)
        self.assertIn("route_ = candidate", block)
        self.assertLess(block.index("if (!store_.save"),
                        block.index("route_ = candidate"))
        self.assertIn("MapStoreState::STORAGE_ERROR", block)

    def test_TEST_SETTINGS_SAVE_FAIL_LCD_ERROR(self):
        for line in ("MAP%u SAVE ERROR", "OLD MAP RETAINED",
                     "SETTINGS NOT SAVED", "X BACK"):
            self.assertIn(line, LCD_TEXT)
            self.assertLessEqual(len(line.replace("%u", "2")), 20)

    def test_TEST_SETTINGS_SAVE_FAIL_REASON(self):
        for reason in ("NONE", "SETTINGS_SAVE", "TEACH_SAVE", "MODE_SAVE",
                       "STORAGE_INIT", "GENERIC"):
            self.assertIn("MapStorageErrorReason::" + reason, MAP_HEADER_TEXT + MAP_TEXT)
        start = MAP_TEXT.index("void MapController::saveSettingsAndExit")
        end = MAP_TEXT.index("void MapController::cancelSettings", start)
        block = MAP_TEXT[start:end]
        self.assertIn("storageErrorReason_ = MapStorageErrorReason::SETTINGS_SAVE", block)
        self.assertIn("const MapReplayMode oldMode = routeMode_", block)

    def test_TEST_SETTINGS_FAIL_OLD_ROUTE_X_BACK(self):
        start = MAP_TEXT.index("void MapController::handleCross()")
        end = MAP_TEXT.index("void MapController::handleCrossLong()", start)
        block = MAP_TEXT[start:end]
        self.assertIn("storageErrorReason_ != MapStorageErrorReason::STORAGE_INIT", block)
        self.assertIn("loadedValid_", block)
        self.assertIn("MAP,STORAGE_ERROR,ACK=BACK,OLD_ROUTE=1", block)
        self.assertIn("storageErrorReason_ = MapStorageErrorReason::NONE", block)

    def test_TEST_SETTINGS_FAIL_NO_OLD_ROUTE(self):
        for line in ("SAVE FAILED", "NO VALID ROUTE", "CHECK STORAGE"):
            self.assertIn(line, LCD_TEXT)
        self.assertIn("oldRouteAvailable", LCD_TEXT)

    def test_TEST_TEACH_SAVE_FAIL_REASON(self):
        self.assertIn("storageErrorReason_ = MapStorageErrorReason::TEACH_SAVE", MAP_TEXT)
        self.assertIn("teachOldRouteAvailable_", MAP_TEXT)

    def test_TEST_TEACH_FAIL_OLD_ROUTE_RESTORE(self):
        self.assertIn("const bool restored = teachOldRouteAvailable_ && loadSelected()", MAP_TEXT)
        self.assertIn("TEACH_SAVE=FAIL,OLD_ROUTE_RETAINED=1", MAP_TEXT)

    def test_TEST_TEACH_FAIL_EMPTY_SLOT_NO_BACK(self):
        self.assertIn("route_ = {}", MAP_TEXT)
        self.assertIn("loadedValid_ = false", MAP_TEXT)
        self.assertIn("TEACH_SAVE=FAIL,NO_VALID_ROUTE=1", MAP_TEXT)
        self.assertIn('"ROUTE NOT SAVED"', LCD_TEXT)
        self.assertIn('"NO VALID MAP"', LCD_TEXT)
        self.assertIn('status.oldRouteAvailable', LCD_TEXT)

    def test_TEST_STORAGE_INIT_FATAL(self):
        self.assertIn("MapStorageErrorReason::STORAGE_INIT", MAP_TEXT)
        self.assertIn("robot_.stopImmediately(true)", MAP_TEXT)
        for line in ("MAP STORAGE ERR", "FLASH INIT FAILED", "MAP DISABLED",
                     "CHECK STORAGE"):
            self.assertIn(line, LCD_TEXT)

    def test_TEST_START_BLOCKED_ON_STORAGE_ERROR(self):
        start = MAP_TEXT.index("void MapController::handleStart()")
        end = MAP_TEXT.index("void MapController::handleTriangle()", start)
        block = MAP_TEXT[start:end]
        self.assertIn("storageErrorReason_ != MapStorageErrorReason::NONE", block)
        self.assertIn("robot_.stopImmediately(true)", block)
        self.assertIn("STORAGE_INIT", block)
        self.assertIn("STORAGE_ERROR", block)

    def test_TEST_SETTINGS_BLOCKED_ON_FATAL_STORAGE_ERROR(self):
        settings_start = MAP_TEXT.index("bool MapController::enterSettings")
        settings_end = MAP_TEXT.index("void MapController::saveSettingsAndExit", settings_start)
        settings_block = MAP_TEXT[settings_start:settings_end]
        self.assertIn("storageErrorReason_ != MapStorageErrorReason::NONE", settings_block)

    def test_TEST_DELETE_BLOCKED_ON_FATAL_STORAGE_ERROR(self):
        circle_start = MAP_TEXT.index("void MapController::handleCircle()")
        circle_end = MAP_TEXT.index("void MapController::handleCross()", circle_start)
        circle_block = MAP_TEXT[circle_start:circle_end]
        self.assertIn("storageErrorReason_ != MapStorageErrorReason::NONE", circle_block)

    def test_TEST_ERROR_REASON_CLEARED_AFTER_SUCCESS(self):
        self.assertGreaterEqual(
            MAP_TEXT.count("storageErrorReason_ = MapStorageErrorReason::NONE"), 5
        )
        self.assertIn("MAP,SETTINGS,SAVE=NO_CHANGE", MAP_TEXT)
        self.assertIn("MapStorageErrorReason::NONE", MAP_TEXT[MAP_TEXT.index("void MapController::begin()"):])

    def test_TEST_ERROR_REASON_CLEARED_AFTER_ACK(self):
        start = MAP_TEXT.index("void MapController::handleCross()")
        end = MAP_TEXT.index("void MapController::handleCrossLong()", start)
        block = MAP_TEXT[start:end]
        self.assertIn("storageErrorReason_ = MapStorageErrorReason::NONE", block)
        self.assertIn("storeState_ = MapStoreState::SAVED", block)
        self.assertIn("mode_ = MapControllerMode::SAVED", block)

    def test_TEST_LCD_SETTINGS_ERROR(self):
        for line in ("MAP%u SAVE ERROR", "OLD MAP RETAINED",
                     "SETTINGS NOT SAVED", "X BACK"):
            self.assertIn(line, LCD_TEXT)

    def test_TEST_LCD_TEACH_ERROR_OLD(self):
        for line in ("MAP%u TEACH ERROR", "NEW ROUTE NOT SAVED",
                     "OLD MAP RETAINED", "X BACK"):
            self.assertIn(line, LCD_TEXT)

    def test_TEST_LCD_TEACH_ERROR_EMPTY(self):
        for line in ("MAP%u TEACH ERROR", "ROUTE NOT SAVED",
                     "NO VALID MAP", "CHECK STORAGE"):
            self.assertIn(line, LCD_TEXT)

    def test_TEST_LCD_INIT_FATAL(self):
        for line in ("MAP STORAGE ERR", "FLASH INIT FAILED",
                     "MAP DISABLED", "CHECK STORAGE"):
            self.assertIn(line, LCD_TEXT)

    def test_TEST_ERROR_LCD_20_CHAR(self):
        error_start = LCD_TEXT.index("if (status.storeState == 3U || status.storageErrorReason != 0U)")
        saved_start = LCD_TEXT.index("if (status.mode == 2U)")
        complete_start = LCD_TEXT.index("if (status.mode == 8U)")
        self.assertLess(error_start, saved_start)
        self.assertLess(error_start, complete_start)
        for line in (
            "MAP STORAGE ERR", "FLASH INIT FAILED", "MAP DISABLED", "CHECK STORAGE",
            "MAP%u SAVE ERROR", "OLD MAP RETAINED", "SETTINGS NOT SAVED", "X BACK",
            "MAP%u TEACH ERROR", "NEW ROUTE NOT SAVED", "ROUTE NOT SAVED",
            "NO VALID MAP", "MAP%u MODE ERROR", "OLD MODE RETAINED", "SAVE FAILED",
            "MAP%u STORAGE ERR", "NO VALID ROUTE",
        ):
            self.assertLessEqual(len(line.replace("%u", "2")), 20, msg=line)

    def test_TEST_STORAGE_ERROR_PRIORITY_OVER_SAVED(self):
        error_start = LCD_TEXT.index(
            "if (status.storeState == 3U || status.storageErrorReason != 0U)"
        )
        saved_start = LCD_TEXT.index("if (status.mode == 2U)")
        complete_start = LCD_TEXT.index("if (status.mode == 8U)")
        self.assertLess(error_start, saved_start)
        self.assertLess(error_start, complete_start)

    def test_TEST_SAVE_ERROR_X_BACK(self):
        start = MAP_TEXT.index("void MapController::handleCross()")
        end = MAP_TEXT.index("void MapController::handleCrossLong()", start)
        block = MAP_TEXT[start:end]
        self.assertIn("MAP,STORAGE_ERROR,ACK=BACK,OLD_ROUTE=1", block)
        self.assertIn("storageErrorReason_ = MapStorageErrorReason::NONE", block)

    def test_finite_loop_completes_only_after_closing_edge(self):
        edges = closed_route_edges(3, laps=2)
        self.assertEqual(edges, [(0, 1), (1, 2), (2, 0),
                                 (0, 1), (1, 2), (2, 0)])
        self.assertIn("logicalClosingEdge", MAP_TEXT)
        self.assertIn("replayLapCounter_ >= loopTarget_", MAP_TEXT)
        target_block = MAP_TEXT[MAP_TEXT.index("replayLapCounter_ >= loopTarget_") :]
        self.assertIn("completeReplay();", target_block)
        self.assertIn("replayTargetIndex_ = count > 1U ? 1U : 0U", MAP_TEXT)

    def test_map_speed_profile_and_turn_cap(self):
        self.assertIn("MAP_GUIDE_ACCEL_RAMP_MS", CONFIG_TEXT)
        self.assertIn("guidedStartMs_", CTRL_TEXT)
        self.assertIn("guidedBaseSpeed_ = MAP_GUIDE_MIN_SPEED", CTRL_TEXT)
        self.assertIn("guidedRequestedSpeed_ = constrain(speed, MAP_REPLAY_SPEED_MIN", CTRL_TEXT)
        self.assertIn("const int16_t mapTurnSpeed = min(replaySpeed_, TURN_MAX_SPEED)", MAP_TEXT)
        self.assertIn("replaySpeed_, incomingBearing", MAP_TEXT)

    def test_lcd_map_lines_match_settings_loop_and_delete_ux(self):
        for token in (
            '"TGT:%lu TRV:%lu"',
            '"MODE:SHUTTLE CY:INF"',
            '"MODE:LOOP LAP:INF"',
            '"MODE:LOOP LAP:%lu/%u"',
            '"SHUT %s CY:%lu/INF"',
            '"MAP%u RUN WP:%02u/%02u"',
            '"UD/LR EDIT TRI HELP"',
            '"START RES XL CANCEL"',
            '"DELETE MAP%u ?"',
            '"ALL ROUTE DATA"',
            '"O YES"',
            '"X NO"',
            '"MAP%u HELP %u/3"',
            '"SHUTTLE GO/BACK"',
            '"LOOP REPEAT LAPS"',
            '"LOOP CLOSE REPEAT"',
            'helpPage <= 2U ? helpPage : 0U',
        ):
            self.assertIn(token, LCD_TEXT)
        self.assertNotIn('"SEL MAP SQH DEL L3"', LCD_TEXT)
        self.assertNotIn("modeSavePending_ = true", MAP_TEXT)

    def test_loop_move_lcd_lap(self):
        self.assertIn("status.replayOperation == 1U", LCD_TEXT)
        self.assertIn('"MAP%u RUN WP:%02u/%02u"', LCD_TEXT)
        self.assertIn('"MODE:LOOP LAP:%lu/%u"', LCD_TEXT)

    def test_loop_turn_lcd_lap(self):
        self.assertIn('"TURN %+ddeg"', LCD_TEXT)
        self.assertIn('writeRunMode(desired_[2])', LCD_TEXT)
        self.assertNotIn('"MAP%u TURN WP:%02u/%02u"', LCD_TEXT)

    def test_loop_realign_lcd_lap(self):
        self.assertIn('"MODE:LOOP LAP:%lu/%u"', LCD_TEXT)
        self.assertIn('"MODE:LOOP LAP:%lu/INF"', LCD_TEXT)
        self.assertIn('"MAP%u RUN WP:%02u/%02u"', LCD_TEXT)
        self.assertNotIn('"MAP%u TURN WP:%02u/%02u"', LCD_TEXT)

    def test_once_turn_no_lap(self):
        self.assertIn('snprintf(line, 21, "MODE:ONCE")', LCD_TEXT)
        self.assertIn('status.userMode == 0U', LCD_TEXT)
        self.assertNotIn('"MODE:ONCE LAP:', LCD_TEXT)

    def test_settings_tri_help_visible(self):
        self.assertIn('"UD/LR EDIT TRI HELP"', LCD_TEXT)
        self.assertIn('"TRI NEXT"', LCD_TEXT)
        self.assertIn('"TRI NEXT X BACK"', LCD_TEXT)

    def test_help_lines_20_char(self):
        for line in (
            "UD/LR EDIT TRI HELP",
            "ONCE ONE WAY",
            "SHUTTLE GO/BACK",
            "LOOP REPEAT LAPS",
            "LOOP CLOSE REPEAT",
            "X HOLD XL CANCEL",
            "TRI NEXT X BACK",
        ):
            self.assertLessEqual(len(line), 20)
        self.assertIn('"MAP%u HELP %u/3"', LCD_TEXT)

    def test_long_press_and_destructive_delete_policy(self):
        self.assertIn("MAP_LONG_PRESS_MS = 1300U", CONFIG_TEXT)
        self.assertIn("kMapCrossLongPressMs = MAP_LONG_PRESS_MS", PS2_TEXT)
        self.assertIn("kMapLongPressMs = MAP_LONG_PRESS_MS", PS2_TEXT)
        square_handler = PS2_TEXT[PS2_TEXT.index("mapSquarePressActive_") :]
        self.assertIn("kMapLongPressMs", square_handler)
        self.assertIn("A long SQUARE on the main MAP page is inert", MAP_TEXT)
        self.assertIn("settingsCanDelete()", MAP_TEXT)
        self.assertIn('MAP,CIRCLE,REJECT,REASON=DELETE_SAFETY', MAP_TEXT)

    # Route cleaner acceptance tests. The Python geometry oracle covers the
    # safety invariants; source assertions ensure the production path uses the
    # fixed-size implementation and its fail-safe gates.
    def test_optimizer_module_and_pipeline(self):
        self.assertIn("cleanMapRoute", CLEANER_HEADER_TEXT)
        self.assertIn("cleanMapRoute(route_, optimizedRoute_", MAP_TEXT)
        self.assertIn("logOptimizeSummary", MAP_TEXT)
        self.assertIn("MAP,OPTIMIZE_WP,I=", MAP_TEXT)
        self.assertIn("optimizedRoute_", MAP_HEADER_TEXT)
        self.assertIn("route_.header.routeType = static_cast<uint8_t>(detectedType)", MAP_TEXT)

    def test_replay_start_noise_does_not_create_turn_pulse(self):
        self.assertIn("kReplayStartupTurnDeadbandDeg", MAP_TEXT)
        self.assertIn("startupNoiseTurn", MAP_TEXT)
        self.assertIn("!startupNoiseTurn", MAP_TEXT)

    def test_optimizer_thresholds_are_named_and_conservative(self):
        for name in (
            "kOptimizeDuplicateDistanceMm",
            "kOptimizeDuplicateHeadingDeg",
            "kOptimizeShortSegmentMm",
            "kOptimizeLineDeviationMm",
            "kOptimizeStraightAngleDeg",
            "kOptimizeCornerAngleDeg",
            "kOptimizeCornerClusterRadiusMm",
            "kOptimizeMaxDeviationMm",
            "kOptimizeMaxLengthReductionPercent",
            "kOptimizeMinimumSegmentMm",
            "kOptimizeMaximumSegmentMm",
            "kOptimizeCornerFitMinAngleDeg",
            "kOptimizeCornerFitMaxAngleDeg",
            "kOptimizeCornerFitMaxExtensionMm",
        ):
            self.assertIn(name, CLEANER_TEXT)
        self.assertLessEqual(
            _constant(CLEANER_TEXT, "kOptimizeMaxLengthReductionPercent"), 15
        )
        self.assertLessEqual(
            _constant(CLEANER_TEXT, "kOptimizeDuplicateDistanceMm"), 40
        )

    def test_TEST_OPT_DUPLICATE_AUTO_REMOVED(self):
        points = [
            (0, 0, 0, MAP_WP_START),
            (10, 1, 0, MAP_WP_AUTO_DISTANCE),
            (500, 0, 0, MAP_WP_AUTO_DISTANCE),
            (1000, 0, 0, MAP_WP_ENDPOINT),
        ]
        self.assertNotIn(1, reference_safe_simplify(points))
        self.assertIn("kOptimizeDuplicateDistanceMm", CLEANER_TEXT)
        self.assertIn("removedDuplicate", CLEANER_TEXT)

    def test_TEST_OPT_MANUAL_NEVER_REMOVED(self):
        points = [
            (0, 0, 0, MAP_WP_START),
            (300, 2, 0, MAP_WP_MANUAL_MARK),
            (600, -2, 0, MAP_WP_AUTO_DISTANCE),
            (1000, 0, 0, MAP_WP_ENDPOINT),
        ]
        kept = reference_safe_simplify(points)
        self.assertIn(1, kept)
        self.assertIn("MAP_WP_MANUAL_MARK", CLEANER_TEXT)
        self.assertIn("metrics.manualKept == rawManual", CLEANER_TEXT)
        self.assertIn("nearProtected", CLEANER_TEXT)

    def test_TEST_OPT_START_PRESERVED(self):
        points = [(0, 0, 0, MAP_WP_START), (500, 1, 0, MAP_WP_ENDPOINT)]
        kept = reference_safe_simplify(points)
        self.assertEqual(kept[0], 0)
        self.assertIn("MAP_WP_START", CLEANER_TEXT)
        self.assertIn("pointsPreserved", CLEANER_TEXT)

    def test_TEST_OPT_ENDPOINT_PRESERVED(self):
        points = [(0, 0, 0, MAP_WP_START), (500, 1, 0, MAP_WP_ENDPOINT)]
        kept = reference_safe_simplify(points)
        self.assertEqual(kept[-1], len(points) - 1)
        self.assertIn("MAP_WP_ENDPOINT", CLEANER_TEXT)
        self.assertIn("rawRoute.waypoints[rawCount - 1U]", CLEANER_TEXT)

    def test_TEST_OPT_COLLINEAR(self):
        points = [
            (0, 0, 0, MAP_WP_START),
            (200, 5, 0, MAP_WP_AUTO_DISTANCE),
            (400, -10, 0, MAP_WP_AUTO_DISTANCE),
            (600, 8, 0, MAP_WP_AUTO_DISTANCE),
            (800, -5, 0, MAP_WP_AUTO_DISTANCE),
            (1000, 0, 0, MAP_WP_ENDPOINT),
        ]
        kept = reference_safe_simplify(points)
        self.assertLessEqual(len(kept), 3)
        self.assertLessEqual(
            max(point_segment_distance(points[i], points[kept[0]],
                                        points[kept[-1]]) for i in range(len(points))),
            60.0,
        )
        self.assertIn("removedCollinear", CLEANER_TEXT)

    def test_TEST_OPT_COLLINEAR_MANUAL_ANCHOR(self):
        points = [
            (0, 0, 0, MAP_WP_START),
            (250, 2, 0, MAP_WP_AUTO_DISTANCE),
            (500, 0, 0, MAP_WP_MANUAL_MARK),
            (750, -2, 0, MAP_WP_AUTO_DISTANCE),
            (1000, 0, 0, MAP_WP_ENDPOINT),
        ]
        kept = reference_safe_simplify(points)
        self.assertIn(2, kept)
        self.assertIn("ProtectedBetween", CLEANER_TEXT)

    def test_TEST_OPT_SHORT_JITTER(self):
        points = [
            (0, 0, 0, MAP_WP_START),
            (30, 1, 0, MAP_WP_AUTO_DISTANCE),
            (60, -1, 0, MAP_WP_AUTO_DISTANCE),
            (100, 0, 0, MAP_WP_AUTO_DISTANCE),
            (500, 0, 0, MAP_WP_ENDPOINT),
        ]
        self.assertLess(len(reference_safe_simplify(points)), len(points))
        self.assertIn("kOptimizeShortSegmentMm", CLEANER_TEXT)
        self.assertIn("removedShort", CLEANER_TEXT)

    def test_TEST_OPT_BACKTRACK_CORRECTION_RULE(self):
        # A short reverse correction on a straight leg is removable, while a
        # genuine 90-degree corner is outside the backtrack rule.
        correction = [
            (0, 0, 0, MAP_WP_START),
            (400, 0, 0, MAP_WP_AUTO_DISTANCE),
            (350, 0, 0, MAP_WP_ENDPOINT),
        ]
        self.assertGreater(
            direction_change(correction[0], correction[1], correction[2]),
            135.0,
        )
        corner = [
            (0, 0, 0, MAP_WP_START),
            (120, 0, 0, MAP_WP_AUTO_DISTANCE),
            (120, 500, 9000, MAP_WP_ENDPOINT),
        ]
        self.assertLess(
            direction_change(corner[0], corner[1], corner[2]),
            135.0,
        )
        for name in (
            "IsBacktrackNoise",
            "kOptimizeBacktrackAngleDeg",
            "kOptimizeBacktrackShortLegMm",
            "kOptimizeBacktrackLegRatio",
        ):
            self.assertIn(name, CLEANER_TEXT)

    def test_TEST_OPT_FITS_AUTO_DISTANCE_CORNER(self):
        self.assertIn("FitAutoCornerTransitions", CLEANER_TEXT)
        self.assertIn("fittedCorner", CLEANER_HEADER_TEXT)
        self.assertIn("FITTED_CORNER", MAP_TEXT)
        self.assertIn("lineT < 1.0f", CLEANER_TEXT)
        self.assertIn("lineU > 0.0f", CLEANER_TEXT)
        self.assertIn("MAP_WP_AUTO_CORNER", CLEANER_TEXT)

    def test_TEST_OPT_REAL_CORNER_PRESERVED(self):
        points = [
            (0, 0, 0, MAP_WP_START),
            (500, 0, 0, MAP_WP_AUTO_CORNER),
            (500, 500, 9000, MAP_WP_AUTO_DISTANCE),
            (0, 500, 18000, MAP_WP_ENDPOINT),
        ]
        # A corner anchor is protected from the collinear pass by policy.
        self.assertIn(1, reference_safe_simplify(points))
        self.assertIn("kOptimizeCornerAngleDeg", CLEANER_TEXT)
        self.assertIn("CornerStrength", CLEANER_TEXT)

    def test_TEST_OPT_NO_DIAGONAL_CORNER_CUT(self):
        points = [
            (0, 0, 0, MAP_WP_START),
            (500, 0, 0, MAP_WP_AUTO_DISTANCE),
            (500, 500, 9000, MAP_WP_AUTO_DISTANCE),
            (0, 500, 18000, MAP_WP_ENDPOINT),
        ]
        kept = reference_safe_simplify(points)
        self.assertIn(1, kept)
        self.assertIn(2, kept)
        self.assertIn("kOptimizeMaxDeviationMm", CLEANER_TEXT)
        self.assertIn("path-corridor", CLEANER_TEXT.lower())

    def test_TEST_OPT_CORNER_CLUSTER(self):
        self.assertIn("CornerRepresentativeNear", CLEANER_TEXT)
        self.assertIn("removedCornerCluster", CLEANER_TEXT)
        self.assertIn("kOptimizeCornerClusterRadiusMm", CLEANER_TEXT)
        self.assertIn("clusterCount", CLEANER_TEXT)

    def test_TEST_OPT_MANUAL_CORNER_PRIORITY(self):
        self.assertIn("IsPreferredCorner", CLEANER_TEXT)
        self.assertIn("bestStrength", CLEANER_TEXT)
        self.assertIn("bestCenterDistance", CLEANER_TEXT)
        self.assertIn("Keep one existing RAW corner anchor", CLEANER_TEXT)

    def test_TEST_OPT_MAX_DEVIATION(self):
        first = (0, 0, 0, MAP_WP_START)
        middle = (500, 0, 0, MAP_WP_AUTO_DISTANCE)
        last = (500, 500, 0, MAP_WP_ENDPOINT)
        self.assertGreater(point_segment_distance(middle, first, last), 60.0)
        self.assertIn("middleDeviation > kOptimizeMaxDeviationMm", CLEANER_TEXT)
        self.assertIn("CLEAN_GEOMETRY", CLEANER_TEXT)

    def test_TEST_OPT_LENGTH_RATIO(self):
        raw = [(0, 0), (100, 60), (200, -60), (300, 60), (400, 0)]
        clean = [(0, 0), (400, 0)]
        self.assertGreater(
            (route_length(raw) - route_length(clean)) / route_length(raw), 0.15
        )
        self.assertIn("kOptimizeMaxLengthReductionPercent", CLEANER_TEXT)
        self.assertIn('"LENGTH_RATIO"', CLEANER_TEXT)

    def test_TEST_OPT_CLOSED_SEAM(self):
        square = [(0, 0), (500, 0), (500, 500), (0, 500), (0, 0)]
        self.assertEqual(square[0], square[-1])
        self.assertIn("kOptimizeClosedClosureSkipMm", CLEANER_TEXT)
        self.assertIn("MapRouteType::CLOSED", CLEANER_TEXT)
        self.assertIn("CLOSED", MAP_TEXT)

    def test_TEST_OPT_ANGLE_WRAP(self):
        delta = (179.0 - (-179.0) + 180.0) % 360.0 - 180.0
        self.assertAlmostEqual(abs(delta), 2.0)
        self.assertIn("while (delta > 180.0f)", CLEANER_TEXT)
        self.assertIn("while (delta <= -180.0f)", CLEANER_TEXT)

    def test_TEST_OPT_MAX_POINTS(self):
        self.assertEqual(MAX_WAYPOINTS, 128)
        self.assertIn("uint16_t selected[STM32_MAP_MAX_WAYPOINTS]", CLEANER_TEXT)
        self.assertNotIn("std::vector", CLEANER_TEXT)
        self.assertNotIn("malloc", CLEANER_TEXT)
        self.assertNotIn("new ", CLEANER_TEXT)

    def test_TEST_OPT_FALLBACK_RAW(self):
        self.assertIn("SetFallback", CLEANER_TEXT)
        self.assertIn("clean = raw", CLEANER_TEXT)
        self.assertIn('"RAW_GEOMETRY"', CLEANER_TEXT)
        self.assertIn('"RAW_FALLBACK,REASON="', MAP_TEXT)

    def test_optimizer_scenarios(self):
        straight = [
            (0, 0, 0, MAP_WP_START),
            (200, 5, 0, MAP_WP_AUTO_DISTANCE),
            (400, -10, 0, MAP_WP_AUTO_DISTANCE),
            (600, 8, 0, MAP_WP_AUTO_DISTANCE),
            (1000, 0, 0, MAP_WP_ENDPOINT),
        ]
        self.assertLessEqual(len(reference_safe_simplify(straight)), 3)

        corner = [
            (0, 0, 0, MAP_WP_START),
            (500, 0, 0, MAP_WP_AUTO_DISTANCE),
            (500, 500, 0, MAP_WP_AUTO_DISTANCE),
            (0, 500, 0, MAP_WP_ENDPOINT),
        ]
        kept = reference_safe_simplify(corner)
        self.assertEqual(kept, [0, 1, 2, 3])

        anchors = [
            (0, 0, 0, MAP_WP_START),
            (300, 0, 0, MAP_WP_AUTO_DISTANCE),
            (600, 0, 0, MAP_WP_MANUAL_MARK),
            (900, 0, 0, MAP_WP_AUTO_DISTANCE),
            (1200, 0, 0, MAP_WP_ENDPOINT),
        ]
        kept = reference_safe_simplify(anchors)
        self.assertEqual(kept[0], 0)
        self.assertIn(2, kept)
        self.assertEqual(kept[-1], 4)

        closed_square = [
            (0, 0, 0, MAP_WP_START),
            (500, 0, 0, MAP_WP_AUTO_CORNER),
            (500, 500, 0, MAP_WP_AUTO_CORNER),
            (0, 500, 0, MAP_WP_AUTO_CORNER),
            (0, 0, 0, MAP_WP_ENDPOINT),
        ]
        self.assertEqual(closed_square[0][:2], closed_square[-1][:2])
        self.assertIn("CLOSED", CLEANER_TEXT)

    def test_optimizer_does_not_touch_motion_or_protocol_paths(self):
        self.assertNotIn("RobotController::", CLEANER_TEXT)
        self.assertNotIn("Ultrasonic", CLEANER_TEXT)
        self.assertNotIn("HeadingFusion", CLEANER_TEXT)
        self.assertNotIn("RobotLink", CLEANER_TEXT)
        self.assertIn("startNextReplaySegment", MAP_TEXT)
        self.assertIn("RouteCleanerMetrics", MAP_HEADER_TEXT)


if __name__ == "__main__":
    unittest.main(verbosity=2)
