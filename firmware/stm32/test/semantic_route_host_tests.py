"""Host geometry tests for the fixed-size semantic MAP optimizer.

The production implementation is deliberately MCU-only, so these tests keep a
small numerical oracle for the safety decisions and inspect the C++ source for
the bounded/fail-safe integration points.
"""

import math
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
HEADER = (ROOT / "include" / "map" / "semantic_route_optimizer.h").read_text(
    encoding="utf-8"
)
SOURCE = (ROOT / "src" / "map" / "semantic_route_optimizer.cpp").read_text(
    encoding="utf-8"
)
MAP_SOURCE = (ROOT / "src" / "map" / "map_controller.cpp").read_text(
    encoding="utf-8"
)


START = 1 << 0
MANUAL = 1 << 1
AUTO = 1 << 2
AUTO_CORNER = 1 << 3
ENDPOINT = 1 << 4


def direction(first, second):
    return math.degrees(math.atan2(second[1] - first[1], second[0] - first[0]))


def angle_delta(first, second):
    delta = first - second
    while delta > 180:
        delta -= 360
    while delta <= -180:
        delta += 360
    return abs(delta)


def path_length(points):
    return sum(math.dist(first, second) for first, second in zip(points, points[1:]))


def line_fit(points):
    mean_x = sum(point[0] for point in points) / len(points)
    mean_y = sum(point[1] for point in points) / len(points)
    xx = sum((point[0] - mean_x) ** 2 for point in points)
    yy = sum((point[1] - mean_y) ** 2 for point in points)
    xy = sum((point[0] - mean_x) * (point[1] - mean_y) for point in points)
    theta = 0.5 * math.atan2(2 * xy, xx - yy)
    ux, uy = math.cos(theta), math.sin(theta)
    if (points[-1][0] - points[0][0]) * ux + (points[-1][1] - points[0][1]) * uy < 0:
        ux, uy = -ux, -uy
    residual = max(
        abs(ux * (point[1] - mean_y) - uy * (point[0] - mean_x))
        for point in points
    )
    return (mean_x, mean_y, ux, uy, residual, path_length(points))


def line_intersection(first, second):
    determinant = first[2] * second[3] - first[3] * second[2]
    if abs(determinant) < 0.01:
        return None
    dx, dy = second[0] - first[0], second[1] - first[1]
    t = (dx * second[3] - dy * second[2]) / determinant
    return first[0] + t * first[2], first[1] + t * first[3]


def point_segment_distance(point, first, second):
    dx, dy = second[0] - first[0], second[1] - first[1]
    length_squared = dx * dx + dy * dy
    if length_squared == 0:
        return math.dist(point, first)
    t = max(0.0, min(1.0, ((point[0] - first[0]) * dx + (point[1] - first[1]) * dy) /
                         length_squared))
    return math.dist(point, (first[0] + t * dx, first[1] + t * dy))


def corner_path_deviation(points, begin, end, corner):
    return max(
        min(point_segment_distance(points[index], points[begin], corner),
            point_segment_distance(points[index], corner, points[end]))
        for index in range(begin, end + 1)
    )


def find_first_semantic_corner(points):
    """Mirror the bounded one-corner search used by the MCU implementation."""
    directions = [direction(points[index], points[index + 1])
                  for index in range(len(points) - 1)]
    first = next(
        (index for index in range(1, len(directions))
         if angle_delta(directions[index], directions[0]) >= 25.0),
        None,
    )
    if first is None:
        return None
    for outgoing_start in range(first, len(directions)):
        outgoing_last = len(directions) - 1
        for index in range(outgoing_start + 1, len(directions)):
            if angle_delta(directions[index], directions[outgoing_start]) >= 25.0:
                outgoing_last = index - 1
                break
        incoming = line_fit(points[: first + 1])
        outgoing = line_fit(points[outgoing_start: outgoing_last + 2])
        if incoming[4] > 60.0 or outgoing[4] > 60.0:
            continue
        if incoming[5] < 200.0 or outgoing[5] < 200.0:
            continue
        angle = angle_delta(
            math.degrees(math.atan2(outgoing[3], outgoing[2])),
            math.degrees(math.atan2(incoming[3], incoming[2])),
        )
        if not 25.0 <= angle <= 150.0:
            continue
        corner = line_intersection(incoming, outgoing)
        if corner is None:
            continue
        shift = min(math.dist(corner, points[index])
                    for index in range(first, outgoing_start + 1))
        deviation = corner_path_deviation(points, 0, outgoing_last + 1, corner)
        synthetic_safe = (
            shift <= 100.0 and deviation <= 60.0 and
            20.0 <= math.dist(points[0], corner) <= 5000.0 and
            20.0 <= math.dist(corner, points[outgoing_last + 1]) <= 5000.0
        )
        return {
            "first": first,
            "outgoing_start": outgoing_start,
            "outgoing_end": outgoing_last + 1,
            "corner": corner,
            "angle": angle,
            "shift": shift,
            "deviation": deviation,
            "synthetic_safe": synthetic_safe,
        }
    return {"first": first, "synthetic_safe": False}


class SemanticRouteHostTests(unittest.TestCase):
    def test_module_is_fixed_size_and_separate(self):
        self.assertIn("optimizeSemanticRoute", MAP_SOURCE)
        self.assertIn("semantic_route_optimizer.h", (ROOT / "include" / "map" / "map_controller.h").read_text(encoding="utf-8"))
        self.assertIn("MapRouteData semanticRoute_", (ROOT / "include" / "map" / "map_controller.h").read_text(encoding="utf-8"))
        self.assertIn("SemanticCornerDiagnostic corners[STM32_MAP_MAX_WAYPOINTS]", HEADER)
        self.assertNotRegex(SOURCE, r"\b(std::vector|malloc|calloc|realloc|new\s)\b")

    def test_pipeline_runs_clean_then_semantic_and_saves_one_route(self):
        clean = MAP_SOURCE.index("cleanMapRoute")
        semantic = MAP_SOURCE.index("optimizeSemanticRoute")
        self.assertLess(clean, semantic)
        self.assertIn("route_ = semanticRoute_", MAP_SOURCE)
        self.assertIn("queueTeachSave", MAP_SOURCE)

    def test_named_geometry_gates(self):
        for name in (
            "kSemanticStraightDirectionDeg",
            "kSemanticMinStraightLengthMm",
            "kSemanticTurnAngleDeg",
            "kSemanticLineFitMaxResidualMm",
            "kSemanticMaxCornerShiftMm",
            "kSemanticMaxPathDeviationMm",
            "kSemanticMaxLengthReductionPercent",
            "kSemanticMinSegmentMm",
            "kSemanticMaxSegmentMm",
            "kSemanticClosedClosureSkipMm",
        ):
            self.assertIn(name, SOURCE)
        self.assertIn("SemanticType", SOURCE)
        self.assertLessEqual(_constant(SOURCE, "kSemanticStraightDirectionDeg"), 12)
        self.assertLessEqual(_constant(SOURCE, "kSemanticMaxCornerShiftMm"), 100)
        self.assertLessEqual(_constant(SOURCE, "kSemanticMaxPathDeviationMm"), 60)
        self.assertLessEqual(_constant(SOURCE, "kSemanticMaxLengthReductionPercent"), 15)

    def test_straight_jitter_is_one_run(self):
        points = [(0, 0), (150, 6), (300, -8), (450, 5), (600, -4), (800, 0)]
        self.assertIsNone(find_first_semantic_corner(points))
        self.assertGreaterEqual(line_fit(points)[5], 200)

    def test_90_degree_noisy_corner_is_one_region(self):
        points = [
            (0, 0), (200, 5), (400, -7), (580, 5), (600, 25),
            (615, 55), (610, 95), (600, 150), (605, 300), (600, 500),
        ]
        corner = find_first_semantic_corner(points)
        self.assertIsNotNone(corner)
        self.assertTrue(corner["synthetic_safe"])
        self.assertAlmostEqual(corner["angle"], 90.0, delta=5.0)
        self.assertLessEqual(corner["shift"], 100.0)

    def test_no_diagonal_cut(self):
        points = [(0, 0), (600, 0), (600, 600)]
        corner = find_first_semantic_corner(points)
        self.assertTrue(corner["synthetic_safe"])
        self.assertAlmostEqual(corner["corner"][0], 600.0, delta=1.0)
        self.assertAlmostEqual(corner["corner"][1], 0.0, delta=1.0)
        self.assertLessEqual(corner["deviation"], 60.0)
        self.assertNotEqual(points[0], points[-1])

    def test_noisy_turn_has_one_corner_not_many(self):
        points = [
            (0, 0), (400, 0), (520, 4), (570, 18), (590, 45),
            (600, 90), (602, 180), (600, 350), (600, 700),
        ]
        corner = find_first_semantic_corner(points)
        self.assertIsNotNone(corner)
        self.assertTrue(corner["synthetic_safe"])
        self.assertGreaterEqual(corner["angle"], 25.0)

    def test_safe_and_unsafe_synthetic_corner_rules(self):
        safe = [(0, 0), (600, 0), (600, 600)]
        unsafe = [(0, 0), (600, 0), (900, 600), (1200, 900), (1500, 1200)]
        self.assertTrue(find_first_semantic_corner(safe)["synthetic_safe"])
        self.assertFalse(find_first_semantic_corner(unsafe)["synthetic_safe"])
        self.assertIn("SetFallback", SOURCE)
        self.assertIn("CORNER_UNSAFE", SOURCE)

    def test_shallow_parallel_vertical_horizontal_and_u_turn_safe(self):
        shallow = [(0, 0), (300, 0), (600, 53), (900, 106)]
        self.assertIsNone(find_first_semantic_corner(shallow))
        parallel = [(0, 0), (300, 0), (600, 1), (900, 2)]
        self.assertIsNone(find_first_semantic_corner(parallel))
        vertical = [(0, 0), (0, 400), (0, 800)]
        horizontal = [(0, 0), (400, 0), (800, 0)]
        self.assertIsNone(find_first_semantic_corner(vertical))
        self.assertIsNone(find_first_semantic_corner(horizontal))
        u_turn = [(0, 0), (500, 0), (300, 0), (300, 400)]
        candidate = find_first_semantic_corner(u_turn)
        self.assertTrue(candidate is None or not candidate.get("synthetic_safe", False))
        self.assertIn("kSemanticMaxCornerAngleDeg", SOURCE)

    def test_manual_and_closed_boundaries(self):
        self.assertIn("ProtectedAnchorsUnchanged", SOURCE)
        self.assertIn("MANUAL_MARK", SOURCE)
        self.assertIn("RecordManualCorners", SOURCE)
        self.assertIn("sectionBegin", SOURCE)
        self.assertIn("MapRouteType::CLOSED", SOURCE)
        self.assertIn("not fitted", SOURCE)
        self.assertIn("START", SOURCE)
        self.assertIn("ENDPOINT", SOURCE)

    def test_metrics_and_fallback_logging(self):
        for token in (
            "MAP,SEMANTIC",
            "SEM_POINTS",
            "STRAIGHTS",
            "TURN_REGIONS",
            "SYNTH_CORNERS",
            "RAW_CORNERS_USED",
            "MANUAL_KEPT",
            "MAP,CORNER,IDX=",
            "SOURCE=",
            "RESULT=",
            "FALLBACK_CLEAN,REASON=",
        ):
            self.assertIn(token, MAP_SOURCE)


def _constant(text, name):
    match = re.search(r"\b" + re.escape(name) + r"\s*=\s*(\d+)", text)
    if not match:
        raise AssertionError(f"constant {name} not found")
    return float(match.group(1))


if __name__ == "__main__":
    unittest.main(verbosity=2)
