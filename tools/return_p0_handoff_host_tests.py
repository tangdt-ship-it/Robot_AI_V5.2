"""Static and behavior-model contracts for the P1-to-P0 Return handoff."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAP = (ROOT / "firmware/stm32/src/map/map_controller.cpp").read_text(
    encoding="utf-8"
)
CONFIG = (ROOT / "firmware/stm32/include/robot_config.h").read_text(
    encoding="utf-8"
)


def require(text: str, needle: str) -> int:
    assert needle in text, needle
    return text.index(needle)


def start_plan(cross_track_mm: float, heading_error_deg: float) -> list[str]:
    """Model only the new P1-to-P0 handoff decision boundary."""
    if cross_track_mm <= 30.0:
        if abs(heading_error_deg) > 0.5:
            return ["BYPASS_REACQUIRE", "COARSE_TURN", "GUIDED_P1"]
        return ["BYPASS_REACQUIRE", "GUIDED_P1"]
    return ["GUIDED_REACQUIRE"]


def main() -> None:
    require(CONFIG, "MAP_RETURN_P0_REACQUIRE_BYPASS_MM = 30U")

    reacquire = MAP[MAP.index("bool MapController::startReturnReacquire()"):MAP.index(
        "bool MapController::startReturnWaypoint()"
    )]
    bypass = require(reacquire, "MAP_RETURN_P0_REACQUIRE_BYPASS_MM")
    assert bypass < reacquire.index("++returnP0ReacquireAttempts_")
    require(reacquire, "returnP0State_ = ReturnP0State::RETURN_WAYPOINT")
    require(reacquire, "return startReturnWaypoint();")

    waypoint = MAP[MAP.index("bool MapController::startReturnWaypoint()"):MAP.index(
        "bool MapController::startReturnP0Position()"
    )]
    require(waypoint, "const float headingError = shortestDeltaDeg")
    require(waypoint, "fabsf(headingError) > MAP_REPLAY_PRETURN_TOLERANCE_DEG")
    turn = require(waypoint, "AiTurnProfile::MAP_COARSE")
    guided = require(waypoint, "robot_.startReplayGuidedWaypoint")
    assert turn < guided, "Return guided motion must not start before coarse pre-turn"
    require(waypoint, 'logReturnP0TraceMotion("WP_PRETURN", current)')

    # Observed HIL geometry: 9.4 mm from the projection, current heading
    # about +92 degrees while P1->P0 requires about -86 degrees.
    assert start_plan(9.4, -178.0) == [
        "BYPASS_REACQUIRE",
        "COARSE_TURN",
        "GUIDED_P1",
    ]
    assert start_plan(9.4, 0.3) == ["BYPASS_REACQUIRE", "GUIDED_P1"]
    assert start_plan(31.0, -178.0) == ["GUIDED_REACQUIRE"]

    print("RETURN_P0_HANDOFF_STATIC=PASS")


if __name__ == "__main__":
    main()
