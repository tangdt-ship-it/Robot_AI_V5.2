"""Static gate contracts for the P1-to-P0 replay-guided handoff."""

from math import isfinite, hypot
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ROBOT = (ROOT / "firmware/stm32/src/control/robot_controller.cpp").read_text(
    encoding="utf-8"
)
MAP = (ROOT / "firmware/stm32/src/map/map_controller.cpp").read_text(
    encoding="utf-8"
)


def reject_reason(**state: object) -> str:
    """Host model preserving the source gate order."""
    if state["brake"]:
        return "BRAKE"
    if state["ai_active"]:
        return "AI_MOTION_ACTIVE"
    if state["ps2_timeout"]:
        return "PS2_FRAME_TIMEOUT"
    if state["map_ui"]:
        return "MAP_UI_CAPTURE"
    if state["owner"] != "NONE":
        return "MOTION_OWNER"
    if state["ps2_motion"]:
        return "PS2_MOTION_ACTIVE"
    if not state["odom_ready"]:
        return "ODOM_NOT_READY"
    if not state["odom_healthy"]:
        return "ODOM_UNHEALTHY"
    if not state["heading"]:
        return "HEADING_UNAVAILABLE"
    if not (isfinite(state["target_x"]) and isfinite(state["target_y"])):
        return "TARGET_NONFINITE"
    if not (isfinite(state["seg_x"]) and isfinite(state["seg_y"])):
        return "SEGMENT_NONFINITE"
    if not isfinite(state["bearing"]):
        return "BEARING_NONFINITE"
    if state["tol"] == 0:
        return "TOLERANCE_ZERO"
    if state["tol"] > 30:
        return "TOLERANCE_RANGE"
    distance = hypot(state["target_x"] - state["seg_x"],
                     state["target_y"] - state["seg_y"])
    if distance <= 0:
        return "DISTANCE_ZERO"
    if distance > 5000:
        return "DISTANCE_RANGE"
    return "NONE"


def valid(**overrides: object) -> dict[str, object]:
    state: dict[str, object] = dict(
        brake=False, ai_active=False, ps2_timeout=False, map_ui=False,
        owner="NONE", ps2_motion=False, odom_ready=True,
        odom_healthy=True, heading=True, target_x=109.5, target_y=-15.6,
        seg_x=908.1, seg_y=-134.4, bearing=-171.5, tol=30,
    )
    state.update(overrides)
    return state


def main() -> None:
    for name in (
        "BRAKE", "AI_MOTION_ACTIVE", "PS2_FRAME_TIMEOUT", "MAP_UI_CAPTURE",
        "MOTION_OWNER", "PS2_MOTION_ACTIVE", "ODOM_NOT_READY",
        "ODOM_UNHEALTHY", "HEADING_UNAVAILABLE", "TARGET_NONFINITE",
        "SEGMENT_NONFINITE", "BEARING_NONFINITE", "TOLERANCE_ZERO",
        "TOLERANCE_RANGE", "DISTANCE_ZERO", "DISTANCE_RANGE",
    ):
        assert f"ReplayGuidedStartReject::{name}" in ROBOT
    for field in (
        "ROBOT,REPLAY_GUIDED_START_REJECT,REASON=", ",OWNER=", ",AI_MODE=",
        ",BRAKE=", ",PS2_FRESH=", ",PS2_TIMEOUT=", ",PS2_MOTION=",
        ",MAP_UI_CAPTURE=", ",ODOM_READY=", ",ODOM_HEALTHY=",
        ",HEADING_AVAILABLE=", ",TARGET_X=", ",TARGET_Y=", ",SEG_X=",
        ",SEG_Y=", ",BEARING=", ",TOL=", ",DIST=", ",GEN=",
    ):
        assert field in ROBOT
    assert "MAP,RETURN_P0,HANDOFF,FROM=P1,TO=P0,STATE=" in MAP

    # A: a completed P1 leaves the owner/mode clear and an ~800 mm P1->P0
    # request with its contractual 30 mm tolerance is admitted.
    assert reject_reason(**valid()) == "NONE"
    assert reject_reason(**valid(ps2_timeout=True)) == "PS2_FRAME_TIMEOUT"
    assert reject_reason(**valid(ps2_motion=True)) == "PS2_MOTION_ACTIVE"
    assert reject_reason(**valid(brake=True)) == "BRAKE"
    assert reject_reason(**valid(map_ui=True)) == "MAP_UI_CAPTURE"
    assert reject_reason(**valid(odom_healthy=False)) == "ODOM_UNHEALTHY"
    assert reject_reason(**valid(heading=False)) == "HEADING_UNAVAILABLE"
    assert reject_reason(**valid(owner="PS2")) == "MOTION_OWNER"
    assert reject_reason(**valid(tol=31)) == "TOLERANCE_RANGE"
    assert reject_reason(**valid(target_x=908.1, target_y=-134.4)) == "DISTANCE_ZERO"

    # Current source explicitly applies passive PS2 freshness before REPLAY
    # ownership, documenting the design-mismatch candidate without changing it.
    can_start = ROBOT[ROBOT.index("bool RobotController::canStartMotion"):ROBOT.index("bool RobotController::startAiMotion")]
    assert can_start.index("ps2_.frameTimedOut(nowMs)") < can_start.index("owner == MotionOwner::REPLAY")
    print("REPLAY_GUIDED_START_HANDOFF_STATIC=PASS")


if __name__ == "__main__":
    main()
