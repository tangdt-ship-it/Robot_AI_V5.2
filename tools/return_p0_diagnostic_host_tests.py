"""Static contracts for Return-P0 reject telemetry (diagnostic-only)."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAP = (ROOT / "firmware/stm32/src/map/map_controller.cpp").read_text(
    encoding="utf-8"
)
HEADER = (ROOT / "firmware/stm32/include/map/map_controller.h").read_text(
    encoding="utf-8"
)
MAIN = (ROOT / "firmware/stm32/src/main.cpp").read_text(encoding="utf-8")


def require(text: str, needle: str) -> None:
    assert needle in text, needle


def main() -> None:
    require(HEADER, "logReturnP0RejectSnapshot(ReturnP0Source source")
    require(MAP, "void MapController::logReturnP0RejectSnapshot")

    # Every field is emitted in one print chain, closed by exactly one newline.
    for field in (
        "MAP,RETURN_P0,REJECT_SNAPSHOT,REASON=",
        ",HOME_VALID=",
        ",HOME_SLOT=",
        ",SELECTED_SLOT=",
        ",HOME_ROUTE_GEN=",
        ",ROUTE_GEN=",
        ",HOME_ODOM_GEN=",
        ",ODOM_GEN=",
        ",HOME_HEADING_GEN=",
        ",HEADING_GEN=",
        ",LOADED=",
        ",MODE=",
        ",REPLAY_ACTIVE=",
        ",RETURN_STATE=",
        ",OWNER=",
        ",MOTORS_STOPPED=",
        ",AI_ACTIVE=",
        ",PS2_MOTION=",
    ):
        require(MAP, field)
    require(MAP, 'logReturnP0RejectSnapshot(source, reason, "ODOM")')
    require(MAP, 'logReturnP0RejectSnapshot(source, reason, "HEADING")')

    # Each direct Return-P0 reject reason is captured before its safety action.
    for reason in (
        "RETURN_P0_ACTIVE",
        "SOURCE",
        "HOME_CONTEXT_INVALID",
        "SLOT_CHANGED",
        "ROUTE_INVALID",
        "ROUTE_CHANGED",
        "MOTION_OWNER",
        "REACQUIRE_START",
        "RETURN_START",
    ):
        marker = f'reason = "{reason}";'
        at = MAP.index(marker)
        after = MAP[at : at + 260]
        assert "logReturnP0RejectSnapshot" in after, reason

    # Logging is pure output; safety calls remain in the request function.
    helper_start = MAP.index("void MapController::logReturnP0RejectSnapshot")
    helper_end = MAP.index("bool MapController::requestReturnToP0Internal", helper_start)
    helper = MAP[helper_start:helper_end]
    for forbidden in (
        "stopImmediately",
        "invalidateHomeContext",
        "nextReplayGeneration",
        "loadSelected",
        "returnP0State_ =",
    ):
        assert forbidden not in helper, forbidden

    # Main logs receipt/result without modifying RobotLink completion behavior.
    require(MAIN, '"MAP,RETURN_P0,DISPATCH,SOURCE=AI_VOICE"')
    require(MAIN, '"MAP,RETURN_P0,DISPATCH_RESULT="')
    dispatch = MAIN.index('"MAP,RETURN_P0,DISPATCH,SOURCE=AI_VOICE"')
    complete = MAIN.index("robotLink.completeMapRequest", dispatch)
    assert dispatch < complete

    print("RETURN_P0_DIAGNOSTIC_STATIC=PASS")


if __name__ == "__main__":
    main()
