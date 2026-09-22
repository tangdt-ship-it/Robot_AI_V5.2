"""Static contracts for accepted Return-P0 accuracy telemetry."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAP = (ROOT / "firmware/stm32/src/map/map_controller.cpp").read_text(
    encoding="utf-8"
)
HEADER = (ROOT / "firmware/stm32/include/map/map_controller.h").read_text(
    encoding="utf-8"
)


def require(text: str, needle: str) -> None:
    assert needle in text, needle


def main() -> None:
    require(HEADER, "logReturnP0TraceBase")
    require(HEADER, "logReturnP0TraceMotion")
    require(MAP, "MAP,RETURN_P0,TRACE,EVENT=")
    for event in (
        "ACCEPT",
        "PROJECTION",
        "REACQUIRE_START",
        "REACQUIRE_DONE",
        "WP_START",
        "WP_DONE",
        "P0_POSITION_START",
        "P0_POSITION_ARRIVAL",
        "P0_POSITION_SETTLE",
        "P0_HEADING_START",
        "P0_HEADING_DONE",
        "FINAL_GATE",
        "COMPLETE",
    ):
        require(MAP, f'logReturnP0Trace{ "Motion" if event != "ACCEPT" and event != "PROJECTION" else "Base" }("{event}"')

    for field in (
        ",LIVE_X=",
        ",LIVE_Y=",
        ",LIVE_H=",
        ",P0_X=",
        ",P0_Y=",
        ",P0_H=",
        ",DX=",
        ",DY=",
        ",POS_ERR=",
        ",HEADING_ERR=",
        ",LEFT_TICKS=",
        ",RIGHT_TICKS=",
        ",FUSED_HEADING=",
        ",ENCODER_HEALTH=",
        ",FUSION_HEALTH=",
        ",GATE_RESULT=",
    ):
        require(MAP, field)

    # The instrumentation is output-only: no algorithm/tolerance edits.
    trace_start = MAP.index("void MapController::logReturnP0TraceBase")
    trace_end = MAP.index("bool MapController::requestReturnToP0Internal", trace_start)
    trace_helpers = MAP[trace_start:trace_end]
    for forbidden in (
        "stopImmediately",
        "invalidateHomeContext",
        "nextReplayGeneration",
        "loadSelected",
        "returnP0State_ =",
    ):
        assert forbidden not in trace_helpers, forbidden
    require(MAP, "MAP_RETURN_P0_POSITION_TOLERANCE_MM")
    require(MAP, "MAP_RETURN_P0_HEADING_TOLERANCE_DEG")

    print("RETURN_P0_ACCURACY_TRACE_STATIC=PASS")


if __name__ == "__main__":
    main()
