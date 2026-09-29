"""Static safety/contracts for V5.2.17 AI MAP/Return-P0 integration."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


STM32_MAP = read("firmware/stm32/src/map/map_controller.cpp")
STM32_MAP_H = read("firmware/stm32/include/map/map_controller.h")
ROBOT_LINK = read("firmware/stm32/src/communication/robot_link_server.cpp")
ROBOT_LINK_H = read("firmware/stm32/include/communication/robot_link_server.h")
MAIN = read("firmware/stm32/src/main.cpp")
UART = read("firmware/esp32-xiaozhi/main/robot/robot_uart.cc")
UART_H = read("firmware/esp32-xiaozhi/main/robot/robot_uart.h")
BOARD = read("firmware/esp32-xiaozhi/main/boards/bread-compact-wifi-s3cam/compact_wifi_board_s3cam.cc")
MCP = read("firmware/esp32-xiaozhi/main/mcp_server.h")


def require(text: str, needle: str) -> None:
    assert needle in text, needle


def main() -> None:
    # Public STM32 request boundary and the single canonical AI tool.
    require(STM32_MAP_H, "requestRunMap(uint8_t slot")
    require(STM32_MAP_H, "requestReturnToP0(ReturnP0Source source")
    require(BOARD, '"self.robot.map_route",')
    require(BOARD, 'run_map1')
    require(BOARD, 'run_map2')
    require(BOARD, 'return_p0')
    require(BOARD, 'chạy về P0')
    require(BOARD, 'không gọi stop, move_distance, turn, return_home')
    require(BOARD, 'completed\\":false')
    require(UART, "SetMode(true, 700)")

    # RobotLink V3 command surface is exact and integrity-protected.
    require(ROBOT_LINK, 'strncmp(frame, "MAP,CMD,", 8)')
    require(ROBOT_LINK, '"MAP,CMD,RUN,1"')
    require(ROBOT_LINK, '"MAP,CMD,RUN,2"')
    require(ROBOT_LINK, '"MAP,CMD,RETURN_P0"')
    require(ROBOT_LINK, "RequiresIntegrity")
    require(ROBOT_LINK_H, "takeMapRequest")
    require(ROBOT_LINK_H, "completeMapRequest")
    require(MAIN, "takeMapRequest")
    require(MAIN, "completeMapRequest")

    # STOP is serviced before any queued map request, and requests go through
    # MapController public APIs rather than private state or fake PS2 input.
    stop_pos = MAIN.index("if (robotLink.takeStopRequest())")
    map_pos = MAIN.index("takeMapRequest", stop_pos)
    motion_pos = MAIN.index("takeMotionRequest", map_pos)
    assert stop_pos < map_pos < motion_pos
    require(MAIN, "requestRunMap")
    require(MAIN, "MapMissionInitiator::AI_VOICE")
    require(MAIN, "requestReturnToP0")
    require(MAIN, "ReturnP0Source::AI_VOICE")

    # AI Return may resume only through its own saved context/generation gates;
    # PS2 remains explicit-start/no-auto-resume.
    require(STM32_MAP, "resumeReturnP0FromObstacleHold")
    require(STM32_MAP, "returnP0Source_ != ReturnP0Source::AI_VOICE")
    require(STM32_MAP, "AI_OBSTACLE_AUTO_RESUME_CLEAR_MS")
    require(STM32_MAP, "returnP0HeldTargetIndex_")
    require(STM32_MAP, "nextReplayGeneration()")
    require(STM32_MAP, "requestReturnToP0(ReturnP0Source::PS2_START")
    require(STM32_MAP, 'abortReturnToP0("EXTERNAL_STOP")')

    # After boot, an accepted AI run may establish only a RAM-bound P0 frame
    # from the existing replay origin.  It remains guarded by reset/route
    # generations and never re-enables the physical post-Teach BACK action.
    require(STM32_MAP, "armAiRunHomeContextIfNeeded")
    require(STM32_MAP, "homeContext_.p0WorldPose = replayOrigin_")
    require(STM32_MAP, "backP0UiDismissed_ = true")

    # ESP32 uses the transaction/ACK path; no direct callback UART command.
    require(UART_H, "RunMap(uint8_t slot")
    require(UART_H, "ReturnToP0(uint32_t timeout_ms")
    require(UART, "SendAndWait")
    require(UART, "MAP,CMD,RUN,")
    require(UART, "MAP,CMD,RETURN_P0")
    require(UART_H, "TRANSPORT_TIMEOUT")

    # HOME remains distinct from MAP Return-P0.
    require(MCP, "self.robot.map_route(action=return_p0)")

    print("V5_2_17_PHASE4_STATIC=PASS")


if __name__ == "__main__":
    main()
