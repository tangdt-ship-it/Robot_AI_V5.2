#!/usr/bin/env python3
"""Static inherited-architecture guardrails for Robot_AI_V5.0 alpha."""
from __future__ import annotations
from collections import Counter
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
errors: list[str] = []

def require(condition: bool, message: str) -> None:
    if not condition:
        errors.append(message)

def text(path: str) -> str:
    p = ROOT / path
    require(p.is_file(), f"missing {path}")
    return p.read_text(errors="replace") if p.is_file() else ""

require((ROOT / "firmware/stm32").is_dir(), "missing production STM32 tree")
require((ROOT / "firmware/esp32-xiaozhi").is_dir(), "missing production Xiaozhi tree")
require(not (ROOT / "legacy").exists(), "legacy tree must not exist")
require(not (ROOT / "logs").exists(), "logs tree must not exist")
version = text("VERSION").strip()
require(re.fullmatch(r"5\.0\.0-alpha\.\d+", version) is not None,
        f"VERSION is not a Robot_AI_V5.0 alpha version: {version!r}")
require((ROOT / "docs/ROBOT_AI_V5_0_PROJECT.md").is_file(),
        "missing V5.0 project definition")

cfg = text("firmware/stm32/include/robot_config.h")
main = text("firmware/stm32/src/main.cpp")
imu = text("firmware/stm32/src/imu/mpu6050.cpp")
fusion = text("firmware/stm32/src/localization/heading_fusion.cpp")
odom = text("firmware/stm32/src/encoders/wheel_odometry.cpp")
server = text("firmware/stm32/src/communication/robot_link_server.cpp")
ini = text("firmware/stm32/platformio.ini")
ruart = text("firmware/esp32-xiaozhi/main/robot/robot_uart.cc")
ruart_h = text("firmware/esp32-xiaozhi/main/robot/robot_uart.h")
mission = text("firmware/esp32-xiaozhi/main/robot/mission_manager.cc")
mission_h = text("firmware/esp32-xiaozhi/main/robot/mission_manager.h")
kconfig = text("firmware/esp32-xiaozhi/main/Kconfig.projbuild")
defaults = text("firmware/esp32-xiaozhi/sdkconfig.robot_ai.defaults")
obstacle_assist = text("firmware/esp32-xiaozhi/main/robot/obstacle_assist.cc")
teach_route = text("firmware/esp32-xiaozhi/main/robot/teach_route.cc")
board = text("firmware/esp32-xiaozhi/main/boards/bread-compact-wifi-s3cam/compact_wifi_board_s3cam.cc")
cmake = text("firmware/esp32-xiaozhi/main/CMakeLists.txt")

require("MPU6050_SCL_PIN = PC8" in cfg and "MPU6050_SDA_PIN = PB5" in cfg,
        "MPU6050 Soft-I2C pins are not PC8/PB5")
require("MPU6050_ADDRESS = 0x68" in cfg, "MPU6050 address is not 0x68")
require("Face-down installation transform" in imu and
        "data_.gyroZDps = -" in imu, "face-down MPU6050 axis transform missing")
require("imu.data().sampleSequence" in main and "compass.sampleSequence()" in main,
        "fusion is not sequence-gated by physical sensor samples")
require("newCompassSample" in fusion and "newImuSample" in fusion,
        "stale IMU/Compass sample protection missing")
require("wheelOdometry.integratePose(headingFusion.headingDeg()" in main,
        "fused heading is not feeding STM32 odometry")
require('GET,IMU' in server and 'GET,FUSION' in server,
        "RobotLink V3 IMU/FUSION telemetry missing")
require('GET,ODOMETRY' in server and 'GetOdometry' in ruart,
        "odometry bridge STM32->ESP32 missing")
require("SyncPoseFromOdometry" in mission and
        "stm32_encoder_plus_fused_heading" in mission,
        "MissionManager is not using STM32 odometry")
require("compass_plus_time_dead_reckoning" not in mission,
        "old compass/time dead reckoning remains")
require("SensorHealth" in cfg or "SensorHealth" in text("firmware/stm32/include/sensors/ultrasonic_sensor.h"),
        "sensor health classification missing")
require("!overallFresh_||!healthy()" in text("firmware/stm32/src/sensors/ultrasonic_sensor.cpp"),
        "forward motion is not fail-closed on sensor health")
require("MotionOwner" in text("firmware/stm32/include/control/robot_controller.h") and
        "motionOwner_" in text("firmware/stm32/src/control/robot_controller.cpp"),
        "motion ownership gate missing")
require("resetGeneration" in text("firmware/stm32/include/encoders/wheel_odometry.h") and
        "RESET_BOUNDARY_UNRESOLVED" in teach_route,
        "encoder reset generation/resume boundary handling missing")
require("compare_exchange_strong" in obstacle_assist and
        "ActiveGuard" in obstacle_assist,
        "obstacle AI cooldown cleanup guard missing")
require("SHORT_SAFETY_TEST" in teach_route and "FULL_PRODUCTION" in teach_route and
        "ROBOT_V5_FULL_REPLAY_PRODUCTION 0" in teach_route,
        "explicit replay mode/default production gate missing")
require("ROUTE,REPLAY=PREFLIGHT" in teach_route and
        "SENSOR_CHANNEL_HEALTH" in teach_route and
        "RESET_BOUNDARY_UNRESOLVED" in teach_route and
        "HEADING_UNRELIABLE" in teach_route,
        "unified replay preflight/degraded safety gate missing")
require("ReplayTerminalStatus" in teach_route and
        "TERMINAL=ROUTE_COMPLETE" in teach_route and
        "TERMINAL=TEST_COMPLETE" in teach_route,
        "replay terminal status contract missing")
require("MotionLeaseActive" in ruart_h and "motion_owner" in ruart_h and
        "OWNER," in server and "MotionOwnerName" in server,
        "motion owner/lease telemetry gate missing")
require("CONFIG_AUTOMATIC_DETOUR 0" in mission and
        "AUTO_DETOUR_DISABLED" in mission and
        'config AUTOMATIC_DETOUR' in kconfig and
        "CONFIG_AUTOMATIC_DETOUR=n" in defaults,
        "automatic detour default-off gate missing")
require((ROOT / "tools/v5_host_selftest.py").is_file(),
        "missing V5 host safety self-test")
require("MissionType::RETURN_HOME" in mission and "RunReturnHome" in mission,
        "RETURN_HOME mission missing")
require("Breadcrumb" in mission_h and "breadcrumbs_" in mission,
        "breadcrumb path memory missing")
require('self.robot.return_home' in board and 'self.robot.set_home' in board,
        "Return Home MCP tools missing")
require('self.robot.get_imu_status' in board and
        'self.robot.get_fusion_status' in board and
        'self.robot.get_odometry' in board,
        "V4.2 localization diagnostic MCP tools missing")
require('default_envs = stm32_robot_v4_2' in ini and
        '[env:stm32_imu_probe]' in ini,
        "V4.2 STM32 production/IMU probe environments missing")
require('HELLO,PROTO,3' in ruart and 'Crc16Ccitt' in ruart,
        "RobotLink V3 CRC/handshake missing")
blackbox_h = text("firmware/esp32-xiaozhi/main/robot/safety_blackbox.h")
blackbox = text("firmware/esp32-xiaozhi/main/robot/safety_blackbox.cc")
require("SID,%lu,OP,%lu" in ruart and "BeginMotionCorrelation" in ruart and
        "MatchMotionCorrelation" in ruart and "ACK_STALE" in ruart,
        "ESP32 strict SID/OP correlation is missing")
require("sessionId" in server and "operationId" in server and
        "PrintCorrelation" in server and "PROGRESS,MOVE" in server,
        "STM32 SID/OP echo for ACK/progress/terminal is missing")
require("activeMotionSessionId_" in text("firmware/stm32/include/communication/robot_link_server.h"),
        "STM32 active operation correlation storage missing")
require("kCapacity = 48" in blackbox_h and "std::array" in blackbox_h and
        "CopyRecent" in blackbox_h and "portENTER_CRITICAL" in blackbox,
        "bounded RAM-only safety black-box missing")
require("MotionDiagnostic" in blackbox_h and "UNCALIBRATED" in blackbox and
        "ClassifyMotionDiagnostic" in blackbox,
        "telemetry-only motion diagnostic classifier missing")
require('"robot/safety_blackbox.cc"' in cmake,
        "ESP32 safety black-box source is not built")
require((ROOT / "docs/ROBOT_AI_V5_HIL_PLAN.md").is_file(),
        "missing Alpha.3 HIL plan")
require("Ran 36 tests" not in text("tools/v5_host_selftest.py"),
        "host self-test must not hard-code a claimed test result")
require('Robot_AI_V4.2 supports ESP32-S3 only' in cmake,
        "V4.2 ESP32 target guard missing")

boards = ROOT / "firmware/esp32-xiaozhi/main/boards"
board_dirs = sorted(p.name for p in boards.iterdir() if p.is_dir()) if boards.is_dir() else []
require(board_dirs == ["bread-compact-wifi-s3cam", "common"],
        f"unexpected ESP32 board dirs: {board_dirs}")
locales = ROOT / "firmware/esp32-xiaozhi/main/assets/locales"
locale_dirs = sorted(p.name for p in locales.iterdir() if p.is_dir()) if locales.is_dir() else []
require(locale_dirs == ["en-US", "vi-VN"], f"unexpected locale dirs: {locale_dirs}")
partitions = sorted(p.relative_to(ROOT / "firmware/esp32-xiaozhi").as_posix()
                    for p in (ROOT / "firmware/esp32-xiaozhi/partitions").rglob('*')
                    if p.is_file())
require(partitions == ["partitions/v2/16m.csv"], f"unexpected partition files: {partitions}")

tool_names = re.findall(r'mcp_server\.AddTool\(\s*\n?\s*"([^"]+)"', board)
dupes = [name for name, count in Counter(tool_names).items() if count > 1]
require(not dupes, f"duplicate MCP tools: {dupes}")

for doc in ["ARCHITECTURE.md", "ROBOTLINK_V3.md", "SAFETY.md", "CONTROL.md",
            "NAVIGATION.md", "HARDWARE_PINOUT.md", "V4_2_VALIDATION_CHECKLIST.md"]:
    require((ROOT / "docs" / doc).is_file(), f"missing V4.2 document {doc}")

if errors:
    print("FAIL Robot_AI_V5.0 inherited static audit")
    for err in errors:
        print(f" - {err}")
    sys.exit(1)
print(f"PASS Robot_AI_V5.0 inherited static audit ({len(tool_names)} MCP tools checked)")
