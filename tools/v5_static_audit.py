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
mission = text("firmware/esp32-xiaozhi/main/robot/mission_manager.cc")
mission_h = text("firmware/esp32-xiaozhi/main/robot/mission_manager.h")
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
