#!/usr/bin/env python3
"""Host-only structural guard for the V5.1 HIL-core integration.

No serial port, build, flash, NVS mutation or motion occurs.
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
MISSION_CC = ROOT / "firmware/esp32-xiaozhi/main/robot/mission_manager.cc"
MISSION_H = ROOT / "firmware/esp32-xiaozhi/main/robot/mission_manager.h"
BOARD = ROOT / "firmware/esp32-xiaozhi/main/boards/bread-compact-wifi-s3cam/compact_wifi_board_s3cam.cc"
SDK = ROOT / "firmware/esp32-xiaozhi/sdkconfig"
VERSION = ROOT / "VERSION"

errors = []
def require(cond, msg):
    if not cond:
        errors.append(msg)

cc = MISSION_CC.read_text(errors="replace")
h = MISSION_H.read_text(errors="replace")
board = BOARD.read_text(errors="replace")
sdk = SDK.read_text(errors="replace")
version = VERSION.read_text().strip()

# Keep Alpha.9 version contract so its host H0/H1/H2 tests remain meaningful.
require(version == "5.0.0-alpha.9", "base VERSION is not Alpha.9")

# Persistent HOME contract.
for token in (
    'kHomeNvsNamespace = "mission_home"',
    'kHomeNvsKey = "route"',
    'kHomeStorageMagic',
    'kHomeStorageVersion = 1',
    'kMaxPersistedBreadcrumbs = 128',
    'esp_rom_crc32_le',
    'nvs_get_blob',
    'nvs_set_blob',
    'nvs_commit',
    'LoadPersistentHome()',
    'PersistHomeRoute()',
    'RestoreOdometryReference()',
):
    require(token in cc, f"missing persistent HOME token: {token}")

# Public synchronization path and mission ownership gate.
require('bool SyncAfterExternalMotion();' in h,
        "SyncAfterExternalMotion declaration missing")
require('bool MissionManager::SyncAfterExternalMotion()' in cc,
        "SyncAfterExternalMotion implementation missing")
require('mission_manager_.IsActive()' in board,
        "move_distance mission ownership gate missing")
require('mission_manager_.SyncAfterExternalMotion()' in board,
        "move_distance pose synchronization missing")
require('"pose_synced"' in board or '\\"pose_synced\\"' in board,
        "move_distance response does not expose pose_synced")

# Return Home memory/tolerance changes.
require('kReturnHomeTaskStackBytes = 32768' in cc,
        "PSRAM return-home stack size missing")
require('xTaskCreatePinnedToCoreWithCaps' in cc,
        "Return Home is not created with caps")
require('MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT' in cc,
        "Return Home stack is not constrained to PSRAM")
require('vTaskDeleteWithCaps(nullptr)' in cc,
        "Return Home WithCaps deleter missing")
require('kWaypointToleranceCm = 4.0f' in cc,
        "Return Home 4 cm tolerance missing")

# Known HIL hardware profile; reject the generic V5.1 archive config.
for token in (
    'CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y',
    'CONFIG_LANGUAGE_VI_VN=y',
    'CONFIG_BOARD_TYPE_BREAD_COMPACT_WIFI_CAM=y',
    'CONFIG_LCD_ST7789_240X240=y',
    'CONFIG_USE_CUSTOM_WAKE_WORD=y',
    'CONFIG_SPIRAM_MODE_OCT=y',
):
    require(token in sdk, f"HIL hardware profile missing: {token}")
for forbidden in (
    'CONFIG_LANGUAGE_ZH_CN=y',
    'CONFIG_BOARD_TYPE_BREAD_COMPACT_WIFI=y',
    'CONFIG_OLED_SSD1306_128X32=y',
    'CONFIG_USE_AFE_WAKE_WORD=y',
):
    require(forbidden not in sdk, f"generic/wrong hardware profile enabled: {forbidden}")

# This package is intentionally pre-Wireless.
for rel in (
    'firmware/esp32-xiaozhi/main/robot/wireless_diagnostic_monitor.cc',
    'firmware/esp32-xiaozhi/main/robot/wireless_hil_control.cc',
    'firmware/esp32-xiaozhi/main/robot/wireless_maintenance.cc',
):
    require(not (ROOT / rel).exists(), f"Wireless implementation unexpectedly present: {rel}")

if errors:
    print("V5_1_HOME_SELFTEST FAIL")
    for e in errors:
        print(" -", e)
    sys.exit(1)
print("V5_1_HOME_SELFTEST PASS")
