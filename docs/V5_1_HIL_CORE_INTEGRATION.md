# Robot_AI V5.1 HIL-Core Candidate

## Goal

This candidate starts from the proven Alpha.9 HIL snapshot and adds only the
useful V5.1 HOME/Return-Home improvements. The Alpha.9 RobotLink/STM32/safety
core is intentionally not replaced by the V5.1 archive.

## Base

- Baseline branch: `baseline/v5-alpha9-hil-passed-pre-wireless`
- Baseline snapshot: `7720eac7c59dc57843b5cdd162298526a42445b4`
- Hardware-tested Alpha.9 checkpoint: `7ec2349cb4878d8acba49c09de288cc95c3aef8c`
- Base version string retained for compatibility: `5.0.0-alpha.9`

## V5.1 changes carried forward

1. `MissionManager` can store HOME and bounded breadcrumbs in NVS.
2. Persistent data is validated by magic/version/count/finiteness and CRC32.
3. HOME can be restored and odometry rebased after a reboot boundary.
4. Operator `move_distance` is rejected while an autonomous mission is active.
5. A successful external distance motion synchronizes MissionManager pose and
   can add a breadcrumb for Return Home.
6. Return Home task stack uses PSRAM via `xTaskCreatePinnedToCoreWithCaps`.
7. Return Home waypoint tolerance is 4 cm instead of 12 cm.

## Alpha.9 invariants preserved

The package keeps the Alpha.9 versions of RobotLink, STM32 motor/safety,
SafetyBlackBox and TeachRoute. The following invariants remain mandatory:

- finite MOVE/TURN requires non-zero SID/OP;
- stale/mismatched/duplicate terminal results cannot complete a new operation;
- STOP physically completes before a finite waiter is released;
- STM32 reboot invalidates the old RobotLink session;
- replay/HOLD reset boundary includes raw RESET_GEN plus STM32 boot epoch;
- protocol recovery is HELLO/PING-only and cannot create motion;
- safe final state is motor zero, navigation idle and no motion lease.

## Hardware profile

The generated `sdkconfig` is taken from the Alpha.9 HIL robot profile, not from
the uploaded V5.1 archive. Expected settings include:

- ESP32-S3;
- 16 MB flash;
- octal PSRAM;
- `bread-compact-wifi-s3cam`;
- ST7789 240x240;
- Vietnamese (`vi-VN`);
- custom wake word.

## Validation boundary

Host/static regression can prove that Alpha.9 guardrails remain present and
that the V5.1 integration is structurally consistent. It does NOT prove that
persistent HOME/Return Home is hardware-commissioned. Before release, run a
small dedicated campaign: set HOME, move a bounded distance, reboot ESP32,
verify HOME restore, optionally reboot STM32, verify rebase, then Return Home
with obstacle/encoder/heading gates active.

## NVS note

The imported V5.1 implementation persists a route whenever a meaningful new
breadcrumb is recorded. This is acceptable for controlled commissioning, but
long-duration autonomous use should characterize NVS write frequency/wear and
may later add a bounded flush/debounce policy if needed.
