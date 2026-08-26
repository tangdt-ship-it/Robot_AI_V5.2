# Project State

## Repository

- Repository: `tangdt-ship-it/Robot_AI`
- Stable / release branch: `main`
- Development branch: `develop/robot-ai-v5.0`
- Current development version: `5.0.0-alpha.4`
- Alpha.4 H0-tested firmware checkpoint: `08affa1997aa5e56cf5974540ded82dcbfc95fb3`.
- Current V5 development head is the latest commit on `develop/robot-ai-v5.0`; documentation-only commits may follow the tested firmware checkpoint without changing firmware bytes.

## Stable baseline

Stable fallback remains the commissioned V4.x line. The current `main` branch carries the stable/release baseline and should not receive experimental V5 feature work directly.

Commissioned capabilities inherited by V5 include:

- Xiaozhi voice/audio/TFT/camera integration;
- RobotLink V3;
- PS2 manual control and safety priority;
- STM32 motor control and watchdog;
- encoder odometry;
- MPU6050 + Compass + heading fusion;
- HC-SR04 obstacle stop authority;
- HOME / breadcrumb / Return Home;
- Teach Route / RouteStore / Smart Waypoint;
- Replay dry-run and commissioned MAP/Replay primitives;
- AI Obstacle Assist advice mode.

Historical MAP/Replay commissioning details previously stored in this file were preserved verbatim in:

`docs/MAP_REPLAY_COMMISSIONING_HISTORY.md`

## V5 Alpha.4 status

Implemented and host-tested:

- fail-closed ultrasonic sensor-health classification;
- motion-owner and motion-lease gates;
- encoder reset generation / reset-boundary protection;
- replay preflight classification;
- explicit `SHORT_SAFETY_TEST` / `FULL_PRODUCTION` modes;
- automatic detour disabled by default;
- strict non-zero `(SID, OP)` correlation for finite MOVE/TURN operations;
- missing finite-operation correlation rejected with `CORRELATION_REQUIRED`;
- stale/mismatched operation result rejection;
- session/STOP/cancel/reset invalidation of pending operations;
- passive negotiated SID logging for no-motion H0 evidence;
- bounded RAM-only safety black-box;
- deterministic V5 host/static tests;
- H0/H1/H2 HIL readiness plan.

## H0 Alpha.4 hardware validation

Status: `PASS`.

Tested firmware checkpoint:

`08affa1997aa5e56cf5974540ded82dcbfc95fb3`

Validated on hardware:

- `TEST_V4_PROTOCOL = PASS`;
- `TEST_V4_2_LOCALIZATION = PASS`;
- `TEST_V5_HOST = PASS`;
- `TEST_V5_STATIC_AUDIT = PASS`;
- `TEST_V5_H0 = PASS`;
- STM32 production build/flash = PASS;
- ESP32-S3 production build/flash = PASS;
- ESP32 application size = `3633488` bytes, free app partition = `495280` bytes;
- RobotLink HELLO/PING online and stable;
- negotiated `SID = 1`, non-zero;
- finite MOVE/TURN correlation guards = PASS;
- missing/zero/mismatched/stale correlation tests = PASS;
- start/final motor commands = `L=0`, `R=0`, `MOVE=0`;
- final motion owner = `NONE`;
- no pending operation and no motion lease observed during H0;
- Compass = `OK`, calibration inactive;
- heading query = PASS, observed heading `0.0 deg`;
- ultrasonic health = `HEALTHY/CLEAR`, observed left/right `70.7 / 56.4 cm`;
- encoder ready = `1`, health = `OK`;
- odometry query = PASS;
- no unexpected motion.

Observed non-blocking anomalies:

- temporary OTA cloud timeout; no reboot and no RobotLink impact;
- firmware boot self-test emitted `COMPASS,RESET` and `STOP`; these were boot self-test actions, not H0 operator motion commands, and no MOVE/TURN occurred;
- opening the ESP32 monitor produced a boot sequence, with no reboot loop;
- STM32 USART3 debug port was unavailable, but required H0 evidence was observable through RobotLink.

H0 exit state: `READY_FOR_H1 = YES`.

## Still requiring HIL / hardware validation

- H1 bounded motion, correlated terminal result and STOP behavior;
- H2 stale/reset/fault handling;
- physical left/right sensor mapping under controlled test geometry;
- STOP latency on the real robot;
- encoder and heading calibration/tolerance characterization;
- Full Replay production commissioning under V5 gates;
- automatic detour/rejoin commissioning.

Until those gates pass, V5 remains `DEVELOPMENT / NOT PRODUCTION`.

## Disabled by default

- Full Replay production;
- automatic detour;
- automatic reverse;
- automatic resume after AI obstacle analysis.

These features must not be enabled only by changing a flag without completing their required HIL gates.

## Naming / compatibility

The project/repository name is now `Robot_AI`.

Some historical names intentionally remain because they are compatibility interfaces or archival records, for example:

- PlatformIO env `stm32_robot_v4_2`;
- scripts `v4_2_localization_selftest.py`, `v4_2_static_audit.py`;
- documents `V4_2_RELEASE_NOTES.md`, `V4_2_VALIDATION_CHECKLIST.md`;
- historical V4.x commissioning references.

Do not rename those identifiers casually unless the corresponding build/test references are migrated in one controlled change.

## Source of truth

- Stable/release source: `main`.
- Active V5 development source: `develop/robot-ai-v5.0`.
- H0-tested Alpha.4 firmware checkpoint: `08affa1997aa5e56cf5974540ded82dcbfc95fb3`.
- Historical MAP/Replay evidence: `docs/MAP_REPLAY_COMMISSIONING_HISTORY.md`.
- V5 scope and release gates: `docs/ROBOT_AI_V5_0_PROJECT.md`.
- V5 Alpha.4 HIL procedure: `docs/ROBOT_AI_V5_HIL_PLAN.md`.
