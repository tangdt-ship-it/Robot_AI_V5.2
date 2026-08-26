# Project State

## Repository

- Repository: `tangdt-ship-it/Robot_AI`
- Stable / release branch: `main`
- Development branch: `develop/robot-ai-v5.0`
- Current development version: `5.0.0-alpha.5`
- Alpha.4 H0-tested firmware checkpoint: `08affa1997aa5e56cf5974540ded82dcbfc95fb3`.
- Alpha.5 H1-tested firmware checkpoint: `dad0a5be9053b004aafce0125410226940afedff`.
- Current V5 development head is the latest commit on `develop/robot-ai-v5.0`; documentation-only commits may follow a tested firmware checkpoint without changing firmware bytes.

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

## V5 Alpha.5 status

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
- cancellation-aware ESP32 STOP lifecycle for finite MOVE/TURN waiters;
- STOP waiter release only after confirmed `DONE,STOP`;
- finite-operation admission blocked while STOP is in progress;
- deterministic V5 host/static/H0/H1 self-tests.

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
- heading query = PASS;
- ultrasonic health = `HEALTHY/CLEAR`;
- encoder ready = `1`, health = `OK`;
- odometry query = PASS;
- no unexpected motion.

H0 exit state: `READY_FOR_H1 = YES`.

## H1 Alpha.4 / Alpha.5 hardware validation

Status: `PASS`.

H1-A bounded finite MOVE was validated on Alpha.4 before the STOP lifecycle fix:

- command: `MOVE,FWD,50,10,SID,1,OP,2`;
- ACK correlation = PASS;
- three MOVE progress frames carried the same `(SID, OP)`;
- terminal: `DONE,MOVE,TARGET,50.0,TRAVEL,50.2,SID,1,OP,2`;
- measured odometry delta = approximately `49.9 mm`;
- reset generation remained `3 -> 3`;
- final motor state = `L=0`, `R=0`, `MOVE=0`;
- no unexpected motion.

Initial H1-B on Alpha.4 exposed an ESP32 operation-lifecycle defect:

- physical STOP was successful and motor-safe;
- `DONE,STOP` arrived in approximately `139.5 ms`;
- old `ERR,TURN,CANCELLED` result was not accepted as completion;
- however the finite TURN waiter did not exit until approximately `12862.5 ms` later.

Alpha.5 fixed that waiter lifecycle on the ESP32 only. STM32 firmware was unchanged.

Alpha.5 H1-B retest validated:

- all V4/V4.2/V5 host/static/H0/H1 tests = PASS;
- ESP32-S3 build = PASS;
- ESP32 application size = `3633712` bytes;
- free app partition = `495056` bytes;
- ESP32 flash = PASS;
- RobotLink online, negotiated `SID = 1`;
- finite turn command: `TURN,REL,LEFT,20,10,SID,1,OP,1`;
- TURN ACK correlation = PASS;
- operator STOP produced `ACK,STOP` and `DONE,STOP`;
- physical STOP latency = approximately `239.8 ms`;
- motor and encoder velocity returned to zero;
- finite TURN waiter released approximately `3.5 ms` after `DONE,STOP`;
- stale `ERR,TURN,CANCELLED,SID,1,OP,1` was not accepted as completion;
- reset generation remained `3 -> 3`;
- final navigation state = `IDLE`;
- final motion lease = `NONE`;
- final motor state = `L=0`, `R=0`, `MOVE=0`;
- no unexpected motion.

Two follow-up idempotent STOP cleanups were observed after the operator STOP. They did not create motion, did not form a STOP loop, and did not block H1 PASS.

H1 exit state: `READY_FOR_H2 = YES`.

## HIL log handling

Runtime/HIL logs must be stored outside the repository worktree so `tools/v5_static_audit.py` can enforce that no `logs/` tree exists in the production repository.

Current external HIL log root:

`F:\Robot\Robot_AI_HIL_Logs`

Historical Alpha.4 logs were preserved under the external archive before H1 Alpha.5 retest.

## Still requiring HIL / hardware validation

- H2 stale/reset/fault handling;
- physical left/right sensor mapping under controlled test geometry;
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
- H1-tested Alpha.5 firmware checkpoint: `dad0a5be9053b004aafce0125410226940afedff`.
- Historical MAP/Replay evidence: `docs/MAP_REPLAY_COMMISSIONING_HISTORY.md`.
- V5 scope and release gates: `docs/ROBOT_AI_V5_0_PROJECT.md`.
- V5 HIL procedure: `docs/ROBOT_AI_V5_HIL_PLAN.md`.
