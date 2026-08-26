# Project State

## Repository

- Repository: `tangdt-ship-it/Robot_AI`
- Stable / release branch: `main`
- Development branch: `develop/robot-ai-v5.0`
- Current development version: `5.0.0-alpha.3`
- Current V5 development head is the latest commit on `develop/robot-ai-v5.0`.

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

## V5 Alpha.3 status

Implemented and host-tested:

- fail-closed ultrasonic sensor-health classification;
- motion-owner and motion-lease gates;
- encoder reset generation / reset-boundary protection;
- replay preflight classification;
- explicit `SHORT_SAFETY_TEST` / `FULL_PRODUCTION` modes;
- automatic detour disabled by default;
- strict non-zero `(SID, OP)` correlation for finite MOVE/TURN operations;
- stale/mismatched operation result rejection;
- session/STOP/cancel/reset invalidation of pending operations;
- bounded RAM-only safety black-box;
- deterministic V5 host/static tests;
- H0/H1/H2 HIL readiness plan.

## Still requiring HIL / hardware validation

- clean build confirmation for both production targets in the operator environment;
- H0 link/session/correlation observation;
- H1 bounded motion and STOP behavior;
- H2 stale/reset/fault handling;
- physical left/right sensor mapping;
- STOP latency on the real robot;
- encoder and heading calibration;
- Full Replay production commissioning;
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
- Historical MAP/Replay evidence: `docs/MAP_REPLAY_COMMISSIONING_HISTORY.md`.
- V5 scope and release gates: `docs/ROBOT_AI_V5_0_PROJECT.md`.
- V5 Alpha.3 HIL procedure: `docs/ROBOT_AI_V5_HIL_PLAN.md`.
