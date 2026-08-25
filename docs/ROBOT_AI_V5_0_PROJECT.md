# Robot_AI_V5.0 — Project Definition

## 1. Project identity

- Product: Robot_AI_V5.0
- Development branch: `develop/robot-ai-v5.0`
- Version state: `5.0.0-alpha.0`
- Inherited source checkpoint: `ea85c29904984c822dab0b53e152ab7e88ed0b81`
- Source branch: `feature/map-replay-v1-hold-resume`
- Hardware compatibility: unchanged from Robot_AI_V4.2/V4.3
- Safety authority: STM32F103VET6
- AI, voice, camera and mission authority: ESP32-S3 N16R8/CAM

V5.0 is an evolutionary release. It must preserve commissioned V4.x functions and hardware pin assignments while replacing incomplete or unsafe control paths. An alpha build must not be described as production firmware until every release gate in this document passes on the real robot.

## 2. Unchanged hardware contract

V5.0 keeps the existing hardware and wiring:

- STM32F103VET6 motor and real-time safety controller;
- ESP32-S3 N16R8/CAM running Xiaozhi, voice, camera and mission management;
- two-wheel differential drive;
- left and right wheel encoders;
- MPU6050 IMU;
- compass and fused heading;
- two front HC-SR04 channels;
- PS2 controller and STM32 LCD;
- ESP32 TFT, microphone, speaker and camera;
- RobotLink UART at the existing pins and baud rate.

No GPIO or electrical connection may be changed in a V5.0 software-only commit unless a separate hardware migration document is approved.

## 3. Safety invariants

These rules apply to every V5.0 feature:

1. STM32 remains the final motor and safety authority.
2. MAP X, R3, PS2 takeover, sensor hard fault, link loss and watchdog may stop motion without waiting for ESP32 or cloud processing.
3. Voice STOP is supported but is not the only emergency-stop path.
4. Only one motion owner may command the robot at one time.
5. Camera or cloud AI may propose a direction but may never override STM32 sensor vetoes.
6. Autonomous motion is rejected when required sensor health is unknown or faulty.
7. A cancelled or emergency-stopped operation never restarts automatically.
8. No reboot may automatically resume a previously interrupted route.
9. Every MOVE and TURN has an operation ID, terminal result and bounded timeout.
10. Production Full Replay and automatic detour stay disabled until their prerequisite gates pass.

## 4. Inherited V4.x capabilities

The following functions are inherited and must be protected by regression tests:

- PS2 manual control and brake behavior;
- STM32 motor PWM control and watchdog;
- RobotLink command transport;
- encoder odometry;
- MPU6050, compass and heading fusion;
- ultrasonic obstacle stop authority;
- Xiaozhi voice, audio, TFT and camera integration;
- MissionManager and MCP robot tools;
- Teach Route and RouteStore persistence;
- Smart Waypoint generation;
- Replay dry run and commissioned B1/B2/B3 primitives;
- HOME/breadcrumb functions;
- AI obstacle description/advice mode;
- existing board pinout and build environments.

Inherited does not mean automatically re-certified. Each item must pass V5 regression tests after the control architecture changes.

## 5. Required V5.0 development stages

### Stage 0 — Branch and release controls

Status at branch creation: COMPLETE

- create the dedicated V5.0 development branch;
- mark version as alpha;
- add feature gates for unsafe functions;
- preserve the exact inherited source checkpoint;
- update audits so they read the current version instead of requiring V4.2.

Default feature policy:

| Feature | Alpha default |
|---|---|
| Manual PS2 | Enabled |
| Teach Route | Enabled |
| Replay dry run | Enabled |
| Replay short safety test | Enabled |
| Full Replay production | Disabled |
| AI obstacle advice | Enabled |
| Automatic detour | Disabled |
| Automatic resume after AI | Disabled |

### Stage 1 — Independent STOP and sensor health

Status: REQUIRED / NOT YET VERIFIED

STOP work:

- make MAP X stop motors locally on STM32 before sending its semantic event;
- retain R3 and PS2 takeover as local intervention paths;
- move blocking MOVE, TURN, camera and cloud work off the ESP32 application task;
- add an urgent STOP/cancel path that is not blocked by a normal UART transaction;
- cancel motion lease and pending operation before sending/confirming STOP;
- introduce a MotionArbiter with NONE, MCP, MISSION, REPLAY and DIAGNOSTIC owners;
- prevent any automatic restart after STOP.

Sensor-health work:

- separate data freshness, echo presence and hardware health;
- add STARTUP, HEALTHY, NO_RETURN, DEGRADED and FAULT states;
- track consecutive timeouts, last valid echo, success window and stuck signal faults;
- transmit health and failure data through RobotLink and parse it on ESP32;
- reject autonomous motion when both front sensors are faulty or unknown;
- define a safe degraded policy when one side is faulty;
- keep manual operation explicit and warn the user when sensor protection is degraded.

Gate S1:

- local X/R3 stop latency target: 100–200 ms;
- motor output zero and motion lease false after STOP;
- STOP works during MOVE, TURN, Replay, camera timeout and cloud timeout;
- an unplugged or stale sensor is never reported as healthy CLEAR;
- sensor fault tests pass on the real robot.

### Stage 2 — AI cooldown and event lifecycle

Status: REQUIRED / LOCKED BY S1

- check cooldown before acquiring the active flag;
- acquire active with compare-exchange rather than exchange-before-return;
- use guaranteed cleanup so camera, cloud, parser and task failures release active;
- do not suppress a more severe zone or a new mission/event ID;
- give each obstacle event a unique ID;
- implement IDLE, QUEUED, ANALYZING, RESULT_READY/FAILED and IDLE transitions;
- run camera/cloud analysis on a cancelable worker.

Gate S2:

- repeated same-zone events do not permanently latch active;
- an EMERGENCY event bypasses a lower-severity cooldown;
- every failure path returns to IDLE;
- the next valid event is accepted without reboot.

### Stage 3 — Correct HOLD/RESUME progress

Status: REQUIRED / LOCKED BY S2

- store an immutable segment target;
- store cumulative completed distance across all interruptions;
- calculate remaining distance from target minus cumulative completion;
- compare command-reported travel with odometry delta;
- reject resume when travel sources disagree beyond tolerance;
- ensure obstacle telemetry does not complete the MOVE waiter before its terminal result;
- use operation IDs to reconcile STOPPED and ERR/RESULT frames;
- add explicit terminal-abort state after the maximum resume count;
- add sensor health, motion ownership and link-session gates to resume;
- never resume automatically after reboot.

Gate S3:

- one, two and three obstacle interruptions on one segment do not cause overshoot;
- delayed, reordered or lost terminal frames end safely;
- pose and heading drift gates behave at their exact boundaries;
- X, R3 and PS2 cancel HOLD and RESUME immediately.

### Stage 4 — Robot-frame left/right verification

Status: REQUIRED / LOCKED BY S3

- define ROBOT_LEFT and ROBOT_RIGHT from the robot's forward direction;
- distinguish mount coordinates from robot coordinates in variable names;
- normalize physical channel wiring exactly once, preferably on STM32;
- remove any second swap on ESP32 if physical testing confirms it;
- parse left/right health and failure fields explicitly;
- log raw mount, STM32 robot-frame, RobotLink and ESP32 application values together;
- verify AI direction advice against sensor vetoes without moving motors.

Gate S4:

- left-only, right-only, center and clear physical tests agree at every layer;
- AI never approves a direction reported blocked by the corresponding robot-frame sensor.

### Stage 5 — Full Replay production

Status: DISABLED / LOCKED BY S1–S4

- replace the hard-coded 150 mm switch with explicit SHORT_SAFETY_TEST and FULL_PRODUCTION modes;
- report TEST_COMPLETE separately from ROUTE_COMPLETE;
- validate motion ownership, link session, cancel state, brake, PS2, sensor health and pose before every primitive;
- run all route segments and turns through one deterministic state machine;
- restore MANUAL, release motion ownership and clear resumable context on all terminal paths;
- report total target distance, travelled distance, error and final pose;
- require start-pose and start-heading confirmation because the robot has no absolute localization.

Gate S5 commissioning sequence:

1. host geometry/state-machine tests;
2. wheels-raised command and STOP test;
3. 150 mm straight test with repeated obstacles;
4. one full straight segment;
5. one isolated turn;
6. known two-segment MAP2 route;
7. L, rectangle, zigzag and longer routes;
8. five to ten repeat runs per accepted route.

### Stage 6 — Guarded obstacle detour and route rejoin

Status: DISABLED / LOCKED BY S5

State flow:

MOVING_ROUTE -> OBSTACLE_STOPPED -> SENSOR_HEALTH_CHECK -> SCAN -> PLAN -> CONFIRM/AUTO_POLICY -> DETOUR -> REJOIN -> RESUME_SEGMENT.

Required behavior:

- STM32 remains the motion and safety authority;
- local sensors veto camera/cloud suggestions;
- use short bounded TURN/MOVE primitives and rescan after every primitive;
- do not automatically reverse with only front sensors;
- limit detour distance, time, attempts and heading change;
- stop in WAIT or ABORT when no safe side exists;
- calculate projection and cross-track error to rejoin the original segment;
- resume only the remaining segment distance;
- keep fully automatic detour opt-in until hardware-in-loop commissioning passes.

Gate S6:

- obstacle left, right and center tests;
- moving-person and no-safe-side tests;
- cloud offline and camera timeout tests;
- one/both sensor fault and link-loss tests;
- X/R3/PS2 intervention tests;
- repeated obstacle during detour test;
- bounded failure with no infinite escape loop;
- route rejoin within the configured pose tolerance.

## 6. Post-foundation V5 optimizations

These are enabled only after Stage 6 or developed behind disabled feature flags:

- multi-route catalog with names, metadata and validation status;
- user PAUSE, RESUME, CANCEL and reason-specific HOLD states;
- multi-step voice missions;
- status explanation using real state-machine data;
- trapezoidal acceleration/deceleration and jerk limiting;
- per-wheel PID, dead-zone compensation and adaptive motor trim;
- encoder/IMU/compass disagreement and wheel-slip detection;
- startup preflight and safe degraded modes;
- calibration of ticks/mm, effective wheelbase, compass offset and motor balance;
- circular black-box event log;
- stronger RobotLink session, sequence, operation ID, CRC and terminal-result semantics;
- optional low-resolution line/color following;
- optional visual marker checkpoints for drift correction;
- patrol checkpoints with bounded camera capture and report generation.

## 7. Current-hardware limitations

V5.0 must not claim capabilities that the current hardware cannot safely support:

- it is not LiDAR SLAM;
- odometry and heading error still accumulate;
- two front HC-SR04 sensors do not provide rear, side or cliff coverage;
- automatic reverse is not safe without rear sensing;
- camera/cloud latency is non-deterministic;
- dynamic obstacle avoidance is limited to low-speed indoor experiments;
- high-assurance autonomous detour requires additional side/rear/cliff sensing.

## 8. Main implementation areas

ESP32-S3:

- `application.cc/.h`
- `mcp_server.cc/.h`
- `robot/robot_uart.cc/.h`
- `robot/mission_manager.cc/.h`
- `robot/teach_route.cc/.h`
- `robot/obstacle_assist.cc/.h`
- `robot/route_store.cc/.h`
- board camera and UI integration files

STM32:

- `control/robot_controller.cpp/.h`
- `communication/robot_link_server.cpp/.h`
- `ps2/ps2_controller.cpp/.h`
- `sensors/ultrasonic_sensor.cpp/.h`
- `safety/safety_watchdog.cpp/.h`
- `motor/motor_controller.cpp/.h`
- `encoders/wheel_odometry.cpp/.h`
- `localization/heading_fusion.cpp/.h`
- `main.cpp`

Tests and documents:

- version-aware static audit;
- RobotLink protocol unit tests;
- route geometry and state-machine tests;
- STOP latency and fault-injection checklist;
- left/right physical verification matrix;
- HOLD/RESUME repeated-interruption tests;
- Full Replay commissioning log;
- detour/rejoin commissioning log.

## 9. Release definition

V5.0 may be tagged production only when:

- both firmware targets build from a clean environment;
- all host tests pass;
- all Stage 1–6 gates have recorded results;
- no safety-critical test is waived;
- V4.x inherited features pass regression checks;
- the exact STM32 and ESP32 commit is flashed and recorded;
- motor-off, STOP, link-loss and sensor-fault behavior is verified on hardware;
- Full Replay and auto-detour defaults match their commissioned status;
- documentation matches the actual firmware behavior.

Until then, the branch remains `5.0.0-alpha.x` and Full Replay/automatic detour remain locked.

## 10. Alpha implementation record

Implemented in the current alpha branch and host-tested:

- independent STM32 fail-closed ultrasonic health classification (`UNKNOWN`, `HEALTHY`, `STALE`, `TIMEOUT`, `INVALID`, `DISCONNECTED_OR_FAULT`, `DEGRADED`);
- idempotent STM32 STOP path, motion-owner gate and RobotLink health/age telemetry;
- encoder reset generation/reason logging and replay resume rejection at an unresolved reset boundary;
- obstacle AI cooldown acquired after cooldown validation with cleanup on every worker exit;
- explicit `SHORT_SAFETY_TEST`/`FULL_PRODUCTION` replay mode, with production mode disabled by default;
- explicit `CONFIG_AUTOMATIC_DETOUR=n` commissioning gate; autonomous navigation stops at an obstacle while it is disabled;
- deterministic host regression suite in `tools/v5_host_selftest.py`.

Still requiring hardware/HIL validation: STOP latency, physical left/right mapping, encoder/heading calibration, Full Replay commissioning, automatic detour/rejoin, camera marker decoding and both clean firmware builds. These are not reported as PASS by the alpha branch.

## 11. Alpha.2 implementation record

### IMPLEMENTED_AND_HOST_TESTED

- Full Replay preflight now emits per-capability `PASS`, `WARN`, `FAIL` or
  `UNAVAILABLE` for link/session, owner, lease, STOP state, route, sensors,
  encoder, odometry/reset boundary and heading; camera is explicitly optional
  for basic replay.
- STM32 state telemetry includes the motion owner, so ESP32 replay refuses an
  unknown or conflicting owner instead of inferring ownership from movement.
- Every Full Replay terminal log names one of `ROUTE_COMPLETE`,
  `TEST_COMPLETE`, `CANCELLED`, `STOPPED`, `ABORTED_SENSOR_FAULT`,
  `ABORTED_LINK_LOSS`, `ABORTED_POSE_UNRELIABLE`,
  `ABORTED_RESET_BOUNDARY` or `INVALID_ROUTE`.
- Host regression coverage is expanded to 20 deterministic tests, including
  route transitions, preflight/degraded matrices, stale-result rejection model,
  owner/lease cleanup and reset-boundary handling.

### DISABLED_BY_DEFAULT

- `SHORT_SAFETY_TEST` remains default; `FULL_PRODUCTION` remains gated by
  `ROBOT_V5_FULL_REPLAY_PRODUCTION=0`.
- `CONFIG_AUTOMATIC_DETOUR=n`, automatic reverse, and automatic resume after
  AI stay disabled.

### IMPLEMENTED_HARDWARE_VALIDATION_REQUIRED

- The code and host tests do not prove hardware health, STOP latency, physical
  sensor side mapping, wheel calibration, fused-heading calibration, Full
  Production route execution, or link-loss behavior on a moving robot.
