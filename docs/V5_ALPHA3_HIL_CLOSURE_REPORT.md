# Robot_AI V5 Alpha.3 HIL Closure Report

## 1. Purpose

This document closes the hardware-in-the-loop hardening campaign that started from `5.0.0-alpha.3` and finished on `5.0.0-alpha.9`.

It is intentionally a compact engineering record for future V5.x/V6 development. Raw serial logs remain outside the repository; this report preserves the conclusions, regression history, tested firmware checkpoints and safety invariants that later work must not break.

## 2. Final status

- Campaign: `V5 Alpha.3 HIL hardening`
- Final result: `PASS`
- Final tested firmware version: `5.0.0-alpha.9`
- Final tested firmware checkpoint: `7ec2349cb4878d8acba49c09de288cc95c3aef8c`
- Development branch after closure: `develop/robot-ai-v5.0`
- H0: `PASS`
- H1: `PASS`
- H2: `PASS`
- Final bounded-motion regression: `PASS`
- Unexpected motion across the closed campaign: `NO`

The campaign PASS does **not** mean every V5 feature is production-ready. Full Replay production, automatic detour/rejoin/resume, calibration and later roadmap features remain separate commissioning gates.

## 3. Firmware checkpoints

| Stage | Version | Tested checkpoint | Main purpose |
|---|---|---|---|
| H0 closure | `5.0.0-alpha.4` | `08affa1997aa5e56cf5974540ded82dcbfc95fb3` | Basic fail-closed/correlation hardware validation |
| H1 closure | `5.0.0-alpha.5` | `dad0a5be9053b004aafce0125410226940afedff` | Finite-operation STOP/waiter lifecycle |
| H2 + final closure | `5.0.0-alpha.9` | `7ec2349cb4878d8acba49c09de288cc95c3aef8c` | Reboot/session/reset-boundary hardening + final motion regression |

Documentation-only commits may exist after a tested firmware checkpoint. The checkpoint SHA above is the firmware source that was actually built/flashed/tested.

## 4. Main defects found and fixed during HIL

### Alpha.3 -> Alpha.4: strict finite-motion correlation

The first H0 run showed that passive evidence and static guards were insufficient to prove strict finite-operation `(SID, OP)` correlation. H0 hardening added/validated:

- non-zero SID/OP requirement for finite MOVE/TURN;
- missing/zero/mismatched correlation rejection;
- stale result rejection;
- fail-closed preflight and owner/lease state;
- H0 observability needed to prove these rules on hardware.

H0 then passed on Alpha.4.

### Alpha.4 -> Alpha.5: STOP did not unblock finite TURN waiter

H1-B found a real ESP32 operation-lifecycle bug:

- physical STOP was successful;
- `DONE,STOP` arrived in about `139.5 ms` in the failing Alpha.4 run;
- motor safety was correct;
- old `ERR,TURN,CANCELLED` was not accepted as completion;
- but the finite TURN task remained blocked for about `12862.5 ms` before returning.

Alpha.5 made public STOP cancellation-aware while keeping physical `DONE,STOP` authoritative.

Retest result:

- physical STOP remained safe;
- finite TURN waiter release improved to about `3.5 ms` after `DONE,STOP`;
- stale cancelled result still was not accepted;
- final motor state remained zero;
- final motion lease was `NONE`.

### Alpha.6: passive H2 black-box observability

Alpha.6 added bounded RAM-only safety black-box telemetry so H2 could observe session/reset/link boundaries without creating motion. The black-box remains diagnostic-only and does not issue actuator commands.

### Alpha.7: deterministic SID reuse across ESP32 reboot

H2 showed that the old in-RAM session ID initialization could cause the first post-reboot SID to return to a fixed small value. This creates a cross-reboot correlation-collision risk if the operation counter also restarts.

Alpha.7 changed the per-boot SID and OP seeds to independent ESP32 hardware-random non-zero values while preserving the existing session-advance logic.

Hardware then observed non-fixed large SID values instead of deterministic `SID=1`.

### Alpha.8: raw RESET_GEN was not a sufficient STM32 reboot boundary

A hardware observation showed raw `RESET_GEN` could change `3 -> 1`. Source audit confirmed that raw reset generation can restart after an STM32 reboot, so the same numeric generation can be reused.

This created an edge case for Replay/HOLD-Resume if reset-boundary validation used only raw `RESET_GEN`.

Alpha.8 added an ESP32-observed `STM32_BOOT_EPOCH` and changed Replay/HOLD reset-boundary validation to use the composite token:

`(RESET_GEN, STM32_BOOT_EPOCH)`

Controlled STM32 reset validated:

- raw `RESET_GEN`: `1 -> 1`;
- STM32 boot epoch: `0 -> 1`;
- composite boundary: `CHANGED / PASS`;
- no automatic Resume;
- motor remained zero.

### Alpha.9: RobotLink did not recover automatically after STM32 reboot

Alpha.8 correctly invalidated the old session after `BOOT,STM32`, but HIL showed that RobotLink did not automatically run HELLO/PING again.

Source audit found that the normal link test was one-shot at ESP32 startup and heartbeat traffic only existed during an active motion lease.

Alpha.9 added a low-priority, non-motion protocol recovery task that:

- observes STM32 boot-epoch change;
- waits until motion/cancellation state is idle;
- performs `CheckProtocol()` only;
- automatically issues HELLO/PING;
- never issues MODE/MOVE/TURN/STOP or reacquires a motion lease.

Hardware validation after controlled STM32 reboot:

- old session invalidated;
- boot epoch changed;
- recovery request observed;
- automatic HELLO/PING observed;
- SID changed `3935795156 -> 3935795157`;
- recovery latency about `476 ms`;
- no recovery motion command;
- no automatic mission/replay Resume;
- final lease `NONE`;
- final motor state zero.

## 5. H0 closure evidence

H0 Alpha.4 validated on hardware:

- all V4/V4.2/V5 host/static/H0 tests passed;
- STM32 and ESP32-S3 production build/flash passed;
- finite MOVE/TURN correlation guards passed;
- negotiated SID was non-zero;
- ultrasonic health was `HEALTHY/CLEAR`;
- encoder was ready and healthy;
- odometry query passed;
- Compass/heading query passed;
- start/final motor command state was zero;
- no unexpected motion.

## 6. H1 closure evidence

### H1-A finite MOVE

Validated command:

`MOVE,FWD,50,10,SID,1,OP,2`

Evidence:

- ACK exact-pair correlation: `PASS`;
- all observed progress frames used the same pair;
- terminal exact-pair correlation: `PASS`;
- terminal travel: `50.2 mm` for `50.0 mm` target;
- measured odometry delta: approximately `49.9 mm`;
- reset generation unchanged;
- final motor state zero;
- no unexpected motion.

### H1-B finite TURN + STOP

Final Alpha.5 retest:

- finite TURN correlation: `PASS`;
- STOP physical safety: `PASS`;
- STOP-to-waiter release: approximately `3.5 ms`;
- stale cancelled TURN result accepted as completion: `NO`;
- reset generation unchanged;
- navigation final `IDLE`;
- motion lease final `NONE`;
- motor final zero.

## 7. H2 closure evidence

H2 validates the non-motion safety/recovery path and host-side stale/duplicate guards.

Hardware validated:

- idle ESP32 reboot did not create automatic motion;
- random SID behavior across reboot;
- controlled STM32 boot was observed and old session invalidated;
- boot epoch changed even when raw reset generation repeated;
- composite reset boundary changed correctly;
- RobotLink auto-recovered using HELLO/PING only;
- recovered SID was new and non-zero;
- recovery latency was approximately `476 ms`;
- read-only RobotLink GET path worked after recovery;
- no watchdog/reboot loop;
- no automatic Resume;
- motor remained zero;
- final lease `NONE`.

Host H2 regression tests guard:

- stale terminal cannot complete a new session;
- duplicate terminal is rejected;
- session invalidation clears an old correlation pair;
- raw reset-generation change blocks Resume;
- STM32 boot-epoch change blocks Resume even if raw reset generation repeats;
- Alpha.9 protocol recovery is non-motion and outside the UART RX task.

## 8. Final bounded-motion regression

The final motion test intentionally used the session that had been recovered by Alpha.9 after an STM32 reboot.

Command:

`MOVE,FWD,50,10,SID,3935795157,OP,277399117`

Result:

- SID matched the recovered session: `YES`;
- ACK correlation: `PASS`;
- progress correlation: `PASS`;
- terminal correlation: `PASS`;
- target: `50.0 mm`;
- internal travelled result: `53.0 mm`;
- measured odometry delta: `46.8 mm`;
- raw reset generation: `1 -> 1`;
- final obstacle state: `HEALTHY/CLEAR`, about `50.8 cm`;
- encoder final velocity: `0 / 0`;
- navigation final: `IDLE`;
- motion lease final: `NONE`;
- motor final: `0 / 0`;
- unexpected motion: `NO`.

This closes the end-to-end path:

STM32 reboot -> old session invalidation -> composite reset boundary -> automatic HELLO/PING recovery -> new SID -> finite correlated MOVE -> ACK/PROGRESS/DONE -> safe stop.

## 9. Safety invariants that future versions must preserve

Any future V5.x/V6 change touching RobotLink, motion, Replay/HOLD, reset handling or recovery must preserve at least these invariants:

1. Finite MOVE/TURN requires non-zero `(SID, OP)` correlation.
2. Missing, zero, mismatched, stale or duplicate terminal correlation must not complete an operation.
3. STOP physical confirmation remains authoritative before releasing a finite waiter.
4. STOP/cancel/session/reset invalidates pending motion correlation.
5. STM32 reboot invalidates the old RobotLink session.
6. Replay/HOLD reset boundary must include both raw `RESET_GEN` and STM32 boot epoch semantics.
7. RobotLink recovery after STM32 reboot must not issue motion commands or automatically Resume a mission.
8. Session/operation seeds must not deterministically reuse the same initial pair after ESP32 reboot.
9. Final safe state after failure/recovery is motor zero, navigation idle and no motion lease.
10. HIL/runtime logs stay outside the repository worktree.

## 10. Known limitations / next gates

The following are **not** closed by this HIL campaign and must be treated as separate development/commissioning work:

- physical left/right ultrasonic mapping characterization under controlled geometry;
- wheel-diameter, wheelbase and heading calibration/tolerance characterization;
- closed-loop two-wheel speed PID;
- Full Replay production commissioning under the hardened gates;
- autonomous obstacle Detour -> Rejoin -> Resume;
- patrol and waypoint actions;
- AprilTag/QR landmark localization;
- camera AI object/person search;
- landmark-assisted Return Home / docking.

Until their own gates pass, V5 remains `DEVELOPMENT / NOT PRODUCTION` even though the Alpha.3 core HIL hardening campaign is closed.

## 11. Raw evidence retention

Raw logs are intentionally **not committed** to GitHub.

External evidence root used during this campaign:

`F:\Robot\Robot_AI_HIL_Logs`

See `docs/V5_HIL_EVIDENCE_MANIFEST.md` for the evidence index and log locations.
