# Robot_AI V5 HIL Evidence Manifest

## Purpose

This file is the index of hardware-in-the-loop evidence for the V5 Alpha.3 hardening campaign.

Raw COM/serial logs are intentionally kept **outside** the Git repository so the production worktree remains clean and `tools/v5_static_audit.py` can enforce that no `logs/` tree exists in the repository.

Primary external evidence root:

`F:\Robot\Robot_AI_HIL_Logs`

Final campaign result:

`HIL_V5_ALPHA3_CAMPAIGN = PASS`

Final tested firmware:

- Version: `5.0.0-alpha.9`
- Checkpoint: `7ec2349cb4878d8acba49c09de288cc95c3aef8c`

For the engineering summary and defect history, see:

`docs/V5_ALPHA3_HIL_CLOSURE_REPORT.md`

## Evidence handling rules

1. Do not copy raw runtime/HIL logs into the repository.
2. Preserve logs under the external evidence root or a backed-up equivalent location.
3. Do not rename historical log folders casually after they have been referenced here.
4. A documentation commit after a firmware checkpoint does not change the tested firmware SHA.
5. When a future test supersedes an older failed/blocked test, keep both if practical; the older log explains why the firmware changed.
6. For future V5.x/V6 campaigns, add new entries to this manifest rather than overwriting the Alpha.3 campaign evidence.

## H0 evidence

### H0 Alpha.3 initial blocker

Purpose:

- establish passive H0 behavior;
- expose insufficient SID/OP observability/guard evidence;
- motivate Alpha.4 H0 hardening.

Result:

`FAIL / BLOCKED` before Alpha.4 correction.

Historical raw logs were part of the Alpha.4-era local HIL tree later archived under:

`F:\Robot\Robot_AI_HIL_Logs\alpha4_archive\hil`

Firmware lineage:

- starting point: `5.0.0-alpha.3`
- H0 closure firmware: `5.0.0-alpha.4`
- H0 tested checkpoint: `08affa1997aa5e56cf5974540ded82dcbfc95fb3`

### H0 Alpha.4 PASS

Result:

`PASS`

Key evidence preserved in project state/closure report:

- all V4/V4.2/V5 host/static/H0 tests passed;
- non-zero SID observed;
- finite MOVE/TURN strict correlation guards passed;
- obstacle/encoder/odometry/heading health passed;
- start/final motors zero;
- no unexpected motion.

## H1 evidence

### H1-A clearance diagnostics

Purpose:

- establish that repeated precheck blockers were due to actual/valid ultrasonic distance observations rather than noisy sensor data;
- verify raw dual HC-SR04 values.

Historical archive root:

`F:\Robot\Robot_AI_HIL_Logs\alpha4_archive\hil`

Known historical subtrees include the Alpha.4 clearance/raw-H1 captures such as:

- `alpha4_clearance_raw`
- `alpha4_h1a_final`
- `alpha4_h1a_final2`
- `alpha4_h1b`

These were moved out of the repository during static-audit cleanup and preserved under the external Alpha.4 archive.

### H1-A finite MOVE PASS

Firmware:

- Version: `5.0.0-alpha.4`
- Tested checkpoint: `08affa1997aa5e56cf5974540ded82dcbfc95fb3`

Command/evidence:

- `MOVE,FWD,50,10,SID,1,OP,2`
- ACK correlation: `PASS`
- progress correlation: `PASS`
- terminal: `DONE,MOVE,TARGET,50.0,TRAVEL,50.2,SID,1,OP,2`
- odometry delta: about `49.9 mm`
- reset generation unchanged
- final motor state zero

Archived raw evidence is under the Alpha.4 archive root above.

### H1-B Alpha.4 waiter defect

Purpose:

- finite TURN active;
- explicit STOP after correlated ACK;
- verify physical stop and finite waiter release.

Firmware:

- Version: `5.0.0-alpha.4`

Result:

`FAIL_STOP_DID_NOT_UNBLOCK_FINITE_WAITER`

Key evidence:

- physical STOP safe;
- `DONE,STOP` approximately `139.5 ms` after operator STOP;
- stale `ERR,TURN,CANCELLED` not accepted;
- TURN task did not return for approximately `12862.5 ms`.

Archived raw evidence:

`F:\Robot\Robot_AI_HIL_Logs\alpha4_archive\hil\alpha4_h1b\...`

This failed log is important and should be retained because it proves the Alpha.5 lifecycle fix was necessary.

### H1-B Alpha.5 PASS

Raw log:

`F:\Robot\Robot_AI_HIL_Logs\alpha5_h1b\com4_h1b.log`

Firmware:

- Version: `5.0.0-alpha.5`
- Tested checkpoint: `dad0a5be9053b004aafce0125410226940afedff`

Result:

`PASS`

Key evidence:

- finite TURN exact-pair ACK: `PASS`
- operator STOP physical safety: `PASS`
- STOP-to-TURN-waiter release: about `3.5 ms`
- stale cancelled terminal accepted: `NO`
- final navigation: `IDLE`
- final lease: `NONE`
- final motor: zero
- unexpected motion: `NO`

## H2 evidence

### Alpha.6 passive black-box precheck

Raw log:

`F:\Robot\Robot_AI_HIL_Logs\alpha6_h2_precheck\com4_h2_precheck.log`

Firmware:

- Version: `5.0.0-alpha.6`

Result:

`BLOCKED_INCOMPLETE_BOOT_SESSION_BLACKBOX_EVIDENCE`

Meaning:

- runtime itself was stable;
- capture started after startup HELLO/PING, so startup session/black-box evidence was missed;
- no motion was run.

This log is diagnostic evidence, not a firmware failure.

### Alpha.6 idle ESP32 session-boundary PASS

Raw log:

`F:\Robot\Robot_AI_HIL_Logs\alpha6_h2_idle_boundary\com4_idle_boundary.log`

Firmware:

- Version: `5.0.0-alpha.6`

Result:

`PASS`

Key evidence:

- ESP32 reset only;
- STM32 remained safe;
- RobotLink recovered;
- `SESSION_CHANGE` black-box observed;
- reset generation unchanged in that run;
- no automatic Resume;
- motor remained zero.

This stage also exposed that the post-reboot SID could return to deterministic `1`, leading to Alpha.7.

### Alpha.7 session-uniqueness / reset-generation anomaly

Raw log:

`F:\Robot\Robot_AI_HIL_Logs\alpha7_session_uniqueness\com4_session_uniqueness.log`

Firmware:

- Version: `5.0.0-alpha.7`
- Branch lineage: session-uniqueness hardening

Result:

`FAIL_UNEXPECTED_RESET_GENERATION_CHANGE`

Key evidence:

- random SID behavior worked; example observed `SID_A=3092733370`;
- raw `RESET_GEN` unexpectedly changed `3 -> 1`;
- second reset was intentionally not run;
- robot remained idle and motor-safe.

This failure triggered STM32 reboot/reset-boundary attribution and the Alpha.8 composite-boundary fix.

### Alpha.7 STM32-reset attribution diagnostic

Raw log:

`F:\Robot\Robot_AI_HIL_Logs\alpha7_stm32_reset_diag\com4_reset_diag.log`

Firmware:

- Version: `5.0.0-alpha.7`

Result:

`STM32_REBOOT_NOT_REPRODUCED`

Key evidence:

- diagnostic ESP32 reset did not reproduce an STM32 reboot;
- no STM32 BOOT frame/reset-cause event observed;
- raw reset generation stayed `1 -> 1`;
- odometry/ticks were essentially unchanged;
- motor remained zero.

This log does **not** explain the earlier `3 -> 1`; it only proves the reset coupling was not reproducible in that diagnostic run.

### Alpha.8 STM32 boot-epoch boundary

Raw log:

`F:\Robot\Robot_AI_HIL_Logs\alpha8_stm32_boot_epoch\com4_stm32_boot_epoch.log`

Firmware:

- Version: `5.0.0-alpha.8`
- Checkpoint lineage: STM32 boot-epoch reset-boundary hardening

Result:

Boundary logic: `PASS`

RobotLink recovery in Alpha.8: `FAIL / NOT RECOVERED`

Key evidence:

- controlled STM32 reset count: `1`;
- STM32 reset cause: `RESET_PIN`;
- raw reset generation: `1 -> 1`;
- STM32 epoch: `0 -> 1`;
- composite boundary changed: `PASS`;
- old RobotLink session invalidated;
- no automatic Resume;
- motor remained zero;
- RobotLink did not automatically renegotiate, leading to Alpha.9.

### Alpha.9 first recovery attempt

Raw log:

`F:\Robot\Robot_AI_HIL_Logs\alpha9_robotlink_recovery\com4_robotlink_recovery.log`

Firmware:

- Version: `5.0.0-alpha.9`
- Tested firmware checkpoint lineage: `7ec2349cb4878d8acba49c09de288cc95c3aef8c`

Result:

`BLOCKED_STARTUP_SID_NOT_CAPTURED`

Meaning:

- all tests/build/flash already passed;
- capture opened after startup negotiation;
- checklist expected a startup marker that was not available in that capture;
- STM32 reset was not run.

This is a procedure/capture blocker, not a firmware failure.

### Alpha.9 RobotLink auto-recovery PASS

Raw log:

`F:\Robot\Robot_AI_HIL_Logs\alpha9_robotlink_recovery_retry\com4_robotlink_recovery_retry.log`

Firmware:

- Version: `5.0.0-alpha.9`
- Tested checkpoint: `7ec2349cb4878d8acba49c09de288cc95c3aef8c`

Result:

`PASS`

Key evidence:

- startup SID before STM32 reset: `3935795156`;
- controlled STM32 reboot observed;
- old session invalidated;
- STM32 epoch changed `0 -> 1`;
- recovery request observed;
- auto HELLO: `YES`;
- auto PING: `YES`;
- recovery retry count: `0`;
- recovered SID: `3935795157`;
- SID changed across STM32 reboot: `YES`;
- boot-to-session-recovery latency: about `476 ms`;
- recovery motion command observed: `NO`;
- raw reset generation changed `3 -> 1`;
- composite boundary: `PASS`;
- read-only RobotLink after recovery: `PASS`;
- final motor zero;
- final lease `NONE`;
- automatic Resume: `NO`;
- unexpected motion: `NO`.

## Final bounded-motion evidence

Raw log:

`F:\Robot\Robot_AI_HIL_Logs\alpha9_final_motion\com4_final_motion.log`

Firmware:

- Version: `5.0.0-alpha.9`
- Final tested checkpoint: `7ec2349cb4878d8acba49c09de288cc95c3aef8c`

Result:

`PASS`

Command:

`MOVE,FWD,50,10,SID,3935795157,OP,277399117`

Key evidence:

- route: `self.robot.move_distance`;
- SID matched the previously recovered RobotLink session;
- ACK exact-pair correlation: `PASS`;
- progress exact-pair correlation: `PASS`;
- terminal exact-pair correlation: `PASS`;
- terminal: `DONE,MOVE,TARGET,50.0,TRAVEL,53.0,SID,3935795157,OP,277399117`;
- internal travelled: `53.0 mm`;
- measured odometry delta: `46.8 mm`;
- raw reset generation unchanged `1 -> 1`;
- final obstacle: `HEALTHY/CLEAR`, approximately `50.8 cm`;
- final encoder velocity: zero;
- final navigation: `IDLE`;
- final lease: `NONE`;
- final motor: zero;
- unexpected motion: `NO`.

This is the final end-to-end HIL evidence closing the Alpha.3 hardening campaign.

## Final host/static regression evidence

At Alpha.9 closure, the following test families were reported `PASS`:

- `tools/v4_protocol_selftest.py`
- `tools/v4_2_localization_selftest.py`
- `tools/v5_host_selftest.py`
- `tools/v5_static_audit.py`
- `tools/v5_h0_selftest.py`
- `tools/v5_h1_selftest.py`
- `tools/v5_h2_selftest.py`

Important H2 host guards include:

- stale terminal rejection across session renewal;
- duplicate terminal rejection;
- raw reset-generation Resume rejection;
- STM32 boot-epoch Resume rejection even when raw generation repeats;
- hardware-random SID/OP seeding guards;
- Alpha.9 background recovery restricted to non-motion protocol renegotiation.

## Evidence preservation checklist for future work

Before deleting/reorganizing local HIL data:

- preserve `alpha5_h1b`;
- preserve `alpha6_h2_idle_boundary`;
- preserve `alpha7_session_uniqueness` and `alpha7_stm32_reset_diag`;
- preserve `alpha8_stm32_boot_epoch`;
- preserve `alpha9_robotlink_recovery_retry`;
- preserve `alpha9_final_motion`;
- preserve the Alpha.4 archive, especially the failing H1-B waiter log and the passing H1-A correlation log.

If the external root is moved to another disk, update this manifest with the new root but keep the historical test names and firmware checkpoint references intact.
