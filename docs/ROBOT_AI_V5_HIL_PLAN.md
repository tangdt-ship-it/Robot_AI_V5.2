# Robot_AI_V5.0 Alpha.9 HIL readiness plan

Alpha.9 is the H2 RobotLink-recovery follow-up after H0 Alpha.4 and H1 Alpha.5
passed on hardware, Alpha.6 established passive black-box observability,
Alpha.7 removed deterministic `(SID, OP)` reuse across ESP32 reboots, and
Alpha.8 made Replay/HOLD reset boundaries aware of STM32 boot epochs.

Alpha.8 hardware proved the boot epoch itself works: an intentional STM32 RESET
changed `STM32_EPOCH 0 -> 1` while raw `RESET_GEN` remained `1 -> 1`, so the
composite reset boundary changed as intended. The robot stayed stopped and no
automatic resume occurred. The same test exposed the remaining defect: after
receiving `BOOT,STM32`, ESP32 invalidated the old RobotLink session but did not
automatically run HELLO/PING again, so `ROBOT_SESSION=READY` did not return.

Alpha.9 fixes only that recovery lifecycle. It does not change STM32 firmware,
RobotLink wire format, motor control, PID, obstacle policy, heading fusion or
Replay motion algorithms.

## Passed gates

- H0 Alpha.4: PASS.
- H1-A bounded correlated MOVE: PASS.
- H1-B finite TURN + explicit STOP: PASS on Alpha.5 after the waiter lifecycle
  fix; waiter release improved from about 12.86 s to 3.5 ms after `DONE,STOP`.
- H2 Alpha.6 idle ESP32 boundary: physical safety, RobotLink GET path and passive
  black-box observation validated.
- H2 Alpha.7 random correlation seed: hardware observed a non-fixed SID and all
  host/static guards passed.
- H2 Alpha.8 STM32 boot epoch: PASS for epoch/boundary detection. Intentional
  STM32 RESET produced `RESET_PIN`, `RESET_GEN 1 -> 1`, `STM32_EPOCH 0 -> 1`,
  session invalidation and final motor 0/0. RobotLink automatic recovery itself
  was not observed, which is the Alpha.9 blocker.

## Alpha.6+ black-box observability

Each `SafetyBlackBox::Record()` event remains stored in the fixed 48-entry,
RAM-only ring and emitted as passive serial telemetry:

`ROBOT_BLACKBOX SEQ=<n> MS=<ms> TYPE=<type> SID=<sid> OP=<op> RESET_GEN=<g> STM32_EPOCH=<e> ROUTE=<r> SEG=<s> REASON=<reason>`

The black-box path remains telemetry-only; it does not issue RobotLink commands
or actuator actions.

## Alpha.7 correlation seed hardening retained

- `motion_session_id_` starts from a non-zero ESP32 hardware-random seed;
- `next_operation_id_` starts from an independent non-zero hardware-random seed;
- successful HELLO/PING advances the session;
- zero remains invalid;
- no NVS write is introduced;
- RobotLink SID/OP remain 32-bit values.

## Alpha.8 STM32 boot epoch boundary retained

`SafetyBlackBox` advances an ESP32-local STM32 boot epoch only for the specific
`LINK_LOSS / STM32_BOOT` breadcrumb generated from an STM32 BOOT frame.
`TeachRoute` stores a composite `ResetBoundaryToken` containing raw
`RESET_GEN + STM32_BOOT_EPOCH`.

Therefore both conditions remain fail-closed:

- encoder/R2 reset changes raw `RESET_GEN` -> old Replay/HOLD context rejected;
- STM32 reboot changes boot epoch -> old context rejected even if raw
  `RESET_GEN` restarts at the same numeric value.

The epoch is RAM-only by design. An ESP32 reboot also destroys all volatile
Replay/HOLD resume context, so no persistent stale resume context survives it.

## Alpha.9 automatic RobotLink renegotiation

Before Alpha.9, `RunLinkTest()` performed HELLO/PING only once after ESP32 boot.
When a later STM32 BOOT frame arrived, `HandleFrame()` correctly invalidated
`protocol_compatible_`, the motion lease and pending finite correlation, but no
background path called `CheckProtocol()` again.

Alpha.9 adds a low-priority `ProtocolRecoveryTask` owned by `RobotUart`:

1. it waits for `RobotUart::Begin()` and the normal startup self-test window;
2. it observes the Alpha.8 `STM32_BOOT_EPOCH`;
3. when the epoch changes, recovery becomes pending;
4. recovery is attempted only when no motion lease, STOP transaction, finite
   correlation, TURN waiter or MOVE waiter is active;
5. recovery calls only `CheckProtocol(700)`, which performs HELLO/PING;
6. failed negotiation retries at a bounded rate;
7. successful negotiation emits the normal `SESSION_CHANGE/HELLO` breadcrumb
   and a non-zero randomized SID;
8. the recovery task never calls MODE, MOVE, TURN, STOP or continuous motion.

The task deliberately runs outside the UART RX task. Calling `CheckProtocol()`
from RX would deadlock because HELLO/PONG responses must be parsed by that same
RX task while the caller waits.

## Preconditions

- Use the branch/commit explicitly named by the H2 execution prompt.
- Run all V4/V4.2/V5 host/static tests including `tools/v5_h2_selftest.py`.
- Store runtime HIL logs outside the repository under
  `F:\Robot\Robot_AI_HIL_Logs`.
- Robot must be stopped, navigation idle, PS2 neutral and motors 0.
- H2 must not create MOVE/TURN merely to obtain recovery evidence.

## H2 remaining gates

1. **Alpha.9 deployment regression**
   - all tests/build/flash PASS;
   - no STM32 firmware change;
   - normal startup HELLO/PING still produces one stable session;
   - no unexpected motion.

2. **STM32 boot + automatic RobotLink recovery**
   - reset STM32 exactly once while robot is idle;
   - observe STM32 BOOT and incremented `STM32_EPOCH`;
   - observe old session invalidation;
   - observe automatic recovery request and HELLO/PING;
   - observe `ROBOT_SESSION=READY,SID=<nonzero>` without operator motion/action;
   - motors remain zero and no mission/replay resumes.

3. **Session uniqueness continuation**
   - after recovery, negotiated SID must remain non-zero and correlation guards
     from Alpha.7 remain intact;
   - no motion is needed merely to read a SID.

4. **Stale/missing/duplicate stage**
   - stale `(SID, OP)` cannot complete a new operation;
   - duplicate terminal is rejected;
   - session invalidation clears an old pending pair;
   - prefer host/synthetic evidence without chassis motion.

5. **Reset generation / HOLD-Resume boundary**
   - an idle encoder reset changes the saved boundary;
   - an STM32 boot changes the saved boundary even if raw `RESET_GEN` repeats;
   - old route/HOLD context is never silently resumed.

6. **Final bounded-motion regression**
   - only after H2 safety gates PASS;
   - one short finite MOVE may verify randomized SID/OP end-to-end on hardware;
   - final motor state must be zero.

## H2 pass criteria

- all host/static/H2 self-tests pass;
- black-box telemetry remains passive;
- Alpha.7 random SID/OP seeding remains intact;
- Alpha.8 STM32 boot epoch changes on STM32 reboot and composite boundaries
  remain fail-closed;
- Alpha.9 automatically re-establishes HELLO/PING and a non-zero session after
  STM32 reboot without any motion command;
- stale/duplicate/session guards remain fail-closed;
- all approved reset boundaries leave motors zero and no active motion lease;
- no mission/replay resumes automatically;
- no watchdog/reboot loop or uncontrolled run occurs.

## Exit criteria

Record tests/build output, resource sizes, external log paths, STM32 BOOT and
reset-cause evidence, `ROBOT_BLACKBOX` epoch/session breadcrumbs, recovery
latency, recovered SID, final idle state and anomalies. Alpha.9 remains
development firmware until this H2 recovery gate passes.
