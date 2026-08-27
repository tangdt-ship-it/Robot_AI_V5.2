# Robot_AI_V5.0 Alpha.8 HIL readiness plan

Alpha.8 is the H2 reset-boundary hardening follow-up after H0 Alpha.4 and H1
Alpha.5 passed on hardware, Alpha.6 established passive black-box observability,
and Alpha.7 removed deterministic `(SID, OP)` reuse across ESP32 reboots.

Alpha.8 does not change STM32 motion control or the RobotLink wire format. Its
firmware change makes Replay/HOLD reset-boundary checks depend on both the raw
STM32 `RESET_GEN` and an ESP32-observed STM32 boot epoch, so a reboot cannot be
hidden by `RESET_GEN` restarting at the same numeric value.

## Passed gates

- H0 Alpha.4: PASS.
- H1-A bounded correlated MOVE: PASS.
- H1-B finite TURN + explicit STOP: PASS on Alpha.5 after the waiter lifecycle
  fix; waiter release improved from about 12.86 s to 3.5 ms after `DONE,STOP`.
- H2 Alpha.6 idle ESP32 session boundary: PASS for physical safety, RobotLink
  recovery and passive black-box observation.
- H2 Alpha.7 correlation seeding: host/static tests PASS and hardware observed a
  non-fixed random SID after reboot (`3092733370`).

## H2 observations that motivated Alpha.8

During the first Alpha.7 session-uniqueness run, operator reset only ESP32 but
STM32 `RESET_GEN` changed from `3` to `1`. A later diagnostic ESP32 reset did not
reproduce an STM32 reboot: `RESET_GEN` stayed `1 -> 1`, odometry/ticks were
stable and no STM32 BOOT/reset event was observed.

Source audit established two facts:

1. STM32 `resetWheelCounts()` only increments `RESET_GEN`; it cannot change
   `3 -> 1`.
2. STM32 odometry `begin()/reset()` initializes a fresh runtime generation, so
   a real STM32 reboot can restart the raw number and eventually reuse a value
   that existed before the reboot.

Replay/HOLD previously compared only raw `RESET_GEN`. That is fail-closed for an
ordinary encoder reset, but it is not a complete reboot boundary if the raw
number repeats.

## Alpha.6+ black-box observability

Each `SafetyBlackBox::Record()` event remains stored in the fixed 48-entry,
RAM-only ring and emitted as passive serial telemetry. Alpha.8 adds the current
STM32 boot epoch to the line:

`ROBOT_BLACKBOX SEQ=<n> MS=<ms> TYPE=<type> SID=<sid> OP=<op> RESET_GEN=<g> STM32_EPOCH=<e> ROUTE=<r> SEG=<s> REASON=<reason>`

The trace path has no RobotUart/actuator dependency and must never acquire a
motion lease or issue MOVE/TURN/STOP.

## Alpha.7 correlation seed hardening retained

- `motion_session_id_` starts from a non-zero ESP32 hardware-random seed;
- `next_operation_id_` starts from an independent non-zero hardware-random
  seed;
- successful HELLO/PING still advances the session;
- zero remains invalid;
- no NVS write is introduced;
- RobotLink SID/OP remain 32-bit values.

## Alpha.8 STM32 boot epoch boundary

`SafetyBlackBox` advances an ESP32-local `STM32_BOOT_EPOCH` only when RobotUart
records the specific `LINK_LOSS / STM32_BOOT` breadcrumb generated from the
STM32 BOOT frame. `InvalidateMotionCorrelation("STM32_BOOT")` cannot double-count
the same reboot because it records a different event type.

`TeachRoute` stores a `ResetBoundaryToken` instead of a bare reset generation.
The token captures:

- raw STM32 `RESET_GEN`;
- current ESP32-observed `STM32_BOOT_EPOCH`.

Existing replay/HOLD comparison sites remain fail-closed:

- raw generation changes -> boundary mismatch;
- STM32 BOOT epoch changes -> boundary mismatch even if raw generation repeats;
- zero keeps its existing "no saved boundary" sentinel meaning.

The epoch is deliberately RAM-only. If ESP32 itself reboots, volatile Replay and
HOLD resume contexts are also destroyed, so no persistent resume context can
survive with an old epoch.

## Preconditions

- Use the branch/commit explicitly named by the H2 execution prompt.
- Run all V4/V4.2/V5 host/static tests, including `tools/v5_h2_selftest.py`.
- Store runtime HIL logs outside the repository under
  `F:\Robot\Robot_AI_HIL_Logs`.
- Robot must be stopped, navigation idle, PS2 neutral and motors 0.
- H2 must not create MOVE/TURN merely to obtain evidence.

## H2 remaining gates

1. **Alpha.8 passive/deployment regression**
   - all tests/build/flash PASS;
   - black-box includes `STM32_EPOCH`;
   - normal ESP32 reset while STM32 stays alive does not change the epoch;
   - no motion occurs.

2. **STM32 boot-boundary proof while idle**
   - only an explicitly approved STM32 reset may be performed;
   - ESP32 must observe `BOOT,STM32` and increment `STM32_EPOCH`;
   - motors remain zero and no mission/replay resumes;
   - raw `RESET_GEN` may restart, but the composite boundary must still change.

3. **Stale/missing/duplicate host trace tests**
   - stale `(SID, OP)` cannot complete a new operation;
   - duplicate terminal is rejected;
   - session invalidation clears an old pending pair.

4. **Reset generation / HOLD-Resume boundary**
   - an idle encoder reset changes the saved boundary;
   - a STM32 boot changes the saved boundary even if raw `RESET_GEN` repeats;
   - old route/HOLD context is never silently resumed.

5. **Final bounded-motion regression**
   - only after H2 safety gates PASS;
   - one short finite MOVE may verify randomized SID/OP end-to-end on hardware;
   - final motor state must be zero.

## H2 pass criteria

- all host/static/H2 self-tests pass;
- black-box telemetry is observable and passive;
- Alpha.7 random SID/OP seeding remains intact;
- STM32 BOOT epoch advances on an observed STM32 reboot and remains stable on an
  ESP32-only reboot;
- Replay/HOLD reset-boundary checks reject either raw-generation or boot-epoch
  changes;
- stale/duplicate/session guards remain fail-closed;
- all approved reset boundaries leave motors at zero and no active motion lease;
- no mission/replay resumes automatically;
- no uncontrolled run occurs.

## Exit criteria

Record tests/build output, resource sizes, external log paths, relevant
`ROBOT_BLACKBOX` excerpts including `STM32_EPOCH`, reset/session observations,
anomalies and final safe state. Alpha.8 remains development firmware until H2
hardware evidence passes.
