# Robot_AI_V5.0 Alpha.7 HIL readiness plan

Alpha.7 is the H2 session-uniqueness hardening follow-up after H0 Alpha.4 and
H1 Alpha.5 passed on hardware and Alpha.6 established passive black-box
observability. It does not change STM32 motion control or the RobotLink wire
format. Its firmware change removes deterministic cross-reboot reuse of the
first `(SID, OP)` pair by seeding both correlation counters from the ESP32
hardware RNG before the first protocol negotiation.

## Passed gates

- H0 Alpha.4: PASS.
- H1-A bounded correlated MOVE: PASS.
- H1-B finite TURN + explicit STOP: PASS on Alpha.5 after the waiter lifecycle
  fix; waiter release improved from about 12.86 s to 3.5 ms after `DONE,STOP`.
- H2 Alpha.6 idle ESP32 session boundary: PASS for physical safety, RobotLink
  recovery, black-box observation and unchanged STM32 `RESET_GEN`.

The Alpha.6 boundary run also exposed a correlation issue: after ESP32 reboot,
`SESSION_SID` returned to `1`, while the operation counter also restarted from
its initial value. A stale pre-reboot frame could therefore collide with the
first post-reboot correlation pair. Alpha.7 closes that gap.

## Alpha.6+ black-box observability

Each `SafetyBlackBox::Record()` event is stored in the fixed 48-entry RAM-only
ring and emitted as a passive serial breadcrumb:

`ROBOT_BLACKBOX SEQ=<n> MS=<ms> TYPE=<type> SID=<sid> OP=<op> RESET_GEN=<g> ROUTE=<r> SEG=<s> REASON=<reason>`

The trace path has no RobotUart/actuator dependency and must never acquire a
motion lease or issue MOVE/TURN/STOP.

## Alpha.7 correlation seed hardening

- `motion_session_id_` starts from a non-zero ESP32 hardware-random seed;
- `next_operation_id_` starts from an independent non-zero hardware-random
  seed;
- the existing successful HELLO/PING session advance remains in place;
- zero remains invalid;
- no NVS write is introduced, so there is no flash-wear penalty from session
  negotiation;
- the RobotLink STM32 protocol is unchanged: SID and OP remain 32-bit values.

The hardware authority for this change is the H2 session-uniqueness test: two
successive ESP32 boots while STM32 remains powered and idle must negotiate
non-zero SIDs that do not repeat in the observed run, and no motion may occur.
The independently randomized OP seed further reduces stale-pair collision risk.

## Preconditions

- Use the branch/commit explicitly named by the H2 execution prompt.
- Run all V4/V4.2/V5 host/static tests, including `tools/v5_h2_selftest.py`.
- Store runtime HIL logs outside the repository under
  `F:\Robot\Robot_AI_HIL_Logs`.
- Robot must be physically stopped, navigation idle, PS2 neutral and motors 0.
- H2 must not create a MOVE/TURN merely to obtain evidence.

## H0 — historical pass

H0 established RobotLink HELLO/PING, non-zero SID, strict finite-motion SID/OP
guards, stopped motor state and no uncontrolled motion.

## H1 — historical pass

H1-A established correlated bounded finite MOVE and matching ACK/progress/DONE.
H1-B established physical STOP plus prompt finite-waiter cancellation, stale
result rejection, lease release and final stopped state.

## H2 — fault/recovery observation, no autonomous motion

1. **Host stale/missing/duplicate trace tests**
   - stale `(SID, OP)` must not complete a new operation;
   - duplicate terminal must be rejected;
   - session invalidation must clear the old pending pair;
   - reset-generation mismatch must block resume/preflight.

2. **Passive black-box trace**
   - capture `ROBOT_BLACKBOX` during boot/session negotiation and H2 checks;
   - sequence values must advance within one boot and fields must be well formed;
   - no black-box observation may create motion.

3. **ESP32 session-boundary uniqueness**
   - keep STM32 powered and robot idle;
   - observe a negotiated SID, reset only ESP32 once, and observe the next SID;
   - both SIDs must be non-zero and different in the HIL run;
   - STM32 `RESET_GEN` must not change;
   - no mission, lease or motor command may resume automatically.

4. **Reset generation**
   - observe `RESET_GEN` before and after any explicitly approved idle encoder
     reset action;
   - if generation changes, old route/hold context must not be silently resumed;
   - no replay is started automatically in H2.

5. **Diagnostic labels**
   - `UNCALIBRATED`, `UNAVAILABLE`, `HEADING_UNRELIABLE` or similar labels are
     observations only; they never authorize an automatic action.

## H2 pass criteria

- all host/static/H2 self-tests pass;
- black-box telemetry is observable and passive;
- cross-reboot SID reuse is not observed in the approved H2 run;
- stale/duplicate/session/reset guards remain fail-closed;
- any approved idle reset/session boundary leaves motors at zero and no active
  motion lease;
- no mission/replay resumes automatically;
- no uncontrolled run occurs.

## Exit criteria

Record host/build output, resource sizes, external log path, relevant
`ROBOT_BLACKBOX` excerpts, pre/post-reset SIDs, reset-generation observations,
anomalies and final safe state. Alpha.7 remains development firmware until H2
hardware evidence passes.
