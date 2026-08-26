# Robot_AI_V5.0 Alpha.6 HIL readiness plan

Alpha.6 is the H2 observability follow-up after H0 Alpha.4 and H1 Alpha.5 both
passed on hardware. It does not change STM32 motion control or the ESP32 motion
protocol. Its only firmware change is passive forensic logging of each event
already committed to the bounded RAM safety black-box.

## Passed gates

- H0 Alpha.4: PASS.
- H1-A bounded correlated MOVE: PASS.
- H1-B finite TURN + explicit STOP: PASS on Alpha.5 after the waiter lifecycle
  fix; waiter release improved from about 12.86 s to 3.5 ms after `DONE,STOP`.

## Alpha.6 observability

Each `SafetyBlackBox::Record()` event is still stored in the fixed 48-entry,
RAM-only ring and is now also emitted as a passive serial breadcrumb:

`ROBOT_BLACKBOX SEQ=<n> MS=<ms> TYPE=<type> SID=<sid> OP=<op> RESET_GEN=<g> ROUTE=<r> SEG=<s> REASON=<reason>`

The trace path has no RobotUart/actuator dependency and must never acquire a
motion lease or issue MOVE/TURN/STOP. It exists only to make H2 evidence
observable in the external HIL log.

## Preconditions

- Use branch/commit explicitly named by the H2 execution prompt.
- Run all V4/V4.2/V5 host/static tests, including `tools/v5_h2_selftest.py`.
- Store runtime HIL logs outside the repository, under
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

## H2 — fault/recovery observation, no motion

H2 uses only safe idle-state operations and host trace playback.

1. **Host stale/missing/duplicate trace tests**
   - stale `(SID, OP)` must not complete a new operation;
   - duplicate terminal must be rejected;
   - session invalidation must clear the old pending pair;
   - reset-generation mismatch must block resume/preflight.

2. **Passive black-box trace**
   - capture `ROBOT_BLACKBOX` lines during boot/session negotiation and all H2
     observations;
   - sequence values must advance and event fields must be well formed;
   - no black-box observation may create motion.

3. **Idle reset/session boundary**
   - perform only the operator-approved reset/reconnect action named in the H2
     execution prompt while motors are already zero;
   - observe session/link invalidation and recovery;
   - verify no motion lease survives the boundary and final state remains idle.

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
- stale/duplicate/session/reset guards are fail-closed;
- any approved idle reset/session boundary leaves motors at zero and no active
  motion lease;
- no mission/replay resumes automatically;
- no uncontrolled run occurs.

## Exit criteria

Record host/build output, resource sizes, external log path, relevant
`ROBOT_BLACKBOX` excerpts, reset/session observations, anomalies and the final
safe state. Alpha.6 remains development firmware until H2 hardware evidence
passes.
