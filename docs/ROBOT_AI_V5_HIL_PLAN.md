# Robot_AI_V5.0 Alpha.3 HIL readiness plan

This is a manual preparation plan only. It contains no flashing, serial-port,
reset, reconnect or motion-control script.

## Preconditions

- Build both production firmware targets from the Alpha.3 commit.
- Run `python tools/v5_host_selftest.py`, `python tools/v5_static_audit.py`,
  `python tools/v4_protocol_selftest.py`, and
  `python tools/v4_2_localization_selftest.py`.
- Confirm the physical area is clear, the robot is supported safely when
  appropriate, an operator has the physical E-stop/STOP procedure available,
  and no autonomous run is armed by this document.
- Record firmware commit, board IDs, date, operator, test area and any known
  sensor/calibration limitations. Do not place credentials or secrets in logs.

## H0 — link and correlation observation (no motion)

1. Establish the normal operator-approved connection using existing tooling.
2. Observe boot, HELLO/PING and state telemetry. Record the newly negotiated
   non-zero SID in the test sheet.
3. Inspect only diagnostic output for a deliberately invalid/blocked preflight;
   expected result is no lease and no motion command.
4. If a protocol trace is captured, verify a finite motion request would use
   `SID,OP` and that malformed, zero, missing and mismatched pairs are rejected.

Pass: link and safety gates are observable; no motion is caused by H0.

## H1 — controlled bounded motion, operator initiated

This phase is executed manually by an authorized operator after H0 passes.
Use the existing stop procedure; this plan never authorizes automatic control.

- Begin with the shortest approved straight-line safety test in a clear area.
- Correlate each accepted ACK, progress and terminal frame by exact `(SID, OP)`.
- Inject no synthetic sensor values and make no calibration/PID/PWM changes.
- On obstacle, encoder, heading, link or reset-boundary concern, stop and
  record the terminal reason. Do not auto-resume.

Pass: each result matches the issued pair; STOP/cancel/link loss leaves no
pending operation or active lease.

## H2 — fault-injection observation and recovery

Use only safe, operator-approved methods that do not move the robot.

- Observe stale/missing/duplicate frame handling from trace playback or an
  isolated approved test fixture; stale frames must not complete a new op.
- Observe session change/reset boundary while idle; the next replay requires a
  fresh preflight and manual start.
- Review the fixed RAM black-box breadcrumbs: preflight, owner/lease,
  command, ACK/result accept-or-stale, hold/resume/reset/stop/cancel/link and
  terminal events. The buffer is RAM-only, bounded, non-persistent and
  contains no secrets.
- Treat `UNCALIBRATED` and `UNAVAILABLE` diagnostics as observation labels,
  not permission for any new automatic action.

Pass: fail-closed behaviour is demonstrated without an uncontrolled run.

## Evidence and exit criteria

Record host/build output, resource sizes, correlation trace excerpts, operator
observations, anomalies and final safety state. Alpha.3 is HIL-ready code and
documentation, not a claim that hardware validation has occurred.
