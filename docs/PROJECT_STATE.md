# Project State

## Stable product baseline

`V4.3 AI Obstacle Assist V1`

Status: `STABLE / COMMISSIONED`

Stable tag/checkpoint already published:

- `v4.3-ai-obstacle-v1-stable`

Core commissioned capabilities:

- `CAMERA_VISION = PASS`
- `CAMERA_YUV422_SVGA = PASS`
- `CAMERA_SRAM_OPTIMIZATION = PASS`
- `AI_OBSTACLE_ASSIST_V1 = COMMISSIONED / PASS`
- `TEST_A/B/C/D = PASS`
- `CAMERA_FRAME = NORMAL`
- `GLOBAL_HOLD = PASS`
- `SUPPRESS_DIAGONAL_BYPASS = PASS`
- `EXPLICIT_CANCEL_CLEAR_HOLD = PASS`
- `PS2_OVERRIDE = PASS`
- `SR04_SAFETY_OVERRIDE = PASS`
- `AI_AUTO_DRIVE = NO`
- `MISSION_RESUME_AFTER_AI = NO`
- `MEMORY_LEAK = NONE`

## SR04 robot frame

STM32 raw SR04 channels are normalized on ESP32 before AI Obstacle safety arbitration:

- `ROBOT_LEFT = RAW_RIGHT`
- `ROBOT_RIGHT = RAW_LEFT`

Raw channels remain available for diagnostics. Camera frame is normal; no camera mirror/flip change is required.

---

# MAP / Teach Route / Replay development state

## Current development branch

`feature/map-replay-v1-turn`

Current commissioned repository head:

`578d751efca50924135565b4f757d7327ce1f776`

This branch contains the commissioned MAP Teach/Smart Waypoint work, Replay Phase A, B1 straight-segment motion, B2 turn-only implementation, and the MAP UI V2 parser regression fix.

## MAP input ownership and UI

Current architecture:

`PS2 -> STM32 -> semantic EVENT,MAP -> ESP32 TeachRoute/RouteStore`

- `INPUT_OWNER = STM32`
- `ESP32_RAW_PS2_MAP_POLLING = OFF`
- `ESP32_MAP_INPUT_TASK = OFF`
- MAP UI is shown on STM32 20x4 LCD.
- ESP32 TFT remains dedicated to Xiaozhi/camera UI.
- MAP UI wire protocol: `MAP_UI_PROTO = 2`
- Replay LCD modes currently cover READY / TEACH / LOADED / DELETE / REPLAY_READY / REPLAY_CHECKED / REPLAY_RUNNING / REPLAY_HOLD / REPLAY_COMPLETE.

### Current MAP buttons

On ROBOT page:

- `R3` = brake toggle (replaces START brake function)
- `START` = unused by ROBOT page and reserved for MAP workflow
- `X/CROSS` = unused by ROBOT page and reserved for MAP workflow

On MAP page:

- `L3` = enter/exit MAP page
- `SELECT` short = select MAP1/MAP2 when state is not locked
- `START` = start Teach / advance Replay state
- `TRIANGLE` = manual important waypoint while teaching
- `SQUARE` short = undo last teach waypoint
- `SQUARE` long = delete request
- `CIRCLE` = load/save/confirm depending on state
- `X` / long SELECT semantic path = cancel/stop

## Smart Waypoint V1

Status: `COMMISSIONED / PASS`

Commissioned configuration:

- `AUTO_DISTANCE = 750 mm`
- `CORNER_TRIGGER = 25 deg`
- `CORNER_STABLE_SAMPLES = 3`
- `MAX_POINTS_PER_MAP = 128`
- manual important points are preserved
- endpoint is added when needed
- old dense `125 mm / 12 deg` behavior is no longer used

Commissioning result:

- legacy MAP1 test route: `21 points / 1084 mm`
- Smart MAP2 test route: `3 points / 950 mm`
- reboot persistence/load = PASS
- motor commands from MAP during Teach = NONE
- robot movement source during Teach = PS2 only

Smart Waypoint implementation checkpoint:

`91d0fa565067799aecbc59913cc16a9eda10a9ae`

## Replay Phase A — dry run

Status: `PASS`

Replay dry-run validates route geometry and storage without motor motion.

MAP2 dry-run evidence:

- WP1 -> WP2 = `771 mm`
- WP2 -> WP3 = `179 mm`
- summed/stored route length = `950 / 950 mm`
- `MOTOR = 0`
- robot movement = NONE

## Replay B1 — first straight segment

Status: `COMMISSIONED / PASS`

Tested on MAP2:

- target WP1 -> WP2 = `771 mm`
- speed = `10`
- turn = OFF
- `START_GATE = PASS / OK`
- `WORKER_PRECHECK = PASS / OK`
- `OBS_VALID=1, OBS_FRESH=1, OBS_ZONE=CLEAR`
- `ECHO=0 + FRESH + CLEAR` is allowed as no-return-clear; it is not treated as an obstacle by itself
- travelled = `771.3 mm`
- internal odometry error = `0.3 mm`
- robot stopped completely at WP2
- WP3 was not run
- no replay turn command was issued
- ESP32 reset = NO
- STM32 reset = NO

B1 tested code checkpoint:

`0a3ac2f718f26110bda118ca341ec504fc0953f9`

Note: `0.3 mm` is encoder/odometry-reported segment error, not an externally measured absolute-position accuracy claim.

## MAP UI V2 parser regression — fixed

The STM32 V2 parser uses 11 numeric conversions plus an optional trailing `%c` validator. A valid V2 frame therefore returns `fields == 11`.

Correct parser condition:

`const bool v2 = fields == 11;`

Regression fix checkpoint:

`d2acc3b91745effbdd805ac7a85bca9b9645293e`

Verified after STM32 flash:

- `MAP,UI,2,1,2,3,128,950,0,0,0,0,0` accepted
- `ERR,MAP_UI = NONE`
- MAP2 LCD leaves `SYNC` and shows `SAVED / 3 points / 950 mm`

Do not revert this condition to `fields == 12` in later branches.

## Replay B2 — waypoint turn

Implementation status: `COMMISSIONED / PASS`

B2 physical commissioning result on MAP2:

- `B2_START_GATE = PASS / OK`
- worker transport precheck attempt `1/3 = PASS / OK`
- `B12 = -1.7 deg`, `B23 = -7.4 deg`, runtime delta = `-5.7 deg`, direction = `RIGHT`
- RobotLink turn command: `TURN,REL,RIGHT,6,10`
- final heading = `2.3 deg`, RobotLink target = `-0.5 deg`, reported error = `-2.8 deg`
- `B2_TURN_DONE`, `CONTINUE=NO`, `MOVE=0`
- manual mode restore = `PASS`
- robot stopped; WP3 was not run; no translation command was issued

The previous physical run exposed a transient `STATE_RX` at the worker precheck. The hardened policy now uses a 900 ms replay safety transaction timeout and retries only `STATE_RX` / `OBS_RX` up to three attempts (100 ms then 150 ms delay). Safety failures are never retried. RobotLink transaction failures log the failure stage (`MUTEX_TIMEOUT`, `SEND_FAIL`, `WAIT_TIMEOUT`, or `NACK`).

B2 design:

- compute `bearing12 = atan2(WP2-WP1)`
- compute `bearing23 = atan2(WP3-WP2)`
- `turnDelta = normalize(bearing23 - bearing12)`
- positive delta = LEFT, negative delta = RIGHT
- use STM32 closed-loop `TurnRelative()` at speed 10
- no timed/open-loop turn
- B2 only turns at WP2 and stops
- no WP2 -> WP3 translation in B2
- no full replay yet
- dynamic turn safety stops if ultrasonic is stale or zone is not CLEAR
- `ECHO=0 + FRESH + CLEAR` remains allowed
- X / R3 / PS2 takeover / obstacle safety retain stop authority

- STM32 session recovery: after STM32 `BOOT`, the ESP32 invalidates the cached
  protocol session; B2 renegotiates `HELLO -> PING -> MODE,AI` before `TURN` and
  restores `MODE,MANUAL` after the turn

WP3 was not run. Do not run B3 or full replay until a separate commissioning
decision is made.

---

# Data ownership / synchronization rules

- Source of truth for code: GitHub repository branch/commit listed above.
- Local workspace must `git pull --ff-only` before each build/flash/test cycle.
- Codex should not change source unless the prompt explicitly requests a patch.
- After an approved local patch passes build/test, commit and push it before starting the next feature phase.
- Before branching to a new phase, preserve the last commissioned code checkpoint in Git history and update this state document.
- MAP route contents themselves live in ESP32 NVS and are not stored in Git. Current MAP1/MAP2 point counts are commissioning data, not source-controlled assets.
