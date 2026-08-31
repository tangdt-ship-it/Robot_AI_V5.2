# Robot AI V5.2.2 — MCP Motion Architecture

## Scope

V5.2.2 starts by simplifying the AI-facing MCP motion surface without changing the commissioned STM32 motor/safety algorithms, RobotLink finite MOVE/TURN primitives, HOME implementation, or TeachRoute/MAP implementation.

The first goal is deterministic tool selection by Xiaozhi/LLM. Legacy callbacks remain registered for compatibility and diagnostics but overlapping motion aliases are hidden from the normal AI `tools/list` response.

## Phase 1 — implemented in 5.2.2-alpha.1

AI-facing motion tools kept visible:

- `self.robot.stop`
- `self.robot.move_distance`
- `self.robot.turn_relative`
- `self.robot.turn_to_heading`
- `self.robot.set_home`
- `self.robot.return_home`
- `self.robot.scan_obstacle` (physical body scan; description explicitly warns that the robot turns)

Supporting read/config tools such as diagnostics, speed and HOME status remain available.

Legacy/overlapping motion tools hidden from normal AI discovery:

- `self.robot.turn_and_move`
- `self.robot.navigate_autonomously`
- `self.robot.cancel_mission`
- `self.robot.move_forward`
- `self.robot.move_backward`
- `self.robot.turn_left`
- `self.robot.turn_right`
- `self.robot.rotate_continuous`

They are not deleted yet, so existing firmware callbacks and manual compatibility paths are preserved while V5.2.2 is commissioned.

## Language mapping policy

### STOP

All of the following map to `self.robot.stop`:

- dừng
- dừng lại
- đứng lại
- ngừng
- ngừng lại
- stop

STOP is always higher priority than any route, sequence or mission.

### Distance motion

`self.robot.move_distance` is the only normal AI tool for forward/backward motion with a requested distance.

Unit conversion before the tool call:

- 1 mm = 1 mm
- 1 cm = 10 mm
- 1 m = 1000 mm
- **1 bước = 5 cm = 50 mm**

Forward language includes: tiến, đi thẳng, tiến về phía trước, đi lên, tiến lên, tiến thẳng, di chuyển lên/trước.

Backward language includes: lùi, lùi lại, lùi về sau, thụt lại, đi lùi, di chuyển về sau.

The current callback still uses `forward=true/false`; a later schema cleanup may replace this boolean with `direction=forward|backward` after HIL regression.

### Relative turn

`self.robot.turn_relative` is the only normal AI tool for a relative left/right angle.

Language includes: quay, xoay, rẽ, quẹo, nghiêng.

- quay đầu = 180 degrees
- quay đầu trái = left 180 degrees
- quay đầu phải = right 180 degrees

Current STM32/RobotUart `TurnRelative()` accepts 1..180 degrees per primitive. Multi-round turns are therefore a Phase 2 wrapper feature and must be decomposed into bounded primitives with STOP/safety checks between primitives.

### HOME

`self.robot.set_home` accepts intents such as set home, đặt vị trí nhà, đặt vị trí xuất phát, đánh dấu điểm xuất phát.

`self.robot.return_home` accepts intents such as về nhà, quay về chỗ cũ, trở lại vị trí ban đầu, quay về điểm xuất phát.

### Environment scan

`self.robot.scan_obstacle` currently remains the implementation name, but its description now explicitly states **PHYSICAL MOTION** because the robot turns its body to scan left/right.

A simple question about the obstacle directly in front should use `self.robot.get_diagnostics(target=obstacle)` and must not cause body motion.

## Automatic detour policy

`CONFIG_AUTOMATIC_DETOUR=n` remains the V5.2.2 default. Therefore `self.robot.navigate_autonomously` is hidden from normal AI discovery until automatic detour is physically commissioned and its advertised behavior matches firmware behavior.

## Phase 2 — next implementation

Add a deterministic motion-sequence layer on ESP32 rather than asking the LLM to improvise primitive ordering.

Target API:

- `self.robot.turn_angle`: wrapper for 1..N degrees / number of rounds, internally decomposed into <=180 degree `TurnRelative()` primitives.
- `self.robot.motion_sequence`: ordered MOVE/TURN steps with a single sequence owner, sequence id, cancellation token, terminal result and STOP authority.

Examples:

- `quay trái 2 vòng` -> turn_angle(left, 720) -> 4 x 180 degree primitives.
- `đi thẳng 0.5 m, rẽ trái 0.5 m, rẽ phải 0.5 m` -> deterministic MOVE/TURN/MOVE sequence.
- a failed/blocked step aborts or holds the sequence; later steps never execute silently after a failure.

## Phase 3 — unified route/MAP

Add one route execution abstraction shared by voice-created routes and PS2 Teach MAP routes.

Target APIs:

- `self.robot.route_record`
- `self.robot.run_route`

Data sources:

1. Voice/command route recording: set HOME / execute motion / mark or auto-mark waypoints.
2. Existing PS2 TeachRoute: manual and Smart Waypoint MAP1/MAP2.

Both should feed a single `RouteExecutor` rather than create a second replay engine.

The existing `TeachRoute::StartFullReplay()` is currently private; MCP must not simulate PS2 button events to bypass its state machine. A public safe route API should be added after the MAP replay production gate is re-commissioned.

## Safety invariants

Every future motion-capable MCP tool must:

1. participate in STOP cancellation generation;
2. respect a single motion owner;
3. fail closed on RobotLink/session/sensor faults;
4. never continue to the next sequence/route primitive after an unhandled failure;
5. never auto-resume after reboot;
6. preserve STM32 as final motor and safety authority;
7. keep automatic detour disabled until HIL commissioning passes.

## V5.2.2 commissioning checklist

Before promoting alpha.1:

- inspect AI `tools/list`: legacy motion aliases are absent;
- verify `stop`, `move_distance`, `turn_relative`, `set_home`, `return_home` are advertised;
- voice test synonyms for STOP;
- voice test 50 mm / 5 cm / 1 step equivalence;
- test forward and backward distance commands;
- test left/right 30, 45, 90 and 180 degree commands;
- test set HOME and return HOME regression;
- ask only "có vật cản phía trước không" and confirm the robot does not rotate;
- ask "quét xung quanh" and confirm scan motion is explicit;
- verify `navigate_autonomously` is not advertised while automatic detour is disabled;
- verify STOP during every active finite motion.

Only after this passes should Phase 2 add sequence/multi-round motion.
