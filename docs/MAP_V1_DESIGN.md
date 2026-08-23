# MAP V1 / Teach Route V1

## Scope and safety

MAP V1 records a route while the operator drives manually using the existing
STM32 PS2 control. It supports exactly two persistent slots, shown to the
operator as **MAP 1** and **MAP 2**.

This version has **NO REPLAY**. `TeachRoute` never sends MOVE, TURN, PWM, AI
mode, navigation, Return Home, or any other motor command. STM32 remains the
sole manual-motor controller. Camera Vision and AI Obstacle Assist V1 are not
changed by this feature.

## Architecture

`TeachRoute` owns UI/input state, a RAM working route, PS2 edge/long-press
handling, and 8 Hz odometry sampling while teaching. `RouteStore` owns only
NVS persistence and integrity validation. Neither uses MissionManager's
occupancy map or Return Home breadcrumbs.

The ESP32 obtains the existing STM32 `PS2,STATUS` `BTN` field and the existing
RobotLink odometry response. No STM32 protocol or firmware change is needed.

## Slots and persistent format

There are exactly two slots:

- `ROUTE_SLOT_1` — NVS key `m1`
- `ROUTE_SLOT_2` — NVS key `m2`

Namespace: `teach_route_v1`.

Each waypoint is compact and fixed-size:

```text
int32 x_mm
int32 y_mm
int16 heading_cdeg
uint8 flags
uint8 reserved
```

`MANUAL_MARK` is bit 0 of `flags`. Route coordinates are relative to the
odometry pose captured at Teach start: the first waypoint is always
`(0, 0, 0)`. Teach does not reset headings, encoders, or odometry.

The route header contains magic, format version, slot, valid flag, waypoint
count, generation, route length, and CRC32. CRC covers the header (with its
CRC field cleared) plus the saved waypoint bytes. A missing slot is `EMPTY`; a
bad magic, version, size, count, or CRC is reported `INVALID` and is never
silently erased.

`MAX_WAYPOINTS_PER_MAP = 128`. Each route takes at most about 1.6 KB, so both
slots remain well within the production NVS budget. NVS is written only on
SAVE and DELETE, never per waypoint. SAVE uses `nvs_set_blob`, `nvs_commit`,
then read-back CRC validation. DELETE erases only the selected key and commits.

## Recording and controls

Automatic points are added when displacement from the previous waypoint is at
least 125 mm or heading delta is at least 12 degrees. Triangle forces a point;
when the current pose duplicates the last point, it instead adds `MANUAL_MARK`.

All input comes from active-low PS2 `BTN`, with press/release edges and
long-press handling:

| Control | Action on MAP page |
| --- | --- |
| L3 | Toggle ROBOT PAGE / MAP PAGE |
| SELECT short, idle | Select MAP 1 / MAP 2 |
| TRIANGLE, idle | Start Teach for selected slot |
| TRIANGLE, teaching | Force mark waypoint |
| SQUARE short, teaching | Undo last point (never START) |
| CIRCLE, teaching | Save and end Teach |
| CIRCLE, idle | Load selected saved slot into RAM only |
| SELECT long (1.5 s), teaching | Cancel and discard RAM working route |
| SQUARE long (2 s), idle | Request deletion of selected slot |
| CIRCLE after deletion request | Confirm deletion |
| SELECT after deletion request | Cancel deletion |

Teach start is rejected unless RobotLink is connected, PS2 is fresh with BTN
available, odometry is valid, MissionManager is idle, and AI Obstacle hold is
not active. The feature does not cancel a mission to obtain these conditions.

## TFT behavior and boot

L3 selects a lightweight MAP page/status overlay without changing the Xiaozhi
chat layout or allocating a new framebuffer. It displays selected slot, saved
state, points, capacity, and mode. Notifications cover selection, teach,
point/mark, undo, save/load, delete confirmation, cancel, full route, and
storage errors.

On boot, `RouteStore` validates metadata for both keys and logs:

```text
ROUTE,BOOT,SLOT=1,STATE=SAVED,POINTS=...
ROUTE,BOOT,SLOT=2,STATE=EMPTY,POINTS=0
```

## Explicit limitations

- No replay, route following, auto navigation, map graph, or motor command.
- No camera landmark, AI Obstacle V2, Find Object, Find Owner, or new MCP tool.
- No STM32 source or flash change.
