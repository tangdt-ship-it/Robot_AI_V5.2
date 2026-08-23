# Project State

## Current stable version

`V4.3 AI Obstacle Assist V1`

## Current status

`STABLE / COMMISSIONED`

## Commissioned capabilities

- `CAMERA_VISION = PASS`
- `CAMERA_YUV422_SVGA = PASS`
- `CAMERA_SRAM_OPTIMIZATION = PASS`
- `AI_OBSTACLE_ASSIST_V1 = COMMISSIONED / PASS`
- `TEST_A = PASS`
- `TEST_B = PASS`
- `TEST_C = PASS`
- `TEST_D = PASS`
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

The STM32 raw channels are normalized on ESP32 before AI Obstacle safety
arbitration:

- `ROBOT_LEFT = RAW_RIGHT`
- `ROBOT_RIGHT = RAW_LEFT`

Raw channels remain available for diagnostics. Camera frame is normal; no
camera mirror, flip, orientation, YUV422/SVGA, or vision-pipeline change is
required.

## Next feature

`MAP V1 / TEACH ROUTE`

## Current feature branch

`MAP V1 / TEACH ROUTE` is in development on `feature/map-v1`. Stable `main`
remains the commissioned V4.3 baseline. Map V1 records and persists manual
PS2 routes only; it does not replay, navigate, or issue motor commands.

No work has started for AI Obstacle V2, replay route, find object, find owner,
or camera landmark.
