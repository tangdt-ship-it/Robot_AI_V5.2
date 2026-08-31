# Wheel-speed PID candidate

The STM32 now contains two allocation-free wheel-speed PID controllers. They
use the existing left/right encoder velocity measurements and adjust only the
applied motor command. The requested command, PS2 arbitration, obstacle veto,
STOP, brake and encoder-stall safety paths remain outside the PID loop.

## Profiles

- `stm32_robot_v4_2`: stable production profile; PID disabled by default.
- `stm32_robot_v4_2_wheel_pid`: commissioning candidate; PID enabled.

PID is intentionally not enabled in the stable profile yet. The default
`WHEEL_PID_TARGET_MM_S_PER_COMMAND` and gains are conservative starting values,
not a claim of calibration for the assembled robot. Measure each wheel's
PWM/command-to-mm/s response and tune left/right independently before a ground
test.

## Control contract

- encoder velocity is the feedback signal;
- the existing signed command is the feed-forward term;
- integral windup is bounded and blocked while output saturation would worsen;
- integral state resets on STOP, brake, zero command and direction reversal;
- PID output can reduce a wheel to zero but cannot reverse it while the owner
  command has the original direction;
- PID never bypasses obstacle/safety decisions and never replaces the final
  motor-zero path;
- no ESP32 buffer or task is added.

First commissioning must be performed with both wheels raised, then at low
speed in an open safe area. Compare `MOTOR` telemetry (`L/R`, `PWM_L/PWM_R`)
with `ENCODER` telemetry (`LV/RV`) and stop immediately if direction, encoder
polarity or the applied PWM is unexpected.
