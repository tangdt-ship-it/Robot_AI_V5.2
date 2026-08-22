#!/usr/bin/env python3
"""Host-side sanity tests for V4.2 axis mapping and Return-Home geometry."""
from __future__ import annotations
import math


def normalize(deg: float) -> float:
    while deg > 180.0:
        deg -= 360.0
    while deg <= -180.0:
        deg += 360.0
    return deg


def robot_axis(raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz):
    # Face-down; connector rear; mounting holes front.
    return raw_ax, -raw_ay, -raw_az, raw_gx, -raw_gy, -raw_gz


def target_heading(x, y, tx, ty):
    return normalize(math.degrees(math.atan2(ty - y, tx - x)))


def main() -> None:
    # Face-down gravity: if IMU +Z points down, raw Az=-1g becomes robot +Z=+1g.
    ax, ay, az, gx, gy, gz = robot_axis(0.0, 0.0, -1.0, 0.0, 0.0, -45.0)
    assert abs(az - 1.0) < 1e-9
    # Positive robot CCW yaw must invert the face-down sensor's raw Gyro Z.
    assert abs(gz - 45.0) < 1e-9
    assert ax == 0.0 and ay == 0.0 and gx == 0.0 and gy == 0.0

    assert target_heading(100.0, 0.0, 0.0, 0.0) == 180.0
    assert abs(target_heading(0.0, 0.0, 100.0, 100.0) - 45.0) < 1e-9
    assert abs(target_heading(0.0, 0.0, 0.0, -100.0) + 90.0) < 1e-9

    # A breadcrumb trail reversed must terminate at HOME (index 0).
    trail = [(0.0, 0.0), (25.0, 0.0), (50.0, 10.0), (75.0, 25.0)]
    reversed_trail = list(reversed(trail))
    assert reversed_trail[-1] == (0.0, 0.0)

    # Wrap-safe shortest heading differences.
    assert normalize(-179.0 - 179.0) == 2.0
    assert normalize(179.0 - (-179.0)) == -2.0

    print("PASS V4.2 localization/axis/return-home host self-test")


if __name__ == "__main__":
    main()
