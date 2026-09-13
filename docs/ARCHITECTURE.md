# Robot_AI — Kiến trúc chuẩn

## 1. Phân tầng

**ESP32-S3**: Xiaozhi, voice/MCP, camera, MissionManager, breadcrumb, Return Home, local planning.

**STM32F103VET6**: PWM/DIR motor, PS2 arbitration, HC-SR04 hard limiter, encoder TIM2/TIM3, MPU6050, Compass, heading fusion, odometry, turn/distance control, LCD, watchdog.

STM32 luôn là motor authority cuối cùng. ESP32 chỉ yêu cầu chuyển động cấp cao.

## 1.1 STM32 Flash contract

The physical STM32 target reports 256 KiB Flash through the F_SIZE register
(0x0100). Firmware reserves the final 12 KiB, from 0x0803D000 through
0x08040000, for MAP and wheel calibration storage. MAP uses four pages from
0x0803D000 through 0x0803F000; WCAL uses the final two pages as an A/B record
store: WCAL_A=0x0803F000 and WCAL_B=0x0803F800. There is no reserved gap
between MAP and WCAL, and WCAL_B ends at 0x08040000. The application linker
region ends before 0x0803D000.

## 1.2 Two-stage cascade control

**Inner loop — external hardware**

Two Driver Motor DC PID V1.0 modules are mandatory. Each driver receives the
STM32 wheel command and direction, reads its wheel encoder, and maintains the
wheel/motor speed.

**Outer motion control — STM32F103VET6**

STM32 owns distance moves, heading turns, sensor fusion, odometry, MAP
Teach/Replay, guided waypoints, path and cross-track correction, Back-to-P0,
Return Home, arbitration, and safety. STM32 does not run a wheel-speed PID in
the production firmware. Command values such as 15/20/25/30 are driver
commands, not mm/s.

## 2. Localization / odometry

```text
Encoder L/R ---> encoder yaw -----+
                                  |
MPU6050 Gyro Z ------------------>+--> HeadingFusion --> fused heading
                                  |
Compass absolute heading -------->+
                                           |
                                           v
Encoder translation ----------> WheelOdometry --> X / Y / theta
                                           |
                                      RobotLink V3
                                           |
                                           v
                                    ESP32 MissionManager
```

Fusion dùng gyro cho động học nhanh, encoder kiểm chứng yaw cơ học và Compass làm tham chiếu tuyệt đối chậm. Compass bị gate khi robot đang chạy nếu sai lệch quá lớn để giảm nhiễu từ motor.

## 3. Quyền ưu tiên

1. hardware E-stop nếu có;
2. STM32 safety/watchdog/brake/obstacle/encoder fault;
3. PS2 manual override;
4. AI/autonomous motion lease;
5. idle.

## 4. Return Home

ESP32 lưu HOME theo odometry STM32 và breadcrumb theo quãng đường/thay đổi heading. Khi Return Home, robot duyệt waypoint theo thứ tự ngược, quay về waypoint rồi chạy **tiến**, không chạy lùi cả hành trình. Mỗi bước vẫn bị HC-SR04/PS2/safety STM32 giới hạn.

Đây là odometry Return Home, chưa phải SLAM/global localization.
