# Robot_AI_V4.2 — Release notes

## Mục tiêu
V4.2 tập trung vào độ chính xác chuyển động và định vị thay vì thêm tính năng AI mới. STM32 vẫn giữ quyền điều khiển motor/safety; ESP32-S3/Xiaozhi làm mission, vision và lập kế hoạch.

## Nâng cấp chính
- MPU6050 trên Soft-I2C riêng: `SCL=PC8`, `SDA=PB5`, `0x68`.
- Hỗ trợ đúng cách lắp MPU6050 face-down: connector về sau, hai lỗ bắt vít về trước.
- Tự hiệu chuẩn gyro khi boot; robot phải đứng yên trong giai đoạn này.
- Heading fusion: Gyro Z + encoder yaw + Compass, có degraded mode khi thiếu một nguồn.
- Odometry STM32 tích hợp X/Y bằng fused heading.
- ESP32 MissionManager bỏ ước lượng vị trí theo thời gian chạy motor; dùng `GET,ODOMETRY` từ STM32.
- Breadcrumb theo odometry thực và mission `RETURN_HOME`.
- MCP mới: odometry, IMU, fusion, set/get home, return home.
- Diagnostic UART3 cho IMU/fusion/odometry.
- Giữ RobotLink V3, PS2 override, HC-SR04 hard safety, watchdog và encoder stall detection.

## Chưa bật trong V4.2
- PID tốc độ bánh xe chưa được bật production vì cần calibration PWM → mm/s trên robot thật.
- Return Home là odometry/breadcrumb, **không phải SLAM**; sai số vẫn tích lũy khi trượt bánh hoặc va chạm.
- Không dùng gia tốc MPU6050 để tích phân trực tiếp ra X/Y.

## Thứ tự commissioning bắt buộc
1. Test `stm32_imu_probe` với robot đứng yên.
2. Xác nhận IMU: `READY=1`, `CAL=1`, `HEALTH=OK`, `AZ≈+1g`; quay trái nhìn từ trên xuống thì `GZ` phải dương theo quy ước robot.
3. Build/nạp `stm32_robot_v4_2` và kiểm tra `IMU`, `FUSION`, `ODOM` trên UART3.
4. Build/nạp ESP32-S3; kiểm tra `GET,IMU`, `GET,FUSION`, `GET,ODOMETRY` trước khi MOVE/TURN.
5. Set HOME rồi thử Return Home trên quãng ngắn, tốc độ thấp, khu vực thoáng.
6. Chỉ tăng quãng đường sau khi hiệu chỉnh ticks/rev, track width và heading fusion.
