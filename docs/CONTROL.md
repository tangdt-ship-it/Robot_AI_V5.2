# Robot_AI — Motion / heading control

## Heading source

Ưu tiên điều khiển dùng `HeadingFusion`:

- Gyro Z MPU6050: phản ứng nhanh;
- encoder differential yaw: phản hồi cơ học;
- Compass: correction tuyệt đối chậm.

Nếu IMU lỗi, fusion có thể degraded bằng Encoder+Compass hoặc một nguồn còn sống. Turn bị từ chối chỉ khi không còn heading source khả dụng.

## Odometry

Encoder được sample trước, sau đó fusion tính heading của vòng hiện tại, cuối cùng `WheelOdometry::integratePose()` dùng heading đó để tích phân X/Y. ESP32 không còn suy ra quãng đường từ thời gian chạy motor.

## Distance

`MOVE_DISTANCE` do STM32 đóng vòng theo encoder. MissionManager dùng primitive này cho Return Home thay vì duration->distance.

## Motor driver cascade

Robot dùng hai Driver Motor DC PID V1.0, mỗi driver điều khiển vòng tốc độ
nội của một bánh bằng encoder riêng. STM32 chỉ phát lệnh trái/phải và chiều
quay ở vòng ngoài; STM32 không chạy wheel-speed PID trong firmware production.
Safety, PS2, obstacle limiter, heading và odometry vẫn thuộc quyền STM32.
