# Robot_AI_V4.2 — Motion / heading control

## Heading source

Ưu tiên điều khiển dùng `HeadingFusion`:

- Gyro Z MPU6050: phản ứng nhanh;
- encoder differential yaw: phản hồi cơ học;
- Compass: correction tuyệt đối chậm.

Nếu IMU lỗi, fusion có thể degraded bằng Encoder+Compass hoặc một nguồn còn sống. Turn bị từ chối chỉ khi không còn heading source khả dụng.

## Odometry

Encoder được sample trước, sau đó fusion tính heading của vòng hiện tại, cuối cùng `WheelOdometry::integratePose()` dùng heading đó để tích phân X/Y. ESP32 không còn suy ra quãng đường từ thời gian chạy motor.

## Distance

`MOVE_DISTANCE` vẫn do STM32 đóng vòng theo encoder. MissionManager V4.2 dùng primitive này cho Return Home thay vì duration->distance.

## Wheel-speed PID

V4.2 **chưa bật PID tốc độ bánh chưa tune**. Trước khi bật cần đo PWM->mm/s, dead-zone và tune riêng từng bánh. Safety/PS2 phải nằm ngoài vòng PID.
