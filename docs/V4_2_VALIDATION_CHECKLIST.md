# Robot_AI_V4.2 — Validation checklist

Các test motor đầu tiên phải kê bánh.

## A. Source/build

- [ ] `python tools\v4_protocol_selftest.py` PASS
- [ ] `python tools\v4_2_localization_selftest.py` PASS
- [ ] `python tools\v4_2_static_audit.py` PASS
- [ ] `pio run -e stm32_robot_v4_2` SUCCESS
- [ ] `idf.py build` SUCCESS

## B. MPU6050 riêng

- [ ] Nối 3.3V/GND, PC8=SCL, PB5=SDA, AD0=GND
- [ ] Build/nạp `stm32_imu_probe`
- [ ] Khi boot giữ robot đứng yên cho đến `READY=1,CAL=1,HEALTH=OK`
- [ ] Đặt robot nằm đúng: AZ xấp xỉ +1g theo hệ robot
- [ ] Xoay trái nhìn từ trên: Gyro Z dương; xoay phải: Gyro Z âm
- [ ] Đứng yên: Gyro Z gần 0 sau calibration

## C. Production STM32

- [ ] Nạp `stm32_robot_v4_2`
- [ ] `GET,IMU` OK
- [ ] `GET,FUSION` READY=1
- [ ] source bình thường là `I+E+C` khi cả 3 cảm biến khỏe
- [ ] tắt/ngắt IMU: fusion chuyển degraded nhưng robot không tự chạy
- [ ] encoder/LCD/SR04/Compass/PS2 vẫn hoạt động

## D. Heading

- [ ] quay tay +90/-90 độ, fused heading đúng chiều
- [ ] test TURN ±45°, ±90° kê bánh rồi mới trên sàn
- [ ] không có nhảy heading lớn khi motor khởi động

## E. Odometry

- [ ] xác nhận lại 6552 tick/rev trái, 7223 tick/rev phải hoặc cập nhật theo đo 10 vòng
- [ ] đi thẳng 500 mm, ghi sai số X/Y
- [ ] quay 90 độ rồi đi 500 mm, kiểm tra trục X/Y hợp lý
- [ ] MCP `get_odometry` thay đổi đúng, không còn position theo timer

## F. HOME / Return Home

- [ ] đặt robot ở điểm A, gọi `set_home`
- [ ] chạy đường ngắn 1–2 m, ít vật cản, tốc độ 10–12
- [ ] kiểm tra breadcrumb tăng
- [ ] gọi `return_home`, robot quay về từng waypoint bằng chạy tiến
- [ ] HOME tolerance thử nghiệm ~12 cm; đo sai số thực tế
- [ ] thử vật cản mới trên đường về: robot phải dừng/né, không ép đi xuyên
- [ ] chạm PS2 trong Return Home: PS2 phải preempt mission

## G. Chưa làm ở V4.2

- wheel-speed PID chưa tune;
- SLAM/global map chưa có;
- Return Home không đảm bảo centimet sau hành trình dài hoặc trượt bánh.
