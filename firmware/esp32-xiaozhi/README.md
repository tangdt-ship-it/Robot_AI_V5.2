# ESP32-S3 Xiaozhi — Robot_AI_V4.2

Firmware ESP32 production duy nhất của Robot_AI_V4.2, tinh giản từ Xiaozhi cho board `bread-compact-wifi-s3cam`.

V4.2 bổ sung:

- RobotLink query odometry/IMU/fusion;
- MissionManager dùng odometry STM32 thay time-dead-reckoning;
- HOME + breadcrumb + Return Home;
- MCP chẩn đoán localization và điều khiển Return Home.

Build bằng ESP-IDF 5.5.x:

```powershell
idf.py fullclean
idf.py build
idf.py -p COMx flash monitor
```
