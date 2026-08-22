# Robot_AI_V4.2

Robot_AI_V4.2 là bản nâng cấp ổn định hóa chuyển động/định vị cho robot vi sai hai bánh dùng **STM32F103VET6 + ESP32-S3 N16R8/CAM + Xiaozhi**.

## Điểm mới chính so với V4.1

- tích hợp **MPU6050** vào STM32 qua Soft-I2C riêng `PC8=SCL`, `PB5=SDA`, địa chỉ `0x68`;
- hỗ trợ đúng cách lắp IMU **mặt linh kiện úp xuống, connector về sau, 2 lỗ bắt vít về trước**;
- tự hiệu chuẩn gyro khi boot, có `stm32_imu_probe`;
- heading fusion: **Encoder + Gyro Z + Compass**, có degraded mode;
- odometry STM32 dùng fused heading để tích phân `X/Y/theta`;
- ESP32 MissionManager bỏ ước lượng vị trí bằng thời gian, dùng `GET,ODOMETRY` thực từ STM32;
- breadcrumb + HOME + mission **RETURN_HOME**;
- thêm MCP: `get_odometry`, `get_imu_status`, `get_fusion_status`, `set_home`, `get_home`, `return_home`;
- RobotLink V3 giữ CRC/sequence cho command và bổ sung query `GET,IMU`, `GET,FUSION`;
- giữ nguyên nguyên tắc: **STM32 là motor/safety authority**, PS2 và HC-SR04 có quyền cao hơn AI.

## Kiến trúc

```text
Xiaozhi / Voice / Camera / MissionManager / Return Home
                         ESP32-S3
                            |
                   RobotLink V3 UART 115200
                            |
                            v
                       STM32F103VE
        +-------------------+-------------------+
        |                   |                   |
     Encoder             MPU6050             Compass
        |                   |                   |
        +-----------> Heading Fusion <-----------+
                            |
                       Fused heading
                            |
                 Odometry X/Y/theta + Motion
                            |
                    Safety / PS2 / SR04
                            |
                          Motors
```

## Build STM32

```powershell
cd firmware\stm32
pio run -e stm32_robot_v4_2
pio run -e stm32_robot_v4_2 -t upload
```

Kiểm tra MPU6050 riêng trước khi production:

```powershell
pio run -e stm32_imu_probe
pio run -e stm32_imu_probe -t upload
```

Debug UART3: `PB10 TX`, baud `115200`.

## Build ESP32-S3

Mở ESP-IDF terminal:

```powershell
cd firmware\esp32-xiaozhi
idf.py fullclean
idf.py build
idf.py -p COMx flash monitor
```

Target cố định: ESP32-S3, board `bread-compact-wifi-s3cam`, `vi-VN`, partition 16 MB.

## Host self-test

```powershell
python tools\v4_protocol_selftest.py
python tools\v4_2_localization_selftest.py
python tools\v4_2_static_audit.py
```

## Quan trọng trước khi chạy Return Home

Return Home của V4.2 là **odometry/breadcrumb**, chưa phải SLAM. Phải hiệu chuẩn encoder và kiểm tra heading fusion trước. Sai số bánh xe/trượt nền vẫn tích lũy theo quãng đường. Xem `docs/V4_2_VALIDATION_CHECKLIST.md`.
