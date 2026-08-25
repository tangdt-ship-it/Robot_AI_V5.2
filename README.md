# Robot_AI_V5.0

Robot_AI_V5.0 là nhánh phát triển kế thừa firmware Robot_AI_V4.2/V4.3 cho robot vi sai hai bánh dùng STM32F103VET6 và ESP32-S3 N16R8/CAM.

## Trạng thái

- Phiên bản: `5.0.0-alpha.0`
- Nhánh: `develop/robot-ai-v5.0`
- Điểm kế thừa: `ea85c29904984c822dab0b53e152ab7e88ed0b81`
- Trạng thái phát hành: DEVELOPMENT / NOT PRODUCTION

V5.0 giữ nguyên phần cứng, pinout và các chức năng V4.x đã được commissioning. Nhánh này sẽ sửa các đường điều khiển chưa đủ an toàn trước khi mở Full Replay và tự né vật cản.

Tài liệu phạm vi, kiến trúc, giai đoạn phát triển và cổng kiểm thử: [docs/ROBOT_AI_V5_0_PROJECT.md](docs/ROBOT_AI_V5_0_PROJECT.md).

## Thứ tự phát triển bắt buộc

1. STOP độc lập và sensor health.
2. AI cooldown và vòng đời obstacle event.
3. HOLD/RESUME theo quãng đường tích lũy.
4. Xác minh robot-left/robot-right bằng thử nghiệm vật lý.
5. Full Replay toàn tuyến.
6. Tự né có giới hạn và quay lại tuyến.
7. Tối ưu chuyển động, nhiều route, mission giọng nói, preflight và black-box log.

Full Replay production và automatic detour bị khóa mặc định cho đến khi các cổng an toàn tương ứng đạt trên robot thật.

## Feature flags alpha

Automatic detour is explicitly disabled by default. Autonomous navigation
stops at an obstacle and reports `automatic_detour_disabled` until physical
commissioning enables `CONFIG_AUTOMATIC_DETOUR=y`.

## Alpha.2 replay boundary

`IMPLEMENTED_AND_HOST_TESTED`: Full Replay uses a unified preflight gate for
RobotLink/session, STM32 motion owner, client motion lease, per-channel sensor
health/freshness, encoder health, odometry/reset generation, and fused-heading
confidence before a TURN. Basic replay does not require camera; its absence is
reported as `WARN` rather than bypassing any motion safety gate.

`DISABLED_BY_DEFAULT`: `SHORT_SAFETY_TEST` remains the default and ends with
`TEST_COMPLETE`. `FULL_PRODUCTION` remains compiled but gated off. Automatic
detour, automatic reverse, and automatic resume after AI remain disabled.

`IMPLEMENTED_HARDWARE_VALIDATION_REQUIRED`: preflight classification and replay
terminal states are host-tested only. STOP latency, physical sensor mapping,
encoder/heading calibration and commissioning of Full Production still require
the real robot.

- `MANUAL_PS2`, Teach Route, Replay dry-run và `SHORT_SAFETY_TEST`: enabled.
- `ROBOT_V5_FULL_REPLAY_PRODUCTION`: `0` mặc định; chỉ bật sau HIL commissioning.
- Automatic detour, automatic reverse và automatic resume sau AI: disabled.
- Physical left/right sensor wiring vẫn cần xác minh; phần mềm dùng một mapping robot-frame canonical và fail-closed khi health chưa xác nhận.

## Kiến trúc giữ nguyên

```text
Xiaozhi / Voice / Camera / Mission / Teach / Replay
                         ESP32-S3
                            |
                   RobotLink UART 115200
                            |
                            v
                       STM32F103VE
        +-------------------+-------------------+
        |                   |                   |
     Encoder             MPU6050             Compass
        |                   |                   |
        +-----------> Heading Fusion <-----------+
                            |
                 Odometry + Motion Control
                            |
             Safety / PS2 / Front HC-SR04
                            |
                          Motors
```

STM32 luôn là motor/safety authority. Camera, cloud và AI chỉ được đề xuất hành vi; chúng không được vượt qua sensor veto hoặc STOP.

## Build STM32

Tên PlatformIO environment được giữ nguyên để không làm hỏng quy trình V4.x:

```powershell
cd firmware\stm32
pio run -e stm32_robot_v4_2
pio run -e stm32_robot_v4_2 -t upload
```

Kiểm tra MPU6050:

```powershell
pio run -e stm32_imu_probe
pio run -e stm32_imu_probe -t upload
```

Debug UART3: `PB10 TX`, baud `115200`.

## Build ESP32-S3

```powershell
cd firmware\esp32-xiaozhi
idf.py fullclean
idf.py build
idf.py -p COMx flash monitor
```

Target: ESP32-S3, board `bread-compact-wifi-s3cam`, locale `vi-VN`, flash 16 MB.

## Kiểm thử host kế thừa

```powershell
python tools\v4_protocol_selftest.py
python tools\v4_2_localization_selftest.py
python tools\v4_2_static_audit.py
python tools\v5_host_selftest.py
python tools\v5_static_audit.py
```

`v5_static_audit.py` kiểm tra phiên bản alpha V5.0 và toàn bộ guardrail kiến trúc kế thừa. Các audit V4.x được giữ lại để kiểm tra checkpoint cũ; lỗi phiên bản của audit V4.x không được coi là lỗi thuật toán robot trên nhánh V5.0.

## Cảnh báo vận hành

- Đây chưa phải firmware V5.0 production.
- Không bật Full Replay bằng cách chỉ đổi cờ giới hạn 150 mm.
- Không bật AI auto-drive hoặc auto-resume trước khi Stage 1–6 đạt.
- Voice STOP không thay thế nút STOP/PS2 vật lý.
- Hai cảm biến trước không bảo vệ phía sau, hai bên hoặc cạnh vực.
- HOME và Replay vẫn dựa trên odometry/heading fusion, chưa phải SLAM định vị tuyệt đối.
