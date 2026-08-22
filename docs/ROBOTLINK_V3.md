# RobotLink V3 — V4.2 profile

Physical link: UART 115200 8-N-1, common GND, ESP32 RX GPIO14 <- STM32 PC10, ESP32 TX GPIO38 -> STM32 PC11.

Command ESP32->STM32 dùng sequence + CRC16:

```text
$RAI,3,SEQ,TYPE,PAYLOAD*CRC16\r\n
```

State-changing command bắt buộc CRC. Response/event STM32->ESP32 vẫn dùng `<...>` human-readable.

## Localization query

```text
GET,ODOMETRY
<VALUE,ODOMETRY,DIST,...,X,...,Y,...,H,...,LT,...,RT,...>

GET,ENCODER
<VALUE,ENCODER,READY,1,HEALTH,OK,LV,...,RV,...>

GET,IMU
<VALUE,IMU,READY,1,CAL,1,HEALTH,OK,GZ,...,AX,...,AY,...,AZ,...>

GET,FUSION
<VALUE,FUSION,READY,1,HEALTH,FUSED,H,...,RATE,...,CONF,...,SRC,I+E+C>

GET,HEADING
<VALUE,HEADING,...>
```

`GET,HEADING` trả fused heading. `GET,COMPASS_STATUS` vẫn là trạng thái riêng của Compass.

Motion heartbeat: ~200 ms; motion lease timeout 700 ms; AI session timeout 30 s.
