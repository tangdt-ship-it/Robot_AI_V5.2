# RobotLink V3 — Robot_AI profile

Physical link: UART 115200 8-N-1, common GND, ESP32 RX GPIO14 <- STM32 PC10, ESP32 TX GPIO38 -> STM32 PC11.

Command ESP32->STM32 dùng sequence + CRC16:

```text
$RAI,3,SEQ,TYPE,PAYLOAD*CRC16\r\n
```

State-changing command bắt buộc CRC. Response/event STM32->ESP32 dùng frame human-readable.

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

## Alpha.3 motion correlation

Finite `MOVE` and `TURN` requests emitted by the V5 replay path carry a non-zero session/operation pair:

```text
MOVE,FWD,500,20,SID,7,OP,31
<ACK,MOVE,FWD,500,MM,20,SID,7,OP,31>
<PROGRESS,MOVE,...,SID,7,OP,31>
<DONE,MOVE,TARGET,500,TRAVEL,499,SID,7,OP,31>

TURN,REL,LEFT,90,20,SID,7,OP,32
<ACK,TURN,REL,LEFT,90,SID,7,OP,32>
<PROGRESS,TURN,...,SID,7,OP,32>
<DONE,TURN,H,...,TGT,...,ERR,...,SID,7,OP,32>
```

The ESP32 accepts an ACK, NACK, progress or terminal frame for an active finite operation only when both `SID` and `OP` are non-zero and exactly equal to the pending pair. IDs are never ordered or compared by age. Missing, zero, old or mismatched IDs are logged as stale and cannot complete a replay step. Duplicate terminal frames are idempotent.

`HELLO`/`PING`, an STM32 boot indication, link loss, STOP, cancellation or a new session invalidates the pending pair. A new manual replay request must pass preflight again; there is no automatic resumption. Legacy frames remain parseable for compatibility, but are deliberately uncorrelated and therefore fail closed for strict Full Replay.
