# Robot_AI_V4.2 — Hardware pinout

## STM32F103VET6

| Chức năng | Pin | Ghi chú |
|---|---|---|
| Motor trái PWM / DIR | PA8 / PA12 | PWM đảo, 255=stop |
| Motor phải PWM / DIR | PA9 / PC0 | PWM đảo, 255=stop |
| Encoder trái A/B | PA1 / PA0 | TIM2 |
| Encoder phải A/B | PA7 / PA6 | TIM3 |
| HC-SR04 TRIG / ECHO | PC12 / PC9 | ECHO phải an toàn 3.3 V |
| Compass RX / TX | PA3 / PA2 | UART 115200 |
| PS2 CLK/ATT/CMD/DAT | PB14/PB13/PB12/PB9 | manual override |
| LCD SCL / SDA | PB7 / PB6 | Soft-I2C, 0x27 |
| **MPU6050 SCL / SDA** | **PC8 / PB5** | **Soft-I2C riêng, 0x68** |
| Debug UART3 TX/RX | PB10/PB11 | 115200 |
| RobotLink UART4 TX/RX | PC10/PC11 | ESP32 |

## MPU6050

- VCC -> 3.3 V
- GND -> GND
- SCL -> PC8
- SDA -> PB5
- AD0 -> GND (`0x68`)
- INT/XDA/XCL: chưa dùng

Cách lắp đã chốt: **mặt có linh kiện úp xuống**, hàng chân hướng về sau robot, hai lỗ bắt vít hướng về đầu robot. Firmware biến đổi trục sang hệ robot `+X forward, +Y left, +Z up`: X giữ dấu, Y/Z đảo dấu; Gyro Z cũng đảo dấu.
