#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <Arduino.h>

// Existing robot wiring. Do not change these pins without changing hardware.
static constexpr uint32_t MOTOR_LEFT_PWM_PIN = PA8;
static constexpr uint32_t MOTOR_LEFT_DIR_PIN = PA12;
static constexpr uint32_t MOTOR_RIGHT_PWM_PIN = PA9;
static constexpr uint32_t MOTOR_RIGHT_DIR_PIN = PC0;

// Quadrature wheel encoders. TIM2 CH1/CH2 is PA0/PA1 and TIM3 CH1/CH2 is
// PA6/PA7. Set either reversed flag true only if forward wheel rotation makes
// its reported tick count decrease in the encoder diagnostic.
static constexpr uint32_t ENCODER_LEFT_A_PIN = PA1;
static constexpr uint32_t ENCODER_LEFT_B_PIN = PA0;
static constexpr uint32_t ENCODER_RIGHT_A_PIN = PA7;
static constexpr uint32_t ENCODER_RIGHT_B_PIN = PA6;
static constexpr bool ENCODER_LEFT_REVERSED = false;
static constexpr bool ENCODER_RIGHT_REVERSED = false;
static constexpr float WHEEL_LEFT_CIRCUMFERENCE_MM = 390.0f;
static constexpr float WHEEL_RIGHT_CIRCUMFERENCE_MM = 390.0f;
// Raised-wheel one-revolution validation: left=6552, right=7223 ticks.
static constexpr float ENCODER_LEFT_TICKS_PER_REV = 6550.0f;
static constexpr float ENCODER_RIGHT_TICKS_PER_REV = 7179.3f;
static constexpr float ENCODER_LEFT_MM_PER_TICK =
	WHEEL_LEFT_CIRCUMFERENCE_MM / ENCODER_LEFT_TICKS_PER_REV;
static constexpr float ENCODER_RIGHT_MM_PER_TICK =
	WHEEL_RIGHT_CIRCUMFERENCE_MM / ENCODER_RIGHT_TICKS_PER_REV;
static constexpr float WHEEL_TRACK_MM = 249.0f;
// Encoder counters are a production dependency in V4. The PlatformIO production
// environment enables them; diagnostic builds may override this to 0.
#ifndef ROBOT_ENCODER_ENABLE
#define ROBOT_ENCODER_ENABLE 0
#endif
static constexpr bool ENCODER_ENABLED = ROBOT_ENCODER_ENABLE != 0;
static constexpr float ENCODER_VELOCITY_FILTER_ALPHA = 0.35f;
static constexpr int16_t ENCODER_STALL_COMMAND_THRESHOLD = 10;
static constexpr uint32_t ENCODER_STALL_TIMEOUT_MS = 900;
static constexpr int32_t ENCODER_STALL_MIN_TICKS = 1;

// Wheel-speed PID candidate. It is deliberately disabled in the stable
// production profile until PWM -> mm/s is measured on the assembled chassis.
// The candidate profile enables it with conservative starting values; all
// safety/PS2/obstacle decisions remain outside this loop.
#ifndef ROBOT_WHEEL_SPEED_PID_ENABLE
#define ROBOT_WHEEL_SPEED_PID_ENABLE 0
#endif
static constexpr bool WHEEL_SPEED_PID_ENABLED =
    ROBOT_WHEEL_SPEED_PID_ENABLE != 0;
static constexpr float WHEEL_PID_TARGET_MM_S_PER_COMMAND = 2.0f;
static constexpr float WHEEL_PID_LEFT_KP = 0.20f;
static constexpr float WHEEL_PID_LEFT_KI = 0.05f;
static constexpr float WHEEL_PID_LEFT_KD = 0.0f;
static constexpr float WHEEL_PID_RIGHT_KP = 0.20f;
static constexpr float WHEEL_PID_RIGHT_KI = 0.05f;
static constexpr float WHEEL_PID_RIGHT_KD = 0.0f;

// Front HC-SR04. PC9/PC12 are used as GPIO so the measurement does not take
// ownership of a hardware timer already used by PWM/Arduino timing.
static constexpr uint32_t ULTRASONIC_TRIG_PIN = PC12;
static constexpr uint32_t ULTRASONIC_ECHO_PIN = PC9;
// Right-mounted module observes the FRONT-LEFT sector.
static constexpr uint32_t ULTRASONIC_RIGHT_TRIG_PIN = PC4;
static constexpr uint32_t ULTRASONIC_RIGHT_ECHO_PIN = PC7;

static constexpr uint32_t LCD_SCL_PIN = PB7;
static constexpr uint32_t LCD_SDA_PIN = PB6;
static constexpr uint8_t LCD_ADDRESS = 0x27;

// MPU6050 IMU on the former PWM7 connector. This is an independent software
// I2C bus so a sensor fault cannot lock the LCD bus. The physical mounting is
// face-down, connector toward the rear and mounting holes toward the front.
static constexpr uint32_t MPU6050_SCL_PIN = PC8;
static constexpr uint32_t MPU6050_SDA_PIN = PB5;
static constexpr uint8_t MPU6050_ADDRESS = 0x68;
static constexpr uint32_t IMU_SAMPLE_PERIOD_MS = 10;  // 100 Hz
static constexpr uint32_t IMU_LOST_TIMEOUT_MS = 250;
static constexpr uint32_t IMU_CALIBRATION_SETTLE_MS = 600;
static constexpr uint16_t IMU_CALIBRATION_SAMPLES = 350;
static constexpr float IMU_CALIBRATION_ACCEL_TOLERANCE_G = 0.12f;
static constexpr float IMU_CALIBRATION_MAX_RATE_DPS = 4.0f;
static constexpr uint8_t IMU_READ_FAULT_COUNT = 5;
static constexpr uint32_t IMU_I2C_HALF_PERIOD_US = 4;
static constexpr uint32_t IMU_I2C_CLOCK_STRETCH_TIMEOUT_US = 500;

// Heading fusion. Short-term yaw comes from MPU6050 Gyro-Z and wheel encoder
// yaw constrains the estimate to chassis kinematics. Large jumps are never
// injected directly.
static constexpr float FUSION_ENCODER_DELTA_WEIGHT = 0.10f;
// Wheel slip must not turn encoder yaw into an absolute heading source.
static constexpr float FUSION_ENCODER_AGREEMENT_DEG = 1.5f;
static constexpr float FUSION_ENCODER_REJECT_DEG = 12.0f;
static constexpr float FUSION_YAW_RATE_FILTER = 0.30f;
static constexpr float FUSION_MIN_DT_S = 0.003f;
static constexpr float FUSION_MAX_DT_S = 0.100f;

static constexpr uint32_t PS2_CLK_PIN = PB14;
static constexpr uint32_t PS2_ATT_PIN = PB13;
static constexpr uint32_t PS2_CMD_PIN = PB12;
static constexpr uint32_t PS2_DAT_PIN = PB9;

// Optional diagnostic UART3. Connect PB10 (TX) to USB-UART RX and common GND.
// PB11 is reserved as RX so HardwareSerial can use the USART3 pin map.
static constexpr uint32_t ROBOT_DEBUG_RX_PIN = PB11;
static constexpr uint32_t ROBOT_DEBUG_TX_PIN = PB10;
static constexpr uint32_t ROBOT_DEBUG_BAUD = 115200;

// ESP32-S3 AI coprocessor link. PC10 is UART4_TX and PC11 is UART4_RX.
// Wiring must cross TX/RX: PC10 -> ESP RX, PC11 <- ESP TX, common 0V.
static constexpr uint32_t ROBOT_LINK_RX_STM32 = PC11;
static constexpr uint32_t ROBOT_LINK_TX_STM32 = PC10;
static constexpr uint32_t ROBOT_LINK_BAUD = 115200;
static constexpr uint32_t ROBOT_LINK_STATUS_MS = 250;
static constexpr uint32_t ROBOT_LINK_TIMEOUT_MS = 1500;
// V4 separates a long-lived AI control session from the short-lived motion
// lease. Losing heartbeat stops motion within 700 ms, while an idle AI session
// may remain armed for 30 s without moving the chassis.
static constexpr uint32_t ROBOT_AI_SESSION_TIMEOUT_MS = 30000;
static constexpr uint32_t ROBOT_MOTION_LEASE_TIMEOUT_MS = 700;
static constexpr uint32_t ROBOT_AI_MOTION_PULSE_MS = 650;
static constexpr int16_t ROBOT_AI_SPEED_MAX = 20;
static constexpr int16_t ROBOT_AI_SPEED_MIN = 10;
static constexpr uint32_t ROBOT_AI_DISTANCE_MAX_MM = 5000;
static constexpr uint32_t ROBOT_AI_DISTANCE_TIMEOUT_MS = 30000;
static constexpr uint32_t ROBOT_AI_DISTANCE_PROGRESS_MS = 250;
static constexpr float DISTANCE_WHEEL_BALANCE_GAIN = 0.0f;
static constexpr int16_t DISTANCE_WHEEL_BALANCE_MAX = 4;
static constexpr uint8_t ROBOT_AI_PROTOCOL_VERSION = 3;
static constexpr uint32_t ROBOT_AI_HEARTBEAT_MS = 200;

// Heading closed-loop turns. The controller estimates yaw rate, predicts the
// mechanical coast angle and uses short correction pulses close to target.
static constexpr int16_t TURN_MIN_SPEED = 10;
static constexpr int16_t TURN_MAX_SPEED = 20;
static constexpr float TURN_SLOW_ZONE_DEG = 25.0f;
static constexpr float TURN_PULSE_ZONE_DEG = 8.0f;
// Angle-command turns are required to settle within half a degree. The
// controller therefore uses short correction pulses close to target instead
// of declaring DONE in the old 3-degree window.
static constexpr float TURN_TOLERANCE_DEG = 0.5f;
static constexpr float TURN_RATE_FILTER = 0.30f;
static constexpr float TURN_PREDICT_TIME_S = 0.18f;
static constexpr float TURN_SETTLE_RATE_DEG_S = 1.0f;
static constexpr float TURN_CORRECTION_START_RATE_DEG_S = 2.5f;
// Speed 10 was not sufficient to overcome static friction on the tested
// chassis. Keep the final correction short, but give it the same torque as
// the normal angle command.
static constexpr int16_t TURN_CORRECTION_SPEED = 15;
static constexpr uint32_t TURN_CORRECTION_PULSE_NEAR_MS = 25;
static constexpr uint32_t TURN_CORRECTION_PULSE_MID_MS = 50;
static constexpr uint32_t TURN_CORRECTION_PULSE_FAR_MS = 100;
static constexpr uint32_t TURN_CORRECTION_COAST_MS = 80;
static constexpr uint32_t TURN_OVERSHOOT_COAST_MS = 120;
static constexpr uint32_t TURN_SETTLE_MS = 250;
static constexpr uint32_t TURN_TIMEOUT_MS = 12000;
static constexpr uint32_t TURN_PROGRESS_MS = 250;
static constexpr int16_t TURN_MAX_RELATIVE_DEG = 180;

static constexpr int16_t SPEED_DEFAULT = 30;
static constexpr int16_t SPEED_MIN = 10;
static constexpr int16_t SPEED_MAX = 255;
static constexpr int16_t REVERSE_SPEED_MAX = 100;

static constexpr uint8_t PWM_STOP = 255;
// Active counter-torque brake. The old PWM 251 was only a 4/255 reverse
// command, so it was practically indistinguishable from coasting. Brake
// strength scales with the last drive command and is capped for safety.
static constexpr uint8_t BRAKE_PWM_LOCK = 251;
static constexpr uint8_t BRAKE_PWM_LOCK_MIN = 251;
static constexpr uint8_t BRAKE_PWM_LOCK_MAX = 254;

// HC-SR04 timing and obstacle policy. A speed-dependent stopping distance is
// added by the sensor policy, while these values define the physical floor.
static constexpr uint32_t ULTRASONIC_SAMPLE_PERIOD_MS = 60;
// Allow acoustic ringing and late echoes from one HC-SR04 to decay before
// triggering the other module. This prevents cross-talk without blocking the
// main loop because Echo capture is interrupt-driven.
static constexpr uint32_t ULTRASONIC_INTER_SENSOR_GUARD_MS = 30;
static constexpr uint32_t ULTRASONIC_ECHO_TIMEOUT_US = 30000;
static constexpr uint32_t ULTRASONIC_FRESH_MS = 350;
// A single HC-SR04 timeout can be caused by an echo collision or a narrow
// electrical glitch.  Keep the fail-safe stop, but allow only a very short,
// slow continuation when both channels recently proved a wide clear path.
static constexpr uint32_t ULTRASONIC_DEGRADED_GRACE_MS = 300;
static constexpr uint32_t ULTRASONIC_DEGRADED_MAX_TIMEOUTS = 2;
static constexpr float ULTRASONIC_DEGRADED_CLEAR_CM = 50.0f;
static constexpr int16_t ULTRASONIC_DEGRADED_MAX_FORWARD_COMMAND = 8;
static constexpr float ULTRASONIC_MIN_CM = 2.0f;
static constexpr float ULTRASONIC_MAX_CM = 400.0f;
// Tuned for the 35 x 40 cm chassis. EMERGENCY is intentionally below 10 cm as
// requested, while BLOCKED/dynamic stop remain earlier so the front corners
// still have room to sweep during an in-place avoidance turn.
static constexpr float OBSTACLE_EMERGENCY_CM = 9.0f;
static constexpr float OBSTACLE_BLOCKED_CM = 16.0f;
static constexpr float OBSTACLE_CAUTION_CM = 30.0f;
static constexpr float OBSTACLE_HYSTERESIS_CM = 4.0f;
static constexpr float OBSTACLE_STOP_BASE_CM = 12.0f;
static constexpr float OBSTACLE_STOP_PER_COMMAND_CM = 0.10f;
static constexpr float OBSTACLE_APPROACH_LOOKAHEAD_S = 0.15f;
static constexpr float OBSTACLE_SLOW_BAND_CM = 18.0f;
static constexpr int16_t OBSTACLE_MIN_FORWARD_COMMAND = 8;
static constexpr float AVOID_MIN_CLEARANCE_CM = 30.0f;
static constexpr float AVOID_SIDE_HYSTERESIS_CM = 8.0f;

static constexpr uint32_t PS2_POLL_MS = 2;
static constexpr uint32_t PS2_RECONNECT_MS = 1000;
static constexpr uint32_t PS2_FAILSAFE_MS = 80;
static constexpr uint32_t PS2_ACTIVITY_MS = 1200;
static constexpr int16_t JOYSTICK_DEADZONE_RAW = 11;
static constexpr int16_t JOYSTICK_MOTION_DEADZONE_RAW = 13;

static constexpr uint32_t CONTROL_PERIOD_MS = 5;

// Independent watchdog. Production enables it through platformio.ini; hardware
// diagnostic environments may disable it to simplify single-step debugging.
#ifndef ROBOT_IWDG_ENABLE
#define ROBOT_IWDG_ENABLE 0
#endif
static constexpr bool IWDG_ENABLED = ROBOT_IWDG_ENABLE != 0;
static constexpr uint32_t IWDG_TIMEOUT_MS = 2500;
static constexpr uint32_t LCD_RENDER_MS = 100;

// Cascaded heading controller. The outer loop requests a deliberately slow
// yaw rate; the inner loop damps measured yaw. This prevents the long S-curve
// oscillation seen when a large wheel differential was held until zero-cross.
static constexpr float HEADING_ANGLE_TO_RATE_GAIN = 0.40f;
static constexpr float HEADING_INTEGRAL_GAIN = 0.08f;
static constexpr float HEADING_YAW_RATE_GAIN = 1.10f;
static constexpr float HEADING_MAX_DESIRED_YAW_RATE_DEG_S = 3.5f;
static constexpr float HEADING_LOCK_ENTER_DEG = 0.25f;
static constexpr float HEADING_LOCK_EXIT_DEG = 0.50f;
static constexpr float HEADING_BRAKE_WINDOW_DEG = 3.0f;
static constexpr float HEADING_INTEGRAL_LIMIT = 70.0f;
static constexpr float HEADING_YAW_RATE_FILTER = 0.35f;
static constexpr int16_t HEADING_SPEED_THRESHOLD = 30;
static constexpr float HEADING_MAX_CORRECTION_RATIO = 0.35f;
static constexpr float HEADING_MIN_CORRECTION_LIMIT = 2.0f;
static constexpr float HEADING_CORRECTION_SLEW_PER_SECOND = 10.0f;
static constexpr float HEADING_YAW_RATE_STOP_DEG_S = 0.35f;
static constexpr float HEADING_MIN_DT_S = 0.005f;
static constexpr float HEADING_MAX_DT_S = 0.100f;
// Detect a poisoned heading signal by checking closed-loop causality. On this
// chassis positive correction must produce negative yaw (and vice versa). If
// a meaningful correction sees sustained yaw in the wrong direction, stop
// using the faulty heading feedback for the remainder of that straight-drive
// session.
static constexpr float HEADING_RESPONSE_TEST_OUTPUT = 2.0f;
static constexpr float HEADING_RESPONSE_TEST_RATE_DEG_S = 0.45f;
static constexpr float HEADING_RESPONSE_FAULT_TIME_S = 1.10f;
static constexpr float HEADING_RESPONSE_RECOVERY_MULTIPLIER = 2.5f;

static constexpr int16_t RAMP_STEP = 1;
static constexpr int16_t RAMP_START = 5;
static constexpr int16_t SPEED_ADJUST_STEP = 20;

#endif
