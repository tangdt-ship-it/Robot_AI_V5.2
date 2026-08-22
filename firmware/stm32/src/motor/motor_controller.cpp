#include <motor/motor_controller.h>
#include <robot_config.h>

void MotorController::begin() {
  analogWriteResolution(8);
  pinMode(MOTOR_LEFT_DIR_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_PWM_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM_PIN, OUTPUT);
  // This is the first hardware action: inverted PWM=255 guarantees no motion.
  stop();
}

void MotorController::writeStopped(uint32_t dirPin, uint32_t pwmPin,
                                   uint8_t pwm, int16_t& speedState,
                                   uint8_t& pwmState) {
  digitalWrite(dirPin, LOW);
  analogWrite(pwmPin, pwm);
  speedState = 0;
  pwmState = pwm;
}

void MotorController::drive(int16_t speed, uint32_t dirPin, uint32_t pwmPin,
                            int16_t& speedState, uint8_t& pwmState) {
  speed = constrain(speed, -255, 255);
  if (speed == 0) {
    writeStopped(dirPin, pwmPin, PWM_STOP, speedState, pwmState);
    return;
  }
  digitalWrite(dirPin, speed > 0 ? HIGH : LOW);
  const uint8_t pwm = static_cast<uint8_t>(255 - abs(speed));
  analogWrite(pwmPin, pwm);
  speedState = speed;
  pwmState = pwm;
}

void MotorController::setSpeedLeft(int16_t speed) {
  drive(speed, MOTOR_LEFT_DIR_PIN, MOTOR_LEFT_PWM_PIN, leftSpeed_, leftPwm_);
}

void MotorController::setSpeedRight(int16_t speed) {
  drive(speed, MOTOR_RIGHT_DIR_PIN, MOTOR_RIGHT_PWM_PIN, rightSpeed_, rightPwm_);
}

void MotorController::setSpeeds(int16_t left, int16_t right) {
  brakeActive_ = false;
  setSpeedLeft(left);
  setSpeedRight(right);
}

void MotorController::stop() {
  writeStopped(MOTOR_LEFT_DIR_PIN, MOTOR_LEFT_PWM_PIN, PWM_STOP,
               leftSpeed_, leftPwm_);
  writeStopped(MOTOR_RIGHT_DIR_PIN, MOTOR_RIGHT_PWM_PIN, PWM_STOP,
               rightSpeed_, rightPwm_);
  brakeActive_ = false;
}

void MotorController::brake() {
  // Hardware-specific locked brake requested by the user: both direction
  // pins LOW and both inverted PWM channels held in the 251..254 range.
  static_assert(BRAKE_PWM_LOCK >= BRAKE_PWM_LOCK_MIN &&
                    BRAKE_PWM_LOCK <= BRAKE_PWM_LOCK_MAX,
                "Brake PWM must stay in the approved 251..254 range");
  writeStopped(MOTOR_LEFT_DIR_PIN, MOTOR_LEFT_PWM_PIN, BRAKE_PWM_LOCK,
               leftSpeed_, leftPwm_);
  writeStopped(MOTOR_RIGHT_DIR_PIN, MOTOR_RIGHT_PWM_PIN, BRAKE_PWM_LOCK,
               rightSpeed_, rightPwm_);
  brakeActive_ = true;
}

void MotorController::freeStop() { stop(); }

void MotorController::update() {}
