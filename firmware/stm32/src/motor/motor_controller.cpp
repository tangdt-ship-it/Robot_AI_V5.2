#include <motor/motor_controller.h>
#include <robot_config.h>

void MotorController::begin() {
  analogWriteResolution(8);
  pinMode(MOTOR_LEFT_DIR_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_PWM_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM_PIN, OUTPUT);
  leftPid_.begin(WHEEL_SPEED_PID_ENABLED, WHEEL_PID_LEFT_KP,
                 WHEEL_PID_LEFT_KI, WHEEL_PID_LEFT_KD,
                 WHEEL_PID_TARGET_MM_S_PER_COMMAND);
  rightPid_.begin(WHEEL_SPEED_PID_ENABLED, WHEEL_PID_RIGHT_KP,
                  WHEEL_PID_RIGHT_KI, WHEEL_PID_RIGHT_KD,
                  WHEEL_PID_TARGET_MM_S_PER_COMMAND);
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
  speedState = speed;
  driveOutput(speed, dirPin, pwmPin, pwmState);
}

void MotorController::driveOutput(int16_t speed, uint32_t dirPin,
                                  uint32_t pwmPin, uint8_t& pwmState) {
  speed = constrain(speed, -255, 255);
  if (speed == 0) {
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, PWM_STOP);
    pwmState = PWM_STOP;
    return;
  }
  digitalWrite(dirPin, speed > 0 ? HIGH : LOW);
  const uint8_t pwm = static_cast<uint8_t>(255 - abs(speed));
  analogWrite(pwmPin, pwm);
  pwmState = pwm;
}

void MotorController::applyRequestedOutputs() {
  leftAppliedCommand_ = leftSpeed_;
  rightAppliedCommand_ = rightSpeed_;
  driveOutput(leftAppliedCommand_, MOTOR_LEFT_DIR_PIN, MOTOR_LEFT_PWM_PIN,
              leftPwm_);
  driveOutput(rightAppliedCommand_, MOTOR_RIGHT_DIR_PIN,
              MOTOR_RIGHT_PWM_PIN, rightPwm_);
}

void MotorController::setSpeedLeft(int16_t speed) {
  leftSpeed_ = constrain(speed, -255, 255);
  leftPid_.reset();
  leftAppliedCommand_ = leftSpeed_;
  driveOutput(leftAppliedCommand_, MOTOR_LEFT_DIR_PIN, MOTOR_LEFT_PWM_PIN,
              leftPwm_);
}

void MotorController::setSpeedRight(int16_t speed) {
  rightSpeed_ = constrain(speed, -255, 255);
  rightPid_.reset();
  rightAppliedCommand_ = rightSpeed_;
  driveOutput(rightAppliedCommand_, MOTOR_RIGHT_DIR_PIN, MOTOR_RIGHT_PWM_PIN,
              rightPwm_);
}

void MotorController::setSpeeds(int16_t left, int16_t right) {
  brakeActive_ = false;
  leftSpeed_ = constrain(left, -255, 255);
  rightSpeed_ = constrain(right, -255, 255);
  applyRequestedOutputs();
}

void MotorController::stop() {
  writeStopped(MOTOR_LEFT_DIR_PIN, MOTOR_LEFT_PWM_PIN, PWM_STOP,
               leftSpeed_, leftPwm_);
  writeStopped(MOTOR_RIGHT_DIR_PIN, MOTOR_RIGHT_PWM_PIN, PWM_STOP,
               rightSpeed_, rightPwm_);
  leftAppliedCommand_ = 0;
  rightAppliedCommand_ = 0;
  leftPid_.reset();
  rightPid_.reset();
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
  leftAppliedCommand_ = 0;
  rightAppliedCommand_ = 0;
  leftPid_.reset();
  rightPid_.reset();
  brakeActive_ = true;
}

void MotorController::freeStop() { stop(); }

void MotorController::update() {}

void MotorController::updateSpeedPid(float leftVelocityMmS,
                                     float rightVelocityMmS) {
#if ROBOT_WHEEL_SPEED_PID_ENABLE
  if (brakeActive_ || (leftSpeed_ == 0 && rightSpeed_ == 0)) {
    if (leftSpeed_ == 0) leftPid_.reset();
    if (rightSpeed_ == 0) rightPid_.reset();
    applyRequestedOutputs();
    return;
  }
  const uint32_t nowMs = millis();
  leftAppliedCommand_ = leftPid_.update(leftSpeed_, leftVelocityMmS, nowMs);
  rightAppliedCommand_ =
      rightPid_.update(rightSpeed_, rightVelocityMmS, nowMs);
  driveOutput(leftAppliedCommand_, MOTOR_LEFT_DIR_PIN, MOTOR_LEFT_PWM_PIN,
              leftPwm_);
  driveOutput(rightAppliedCommand_, MOTOR_RIGHT_DIR_PIN,
              MOTOR_RIGHT_PWM_PIN, rightPwm_);
#else
  (void)leftVelocityMmS;
  (void)rightVelocityMmS;
#endif
}
