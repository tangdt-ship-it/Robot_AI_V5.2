#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>

class MotorController {
 public:
  void begin();
  void setSpeedLeft(int16_t speed);
  void setSpeedRight(int16_t speed);
  void setSpeeds(int16_t left, int16_t right);
  void stop();
  void brake();
  void freeStop();
  void update();

  int16_t leftSpeed() const { return leftSpeed_; }
  int16_t rightSpeed() const { return rightSpeed_; }
  uint8_t leftPwm() const { return leftPwm_; }
  uint8_t rightPwm() const { return rightPwm_; }
  bool isBraking() const { return brakeActive_; }

 private:
  static void drive(int16_t speed, uint32_t dirPin, uint32_t pwmPin,
                    int16_t& speedState, uint8_t& pwmState);
  static void writeStopped(uint32_t dirPin, uint32_t pwmPin, uint8_t pwm,
                           int16_t& speedState, uint8_t& pwmState);

  int16_t leftSpeed_ = 0;
  int16_t rightSpeed_ = 0;
  uint8_t leftPwm_ = 255;
  uint8_t rightPwm_ = 255;
  bool brakeActive_ = false;
};

#endif
