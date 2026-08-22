#include <imu/mpu6050.h>
#include <robot_config.h>

#include <math.h>

namespace {
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr float kAccelLsbPerG = 16384.0f;  // +/-2 g
constexpr float kGyroLsbPerDps = 131.0f;   // +/-250 deg/s
}

void Mpu6050::SoftI2C::lowScl() {
  pinMode(sclPin_, OUTPUT);
  digitalWrite(sclPin_, LOW);
}
void Mpu6050::SoftI2C::lowSda() {
  pinMode(sdaPin_, OUTPUT);
  digitalWrite(sdaPin_, LOW);
}
void Mpu6050::SoftI2C::releaseScl() { pinMode(sclPin_, INPUT); }
void Mpu6050::SoftI2C::releaseSda() { pinMode(sdaPin_, INPUT); }

bool Mpu6050::SoftI2C::waitSclHigh() {
  const uint32_t started = micros();
  while (digitalRead(sclPin_) == LOW) {
    if ((micros() - started) > IMU_I2C_CLOCK_STRETCH_TIMEOUT_US) return false;
  }
  return true;
}

void Mpu6050::SoftI2C::begin() {
  releaseSda();
  releaseScl();
  delay(2);
  // Bus recovery for a sensor reset in the middle of a byte.
  for (uint8_t i = 0; i < 9 && digitalRead(sdaPin_) == LOW; ++i) {
    lowScl();
    delayMicroseconds(IMU_I2C_HALF_PERIOD_US);
    releaseScl();
    (void)waitSclHigh();
    delayMicroseconds(IMU_I2C_HALF_PERIOD_US);
  }
  stop();
}

void Mpu6050::SoftI2C::start() {
  releaseSda();
  releaseScl();
  (void)waitSclHigh();
  delayMicroseconds(IMU_I2C_HALF_PERIOD_US);
  lowSda();
  delayMicroseconds(IMU_I2C_HALF_PERIOD_US);
  lowScl();
}

void Mpu6050::SoftI2C::stop() {
  lowSda();
  delayMicroseconds(IMU_I2C_HALF_PERIOD_US);
  releaseScl();
  (void)waitSclHigh();
  delayMicroseconds(IMU_I2C_HALF_PERIOD_US);
  releaseSda();
  delayMicroseconds(IMU_I2C_HALF_PERIOD_US);
}

bool Mpu6050::SoftI2C::writeByte(uint8_t value) {
  for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
    if ((value & mask) != 0) releaseSda(); else lowSda();
    delayMicroseconds(IMU_I2C_HALF_PERIOD_US);
    releaseScl();
    if (!waitSclHigh()) { lowScl(); return false; }
    delayMicroseconds(IMU_I2C_HALF_PERIOD_US);
    lowScl();
  }
  releaseSda();
  delayMicroseconds(IMU_I2C_HALF_PERIOD_US);
  releaseScl();
  if (!waitSclHigh()) { lowScl(); return false; }
  delayMicroseconds(IMU_I2C_HALF_PERIOD_US);
  const bool ack = digitalRead(sdaPin_) == LOW;
  lowScl();
  return ack;
}

uint8_t Mpu6050::SoftI2C::readByte(bool ack) {
  uint8_t value = 0;
  releaseSda();
  for (uint8_t i = 0; i < 8; ++i) {
    value <<= 1;
    delayMicroseconds(IMU_I2C_HALF_PERIOD_US);
    releaseScl();
    (void)waitSclHigh();
    if (digitalRead(sdaPin_) != LOW) value |= 1U;
    delayMicroseconds(IMU_I2C_HALF_PERIOD_US);
    lowScl();
  }
  if (ack) lowSda(); else releaseSda();
  delayMicroseconds(IMU_I2C_HALF_PERIOD_US);
  releaseScl();
  (void)waitSclHigh();
  delayMicroseconds(IMU_I2C_HALF_PERIOD_US);
  lowScl();
  releaseSda();
  return value;
}

bool Mpu6050::SoftI2C::probe(uint8_t address) {
  start();
  const bool ack = writeByte(static_cast<uint8_t>(address << 1));
  stop();
  return ack;
}

bool Mpu6050::SoftI2C::writeReg(uint8_t address, uint8_t reg, uint8_t value) {
  start();
  const bool ok = writeByte(static_cast<uint8_t>(address << 1)) &&
                  writeByte(reg) && writeByte(value);
  stop();
  return ok;
}

bool Mpu6050::SoftI2C::readRegs(uint8_t address, uint8_t reg, uint8_t* data,
                                size_t length) {
  if (data == nullptr || length == 0) return false;
  start();
  if (!writeByte(static_cast<uint8_t>(address << 1)) || !writeByte(reg)) {
    stop();
    return false;
  }
  start();
  if (!writeByte(static_cast<uint8_t>((address << 1) | 1U))) {
    stop();
    return false;
  }
  for (size_t i = 0; i < length; ++i) {
    data[i] = readByte(i + 1U < length);
  }
  stop();
  return true;
}

int16_t Mpu6050::makeInt16(uint8_t high, uint8_t low) {
  return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

bool Mpu6050::configureDevice() {
  uint8_t who = 0;
  if (!bus_.readRegs(address_, REG_WHO_AM_I, &who, 1)) return false;
  if ((who & 0x7EU) != 0x68U) return false;

  // Reset then select PLL with X gyroscope as the clock source.
  if (!bus_.writeReg(address_, REG_PWR_MGMT_1, 0x80)) return false;
  delay(100);
  if (!bus_.writeReg(address_, REG_PWR_MGMT_1, 0x01)) return false;
  delay(10);
  // DLPF=3 (~44 Hz gyro bandwidth), 1 kHz internal sample / (1+9) = 100 Hz.
  if (!bus_.writeReg(address_, REG_CONFIG, 0x03)) return false;
  if (!bus_.writeReg(address_, REG_SMPLRT_DIV, 9)) return false;
  if (!bus_.writeReg(address_, REG_GYRO_CONFIG, 0x00)) return false;
  if (!bus_.writeReg(address_, REG_ACCEL_CONFIG, 0x00)) return false;
  return true;
}

bool Mpu6050::begin() {
  bus_.begin();
  health_ = ImuHealth::DISCONNECTED;
  calibrated_ = false;
  if (!bus_.probe(address_) || !configureDevice()) return false;
  beginMs_ = millis();
  lastGoodMs_ = beginMs_;
  lastRequestMs_ = 0;
  consecutiveReadErrors_ = 0;
  restartCalibration();
  return true;
}

void Mpu6050::restartCalibration() {
  calibrated_ = false;
  calibrationSamples_ = 0;
  calibrationSumGx_ = calibrationSumGy_ = calibrationSumGz_ = 0.0f;
  gyroBiasXDegS_ = gyroBiasYDegS_ = gyroBiasZDegS_ = 0.0f;
  beginMs_ = millis();
  if (lastGoodMs_ != 0U) health_ = ImuHealth::CALIBRATING;
}

bool Mpu6050::sampleStationary() const {
  const float accelMagnitude = sqrtf(data_.accelXg * data_.accelXg +
                                     data_.accelYg * data_.accelYg +
                                     data_.accelZg * data_.accelZg);
  return fabsf(accelMagnitude - 1.0f) <= IMU_CALIBRATION_ACCEL_TOLERANCE_G &&
         fabsf(data_.gyroXDps) <= IMU_CALIBRATION_MAX_RATE_DPS &&
         fabsf(data_.gyroYDps) <= IMU_CALIBRATION_MAX_RATE_DPS &&
         fabsf(data_.gyroZDps) <= IMU_CALIBRATION_MAX_RATE_DPS;
}

bool Mpu6050::readSample() {
  uint8_t bytes[14] = {};
  if (!bus_.readRegs(address_, REG_ACCEL_XOUT_H, bytes, sizeof(bytes))) {
    return false;
  }
  const int16_t rawAx = makeInt16(bytes[0], bytes[1]);
  const int16_t rawAy = makeInt16(bytes[2], bytes[3]);
  const int16_t rawAz = makeInt16(bytes[4], bytes[5]);
  const int16_t rawTemp = makeInt16(bytes[6], bytes[7]);
  const int16_t rawGx = makeInt16(bytes[8], bytes[9]);
  const int16_t rawGy = makeInt16(bytes[10], bytes[11]);
  const int16_t rawGz = makeInt16(bytes[12], bytes[13]);

  // Face-down installation transform to robot coordinates.
  data_.accelXg = static_cast<float>(rawAx) / kAccelLsbPerG;
  data_.accelYg = -static_cast<float>(rawAy) / kAccelLsbPerG;
  data_.accelZg = -static_cast<float>(rawAz) / kAccelLsbPerG;
  data_.gyroXDps = static_cast<float>(rawGx) / kGyroLsbPerDps;
  data_.gyroYDps = -static_cast<float>(rawGy) / kGyroLsbPerDps;
  data_.gyroZDps = -static_cast<float>(rawGz) / kGyroLsbPerDps;
  data_.temperatureC = static_cast<float>(rawTemp) / 340.0f + 36.53f;
  return true;
}

void Mpu6050::update() {
  const uint32_t now = millis();
  if ((now - lastRequestMs_) < IMU_SAMPLE_PERIOD_MS) return;
  lastRequestMs_ = now;
  if (!readSample()) {
    if (consecutiveReadErrors_ < 255U) ++consecutiveReadErrors_;
    if (consecutiveReadErrors_ >= IMU_READ_FAULT_COUNT) {
      health_ = ImuHealth::READ_FAULT;
    }
    return;
  }

  consecutiveReadErrors_ = 0;
  lastGoodMs_ = now;
  data_.lastSampleMs = now;
  ++data_.sampleSequence;

  if (!calibrated_) {
    health_ = ImuHealth::CALIBRATING;
    if ((now - beginMs_) < IMU_CALIBRATION_SETTLE_MS) return;
    if (!sampleStationary()) {
      // Calibration must represent a physically stationary chassis. Restart
      // accumulation after movement instead of baking motion into gyro bias.
      calibrationSamples_ = 0;
      calibrationSumGx_ = calibrationSumGy_ = calibrationSumGz_ = 0.0f;
      return;
    }
    calibrationSumGx_ += data_.gyroXDps;
    calibrationSumGy_ += data_.gyroYDps;
    calibrationSumGz_ += data_.gyroZDps;
    ++calibrationSamples_;
    if (calibrationSamples_ >= IMU_CALIBRATION_SAMPLES) {
      const float inv = 1.0f / static_cast<float>(calibrationSamples_);
      gyroBiasXDegS_ = calibrationSumGx_ * inv;
      gyroBiasYDegS_ = calibrationSumGy_ * inv;
      gyroBiasZDegS_ = calibrationSumGz_ * inv;
      calibrated_ = true;
      health_ = ImuHealth::OK;
    }
  }

  if (calibrated_) {
    data_.gyroXDps -= gyroBiasXDegS_;
    data_.gyroYDps -= gyroBiasYDegS_;
    data_.gyroZDps -= gyroBiasZDegS_;
    health_ = ImuHealth::OK;
  }
}

bool Mpu6050::connected() const {
  return lastGoodMs_ != 0U &&
         (millis() - lastGoodMs_) <= IMU_LOST_TIMEOUT_MS &&
         health_ != ImuHealth::DISCONNECTED;
}

const char* Mpu6050::healthText() const {
  switch (health_) {
    case ImuHealth::DISCONNECTED: return "LOST";
    case ImuHealth::CALIBRATING: return "CAL";
    case ImuHealth::OK: return "OK";
    case ImuHealth::READ_FAULT: return "FAULT";
  }
  return "UNKNOWN";
}
