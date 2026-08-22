#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>

enum class ImuHealth : uint8_t {
  DISCONNECTED = 0,
  CALIBRATING = 1,
  OK = 2,
  READ_FAULT = 3,
};

struct ImuData {
  float accelXg = 0.0f;
  float accelYg = 0.0f;
  float accelZg = 0.0f;
  float gyroXDps = 0.0f;
  float gyroYDps = 0.0f;
  float gyroZDps = 0.0f;
  float temperatureC = 0.0f;
  uint32_t sampleSequence = 0;
  uint32_t lastSampleMs = 0;
};

// MPU6050 driver for Robot_AI_V4.2.
// The module is mounted face-down, connector toward the rear of the robot and
// mounting holes toward the front. Robot coordinates are +X forward, +Y left,
// +Z up, so the board axes are transformed as X=+Ximu, Y=-Yimu, Z=-Zimu.
class Mpu6050 {
 public:
  Mpu6050(uint32_t sclPin, uint32_t sdaPin, uint8_t address = 0x68)
      : bus_(sclPin, sdaPin), address_(address) {}

  bool begin();
  void update();
  void restartCalibration();

  bool connected() const;
  bool calibrated() const { return calibrated_; }
  bool ready() const { return connected() && calibrated_ && health_ == ImuHealth::OK; }
  ImuHealth health() const { return health_; }
  const char* healthText() const;
  const ImuData& data() const { return data_; }
  float gyroBiasZDegS() const { return gyroBiasZDegS_; }
  uint16_t calibrationSamples() const { return calibrationSamples_; }

 private:
  class SoftI2C {
   public:
    SoftI2C(uint32_t sclPin, uint32_t sdaPin)
        : sclPin_(sclPin), sdaPin_(sdaPin) {}
    void begin();
    bool probe(uint8_t address);
    bool writeReg(uint8_t address, uint8_t reg, uint8_t value);
    bool readRegs(uint8_t address, uint8_t reg, uint8_t* data, size_t length);

   private:
    void lowScl();
    void lowSda();
    void releaseScl();
    void releaseSda();
    bool waitSclHigh();
    void start();
    void stop();
    bool writeByte(uint8_t value);
    uint8_t readByte(bool ack);

    uint32_t sclPin_;
    uint32_t sdaPin_;
  };

  bool configureDevice();
  bool readSample();
  bool sampleStationary() const;
  static int16_t makeInt16(uint8_t high, uint8_t low);

  SoftI2C bus_;
  uint8_t address_ = 0x68;
  ImuData data_;
  ImuHealth health_ = ImuHealth::DISCONNECTED;
  bool calibrated_ = false;
  uint32_t beginMs_ = 0;
  uint32_t lastRequestMs_ = 0;
  uint32_t lastGoodMs_ = 0;
  uint8_t consecutiveReadErrors_ = 0;
  uint16_t calibrationSamples_ = 0;
  float calibrationSumGx_ = 0.0f;
  float calibrationSumGy_ = 0.0f;
  float calibrationSumGz_ = 0.0f;
  float gyroBiasXDegS_ = 0.0f;
  float gyroBiasYDegS_ = 0.0f;
  float gyroBiasZDegS_ = 0.0f;
};

#endif
