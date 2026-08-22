#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <Arduino.h>

struct LcdDisplayData {
  float compassAngle = 0.0f;
  const char* ps2Status = "WAIT";
  int16_t speed = 0;
  int16_t left = 0;
  int16_t right = 0;
  int32_t leftEncoderTicks = 0;
  int32_t rightEncoderTicks = 0;
  bool encoderReady = false;
  const char* encoderHealth = "DISABLED";
  bool headingEnabled = false;
  float headingError = 0.0f;
  float headingTarget = 0.0f;
  bool rampEnabled = false;
  bool ultrasonicValid = false;
  float ultrasonicDistanceCm = 0.0f;
  const char* obstacleZone = "UNKNOWN";
  bool brakeLocked = false;
  const char* imuStatus = "LOST";
  const char* fusionStatus = "LOST";
};

class LcdDisplay {
 public:
  LcdDisplay(uint32_t sclPin, uint32_t sdaPin, uint8_t address);
  bool begin();
  void setData(const LcdDisplayData& data);
  void update();
  void forceRefresh();
  bool isAvailable() const { return available_; }

 private:
  class SoftI2C {
   public:
    SoftI2C(uint32_t sclPin, uint32_t sdaPin);
    void begin();
    bool probe(uint8_t address);
    bool writeByte(uint8_t address, uint8_t value);

   private:
    void lowScl();
    void lowSda();
    void releaseScl();
    void releaseSda();
    void waitHigh(uint32_t pin);
    void start();
    void stop();
    bool writeRaw(uint8_t value);
    uint32_t sclPin_;
    uint32_t sdaPin_;
  };

  void clearHardware();
  void command(uint8_t value);
  void send(uint8_t value, uint8_t rs);
  void write4(uint8_t value);
  void expanderWrite(uint8_t value);
  void pulseEnable(uint8_t value);
  void setCursor(uint8_t column, uint8_t row);
  void buildDesiredLines();
  bool sendOneDirtyCharacter();
  static void padLine(char (&line)[21]);

  SoftI2C bus_;
  uint8_t address_;
  bool available_ = false;
  LcdDisplayData data_;
  char desired_[4][21] = {};
  char sent_[4][21] = {};
  uint32_t lastRenderMs_ = 0;
  uint32_t lastCharacterMs_ = 0;
  uint8_t scanRow_ = 0;
  uint8_t scanColumn_ = 0;
};

#endif
