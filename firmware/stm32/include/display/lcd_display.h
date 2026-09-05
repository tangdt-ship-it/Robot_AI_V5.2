#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <Arduino.h>

enum class LcdPage : uint8_t {
  ROBOT = 0,
  MAP = 1,
};

struct LcdDisplayData {
  float headingDeg = 0.0f;
  const char* controlMode = "AI";
  const char* ps2Status = "WAIT";
  const char* motionOwner = "NONE";
  bool ps2Connected = false;
  bool ps2Fresh = false;
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
  float frontLeftDistanceCm = 0.0f;
  float frontRightDistanceCm = 0.0f;
  bool frontLeftFresh = false;
  bool frontRightFresh = false;
  bool frontLeftEchoValid = false;
  bool frontRightEchoValid = false;
  bool frontLeftDisplayDistanceValid = false;
  bool frontRightDisplayDistanceValid = false;
  bool frontLeftDisplayFar = false;
  bool frontRightDisplayFar = false;
  const char* frontLeftZone = "UNKNOWN";
  const char* frontRightZone = "UNKNOWN";
  const char* frontLeftHealth = "UNKNOWN";
  const char* frontRightHealth = "UNKNOWN";
  bool brakeLocked = false;
  const char* imuStatus = "LOST";
  const char* fusionStatus = "LOST";
};

struct LcdMapStatus {
  bool valid = false;
  uint8_t storeState = 0;  // EMPTY/SAVED/INVALID/STORAGE_ERROR.
  // STM32-local MAP modes: READY, TEACH, SAVED, DELETE, CHECK, CHECKED,
  // RUNNING, HOLD, COMPLETE and CLOSED_CONFIRM.
  uint8_t mode = 0;
  uint8_t routeType = 0;   // 0 OPEN, 1 CLOSED.
  uint8_t replayMode = 0;  // 0 ONCE, 1 LOOP, 2 RETURN, 3 PING_PONG.
  uint16_t points = 0;
  uint16_t maxPoints = 128;
  uint32_t lengthMm = 0;
  uint16_t replayWp = 0;
  uint16_t replayTotal = 0;
  uint32_t replayTargetMm = 0;
  uint32_t replayTravelMm = 0;
  uint32_t replayErrorMm = 0;
  uint8_t replayOperation = 0;  // 0 NONE, 1 MOVE, 2 TURN, 3 RESUMABLE HOLD.
  uint8_t holdReason = 0;       // 0 NONE, 1 USER, 2 OBSTACLE, ...
  int16_t replayTargetDeg = 0;
  uint32_t replayLapCounter = 0;
  uint32_t closeCandidateDistanceMm = 0;
  int16_t closeCandidateHeadingDeg = 0;
  uint8_t settingsItem = 0;  // MODE/SPEED/LAP/DELETE MAP.
  int16_t settingsSpeed = 20;
  uint8_t settingsLoopTarget = 0;  // 0 = INF.
  uint8_t helpPage = 0;
  uint8_t storageErrorReason = 0;  // NONE/SETTINGS/TEACH/MODE/INIT/GENERIC.
  bool oldRouteAvailable = false;
};

class LcdDisplay {
 public:
  LcdDisplay(uint32_t sclPin, uint32_t sdaPin, uint8_t address);
  bool begin();
  void setData(const LcdDisplayData& data);
  void update();
  void forceRefresh();
  bool isAvailable() const { return available_; }

  // The STM32 LCD owns the lightweight Map page. The ESP32 TFT remains
  // dedicated to Xiaozhi/chat/camera. These methods only change presentation;
  // they never affect motor authority or navigation state.
  void setPage(LcdPage page);
  void togglePage();
  LcdPage page() const { return page_; }
  bool isMapPage() const { return page_ == LcdPage::MAP; }
  void setMapSlot(uint8_t slot);
  void toggleMapSlot();
  uint8_t mapSlot() const { return mapSlot_; }
  bool mapSlotLocked() const {
    const LcdMapStatus& status = mapStatus_[mapSlot_ - 1U];
    return status.valid && status.mode >= 1U && status.mode != 2U &&
           status.mode != 8U;
  }
  void setMapStatus(uint8_t slot, uint8_t storeState, uint8_t mode,
                    uint16_t points, uint16_t maxPoints,
                    uint32_t lengthMm, uint16_t replayWp = 0,
                    uint16_t replayTotal = 0, uint32_t replayTargetMm = 0,
                    uint32_t replayTravelMm = 0,
                    uint32_t replayErrorMm = 0,
                    uint8_t replayOperation = 0,
                    uint8_t routeType = 0,
                    uint8_t replayMode = 0,
                    uint8_t holdReason = 0,
                    int16_t replayTargetDeg = 0,
                    uint32_t replayLapCounter = 0,
                    uint32_t closeCandidateDistanceMm = 0,
                    int16_t closeCandidateHeadingDeg = 0,
                    uint8_t settingsItem = 0,
                    int16_t settingsSpeed = 20,
                    uint8_t settingsLoopTarget = 0,
                    uint8_t helpPage = 0,
                    uint8_t storageErrorReason = 0,
                    bool oldRouteAvailable = false);

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
  void buildRobotLines();
  void buildMapLines();
  bool sendOneDirtyCharacter();
  static void padLine(char (&line)[21]);

  SoftI2C bus_;
  uint8_t address_;
  bool available_ = false;
  LcdDisplayData data_;
  LcdPage page_ = LcdPage::ROBOT;
  uint8_t mapSlot_ = 1;
  LcdMapStatus mapStatus_[2] = {};
  char desired_[4][21] = {};
  char sent_[4][21] = {};
  uint32_t lastRenderMs_ = 0;
  uint32_t lastCharacterMs_ = 0;
  uint8_t scanRow_ = 0;
  uint8_t scanColumn_ = 0;
};

#endif
