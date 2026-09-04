#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H
#include <Arduino.h>
enum class ObstacleZone : uint8_t { UNKNOWN, CLEAR, CAUTION, BLOCKED, EMERGENCY };
enum class AvoidanceDirection : uint8_t { NONE, LEFT, RIGHT, STOP };
// Health is deliberately separate from ObstacleZone.  A timeout is not an
// empty corridor: it is an absence of evidence and must fail closed.
enum class SensorHealth : uint8_t {
  UNKNOWN,
  HEALTHY,
  STALE,
  TIMEOUT,
  INVALID,
  DISCONNECTED_OR_FAULT,
  DEGRADED,
};
struct UltrasonicReading {
  float distanceCm=0, rawDistanceCm=0, rateCmS=0;
  bool valid=false, fresh=false, echoValid=false, displayFar=false;
  // True when the LCD can safely present the last real Echo during the
  // bounded dropout hold, or the established no-Echo/out-of-range display
  // state. It is separate from echoValid, which describes the current
  // measurement attempt.
  bool displayDistanceValid=false;
  uint32_t lastUpdateMs=0, ageMs=0, failureCount=0;
  ObstacleZone zone=ObstacleZone::UNKNOWN;
  SensorHealth health=SensorHealth::UNKNOWN;
};
class UltrasonicSensor {
 public:
  void begin(); void update();
  bool hasMeasurement() const { return measurementSequence_ != 0U; }
  bool echoValid() const { return overallEchoValid_; }
  bool isFresh() const { return overallFresh_; }
  bool healthy() const { return overallHealth_ == SensorHealth::HEALTHY; }
  SensorHealth health() const { return overallHealth_; }
  float distanceCm() const { return nearestDistanceCm_; }
  float rawDistanceCm() const { return nearestRawDistanceCm_; }
  float approachRateCmS() const { return nearestRateCmS_; }
  ObstacleZone zone() const { return overallZone_; }
  bool hardBlocked() const;
  uint32_t measurementSequence() const { return measurementSequence_; }
  uint32_t zoneSequence() const { return zoneSequence_; }
  const UltrasonicReading& frontLeft() const { return frontLeft_; }
  const UltrasonicReading& frontRight() const { return frontRight_; }
  float frontLeftDistanceCm() const { return frontLeft_.distanceCm; }
  float frontRightDistanceCm() const { return frontRight_.distanceCm; }
  uint32_t frontLeftTriggerCount() const { return channels_[0].triggerCount; }
  uint32_t frontRightTriggerCount() const { return channels_[1].triggerCount; }
  uint32_t frontLeftIsrCount() const { return channels_[0].isrCount; }
  uint32_t frontRightIsrCount() const { return channels_[1].isrCount; }
  uint32_t frontLeftCapturedEdgeCount() const { return channels_[0].capturedEdgeCount; }
  uint32_t frontRightCapturedEdgeCount() const { return channels_[1].capturedEdgeCount; }
  uint32_t frontLeftLastPulseUs() const { return channels_[0].lastPulseUs; }
  uint32_t frontRightLastPulseUs() const { return channels_[1].lastPulseUs; }
  ObstacleZone frontLeftZone() const { return frontLeft_.zone; }
  ObstacleZone frontRightZone() const { return frontRight_.zone; }
  ObstacleZone overallZone() const { return overallZone_; }
  AvoidanceDirection suggestedAvoidance() const { return suggestion_; }
  static const char* avoidanceText(AvoidanceDirection direction);
  static const char* zoneText(ObstacleZone zone);
  static const char* healthText(SensorHealth health);
  float stoppingDistanceCm(int16_t forwardCommand) const;
  int16_t limitForwardCommand(int16_t forwardCommand) const;
  // Bounded grace for in-place turns after a recently validated wide-clear
  // sector. This never treats an unvalidated/startup sensor as clear.
  bool hasRecentClearWindow(uint32_t nowMs) const;
 private:
  enum class TriggerState : uint8_t { IDLE, WAIT_ECHO };
  struct Channel {
    uint32_t trigPin=0, echoPin=0;
    volatile uint32_t echoRiseUs=0, echoPulseUs=0;
    volatile bool echoPulseReady=false;
    volatile TriggerState state=TriggerState::IDLE;
    volatile uint32_t isrCount=0, capturedEdgeCount=0;
    uint32_t triggerCount=0;
    volatile uint32_t lastPulseUs=0;
    uint32_t waitEchoStartedUs=0;
    uint32_t lastTriggerMs=0, lastMeasurementMs=0, measurementSequence=0, failureCount=0;
    float history[5]={}; uint8_t historyCount=0, historyIndex=0;
    float rawDistanceCm=0, filteredDistanceCm=0, approachRateCmS=0;
    float outlierCandidateCm=0;
    uint8_t outlierConfirmations=0;
    bool filterReady=false, echoValid=false; ObstacleZone zone=ObstacleZone::UNKNOWN;
    SensorHealth health=SensorHealth::UNKNOWN;
    uint32_t lastValidEchoMs=0, consecutiveTimeouts=0, invalidCount=0;
    uint8_t closerConfirmations=0, fartherConfirmations=0;
    bool displayFar=false;
    bool noEchoFar=false;
  } channels_[2];
  static UltrasonicSensor* instance_;
  static void echoIsrMountLeft(); static void echoIsrMountRight();
  void handleEchoEdge(uint8_t index);
  void acceptPulse(uint8_t index,uint32_t pulseUs,uint32_t nowMs);
  void acceptTimeout(uint8_t index,uint32_t nowMs,bool noEcho=true);
  void updateChannelZone(uint8_t index,float distanceCm);
  float medianHistory(const Channel& channel) const; bool channelFresh(const Channel& channel,uint32_t nowMs) const;
  bool hasRecentValidEcho(const Channel& channel,uint32_t nowMs) const;
  bool degradedClearWindow(uint32_t nowMs) const;
  void recomputeObstacleModel(uint32_t nowMs);
  UltrasonicReading frontLeft_,frontRight_; float nearestDistanceCm_=0,nearestRawDistanceCm_=0,nearestRateCmS_=0;
  bool overallEchoValid_=false,overallFresh_=false; SensorHealth overallHealth_=SensorHealth::UNKNOWN;
  ObstacleZone overallZone_=ObstacleZone::UNKNOWN;
  AvoidanceDirection suggestion_=AvoidanceDirection::STOP;
  volatile uint8_t activeChannel_=0xFF; uint8_t nextChannel_=0;
  uint32_t nextTriggerAllowedMs_=0,measurementSequence_=0,zoneSequence_=0;
};
#endif
