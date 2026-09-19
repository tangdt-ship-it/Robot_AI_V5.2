#ifndef OBSTACLE_CLASSIFIER_H
#define OBSTACLE_CLASSIFIER_H

#include <Arduino.h>
#include <sensors/ultrasonic_sensor.h>

enum class ObstacleClass : uint8_t {
  CLEAR,
  LEFT,
  RIGHT,
  CENTER,
  BOTH_BLOCKED,
  UNKNOWN
};

enum class ObstacleDecision : uint8_t {
  NONE,
  AVOID_LEFT,
  AVOID_RIGHT,
  HOLD
};

// A pure snapshot keeps the classifier testable and makes the data-validity
// boundary explicit.  No LCD/presentation state is included here.
struct ObstacleClassificationInput {
  bool leftFresh = false;
  bool leftHealthy = false;
  bool leftValid = false;
  float leftDistanceCm = 0.0f;
  ObstacleZone leftZone = ObstacleZone::UNKNOWN;
  bool rightFresh = false;
  bool rightHealthy = false;
  bool rightValid = false;
  float rightDistanceCm = 0.0f;
  ObstacleZone rightZone = ObstacleZone::UNKNOWN;
};

struct ObstacleClassificationResult {
  ObstacleClass classification = ObstacleClass::UNKNOWN;
  ObstacleDecision recommendation = ObstacleDecision::HOLD;
  bool valid = false;
};

class ObstacleClassifier {
 public:
  ObstacleClassifier(const UltrasonicSensor& ultrasonic, Print& debug)
      : ultrasonic_(ultrasonic), debug_(debug) {}

  void update(uint32_t nowMs = millis());

  ObstacleClass classification() const { return classification_; }
  ObstacleDecision decision() const { return decision_; }
  bool stable() const { return stable_; }
  uint32_t candidateAgeMs(uint32_t nowMs = millis()) const {
    return candidateInitialized_ ? nowMs - candidateSinceMs_ : 0U;
  }

  static ObstacleClassificationResult classify(
      const ObstacleClassificationInput& input);
  static ObstacleDecision recommend(const ObstacleClassificationInput& input,
                                    ObstacleClass classification);
  static const char* classText(ObstacleClass classification);
  static const char* decisionText(ObstacleDecision decision);

 private:
  static ObstacleClassificationInput snapshot(const UltrasonicSensor& sensor);
  static bool channelValid(bool fresh, bool healthy, bool valid, float distance,
                           ObstacleZone zone);
  static bool blocked(ObstacleZone zone);
  static bool stopZone(ObstacleZone zone);
  void logState(const ObstacleClassificationInput& input);

  const UltrasonicSensor& ultrasonic_;
  Print& debug_;
  ObstacleClass candidateClass_ = ObstacleClass::UNKNOWN;
  ObstacleDecision candidateDecision_ = ObstacleDecision::HOLD;
  ObstacleClass classification_ = ObstacleClass::UNKNOWN;
  ObstacleDecision decision_ = ObstacleDecision::HOLD;
  uint32_t candidateSinceMs_ = 0U;
  bool candidateInitialized_ = false;
  bool stable_ = false;
  bool logInitialized_ = false;
  ObstacleClass lastLoggedClass_ = ObstacleClass::UNKNOWN;
  ObstacleDecision lastLoggedDecision_ = ObstacleDecision::HOLD;
  bool lastLoggedStable_ = false;
};

#endif
