#include <sensors/obstacle_classifier.h>

#include <math.h>

#include <robot_config.h>

namespace {
bool sameResult(ObstacleClass leftClass, ObstacleDecision leftDecision,
                ObstacleClass rightClass, ObstacleDecision rightDecision) {
  return leftClass == rightClass && leftDecision == rightDecision;
}
}

ObstacleClassificationInput ObstacleClassifier::snapshot(
    const UltrasonicSensor& sensor) {
  const UltrasonicReading& left = sensor.frontLeft();
  const UltrasonicReading& right = sensor.frontRight();
  ObstacleClassificationInput input;
  input.leftFresh = left.fresh;
  input.leftHealthy = left.health == SensorHealth::HEALTHY;
  input.leftValid = left.valid && left.echoValid;
  input.leftDistanceCm = left.distanceCm;
  input.leftZone = left.zone;
  input.rightFresh = right.fresh;
  input.rightHealthy = right.health == SensorHealth::HEALTHY;
  input.rightValid = right.valid && right.echoValid;
  input.rightDistanceCm = right.distanceCm;
  input.rightZone = right.zone;
  return input;
}

bool ObstacleClassifier::channelValid(bool fresh, bool healthy, bool valid,
                                      float distance, ObstacleZone zone) {
  return fresh && healthy && valid && isfinite(distance) &&
         distance >= ULTRASONIC_MIN_CM && distance <= ULTRASONIC_MAX_CM &&
         zone != ObstacleZone::UNKNOWN;
}

bool ObstacleClassifier::blocked(ObstacleZone zone) {
  return zone == ObstacleZone::CAUTION || zone == ObstacleZone::BLOCKED ||
         zone == ObstacleZone::EMERGENCY;
}

bool ObstacleClassifier::stopZone(ObstacleZone zone) {
  return zone == ObstacleZone::BLOCKED || zone == ObstacleZone::EMERGENCY;
}

ObstacleClassificationResult ObstacleClassifier::classify(
    const ObstacleClassificationInput& input) {
  const bool leftOk = channelValid(input.leftFresh, input.leftHealthy,
                                   input.leftValid, input.leftDistanceCm,
                                   input.leftZone);
  const bool rightOk = channelValid(input.rightFresh, input.rightHealthy,
                                    input.rightValid, input.rightDistanceCm,
                                    input.rightZone);
  if (!leftOk || !rightOk) {
    return {ObstacleClass::UNKNOWN, ObstacleDecision::HOLD, false};
  }

  const bool leftBlocked = blocked(input.leftZone);
  const bool rightBlocked = blocked(input.rightZone);
  ObstacleClass classification = ObstacleClass::CLEAR;
  if (leftBlocked && !rightBlocked) {
    classification = ObstacleClass::LEFT;
  } else if (!leftBlocked && rightBlocked) {
    classification = ObstacleClass::RIGHT;
  } else if (leftBlocked && rightBlocked) {
    const float differenceMm = fabsf(input.leftDistanceCm -
                                     input.rightDistanceCm) * 10.0f;
    classification = differenceMm <= OBSTACLE_CLASS_CENTER_BALANCE_MM
                         ? ObstacleClass::CENTER
                         : ObstacleClass::BOTH_BLOCKED;
  }
  return {classification, recommend(input, classification), true};
}

ObstacleDecision ObstacleClassifier::recommend(
    const ObstacleClassificationInput& input, ObstacleClass classification) {
  switch (classification) {
    case ObstacleClass::CLEAR:
      return ObstacleDecision::NONE;
    case ObstacleClass::LEFT:
      return ObstacleDecision::AVOID_RIGHT;
    case ObstacleClass::RIGHT:
      return ObstacleDecision::AVOID_LEFT;
    case ObstacleClass::CENTER:
    case ObstacleClass::BOTH_BLOCKED: {
      const float rightAdvantageMm =
          (input.rightDistanceCm - input.leftDistanceCm) * 10.0f;
      const float leftAdvantageMm = -rightAdvantageMm;
      if (rightAdvantageMm >= OBSTACLE_CLASS_SIDE_CLEARANCE_MARGIN_MM &&
          !stopZone(input.rightZone)) {
        return ObstacleDecision::AVOID_RIGHT;
      }
      if (leftAdvantageMm >= OBSTACLE_CLASS_SIDE_CLEARANCE_MARGIN_MM &&
          !stopZone(input.leftZone)) {
        return ObstacleDecision::AVOID_LEFT;
      }
      return ObstacleDecision::HOLD;
    }
    case ObstacleClass::UNKNOWN:
    default:
      return ObstacleDecision::HOLD;
  }
}

void ObstacleClassifier::update(uint32_t nowMs) {
  const ObstacleClassificationInput input = snapshot(ultrasonic_);
  const ObstacleClassificationResult result = classify(input);

  // Loss of freshness/health is an immediate fail-closed transition. Never
  // retain a previous side recommendation while the sensor evidence is bad.
  if (!result.valid) {
    candidateInitialized_ = false;
    stable_ = false;
    classification_ = ObstacleClass::UNKNOWN;
    decision_ = ObstacleDecision::HOLD;
    logState(input);
    return;
  }

  if (!candidateInitialized_ ||
      !sameResult(candidateClass_, candidateDecision_, result.classification,
                  result.recommendation)) {
    candidateClass_ = result.classification;
    candidateDecision_ = result.recommendation;
    candidateSinceMs_ = nowMs;
    candidateInitialized_ = true;
    stable_ = false;
  }

  classification_ = candidateClass_;
  if (nowMs - candidateSinceMs_ >= OBSTACLE_CLASS_STABLE_MS) {
    decision_ = candidateDecision_;
    stable_ = true;
  } else {
    // The candidate class is useful diagnostic information, but no direction
    // is actionable until the time gate completes.
    decision_ = ObstacleDecision::HOLD;
    stable_ = false;
  }
  logState(input);
}

void ObstacleClassifier::logState(const ObstacleClassificationInput& input) {
  if (logInitialized_ && classification_ == lastLoggedClass_ &&
      decision_ == lastLoggedDecision_ && stable_ == lastLoggedStable_) {
    return;
  }
  debug_.print("OBS,CLASS,");
  debug_.print(classText(classification_));
  debug_.print(",L=");
  debug_.print(input.leftDistanceCm, 1);
  debug_.print(",R=");
  debug_.print(input.rightDistanceCm, 1);
  debug_.print(",LZONE=");
  debug_.print(UltrasonicSensor::zoneText(input.leftZone));
  debug_.print(",RZONE=");
  debug_.print(UltrasonicSensor::zoneText(input.rightZone));
  debug_.print(",STABLE=");
  debug_.println(stable_ ? 1 : 0);

  debug_.print("OBS,DECIDE,");
  debug_.print(decisionText(decision_));
  debug_.print(",CLASS=");
  debug_.print(classText(classification_));
  debug_.print(",STABLE=");
  debug_.println(stable_ ? 1 : 0);
  lastLoggedClass_ = classification_;
  lastLoggedDecision_ = decision_;
  lastLoggedStable_ = stable_;
  logInitialized_ = true;
}

const char* ObstacleClassifier::classText(ObstacleClass classification) {
  switch (classification) {
    case ObstacleClass::CLEAR:
      return "CLEAR";
    case ObstacleClass::LEFT:
      return "LEFT";
    case ObstacleClass::RIGHT:
      return "RIGHT";
    case ObstacleClass::CENTER:
      return "CENTER";
    case ObstacleClass::BOTH_BLOCKED:
      return "BOTH_BLOCKED";
    case ObstacleClass::UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

const char* ObstacleClassifier::decisionText(ObstacleDecision decision) {
  switch (decision) {
    case ObstacleDecision::NONE:
      return "NONE";
    case ObstacleDecision::AVOID_LEFT:
      return "AVOID_LEFT";
    case ObstacleDecision::AVOID_RIGHT:
      return "AVOID_RIGHT";
    case ObstacleDecision::HOLD:
    default:
      return "HOLD";
  }
}
