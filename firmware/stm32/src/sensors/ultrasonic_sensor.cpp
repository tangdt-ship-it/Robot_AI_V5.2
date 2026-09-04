#include <sensors/ultrasonic_sensor.h>
#include <robot_config.h>
#include <math.h>

UltrasonicSensor* UltrasonicSensor::instance_ = nullptr;
namespace { constexpr uint8_t LEFT_MOUNT=0, RIGHT_MOUNT=1; }

void UltrasonicSensor::begin() {
  instance_=this;
  channels_[LEFT_MOUNT].trigPin=ULTRASONIC_TRIG_PIN;
  channels_[LEFT_MOUNT].echoPin=ULTRASONIC_ECHO_PIN;
  channels_[RIGHT_MOUNT].trigPin=ULTRASONIC_RIGHT_TRIG_PIN;
  channels_[RIGHT_MOUNT].echoPin=ULTRASONIC_RIGHT_ECHO_PIN;
  const uint32_t now=millis();
  for (uint8_t i=0; i<2; ++i) {
    Channel& c=channels_[i];
    pinMode(c.trigPin,OUTPUT);
    digitalWrite(c.trigPin,LOW);
    // HC-SR04 Echo is push-pull, but the long harness can briefly float while
    // the module is idle or between valid pulses. Keep a weak pull-down so
    // noise cannot create a false edge; it is negligible while Echo drives
    // HIGH and does not alter the motor/safety policy.
    pinMode(c.echoPin,INPUT_PULLDOWN);
    c.lastTriggerMs=now-ULTRASONIC_SAMPLE_PERIOD_MS;
  }
  attachInterrupt(digitalPinToInterrupt(channels_[LEFT_MOUNT].echoPin),
                  echoIsrMountLeft,CHANGE);
  attachInterrupt(digitalPinToInterrupt(channels_[RIGHT_MOUNT].echoPin),
                  echoIsrMountRight,CHANGE);
  activeChannel_=0xFF;
  nextChannel_=LEFT_MOUNT;
  nextTriggerAllowedMs_=now;
}
void UltrasonicSensor::echoIsrMountLeft(){if(instance_)instance_->handleEchoEdge(LEFT_MOUNT);}
void UltrasonicSensor::echoIsrMountRight(){if(instance_)instance_->handleEchoEdge(RIGHT_MOUNT);}
void UltrasonicSensor::handleEchoEdge(uint8_t i){
  Channel& c=channels_[i];
  const uint32_t us=micros();
  ++c.isrCount;
  if(activeChannel_ != i || c.state != TriggerState::WAIT_ECHO) return;
  ++c.capturedEdgeCount;
  if(digitalRead(c.echoPin)==HIGH){
    c.echoRiseUs=us;
  }else if(c.echoRiseUs != 0U){
    c.echoPulseUs=us-c.echoRiseUs;
    c.lastPulseUs=c.echoPulseUs;
    c.echoRiseUs=0;
    c.echoPulseReady=true;
  }
}
bool UltrasonicSensor::channelFresh(const Channel& c,uint32_t now)const{return c.measurementSequence && now-c.lastMeasurementMs<=ULTRASONIC_FRESH_MS;}
bool UltrasonicSensor::hasRecentValidEcho(const Channel& c,uint32_t now)const{
  return c.filterReady && c.lastValidEchoMs!=0U &&
         now-c.lastValidEchoMs<=ULTRASONIC_DEGRADED_GRACE_MS &&
         c.consecutiveTimeouts<=ULTRASONIC_DEGRADED_MAX_TIMEOUTS;
}
float UltrasonicSensor::medianHistory(const Channel& c)const{
  float a[5]; for(uint8_t i=0;i<c.historyCount;i++)a[i]=c.history[i];
  for(uint8_t i=1;i<c.historyCount;i++){float v=a[i];int8_t j=i-1;while(j>=0&&a[j]>v){a[j+1]=a[j];--j;}a[j+1]=v;}
  return c.historyCount?a[c.historyCount/2]:0;
}
void UltrasonicSensor::updateChannelZone(uint8_t i,float d){
  Channel& c=channels_[i]; ObstacleZone n=ObstacleZone::CLEAR;
  if(d<=OBSTACLE_EMERGENCY_CM)n=ObstacleZone::EMERGENCY;else if(d<=OBSTACLE_BLOCKED_CM)n=ObstacleZone::BLOCKED;else if(d<=OBSTACLE_CAUTION_CM)n=ObstacleZone::CAUTION;
  if(c.zone==ObstacleZone::UNKNOWN){c.zone=n;++zoneSequence_;return;}
  if((uint8_t)n>(uint8_t)c.zone){c.fartherConfirmations=0;if(n==ObstacleZone::EMERGENCY||++c.closerConfirmations>=2){c.zone=n;c.closerConfirmations=0;++zoneSequence_;}return;}
  c.closerConfirmations=0;if(n==c.zone){c.fartherConfirmations=0;return;}
  float release=OBSTACLE_CAUTION_CM+OBSTACLE_HYSTERESIS_CM;if(c.zone==ObstacleZone::EMERGENCY)release=OBSTACLE_EMERGENCY_CM+OBSTACLE_HYSTERESIS_CM;else if(c.zone==ObstacleZone::BLOCKED)release=OBSTACLE_BLOCKED_CM+OBSTACLE_HYSTERESIS_CM;
  if(d>=release&&++c.fartherConfirmations>=3){c.zone=n;c.fartherConfirmations=0;++zoneSequence_;}
}
void UltrasonicSensor::acceptPulse(uint8_t i,uint32_t pulse,uint32_t now){
  Channel& c=channels_[i]; const float d=pulse*0.01715f;if(d<ULTRASONIC_MIN_CM||d>ULTRASONIC_MAX_CM){
    ++c.invalidCount;
    // Treat an isolated out-of-range pulse like a dropped sample. A recent
    // valid Echo remains authoritative for a bounded time; a persistent fault
    // is still promoted to INVALID/TIMEOUT by acceptTimeout().
    acceptTimeout(i,now,false);
    return;
  }
  c.echoValid=true;c.health=SensorHealth::HEALTHY;c.lastValidEchoMs=now;
  c.noEchoFar=false;
  c.consecutiveTimeouts=0;c.lastMeasurementMs=now;++c.measurementSequence;c.rawDistanceCm=d;
  // A single large jump is commonly an acoustic cross-reflection from the
  // other front-facing SR04. Keep the last stable range until the new range
  // repeats. A close reading is never suppressed by this filter because the
  // safety zone must react quickly to a real nearby obstacle.
  const float historyMedian=c.historyCount?medianHistory(c):d;
  const bool largeJump=c.filterReady&&c.historyCount>=3&&
                       fabsf(d-historyMedian)>ULTRASONIC_OUTLIER_REJECT_DELTA_CM;
  if(largeJump&&d>OBSTACLE_CAUTION_CM){
    if(c.outlierConfirmations!=0U&&
       fabsf(d-c.outlierCandidateCm)<=ULTRASONIC_OUTLIER_CONFIRM_TOLERANCE_CM){
      ++c.outlierConfirmations;
    }else{
      c.outlierCandidateCm=d;
      c.outlierConfirmations=1;
    }
    if(c.outlierConfirmations<ULTRASONIC_OUTLIER_CONFIRMATIONS){
      // This is a valid electrical pulse, but not yet a trusted range sample.
      // Keep the filtered distance, display state and obstacle zone unchanged.
      return;
    }
    c.outlierConfirmations=0;
    // Keep the previous history when a non-emergency range jump is confirmed.
    // A short-lived second echo must not replace the established range in one
    // step; the normal rolling median will adopt a genuinely persistent new
    // target, while alternating 82/127/145 cm reflections remain rejected.
  }else{
    c.outlierConfirmations=0;
  }
  c.history[c.historyIndex]=d;c.historyIndex=(c.historyIndex+1)%5;if(c.historyCount<5)c.historyCount++;
  const float med=medianHistory(c);if(!c.filterReady){c.filteredDistanceCm=d;c.filterReady=true;}else{const float target=d<c.filteredDistanceCm?d:med;const float alpha=target<c.filteredDistanceCm?0.72f:0.24f;c.filteredDistanceCm+=alpha*(target-c.filteredDistanceCm);}
  // Drive the far/OK presentation from the filtered range, never from the
  // current raw pulse. The hysteresis prevents a single noisy sample near the
  // 100 cm presentation boundary from making the LCD flicker.
  if(!c.displayFar && c.filteredDistanceCm>=ULTRASONIC_DISPLAY_FAR_ENTER_CM) {
    c.displayFar=true;
  }else if(c.displayFar && c.filteredDistanceCm<=ULTRASONIC_DISPLAY_FAR_EXIT_CM) {
    c.displayFar=false;
  }
  static uint32_t prevMs[2]={};static float prevD[2]={};if(prevMs[i]&&now>prevMs[i]){const float rate=(prevD[i]-c.filteredDistanceCm)/(float)(now-prevMs[i])*1000.0f;c.approachRateCmS=constrain(0.30f*rate+0.70f*c.approachRateCmS,-250.0f,250.0f);}prevMs[i]=now;prevD[i]=c.filteredDistanceCm;updateChannelZone(i,c.filteredDistanceCm);
}
void UltrasonicSensor::acceptTimeout(uint8_t i,uint32_t now,bool noEcho){
  Channel& c=channels_[i];
  c.echoValid=false;
  c.lastMeasurementMs=now;
  ++c.measurementSequence;
  ++c.failureCount;
  // A timeout is not a distance measurement. Preserve the last real raw
  // value; replacing it with a synthetic 500 cm sample creates false jumps
  // in filtering and unstable obstacle status.
  c.consecutiveTimeouts=static_cast<uint32_t>(min<uint32_t>(255U,c.consecutiveTimeouts+1U));
  const bool recentValid=hasRecentValidEcho(c,now);
  if(recentValid){
    // Keep the last validated obstacle zone during a transient dropout. This
    // removes UNKNOWN/CLEAR oscillation without allowing a near obstacle to
    // become clear: degradedClearWindow() still rejects near distances.
    c.health=SensorHealth::DEGRADED;
    if(!c.displayFar && c.filteredDistanceCm>=ULTRASONIC_DISPLAY_FAR_ENTER_CM) {
      c.displayFar=true;
    }else if(c.displayFar && c.filteredDistanceCm<=ULTRASONIC_DISPLAY_FAR_EXIT_CM) {
      c.displayFar=false;
    }
    if(noEcho &&
       c.consecutiveTimeouts>=ULTRASONIC_DISPLAY_NO_ECHO_FAR_TIMEOUTS) {
      c.noEchoFar=true;
    }
  }else{
    c.health=noEcho?SensorHealth::TIMEOUT:SensorHealth::INVALID;
    if(!noEcho) c.noEchoFar=false;
    c.displayFar=false;
    c.zone=ObstacleZone::UNKNOWN;
  }
  // A timeout is no measurement, not a synthetic 400 cm sample. Preserve
  // the last real value so a later valid Echo recovers without filter drift.
  if(!c.filterReady){c.filteredDistanceCm=ULTRASONIC_MAX_CM;c.filterReady=true;}
  else c.approachRateCmS*=0.70f;
}
void UltrasonicSensor::recomputeObstacleModel(uint32_t now){
  auto fill=[this,now](const Channel& c,UltrasonicReading& o){
    const bool recentValid=hasRecentValidEcho(c,now);
    const uint32_t validAge=c.lastValidEchoMs!=0U?now-c.lastValidEchoMs:0U;
    o.distanceCm=c.filteredDistanceCm;
    o.rawDistanceCm=c.rawDistanceCm;
    o.valid=c.filterReady;
    o.echoValid=c.echoValid;
    o.displayDistanceValid=(recentValid&&validAge<=ULTRASONIC_DISPLAY_HOLD_MS) ||
                           c.noEchoFar;
    o.displayFar=o.displayDistanceValid&&(c.displayFar||c.noEchoFar);
    o.fresh=c.echoValid&&recentValid&&validAge<=ULTRASONIC_FRESH_MS;
    o.lastUpdateMs=c.lastValidEchoMs;
    o.ageMs=validAge;
    o.rateCmS=c.approachRateCmS;
    o.zone=c.zone;
    o.failureCount=c.failureCount;
    o.health=c.health;
    if(c.lastValidEchoMs!=0U&&!o.fresh&&c.health==SensorHealth::HEALTHY)o.health=SensorHealth::STALE;
  };
  // Keep the presentation and telemetry names tied to the physical modules:
  // PC12/PC9 is the left SR04 and PC4/PC7 is the right SR04.
  fill(channels_[LEFT_MOUNT],frontLeft_);fill(channels_[RIGHT_MOUNT],frontRight_);
  const bool l=hasRecentValidEcho(channels_[LEFT_MOUNT],now)&&frontLeft_.zone!=ObstacleZone::UNKNOWN;
  const bool r=hasRecentValidEcho(channels_[RIGHT_MOUNT],now)&&frontRight_.zone!=ObstacleZone::UNKNOWN;
  overallFresh_=frontLeft_.fresh&&frontRight_.fresh;
  overallEchoValid_=frontLeft_.echoValid&&frontRight_.echoValid;
  if(!frontLeft_.valid||!frontRight_.valid) overallHealth_=SensorHealth::UNKNOWN;
  else if(!l&&!r) overallHealth_=SensorHealth::DISCONNECTED_OR_FAULT;
  else if(!l||!r) overallHealth_=SensorHealth::TIMEOUT;
  else if(frontLeft_.health==SensorHealth::HEALTHY&&frontRight_.health==SensorHealth::HEALTHY) overallHealth_=SensorHealth::HEALTHY;
  else overallHealth_=SensorHealth::DEGRADED;
  // A failed/unknown channel can never produce a CLEAR decision. A recently
  // validated channel may retain its zone during a bounded dropout.
  if(!l||!r){overallZone_=ObstacleZone::UNKNOWN;suggestion_=AvoidanceDirection::STOP;return;}
  nearestDistanceCm_=min(frontLeft_.distanceCm,frontRight_.distanceCm);nearestRawDistanceCm_=min(frontLeft_.rawDistanceCm,frontRight_.rawDistanceCm);nearestRateCmS_=max(frontLeft_.rateCmS,frontRight_.rateCmS);
  if(frontLeft_.zone==ObstacleZone::EMERGENCY||frontRight_.zone==ObstacleZone::EMERGENCY)overallZone_=ObstacleZone::EMERGENCY;else if(frontLeft_.zone==ObstacleZone::BLOCKED||frontRight_.zone==ObstacleZone::BLOCKED)overallZone_=ObstacleZone::BLOCKED;else if(frontLeft_.zone==ObstacleZone::CAUTION||frontRight_.zone==ObstacleZone::CAUTION)overallZone_=ObstacleZone::CAUTION;else overallZone_=ObstacleZone::CLEAR;
  const float delta=frontLeft_.distanceCm-frontRight_.distanceCm;if(overallZone_>=ObstacleZone::EMERGENCY||fabsf(delta)<=AVOID_SIDE_HYSTERESIS_CM)suggestion_=AvoidanceDirection::STOP;else suggestion_=delta<0?AvoidanceDirection::RIGHT:AvoidanceDirection::LEFT;
}
bool UltrasonicSensor::degradedClearWindow(uint32_t nowMs) const{
  for(uint8_t i=0;i<2;++i){
    const Channel& c=channels_[i];
    if(!c.filterReady || c.lastValidEchoMs==0 || nowMs-c.lastValidEchoMs>ULTRASONIC_DEGRADED_GRACE_MS || c.consecutiveTimeouts>ULTRASONIC_DEGRADED_MAX_TIMEOUTS || c.filteredDistanceCm<=ULTRASONIC_DEGRADED_CLEAR_CM) return false;
  }
  return true;
}
bool UltrasonicSensor::hasRecentClearWindow(uint32_t nowMs) const {
  return degradedClearWindow(nowMs);
}
void UltrasonicSensor::update(){
  const uint32_t ms=millis();
  if(activeChannel_==0xFF && (int32_t)(ms-nextTriggerAllowedMs_)>=0) {
    for(uint8_t n=0;n<2;n++) {
      const uint8_t i=(nextChannel_+n)%2;
      Channel& c=channels_[i];
      if(ms-c.lastTriggerMs<ULTRASONIC_SAMPLE_PERIOD_MS) continue;

      // Check only the channel about to be triggered. A disconnected or
      // floating Echo input must not globally block the healthy SR04 and make
      // both LCD fields expire to ----. The 45 ms inter-sensor guard still
      // separates acoustic bursts; a channel whose own Echo is stuck HIGH is
      // skipped and retried on a later round.
      if(digitalRead(c.echoPin)==HIGH) continue;

      // Clear a previous capture before selecting the channel.  TRIG and the
      // start of Echo capture are kept in one call; a delayed second loop
      // could otherwise lose a close Echo pulse while still in TRIGGER_HIGH.
      noInterrupts();
      c.echoPulseReady=false;
      c.echoRiseUs=0;
      activeChannel_=i;
      c.waitEchoStartedUs=micros();
      c.state=TriggerState::WAIT_ECHO;
      interrupts();

      // Arm Echo before raising TRIG. This makes the capture invariant true
      // for both modules, including a very close target whose Echo can begin
      // immediately after the trigger pulse.
      ++c.triggerCount;
      nextChannel_=(i+1)%2;
      c.lastTriggerMs=ms;
      digitalWrite(c.trigPin,HIGH);
      delayMicroseconds(12);
      digitalWrite(c.trigPin,LOW);
      break;
    }
  } else if(activeChannel_!=0xFF) {
    const uint8_t i=activeChannel_;
    Channel& c=channels_[i];
    uint32_t pulse=0;
    bool ready=false;
    noInterrupts();
    if(c.echoPulseReady){
      pulse=c.echoPulseUs;
      c.echoPulseReady=false;
      ready=true;
    }
    interrupts();
    const uint32_t us=micros();
    const bool timedOut=(us-c.waitEchoStartedUs)>=ULTRASONIC_ECHO_TIMEOUT_US;
    if(ready){
      acceptPulse(i,pulse,millis());
      ++measurementSequence_;
    }else if(timedOut){
      acceptTimeout(i,millis());
      ++measurementSequence_;
    }
    if(ready||timedOut){
      c.state=TriggerState::IDLE;
      activeChannel_=0xFF;
      nextTriggerAllowedMs_=millis()+ULTRASONIC_INTER_SENSOR_GUARD_MS;
    }
  }
  recomputeObstacleModel(millis());
}
bool UltrasonicSensor::hardBlocked()const{return overallZone_==ObstacleZone::BLOCKED||overallZone_==ObstacleZone::EMERGENCY;}
float UltrasonicSensor::stoppingDistanceCm(int16_t cmd)const{const float speed=abs(cmd)*OBSTACLE_STOP_PER_COMMAND_CM;const float approach=max(0.0f,nearestRateCmS_)*OBSTACLE_APPROACH_LOOKAHEAD_S;return constrain(OBSTACLE_STOP_BASE_CM+speed+approach,OBSTACLE_EMERGENCY_CM,OBSTACLE_CAUTION_CM);}
int16_t UltrasonicSensor::limitForwardCommand(int16_t cmd)const{
  if(cmd<=0)return cmd;
  if(!overallFresh_||!healthy()){
    // Do not turn a brief, known-clear echo loss into an immediate mission
    // abort.  This path is deliberately capped and expires quickly; a
    // missing startup reading, close/stale reading, or repeated timeout still
    // fails closed at zero.
    if(!degradedClearWindow(millis()))return 0;
    const float degradedNearest=min(channels_[LEFT_MOUNT].filteredDistanceCm,channels_[RIGHT_MOUNT].filteredDistanceCm);
    if(degradedNearest<=stoppingDistanceCm(cmd) || degradedNearest<=OBSTACLE_CAUTION_CM)return 0;
    return min(cmd,ULTRASONIC_DEGRADED_MAX_FORWARD_COMMAND);
  }
  if(nearestDistanceCm_<=stoppingDistanceCm(cmd)||hardBlocked())return 0;
  if(nearestDistanceCm_>=stoppingDistanceCm(cmd)+OBSTACLE_SLOW_BAND_CM)return cmd;
  int16_t limited=(int16_t)lroundf(cmd*constrain((nearestDistanceCm_-stoppingDistanceCm(cmd))/OBSTACLE_SLOW_BAND_CM,0.0f,1.0f));
  if(limited>0&&limited<OBSTACLE_MIN_FORWARD_COMMAND)limited=OBSTACLE_MIN_FORWARD_COMMAND;
  return min(cmd,limited);
}
const char* UltrasonicSensor::zoneText(ObstacleZone z){switch(z){case ObstacleZone::CLEAR:return "CLEAR";case ObstacleZone::CAUTION:return "CAUTION";case ObstacleZone::BLOCKED:return "BLOCKED";case ObstacleZone::EMERGENCY:return "EMERGENCY";default:return "UNKNOWN";}}
const char* UltrasonicSensor::healthText(SensorHealth h){switch(h){case SensorHealth::HEALTHY:return "HEALTHY";case SensorHealth::STALE:return "STALE";case SensorHealth::TIMEOUT:return "TIMEOUT";case SensorHealth::INVALID:return "INVALID";case SensorHealth::DISCONNECTED_OR_FAULT:return "DISCONNECTED_OR_FAULT";case SensorHealth::DEGRADED:return "DEGRADED";default:return "UNKNOWN";}}
const char* UltrasonicSensor::avoidanceText(AvoidanceDirection d){switch(d){case AvoidanceDirection::LEFT:return "LEFT";case AvoidanceDirection::RIGHT:return "RIGHT";case AvoidanceDirection::STOP:return "STOP";default:return "NONE";}}
