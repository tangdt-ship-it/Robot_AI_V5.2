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
  for (uint8_t i=0; i<2; ++i) { Channel& c=channels_[i]; pinMode(c.trigPin,OUTPUT); digitalWrite(c.trigPin,LOW); pinMode(c.echoPin,INPUT_PULLDOWN); c.lastTriggerMs=now-ULTRASONIC_SAMPLE_PERIOD_MS; }
  attachInterrupt(digitalPinToInterrupt(channels_[LEFT_MOUNT].echoPin),echoIsrMountLeft,CHANGE);
  attachInterrupt(digitalPinToInterrupt(channels_[RIGHT_MOUNT].echoPin),echoIsrMountRight,CHANGE);
  nextTriggerAllowedMs_=now;
}
void UltrasonicSensor::echoIsrMountLeft(){if(instance_)instance_->handleEchoEdge(LEFT_MOUNT);}
void UltrasonicSensor::echoIsrMountRight(){if(instance_)instance_->handleEchoEdge(RIGHT_MOUNT);}
void UltrasonicSensor::handleEchoEdge(uint8_t i){
  Channel& c=channels_[i]; const uint32_t us=micros();
  if (activeChannel_ != i || c.state != TriggerState::WAIT_ECHO) return;
  if(digitalRead(c.echoPin)==HIGH)c.echoRiseUs=us;
  else if(c.echoRiseUs){c.echoPulseUs=us-c.echoRiseUs;c.echoRiseUs=0;c.echoPulseReady=true;}
}
bool UltrasonicSensor::channelFresh(const Channel& c,uint32_t now)const{return c.measurementSequence && now-c.lastMeasurementMs<=ULTRASONIC_FRESH_MS;}
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
    c.echoValid=false; ++c.invalidCount; c.health=SensorHealth::INVALID;
    acceptTimeout(i,now); c.health=SensorHealth::INVALID; return;
  }
  c.echoValid=true;c.health=SensorHealth::HEALTHY;c.lastValidEchoMs=now;
  c.consecutiveTimeouts=0;c.lastMeasurementMs=now;++c.measurementSequence;c.rawDistanceCm=d;c.history[c.historyIndex]=d;c.historyIndex=(c.historyIndex+1)%5;if(c.historyCount<5)c.historyCount++;
  const float med=medianHistory(c);if(!c.filterReady){c.filteredDistanceCm=d;c.filterReady=true;}else{const float target=d<c.filteredDistanceCm?d:med;const float alpha=target<c.filteredDistanceCm?0.72f:0.24f;c.filteredDistanceCm+=alpha*(target-c.filteredDistanceCm);}
  static uint32_t prevMs[2]={};static float prevD[2]={};if(prevMs[i]&&now>prevMs[i]){const float rate=(prevD[i]-c.filteredDistanceCm)/(float)(now-prevMs[i])*1000.0f;c.approachRateCmS=constrain(0.30f*rate+0.70f*c.approachRateCmS,-250.0f,250.0f);}prevMs[i]=now;prevD[i]=c.filteredDistanceCm;updateChannelZone(i,c.filteredDistanceCm);
}
void UltrasonicSensor::acceptTimeout(uint8_t i,uint32_t now){
  Channel& c=channels_[i];c.echoValid=false;c.health=SensorHealth::TIMEOUT;c.lastMeasurementMs=now;++c.measurementSequence;++c.failureCount;++c.consecutiveTimeouts;c.rawDistanceCm=ULTRASONIC_MAX_CM;
  if(!c.filterReady){c.filteredDistanceCm=ULTRASONIC_MAX_CM;c.filterReady=true;}else{c.history[c.historyIndex]=ULTRASONIC_MAX_CM;c.historyIndex=(c.historyIndex+1)%5;if(c.historyCount<5)c.historyCount++;c.filteredDistanceCm+=0.24f*(medianHistory(c)-c.filteredDistanceCm);c.approachRateCmS*=0.70f;}c.zone=ObstacleZone::UNKNOWN;
}
void UltrasonicSensor::recomputeObstacleModel(uint32_t now){
  auto fill=[now](const Channel& c,UltrasonicReading& o){o.distanceCm=c.filteredDistanceCm;o.rawDistanceCm=c.rawDistanceCm;o.valid=c.filterReady;o.echoValid=c.echoValid;o.fresh=c.measurementSequence&&now-c.lastMeasurementMs<=ULTRASONIC_FRESH_MS;o.lastUpdateMs=c.lastMeasurementMs;o.ageMs=c.measurementSequence?now-c.lastMeasurementMs:0;o.rateCmS=c.approachRateCmS;o.zone=c.zone;o.failureCount=c.failureCount;o.health=c.health;if(c.measurementSequence && !o.fresh)o.health=SensorHealth::STALE;};
  // Mount-left looks right; mount-right looks left.
  fill(channels_[RIGHT_MOUNT],frontLeft_);fill(channels_[LEFT_MOUNT],frontRight_);
  const bool l=frontLeft_.valid&&frontLeft_.fresh,r=frontRight_.valid&&frontRight_.fresh;overallFresh_=l&&r;overallEchoValid_=l&&r&&frontLeft_.echoValid&&frontRight_.echoValid;
  if(!frontLeft_.valid || !frontRight_.valid) overallHealth_=SensorHealth::UNKNOWN;
  else if(!frontLeft_.fresh || !frontRight_.fresh) overallHealth_=SensorHealth::STALE;
  else if(!frontLeft_.echoValid && !frontRight_.echoValid) overallHealth_=SensorHealth::DISCONNECTED_OR_FAULT;
  else if(!frontLeft_.echoValid || !frontRight_.echoValid) overallHealth_=SensorHealth::DEGRADED;
  else if(frontLeft_.health==SensorHealth::INVALID || frontRight_.health==SensorHealth::INVALID) overallHealth_=SensorHealth::INVALID;
  else overallHealth_=SensorHealth::HEALTHY;
  // A failed/unknown channel can never produce a CLEAR decision.
  if(!l||!r||!healthy()){overallZone_=ObstacleZone::UNKNOWN;suggestion_=AvoidanceDirection::STOP;return;}
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
void UltrasonicSensor::update(){
  const uint32_t ms=millis(),us=micros();
  if(activeChannel_==0xFF){if((int32_t)(ms-nextTriggerAllowedMs_)>=0)for(uint8_t n=0;n<2;n++){uint8_t i=(nextChannel_+n)%2;if(ms-channels_[i].lastTriggerMs>=ULTRASONIC_SAMPLE_PERIOD_MS){activeChannel_=i;nextChannel_=(i+1)%2;Channel& c=channels_[i];digitalWrite(c.trigPin,HIGH);c.triggerStartedUs=us;c.lastTriggerMs=ms;c.state=TriggerState::TRIGGER_HIGH;break;}}}
  else {Channel& c=channels_[activeChannel_];if(c.state==TriggerState::TRIGGER_HIGH&&us-c.triggerStartedUs>=12){digitalWrite(c.trigPin,LOW);noInterrupts();c.echoPulseReady=false;c.echoRiseUs=0;interrupts();c.waitEchoStartedUs=us;c.state=TriggerState::WAIT_ECHO;}else if(c.state==TriggerState::WAIT_ECHO){uint32_t pulse=0;bool ready=false;noInterrupts();if(c.echoPulseReady){pulse=c.echoPulseUs;c.echoPulseReady=false;ready=true;}interrupts();if(ready){acceptPulse(activeChannel_,pulse,ms);++measurementSequence_;}else if(us-c.waitEchoStartedUs>=ULTRASONIC_ECHO_TIMEOUT_US){acceptTimeout(activeChannel_,ms);++measurementSequence_;}if(ready||us-c.waitEchoStartedUs>=ULTRASONIC_ECHO_TIMEOUT_US){c.state=TriggerState::IDLE;activeChannel_=0xFF;nextTriggerAllowedMs_=ms+ULTRASONIC_INTER_SENSOR_GUARD_MS;}}}
  recomputeObstacleModel(ms);
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
