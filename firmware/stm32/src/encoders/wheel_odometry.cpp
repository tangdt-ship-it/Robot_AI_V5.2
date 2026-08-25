#include <encoders/wheel_odometry.h>
#include <robot_config.h>

#include <math.h>

namespace {
bool configureEncoder(TIM_HandleTypeDef& timer, TIM_TypeDef* instance) {
  timer.Instance = instance;
  timer.Init.Prescaler = 0;
  timer.Init.CounterMode = TIM_COUNTERMODE_UP;
  timer.Init.Period = 0xFFFFU;
  timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  TIM_Encoder_InitTypeDef config = {};
  config.EncoderMode = TIM_ENCODERMODE_TI12;
  config.IC1Polarity = TIM_ICPOLARITY_RISING;
  config.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  config.IC1Prescaler = TIM_ICPSC_DIV1;
  config.IC1Filter = 0;
  config.IC2Polarity = TIM_ICPOLARITY_RISING;
  config.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  config.IC2Prescaler = TIM_ICPSC_DIV1;
  config.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&timer, &config) != HAL_OK) return false;
  return HAL_TIM_Encoder_Start(&timer, TIM_CHANNEL_ALL) == HAL_OK;
}
constexpr float kDegToRad = 0.017453292519943295f;
constexpr float kPi = 3.14159265358979323846f;
}  // namespace

bool WheelOdometry::begin() {
  initialized_ = false;
  health_ = EncoderHealth::INIT_FAILED;
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  GPIO_InitTypeDef gpio = {};
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &gpio);

  if (!configureEncoder(leftTimer_, TIM2)) return false;
  if (!configureEncoder(rightTimer_, TIM3)) {
    HAL_TIM_Encoder_Stop(&leftTimer_, TIM_CHANNEL_ALL);
    return false;
  }
  __HAL_TIM_SET_COUNTER(&leftTimer_, 0);
  __HAL_TIM_SET_COUNTER(&rightTimer_, 0);
  initialized_ = true;
  health_ = EncoderHealth::OK;
  reset();
  return true;
}

int32_t WheelOdometry::counterDelta(uint32_t current, uint32_t previous) {
  return static_cast<int16_t>(static_cast<uint16_t>(current - previous));
}

float WheelOdometry::normalizeRad(float radians) {
  while (radians > kPi) radians -= 2.0f * kPi;
  while (radians <= -kPi) radians += 2.0f * kPi;
  return radians;
}

void WheelOdometry::updateStallHealth(uint32_t nowMs, int32_t leftDelta,
                                      int32_t rightDelta,
                                      int16_t leftCommand,
                                      int16_t rightCommand) {
  auto stalled = [nowMs](int16_t command, int32_t delta,
                         uint32_t& commandStartMs,
                         uint32_t& lastMotionMs) {
    if (abs(command) < ENCODER_STALL_COMMAND_THRESHOLD) {
      commandStartMs = 0;
      lastMotionMs = nowMs;
      return false;
    }
    if (commandStartMs == 0U) {
      commandStartMs = nowMs;
      lastMotionMs = nowMs;
    }
    if (abs(delta) >= ENCODER_STALL_MIN_TICKS) lastMotionMs = nowMs;
    const uint32_t reference = lastMotionMs > commandStartMs
                                   ? lastMotionMs
                                   : commandStartMs;
    return (nowMs - reference) > ENCODER_STALL_TIMEOUT_MS;
  };

  const bool leftStall = stalled(leftCommand, leftDelta, leftCommandStartMs_,
                                 leftLastMotionMs_);
  const bool rightStall = stalled(rightCommand, rightDelta,
                                  rightCommandStartMs_, rightLastMotionMs_);
  if (leftStall && rightStall) health_ = EncoderHealth::BOTH_STALL;
  else if (leftStall) health_ = EncoderHealth::LEFT_STALL;
  else if (rightStall) health_ = EncoderHealth::RIGHT_STALL;
  else health_ = EncoderHealth::OK;
}

void WheelOdometry::update(int16_t leftCommand, int16_t rightCommand) {
  if (!initialized_) return;
  const uint32_t nowMs = millis();
  const uint32_t leftCount = __HAL_TIM_GET_COUNTER(&leftTimer_);
  const uint32_t rightCount = __HAL_TIM_GET_COUNTER(&rightTimer_);
  int32_t leftDelta = counterDelta(leftCount, leftPreviousCount_);
  int32_t rightDelta = counterDelta(rightCount, rightPreviousCount_);
  leftPreviousCount_ = leftCount;
  rightPreviousCount_ = rightCount;
  if (ENCODER_LEFT_REVERSED) leftDelta = -leftDelta;
  if (ENCODER_RIGHT_REVERSED) rightDelta = -rightDelta;

  const uint32_t dtMs = lastUpdateMs_ == 0U ? 0U : nowMs - lastUpdateMs_;
  lastUpdateMs_ = nowMs;
  if (dtMs > 0U && dtMs <= 250U) {
    const float scale = 1000.0f / static_cast<float>(dtMs);
    const float leftInstant = static_cast<float>(leftDelta) *
                              ENCODER_LEFT_MM_PER_TICK * scale;
    const float rightInstant = static_cast<float>(rightDelta) *
                               ENCODER_RIGHT_MM_PER_TICK * scale;
    data_.leftVelocityMmS += ENCODER_VELOCITY_FILTER_ALPHA *
                            (leftInstant - data_.leftVelocityMmS);
    data_.rightVelocityMmS += ENCODER_VELOCITY_FILTER_ALPHA *
                             (rightInstant - data_.rightVelocityMmS);
    data_.linearVelocityMmS =
        (data_.leftVelocityMmS + data_.rightVelocityMmS) * 0.5f;
    data_.angularVelocityRadS =
        (data_.rightVelocityMmS - data_.leftVelocityMmS) / WHEEL_TRACK_MM;
  }

  updateStallHealth(nowMs, leftDelta, rightDelta, leftCommand, rightCommand);
  if (leftDelta == 0 && rightDelta == 0) return;

  data_.leftTicks += leftDelta;
  data_.rightTicks += rightDelta;
  const float leftMm = static_cast<float>(leftDelta) * ENCODER_LEFT_MM_PER_TICK;
  const float rightMm = static_cast<float>(rightDelta) * ENCODER_RIGHT_MM_PER_TICK;
  data_.leftDistanceMm += leftMm;
  data_.rightDistanceMm += rightMm;
  const float travelMm = (leftMm + rightMm) * 0.5f;
  const float headingDelta = (rightMm - leftMm) / WHEEL_TRACK_MM;
  data_.encoderHeadingRad = normalizeRad(data_.encoderHeadingRad + headingDelta);
  data_.distanceMm += travelMm;
  pendingTravelMm_ += travelMm;
}

void WheelOdometry::integratePose(float headingDeg, bool externalHeadingValid) {
  const float newHeading = externalHeadingValid
                               ? normalizeRad(headingDeg * kDegToRad)
                               : data_.encoderHeadingRad;
  const float headingDelta = normalizeRad(newHeading - data_.headingRad);
  if (pendingTravelMm_ != 0.0f) {
    const float midpoint = normalizeRad(data_.headingRad + headingDelta * 0.5f);
    data_.xMm += pendingTravelMm_ * cosf(midpoint);
    data_.yMm += pendingTravelMm_ * sinf(midpoint);
    pendingTravelMm_ = 0.0f;
  }
  data_.headingRad = newHeading;
}

void WheelOdometry::reset() {
  if (initialized_) {
    leftPreviousCount_ = __HAL_TIM_GET_COUNTER(&leftTimer_);
    rightPreviousCount_ = __HAL_TIM_GET_COUNTER(&rightTimer_);
  } else {
    leftPreviousCount_ = rightPreviousCount_ = 0;
  }
  data_ = {};
  pendingTravelMm_ = 0.0f;
  lastUpdateMs_ = millis();
  leftCommandStartMs_ = rightCommandStartMs_ = 0;
  leftLastMotionMs_ = rightLastMotionMs_ = lastUpdateMs_;
  if (initialized_) health_ = EncoderHealth::OK;
  ++resetGeneration_;
  lastResetReason_ = EncoderResetReason::BOOT;
}

void WheelOdometry::resetWheelCounts(EncoderResetReason reason) {
  if (!initialized_) return;

  // Rebase the hardware counters and software totals together. Navigation
  // pose and encoderHeadingRad deliberately remain unchanged: R2 is an
  // operator counter-zero command, not an odometry/home reset.
  __HAL_TIM_SET_COUNTER(&leftTimer_, 0);
  __HAL_TIM_SET_COUNTER(&rightTimer_, 0);
  leftPreviousCount_ = 0;
  rightPreviousCount_ = 0;
  data_.leftTicks = 0;
  data_.rightTicks = 0;
  data_.leftVelocityMmS = 0.0f;
  data_.rightVelocityMmS = 0.0f;
  data_.linearVelocityMmS = 0.0f;
  data_.angularVelocityRadS = 0.0f;

  const uint32_t nowMs = millis();
  lastUpdateMs_ = nowMs;
  leftCommandStartMs_ = rightCommandStartMs_ = 0;
  leftLastMotionMs_ = rightLastMotionMs_ = nowMs;
  health_ = EncoderHealth::OK;
  ++resetGeneration_;
  lastResetReason_ = reason;
}

const char* WheelOdometry::resetReasonText(EncoderResetReason reason) {
  switch (reason) {
    case EncoderResetReason::BOOT: return "BOOT";
    case EncoderResetReason::PS2_R2: return "PS2_R2";
    case EncoderResetReason::ROBOTLINK: return "ROBOTLINK";
    case EncoderResetReason::UNKNOWN: return "UNKNOWN";
  }
  return "UNKNOWN";
}

const char* WheelOdometry::healthText() const {
  switch (health_) {
    case EncoderHealth::DISABLED: return "DISABLED";
    case EncoderHealth::OK: return "OK";
    case EncoderHealth::INIT_FAILED: return "INIT_FAILED";
    case EncoderHealth::LEFT_STALL: return "LEFT_STALL";
    case EncoderHealth::RIGHT_STALL: return "RIGHT_STALL";
    case EncoderHealth::BOTH_STALL: return "BOTH_STALL";
  }
  return "UNKNOWN";
}
