#include <display/lcd_display.h>
#include <robot_config.h>
#include <stdio.h>
#include <string.h>

LcdDisplay::SoftI2C::SoftI2C(uint32_t sclPin, uint32_t sdaPin)
    : sclPin_(sclPin), sdaPin_(sdaPin) {}

void LcdDisplay::SoftI2C::lowScl() { pinMode(sclPin_, OUTPUT); digitalWrite(sclPin_, LOW); }
void LcdDisplay::SoftI2C::lowSda() { pinMode(sdaPin_, OUTPUT); digitalWrite(sdaPin_, LOW); }
void LcdDisplay::SoftI2C::releaseScl() { pinMode(sclPin_, INPUT); }
void LcdDisplay::SoftI2C::releaseSda() { pinMode(sdaPin_, INPUT); }

void LcdDisplay::SoftI2C::waitHigh(uint32_t pin) {
  const uint32_t startUs = micros();
  while (digitalRead(pin) == LOW && (micros() - startUs) < 2000U) {}
}

void LcdDisplay::SoftI2C::start() {
  releaseSda(); releaseScl(); waitHigh(sclPin_); delayMicroseconds(8);
  lowSda(); delayMicroseconds(8); lowScl();
}
void LcdDisplay::SoftI2C::stop() {
  lowSda(); delayMicroseconds(8); releaseScl(); waitHigh(sclPin_);
  delayMicroseconds(8); releaseSda(); delayMicroseconds(8);
}

bool LcdDisplay::SoftI2C::writeRaw(uint8_t value) {
  for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
    if ((value & mask) != 0) releaseSda(); else lowSda();
    delayMicroseconds(5); releaseScl(); waitHigh(sclPin_);
    delayMicroseconds(8); lowScl(); delayMicroseconds(5);
  }
  releaseSda(); delayMicroseconds(5); releaseScl(); waitHigh(sclPin_);
  delayMicroseconds(8);
  const bool ack = digitalRead(sdaPin_) == LOW;
  lowScl(); delayMicroseconds(5);
  return ack;
}

void LcdDisplay::SoftI2C::begin() {
  releaseSda(); releaseScl(); delay(10);
  for (uint8_t i = 0; i < 9 && digitalRead(sdaPin_) == LOW; ++i) {
    lowScl(); delayMicroseconds(10); releaseScl(); waitHigh(sclPin_);
    delayMicroseconds(10);
  }
  stop();
}

bool LcdDisplay::SoftI2C::probe(uint8_t address) {
  start(); const bool ack = writeRaw(static_cast<uint8_t>(address << 1)); stop();
  return ack;
}

bool LcdDisplay::SoftI2C::writeByte(uint8_t address, uint8_t value) {
  start();
  const bool addressAck = writeRaw(static_cast<uint8_t>(address << 1));
  const bool dataAck = writeRaw(value);
  stop();
  return addressAck && dataAck;
}

LcdDisplay::LcdDisplay(uint32_t sclPin, uint32_t sdaPin, uint8_t address)
    : bus_(sclPin, sdaPin), address_(address) {}

void LcdDisplay::expanderWrite(uint8_t value) {
  bus_.writeByte(address_, static_cast<uint8_t>(value | 0x08));
}
void LcdDisplay::pulseEnable(uint8_t value) {
  expanderWrite(static_cast<uint8_t>(value | 0x04));
  delayMicroseconds(2);
  expanderWrite(static_cast<uint8_t>(value & ~0x04));
  delayMicroseconds(60);
}
void LcdDisplay::write4(uint8_t value) { expanderWrite(value); pulseEnable(value); }
void LcdDisplay::send(uint8_t value, uint8_t rs) {
  write4(static_cast<uint8_t>((value & 0xF0) | rs));
  write4(static_cast<uint8_t>(((value << 4) & 0xF0) | rs));
}
void LcdDisplay::command(uint8_t value) { send(value, 0); }
void LcdDisplay::clearHardware() { command(0x01); delay(3); }
void LcdDisplay::setCursor(uint8_t column, uint8_t row) {
  static constexpr uint8_t offsets[4] = {0x00, 0x40, 0x14, 0x54};
  command(static_cast<uint8_t>(0x80 | (column + offsets[row & 0x03])));
}

bool LcdDisplay::begin() {
  bus_.begin();
  if (!bus_.probe(address_)) return false;
  delay(60); expanderWrite(0); delay(100);
  write4(0x30); delayMicroseconds(4500);
  write4(0x30); delayMicroseconds(4500);
  write4(0x30); delayMicroseconds(200);
  write4(0x20);
  command(0x28); command(0x08); clearHardware(); command(0x06); command(0x0C);
  available_ = true;
  page_ = LcdPage::ROBOT;
  mapSlot_ = 1;
  forceRefresh();
  return true;
}

void LcdDisplay::padLine(char (&line)[21]) {
  size_t length = strnlen(line, 20);
  while (length < 20) line[length++] = ' ';
  line[20] = '\0';
}

void LcdDisplay::setData(const LcdDisplayData& data) { data_ = data; }

void LcdDisplay::setPage(LcdPage page) {
  if (page_ == page) return;
  page_ = page;
  forceRefresh();
}

void LcdDisplay::togglePage() {
  setPage(page_ == LcdPage::ROBOT ? LcdPage::MAP : LcdPage::ROBOT);
}

void LcdDisplay::setMapSlot(uint8_t slot) {
  const uint8_t normalized = slot == 2U ? 2U : 1U;
  if (mapSlot_ == normalized) return;
  mapSlot_ = normalized;
  forceRefresh();
}

void LcdDisplay::toggleMapSlot() {
  setMapSlot(mapSlot_ == 1U ? 2U : 1U);
}

void LcdDisplay::setMapStatus(uint8_t slot, uint8_t storeState, uint8_t mode,
                              uint16_t points, uint16_t maxPoints,
                              uint32_t lengthMm, uint16_t replayWp,
                              uint16_t replayTotal, uint32_t replayTargetMm,
                              uint32_t replayTravelMm,
                              uint32_t replayErrorMm,
                              uint8_t replayOperation, uint8_t routeType,
                              uint8_t replayMode, uint8_t holdReason,
                              int16_t replayTargetDeg,
                              uint32_t replayLapCounter,
                              uint32_t closeCandidateDistanceMm,
                              int16_t closeCandidateHeadingDeg) {
  const uint8_t normalized = slot == 2U ? 2U : 1U;
  LcdMapStatus& status = mapStatus_[normalized - 1U];
  status.valid = true;
  status.storeState = storeState;
  status.mode = mode;
  status.points = points;
  status.maxPoints = maxPoints == 0U ? 128U : maxPoints;
  status.lengthMm = lengthMm;
  status.replayWp = replayWp;
  status.replayTotal = replayTotal;
  status.replayTargetMm = replayTargetMm;
  status.replayTravelMm = replayTravelMm;
  status.replayErrorMm = replayErrorMm;
  status.replayOperation = replayOperation;
  status.holdReason = holdReason;
  status.routeType = routeType <= 1U ? routeType : 0U;
  status.replayMode = replayMode <= 3U ? replayMode : 0U;
  status.replayTargetDeg = replayTargetDeg;
  status.replayLapCounter = replayLapCounter;
  status.closeCandidateDistanceMm = closeCandidateDistanceMm;
  status.closeCandidateHeadingDeg = closeCandidateHeadingDeg;
  if (mapSlot_ == normalized) forceRefresh();
}

void LcdDisplay::buildRobotLines() {
  const int headingDeg = static_cast<int>(lroundf(data_.headingDeg));
  const char* mode = data_.controlMode != nullptr ? data_.controlMode : "AI";
  const bool leftDisplayValid = data_.frontLeftDisplayDistanceValid;
  const bool rightDisplayValid = data_.frontRightDisplayDistanceValid;
  char leftDistance[5] = "----";
  char rightDistance[5] = "----";
  auto formatDistance = [](char (&output)[5], bool displayValid, bool displayFar,
                           float distanceCm) {
    if (!displayValid) return;
    if (displayFar) {
      snprintf(output, sizeof(output), " OK ");
      return;
    }
    // Show useful numeric data for a nearby object. At or beyond 100 cm the
    // display stays compact; this does not change the SR04 safety thresholds.
    if (distanceCm >= 100.0f) {
      snprintf(output, sizeof(output), " OK ");
    } else {
      // Numeric distances are at most two digits. Left-align the compact
      // value in its fixed 4-column slot so there are no leading blanks;
      // ---- remains four characters for a sensor fault.
      snprintf(output, sizeof(output), "%d",
               static_cast<int>(lroundf(distanceCm)));
    }
  };
  formatDistance(leftDistance, leftDisplayValid, data_.frontLeftDisplayFar,
                 data_.frontLeftDistanceCm);
  formatDistance(rightDistance, rightDisplayValid, data_.frontRightDisplayFar,
                 data_.frontRightDistanceCm);

  // Fixed 20-column main page. Keep the two SR04 channels independent:
  // nearby values are numeric, confirmed far/held-far is OK, and an expired
  // value is ----. The display hold never changes the fail-safe motion gate.
  // Exact RF pair state is not exposed by the PS2 receiver. Restore the
  // previous truthful presentation: show the PS2 receiver state rather than
  // claiming that MODEL=PS2 means the handheld is paired.
  (void)mode;
  snprintf(desired_[0], 21, "HDG:%4d PS2:%-4.4s", headingDeg,
           data_.ps2Status != nullptr ? data_.ps2Status : "WAIT");
  snprintf(desired_[1], 21, "SPD:%3d L:%3d R:%3d", data_.speed, data_.left,
           data_.right);
  if (data_.brakeLocked) {
    // 20 columns exactly; BRK is only present while the active brake is on.
    snprintf(desired_[2], 21, "L04:%-4s R04:%-4sBRK", leftDistance,
             rightDistance);
  } else {
    snprintf(desired_[2], 21, "L04:%-4s R04:%-4s", leftDistance,
             rightDistance);
  }
  snprintf(desired_[3], 21, "ENL:%5ld ENR:%5ld",
           static_cast<long>(data_.leftEncoderTicks),
           static_cast<long>(data_.rightEncoderTicks));
}

void LcdDisplay::buildMapLines() {
  const LcdMapStatus& status = mapStatus_[mapSlot_ - 1U];
  const char* routeType = status.routeType == 1U ? "CLOSED" : "OPEN";
  const char* replayMode = status.replayMode == 1U ? "LOOP" :
                           status.replayMode == 2U ? "RETURN" :
                           status.replayMode == 3U ? "PING" : "ONCE";
  if (!status.valid) {
    snprintf(desired_[0], 21, "MAP%u EMPTY", mapSlot_);
    snprintf(desired_[1], 21, "PTS:000/128 L:0.0m");
    snprintf(desired_[2], 21, "TRI TEACH");
    snprintf(desired_[3], 21, "SEL MAP L3 EXIT");
    return;
  }

  const uint32_t lengthTenthsM = (status.lengthMm + 50U) / 100U;
  const unsigned long wholeM = static_cast<unsigned long>(lengthTenthsM / 10U);
  const unsigned long tenthM = static_cast<unsigned long>(lengthTenthsM % 10U);

  if (status.mode == 9U) {
    snprintf(desired_[0], 21, "MAP%u FINISH", mapSlot_);
    snprintf(desired_[1], 21, "PTS:%03u C:%lumm",
             static_cast<unsigned>(status.points),
             static_cast<unsigned long>(status.closeCandidateDistanceMm));
    snprintf(desired_[2], 21, "X=OPEN");
    snprintf(desired_[3], 21, "O=CLOSED");
    return;
  }
  if (status.mode == 4U) {
    snprintf(desired_[0], 21, "MAP%u CHECK %s", mapSlot_, routeType);
    snprintf(desired_[1], 21, "WP:%02u/%02u L:%lu.%lum",
             static_cast<unsigned>(status.replayWp),
             static_cast<unsigned>(status.replayTotal), wholeM, tenthM);
    snprintf(desired_[2], 21, "MODE:%s", replayMode);
    snprintf(desired_[3], 21, "START RUN X CANCEL");
    return;
  }
  if (status.mode == 5U) {
    snprintf(desired_[0], 21, "MAP%u CHECK PASS", mapSlot_);
    snprintf(desired_[1], 21, "WP:%02u/%02u T:%lumm",
             static_cast<unsigned>(status.replayWp),
             static_cast<unsigned>(status.replayTotal),
             static_cast<unsigned long>(status.replayTargetMm));
    snprintf(desired_[2], 21, "MODE:%s", replayMode);
    snprintf(desired_[3], 21, "START GO X CANCEL");
    return;
  }
  if (status.mode == 6U) {
    if (status.replayMode == 1U) {
      snprintf(desired_[0], 21, "MAP%u LOOP %02u/%02u", mapSlot_,
               static_cast<unsigned>(status.replayWp),
               static_cast<unsigned>(status.replayTotal));
      snprintf(desired_[1], 21, "LAP:%lu T:%lumm",
               static_cast<unsigned long>(status.replayLapCounter),
               static_cast<unsigned long>(status.replayTargetMm));
      snprintf(desired_[2], 21, "V:%lumm MODE:LOOP",
               static_cast<unsigned long>(status.replayTravelMm));
      snprintf(desired_[3], 21, "X HOLD X-LONG CANCEL");
      return;
    }
    if (status.replayOperation == 1U) {
      snprintf(desired_[0], 21, "MAP%u RUN WP:%02u/%02u", mapSlot_,
               static_cast<unsigned>(status.replayWp),
               static_cast<unsigned>(status.replayTotal));
      snprintf(desired_[1], 21, "T:%lumm V:%lumm",
               static_cast<unsigned long>(status.replayTargetMm),
               static_cast<unsigned long>(status.replayTravelMm));
      snprintf(desired_[2], 21, "MODE:%s", replayMode);
      snprintf(desired_[3], 21, "X HOLD X-LONG CANCEL");
      return;
    }
    if (status.replayOperation == 2U) {
      snprintf(desired_[0], 21, "MAP%u TURN WP:%02u/%02u", mapSlot_,
               static_cast<unsigned>(status.replayWp),
               static_cast<unsigned>(status.replayTotal));
      snprintf(desired_[1], 21, "TURN %+ddeg", status.replayTargetDeg);
      snprintf(desired_[2], 21, "MODE:%s", replayMode);
      snprintf(desired_[3], 21, "X HOLD X-LONG CANCEL");
      return;
    }
    snprintf(desired_[0], 21, "MAP%u RUN %02u/%02u", mapSlot_,
             static_cast<unsigned>(status.replayWp),
             static_cast<unsigned>(status.replayTotal));
    snprintf(desired_[1], 21, "TURN %+ddeg", status.replayTargetDeg);
    snprintf(desired_[2], 21, "MODE:%s", replayMode);
    snprintf(desired_[3], 21, "X HOLD X-LONG CANCEL");
    return;
  }
  if (status.mode == 7U) {
    snprintf(desired_[0], 21, "MAP%u HOLD %02u/%02u", mapSlot_,
             static_cast<unsigned>(status.replayWp),
             static_cast<unsigned>(status.replayTotal));
    if (status.replayMode == 1U) {
      snprintf(desired_[1], 21, "LOOP LAP:%lu",
               static_cast<unsigned long>(status.replayLapCounter));
      snprintf(desired_[2], 21, "START RESUME");
      snprintf(desired_[3], 21, "X-LONG CANCEL");
      return;
    }
    if (status.holdReason == 1U) {
      snprintf(desired_[1], 21, "USER HOLD");
      snprintf(desired_[2], 21, "START RESUME");
      snprintf(desired_[3], 21, "X-LONG CANCEL");
    } else if (status.holdReason == 2U) {
      snprintf(desired_[1], 21, "OBS BLOCKED");
      snprintf(desired_[2], 21, "START RESUME");
      snprintf(desired_[3], 21, "X-LONG CANCEL");
    } else {
      snprintf(desired_[1], 21, "STOPPED");
      snprintf(desired_[2], 21, "RESUME LOCKED");
      snprintf(desired_[3], 21, "X-LONG CANCEL");
    }
    return;
  }
  if (status.mode == 8U) {
    snprintf(desired_[0], 21, "MAP%u COMPLETE", mapSlot_);
    snprintf(desired_[1], 21, "MODE:%s", replayMode);
    snprintf(desired_[2], 21, "STOPPED");
    snprintf(desired_[3], 21, "START AGAIN");
    return;
  }

  const char* title = "READY";
  if (status.mode == 1U) {
    title = "TEACH MAN";
  } else if (status.mode == 2U) {
    title = "SAVED";
  } else if (status.mode == 3U) {
    title = "DELETE?";
  } else if (status.storeState == 0U) {
    title = "EMPTY";
  } else if (status.storeState == 1U) {
    title = "SAVED";
  } else if (status.storeState >= 2U) {
    title = "ERROR";
  }

  if (status.mode == 3U) {
    snprintf(desired_[0], 21, "DELETE MAP%u?", mapSlot_);
  } else {
    snprintf(desired_[0], 21, "MAP%u %s %s", mapSlot_, title, routeType);
  }
  snprintf(desired_[1], 21, "PTS:%03u/%03u L:%lu.%lum",
           static_cast<unsigned>(status.points),
           static_cast<unsigned>(status.maxPoints), wholeM, tenthM);

  if (status.mode == 1U) {
    snprintf(desired_[2], 21, "TRI=MARK SQ=UNDO");
    snprintf(desired_[3], 21, "O=SAVE X=CANCEL");
  } else if (status.mode == 2U) {
    snprintf(desired_[2], 21, "MODE:%s", replayMode);
    snprintf(desired_[3], 21, "START RUN L3 EXIT");
  } else if (status.mode == 3U) {
    snprintf(desired_[2], 21, "CIR YES");
    snprintf(desired_[3], 21, "SELH CANCEL L3 EXIT");
  } else if (status.storeState == 0U) {
    snprintf(desired_[2], 21, "START TEACH");
    snprintf(desired_[3], 21, "SEL MAP L3 EXIT");
  } else if (status.storeState == 1U) {
    snprintf(desired_[2], 21, "START TEACH CIR MODE");
    snprintf(desired_[3], 21, "SEL MAP SQH DEL L3");
  } else {
    snprintf(desired_[2], 21, "MAP STORAGE ERROR");
    snprintf(desired_[3], 21, "SEL MAP L3 EXIT");
  }
}

void LcdDisplay::buildDesiredLines() {
  if (page_ == LcdPage::MAP) {
    buildMapLines();
  } else {
    buildRobotLines();
  }
  for (uint8_t row = 0; row < 4; ++row) padLine(desired_[row]);
}

void LcdDisplay::forceRefresh() {
  memset(sent_, 0, sizeof(sent_));
  scanRow_ = 0;
  scanColumn_ = 0;
}

bool LcdDisplay::sendOneDirtyCharacter() {
  for (uint8_t examined = 0; examined < 80; ++examined) {
    const uint8_t row = scanRow_;
    const uint8_t column = scanColumn_;
    if (++scanColumn_ >= 20) { scanColumn_ = 0; scanRow_ = (scanRow_ + 1U) & 0x03U; }
    if (desired_[row][column] == sent_[row][column]) continue;
    setCursor(column, row);
    send(static_cast<uint8_t>(desired_[row][column]), 1);
    sent_[row][column] = desired_[row][column];
    return true;
  }
  return false;
}

void LcdDisplay::update() {
  if (!available_) return;
  const uint32_t now = millis();
  if ((now - lastRenderMs_) >= LCD_RENDER_MS) {
    lastRenderMs_ = now;
    buildDesiredLines();
  }
  if ((now - lastCharacterMs_) >= 5U) {
    lastCharacterMs_ = now;
    (void)sendOneDirtyCharacter();
  }
}
