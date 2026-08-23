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
                              uint32_t lengthMm) {
  const uint8_t normalized = slot == 2U ? 2U : 1U;
  LcdMapStatus& status = mapStatus_[normalized - 1U];
  status.valid = true;
  status.storeState = storeState;
  status.mode = mode;
  status.points = points;
  status.maxPoints = maxPoints == 0U ? 128U : maxPoints;
  status.lengthMm = lengthMm;
  if (mapSlot_ == normalized) forceRefresh();
}

void LcdDisplay::buildRobotLines() {
  const int compassTenths = static_cast<int>(roundf(data_.compassAngle * 10.0f));
  const int targetTenths = static_cast<int>(roundf(data_.headingTarget * 10.0f));
  snprintf(desired_[0], 21, "HDG:%4d.%1d PS2:%-4s",
           compassTenths / 10, abs(compassTenths % 10), data_.ps2Status);
  snprintf(desired_[1], 21, "SPD:%3d L:%3d R:%3d", data_.speed, data_.left, data_.right);
  if (!data_.encoderReady) {
    snprintf(desired_[2], 21, "ENC:%-16.16s",
             data_.encoderHealth != nullptr ? data_.encoderHealth : "NOT_READY");
  } else {
    snprintf(desired_[2], 21, "EL:%6ld ER:%6ld",
             static_cast<long>(data_.leftEncoderTicks),
             static_cast<long>(data_.rightEncoderTicks));
  }
  if (data_.brakeLocked ||
      (data_.obstacleZone != nullptr &&
       strcmp(data_.obstacleZone, "CLEAR") != 0 &&
       strcmp(data_.obstacleZone, "UNKNOWN") != 0)) {
    snprintf(desired_[3], 21, "US:%3dcm %-5s B:%s",
             data_.ultrasonicValid
                 ? static_cast<int>(lroundf(data_.ultrasonicDistanceCm))
                 : 0,
             data_.obstacleZone != nullptr ? data_.obstacleZone : "UNK",
             data_.brakeLocked ? "ON" : "OFF");
  } else if (data_.headingEnabled) {
    snprintf(desired_[3], 21, "TGT:%4d.%1d RAMP:%s", targetTenths / 10,
             abs(targetTenths % 10), data_.rampEnabled ? "ON" : "OFF");
  } else {
    snprintf(desired_[3], 21, "IMU:%-4.4s FUS:%-4.4s",
             data_.imuStatus != nullptr ? data_.imuStatus : "LOST",
             data_.fusionStatus != nullptr ? data_.fusionStatus : "LOST");
  }
}

void LcdDisplay::buildMapLines() {
  const LcdMapStatus& status = mapStatus_[mapSlot_ - 1U];
  if (!status.valid) {
    snprintf(desired_[0], 21, "MAP%u SYNC", mapSlot_);
    snprintf(desired_[1], 21, "P:---/--- L:---");
    snprintf(desired_[2], 21, "WAIT ESP32 STATUS");
    snprintf(desired_[3], 21, "SEL MAP L3 EXIT");
    return;
  }

  const uint32_t lengthTenthsM = (status.lengthMm + 50U) / 100U;
  const unsigned long wholeM = static_cast<unsigned long>(lengthTenthsM / 10U);
  const unsigned long tenthM = static_cast<unsigned long>(lengthTenthsM % 10U);
  const char* title = "READY";
  if (status.mode == 1U) {
    title = "TEACH";
  } else if (status.mode == 2U) {
    title = "LOADED";
  } else if (status.mode == 3U) {
    title = "DELETE?";
  } else if (status.mode == 4U) {
    title = "REPLAY READY";
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
    snprintf(desired_[0], 21, "MAP%u %s", mapSlot_, title);
  }
  snprintf(desired_[1], 21, "P:%03u/%03u L:%lu.%lum",
           static_cast<unsigned>(status.points),
           static_cast<unsigned>(status.maxPoints), wholeM, tenthM);

  if (status.mode == 1U) {
    snprintf(desired_[2], 21, "TR MARK SQ UNDO");
    snprintf(desired_[3], 21, "CIR SAVE X CANCEL");
  } else if (status.mode == 2U) {
    snprintf(desired_[2], 21, "START REPLAY");
    snprintf(desired_[3], 21, "CIR LOAD L3 EXIT");
  } else if (status.mode == 3U) {
    snprintf(desired_[2], 21, "CIR YES");
    snprintf(desired_[3], 21, "SELH CANCEL L3 EXIT");
  } else if (status.mode == 4U) {
    snprintf(desired_[2], 21, "START CHECK NO MOTOR");
    snprintf(desired_[3], 21, "X CANCEL L3 EXIT");
  } else if (status.storeState == 0U) {
    snprintf(desired_[2], 21, "START TEACH");
    snprintf(desired_[3], 21, "SEL MAP L3 EXIT");
  } else if (status.storeState == 1U) {
    snprintf(desired_[2], 21, "START TEACH CIR LOAD");
    snprintf(desired_[3], 21, "SEL MAP SQH DEL L3");
  } else {
    snprintf(desired_[2], 21, "CHECK ESP32/NVS");
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
