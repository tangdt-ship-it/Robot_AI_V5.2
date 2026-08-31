#include "Compass_Uart_V2_STM32.h"

Compass_Uart_V2::Compass_Uart_V2(uint32_t rxPin, uint32_t txPin)
    : serial_(rxPin, txPin), lastReadOk_(false), rxState_(RxState::IDLE),
      requestStartUs_(0), byteStartUs_(0), highByte_(0), timedOut_(false) {}

void Compass_Uart_V2::begin(uint32_t baud) {
  serial_.begin(baud);
  delay(50);
  clearRx();
  cancelRead();
}

void Compass_Uart_V2::reset() {
  // Reset is deliberately non-blocking; the controller waits for a later sample.
  cancelRead();
  clearRx();
  serial_.write('a');
}

void Compass_Uart_V2::clearRx() {
  while (serial_.available() > 0) (void)serial_.read();
}

bool Compass_Uart_V2::waitForBytes(size_t count, uint32_t timeoutUs) {
  const uint32_t startUs = micros();
  while (serial_.available() < static_cast<int>(count)) {
    if ((micros() - startUs) >= timeoutUs) return false;
    yield();
  }
  return true;
}

int16_t Compass_Uart_V2::read() {
  lastReadOk_ = false;
  cancelRead();
  clearRx();
  serial_.write('z');
  if (!waitForBytes(1U, RESPONSE_TIMEOUT_US)) return 0;
  const int high = serial_.read();
  if (high < 0 || !waitForBytes(1U, BYTE_GAP_TIMEOUT_US)) return 0;
  const int low = serial_.read();
  if (low < 0) return 0;
  lastReadOk_ = true;
  return static_cast<int16_t>((static_cast<uint16_t>(static_cast<uint8_t>(high)) << 8) |
                              static_cast<uint16_t>(static_cast<uint8_t>(low)));
}

bool Compass_Uart_V2::requestRead() {
  if (rxState_ != RxState::IDLE) return false;
  lastReadOk_ = false;
  timedOut_ = false;
  clearRx();
  serial_.write('z');
  requestStartUs_ = micros();
  byteStartUs_ = requestStartUs_;
  rxState_ = RxState::WAIT_HIGH;
  return true;
}

bool Compass_Uart_V2::pollRead(int16_t& value) {
  if (rxState_ == RxState::IDLE) return false;
  const uint32_t nowUs = micros();
  if (rxState_ == RxState::WAIT_HIGH) {
    if (serial_.available() > 0) {
      const int byte = serial_.read();
      if (byte < 0) { cancelRead(); return false; }
      highByte_ = static_cast<uint8_t>(byte);
      byteStartUs_ = nowUs;
      rxState_ = RxState::WAIT_LOW;
    } else if ((nowUs - requestStartUs_) >= RESPONSE_TIMEOUT_US) {
      timedOut_ = true;
      cancelRead();
    }
    return false;
  }
  if (serial_.available() > 0) {
    const int byte = serial_.read();
    if (byte < 0) { cancelRead(); return false; }
    value = static_cast<int16_t>((static_cast<uint16_t>(highByte_) << 8) |
                                 static_cast<uint16_t>(static_cast<uint8_t>(byte)));
    lastReadOk_ = true;
    rxState_ = RxState::IDLE;
    timedOut_ = false;
    return true;
  }
  if ((nowUs - byteStartUs_) >= BYTE_GAP_TIMEOUT_US) {
    timedOut_ = true;
    cancelRead();
  }
  return false;
}

bool Compass_Uart_V2::readPending() const { return rxState_ != RxState::IDLE; }
bool Compass_Uart_V2::readTimedOut() const { return timedOut_; }
void Compass_Uart_V2::cancelRead() { rxState_ = RxState::IDLE; }
bool Compass_Uart_V2::lastReadOk() const { return lastReadOk_; }
