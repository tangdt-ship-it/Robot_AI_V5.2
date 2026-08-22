#ifndef ROBOT_LINK_PROTOCOL_H
#define ROBOT_LINK_PROTOCOL_H

#include <Arduino.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace RobotLink {

static constexpr uint8_t PROTOCOL_VERSION = 3;
static constexpr size_t MAX_FRAME = 128;
static constexpr size_t MAX_TYPE = 12;
static constexpr size_t MAX_PAYLOAD = 80;

namespace MessageType {
static constexpr char BOOT[] = "BOOT";
static constexpr char HELLO[] = "HELLO";
static constexpr char PING[] = "PING";
static constexpr char PONG[] = "PONG";
static constexpr char ACK[] = "ACK";
static constexpr char STATUS[] = "STATUS";
static constexpr char STOP[] = "STOP";
static constexpr char RESULT[] = "RESULT";
}  // namespace MessageType

struct Frame {
  uint16_t sequence = 0;
  char type[MAX_TYPE] = {};
  char payload[MAX_PAYLOAD] = {};
};

inline uint16_t crc16Ccitt(const char* data, size_t length) {
  uint16_t crc = 0xFFFFU;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(static_cast<uint8_t>(data[i])) << 8U;
    for (uint8_t bit = 0; bit < 8U; ++bit) {
      crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1U) ^ 0x1021U)
                            : static_cast<uint16_t>(crc << 1U);
    }
  }
  return crc;
}

inline bool send(Print& stream, uint16_t sequence, const char* type,
                 const char* payload = "") {
  char body[MAX_FRAME];
  const int written = snprintf(body, sizeof(body), "RAI,%u,%u,%s,%s",
                               static_cast<unsigned>(PROTOCOL_VERSION),
                               static_cast<unsigned>(sequence), type, payload);
  if (written <= 0 || static_cast<size_t>(written) >= sizeof(body)) return false;
  const uint16_t crc = crc16Ccitt(body, static_cast<size_t>(written));
  char suffix[8];
  snprintf(suffix, sizeof(suffix), "*%04X\r\n", static_cast<unsigned>(crc));
  stream.write('$');
  stream.write(reinterpret_cast<const uint8_t*>(body), written);
  stream.print(suffix);
  return true;
}

class Parser {
 public:
  bool push(char value, Frame& output) {
    if (value == '$') {
      length_ = 0;
      buffer_[length_++] = value;
      receiving_ = true;
      return false;
    }
    if (!receiving_) return false;
    if (value == '\r') return false;
    if (value == '\n') {
      buffer_[length_] = '\0';
      receiving_ = false;
      return decode(output);
    }
    if (length_ >= sizeof(buffer_) - 1U) {
      reset();
      return false;
    }
    buffer_[length_++] = value;
    return false;
  }

  void reset() {
    length_ = 0;
    receiving_ = false;
  }

 private:
  static bool parseHex16(const char* text, uint16_t& value) {
    if (strlen(text) != 4U) return false;
    char* end = nullptr;
    const unsigned long parsed = strtoul(text, &end, 16);
    if (end == nullptr || *end != '\0' || parsed > 0xFFFFUL) return false;
    value = static_cast<uint16_t>(parsed);
    return true;
  }

  bool decode(Frame& output) {
    if (length_ < 12U || buffer_[0] != '$') return false;
    char* star = strrchr(buffer_, '*');
    if (star == nullptr) return false;
    uint16_t receivedCrc = 0;
    if (!parseHex16(star + 1, receivedCrc)) return false;
    *star = '\0';
    const char* body = buffer_ + 1;
    if (crc16Ccitt(body, strlen(body)) != receivedCrc) return false;

    char* cursor = buffer_ + 1;
    char* comma = strchr(cursor, ',');
    if (comma == nullptr) return false;
    *comma = '\0';
    if (strcmp(cursor, "RAI") != 0) return false;

    cursor = comma + 1;
    comma = strchr(cursor, ',');
    if (comma == nullptr) return false;
    *comma = '\0';
    if (strtoul(cursor, nullptr, 10) != PROTOCOL_VERSION) return false;

    cursor = comma + 1;
    comma = strchr(cursor, ',');
    if (comma == nullptr) return false;
    *comma = '\0';
    char* sequenceEnd = nullptr;
    const unsigned long sequence = strtoul(cursor, &sequenceEnd, 10);
    if (sequenceEnd == nullptr || *sequenceEnd != '\0' || sequence > 65535UL)
      return false;

    cursor = comma + 1;
    comma = strchr(cursor, ',');
    if (comma == nullptr) return false;
    *comma = '\0';
    if (*cursor == '\0' || strlen(cursor) >= sizeof(output.type)) return false;
    strncpy(output.type, cursor, sizeof(output.type));
    output.type[sizeof(output.type) - 1U] = '\0';

    cursor = comma + 1;
    if (strlen(cursor) >= sizeof(output.payload)) return false;
    strncpy(output.payload, cursor, sizeof(output.payload));
    output.payload[sizeof(output.payload) - 1U] = '\0';
    output.sequence = static_cast<uint16_t>(sequence);
    return true;
  }

  char buffer_[MAX_FRAME] = {};
  size_t length_ = 0;
  bool receiving_ = false;
};

}  // namespace RobotLink

#endif
