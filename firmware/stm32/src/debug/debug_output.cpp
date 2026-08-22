#include <debug/debug_output.h>

namespace {
struct RttBuffer {
  const char* name;
  char* buffer;
  uint32_t size;
  volatile uint32_t writeOffset;
  volatile uint32_t readOffset;
  uint32_t flags;
};

struct RttControlBlock {
  char id[16];
  int32_t maxUpBuffers;
  int32_t maxDownBuffers;
  RttBuffer up[1];
  RttBuffer down[1];
};

char rttUpBuffer[2048];
char rttDownBuffer[16];
const char rttUpName[] = "Robot_AI";
const char rttDownName[] = "Terminal";

// The identifier and layout are consumed by OpenOCD's built-in RTT client.
RttControlBlock rttControl = {
    {'S','E','G','G','E','R',' ','R','T','T',0,0,0,0,0,0},
    1,
    1,
    {{rttUpName, rttUpBuffer, sizeof(rttUpBuffer), 0, 0, 0}},
    {{rttDownName, rttDownBuffer, sizeof(rttDownBuffer), 0, 0, 0}}};
}

size_t RttStream::write(uint8_t value) {
  RttBuffer& channel = rttControl.up[0];
  const uint32_t writeOffset = channel.writeOffset;
  const uint32_t next = (writeOffset + 1U) % channel.size;
  if (next == channel.readOffset) return 0;
  channel.buffer[writeOffset] = static_cast<char>(value);
  __DMB();
  channel.writeOffset = next;
  return 1;
}

size_t RttStream::write(const uint8_t* buffer, size_t size) {
  size_t written = 0;
  while (written < size && write(buffer[written]) == 1U) ++written;
  return written;
}

size_t DebugOutput::write(uint8_t value) {
  (void)rtt_.write(value);
  return uart_.write(value);
}

size_t DebugOutput::write(const uint8_t* buffer, size_t size) {
  (void)rtt_.write(buffer, size);
  return uart_.write(buffer, size);
}

