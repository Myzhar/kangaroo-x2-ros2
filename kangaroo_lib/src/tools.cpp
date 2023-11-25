#include "tools.hpp"

uint64_t millis() {
  auto now = std::chrono::system_clock::now();
  auto duration = now.time_since_epoch();
  uint64_t msec =
      std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

  return msec;
}