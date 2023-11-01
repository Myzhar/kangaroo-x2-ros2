#ifndef TOOLS_HPP
#define TOOLS_HPP

#include "types.hpp"
#include <chrono>

template <class T>
const T& constrain(const T& x, const T& a, const T& b) {
  if (x < a) {
    return a;
  } else if (b < x) {
    return b;
  } else
    return x;
}

uint64_t millis() {
  auto now = std::chrono::system_clock::now();
  auto duration = now.time_since_epoch();
  uint64_t msec =
      std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

  return msec;
}

#endif  // TOOLS_HPP