#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <chrono>

#include "types.hpp"

template <class T>
const T& constrain(const T& x, const T& a, const T& b) {
  if (x < a) {
    return a;
  } else if (b < x) {
    return b;
  } else
    return x;
}

uint64_t millis();

#endif  // TOOLS_HPP