#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <chrono>
#include <string>

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

std::string toString(KangarooError err);

#endif  // TOOLS_HPP