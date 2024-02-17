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

bool calculateDiffDriveUnits(const float wheel_radius, const float wheel_dist,
                             const uint32_t encoder_lines,
                             const float gear_ratio_mul,
                             uint32_t& out_drive_dist,
                             uint32_t& out_drive_lines,
                             uint32_t& out_turn_lines);

#endif  // TOOLS_HPP