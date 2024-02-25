#include "tools.hpp"

#include <math.h>

uint64_t millis()
{
  auto now = std::chrono::system_clock::now();
  auto duration = now.time_since_epoch();
  uint64_t msec =
    std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

  return msec;
}

std::string toString(KangarooError err)
{
  switch (err) {
    case KangarooError::KANGAROO_NO_ERROR:
      return std::string("no error");

    case KangarooError::KANGAROO_NOT_STARTED:
      return std::string("channel not started");

    case KangarooError::KANGAROO_NOT_HOMED:
      return std::string("channel not homed");

    case KangarooError::KANGAROO_CONTROL_ERROR:
      return std::string("channel control error");

    case KangarooError::KANGAROO_WRONG_MODE:
      return std::string("Kangaroo x2 wrong control mode");

    case KangarooError::KANGAROO_UNRECOGNIZED_CODE:
      return std::string("code not recognized");

    case KangarooError::KANGAROO_SERIAL_TIMEOUT:
      return std::string("serial timeout");

    case KangarooError::KANGAROO_INVALID_STATUS:
      return std::string("invalid status");

    case KangarooError::KANGAROO_TIMED_OUT:
      return std::string("request time out");

    case KangarooError::KANGAROO_PORT_NOT_OPEN:
      return std::string("serial port not open");

    default:
      return std::string("unknown error code");
  }
}

bool calculateDiffDriveUnits(
  const float wheel_radius, const float wheel_dist,
  const uint32_t encoder_lines,
  const float gear_ratio_mul,
  uint32_t & out_drive_dist,
  uint32_t & out_drive_lines,
  uint32_t & out_turn_lines)
{
  // ----> Forwarding
  float F = 2.0f * M_PI * wheel_radius;
  out_drive_dist = static_cast<uint32_t>(std::round(F));
  float F_L = gear_ratio_mul * encoder_lines;
  out_drive_lines = static_cast<uint32_t>(std::round(F_L));
  // ----> Forwarding

  // ----> Turning
  float T = F_L * M_PI * wheel_dist / F;
  out_turn_lines = static_cast<uint32_t>(std::round(T));
  // <---- Turning

  return out_drive_dist != 0 && out_drive_lines != 0 && out_turn_lines != 0;
}
