#include "tools.hpp"

uint64_t millis() {
  auto now = std::chrono::system_clock::now();
  auto duration = now.time_since_epoch();
  uint64_t msec =
      std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

  return msec;
}

std::string toString(KangarooError err) {
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
      return std::string("serial timeour");

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