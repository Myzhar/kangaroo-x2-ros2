#ifndef KANGAROO_STATUS_HPP
#define KANGAROO_STATUS_HPP

#include "types.hpp"

/*!
\class KangarooStatus
\brief Stores the response to a status request.

Returned by KangarooChannel::getPos(), KangarooMonitor::status(), and others.
*/
class KangarooStatus {
 public:
  KangarooStatus();
  KangarooStatus(const uint8_t* data, size_t length);

 public:
  static KangarooStatus createInvalidStatus();
  static KangarooStatus createTimedOut();

 public:
  /*!
  Gets the channel name.
  \return The channel name.
  */
  inline char channel() const { return (char)_channel; }

  /*!
  Gets the status flags associated with this response.
  \return The status flags.
  */
  inline KangarooStatusFlags flags() const {
    return (KangarooStatusFlags)_flags;
  }

  /*!
  Gets the type of the request.
  \return The type of the request.
  */
  inline KangarooGetType type() const { return (KangarooGetType)_type; }

  /*!
  Gets the value associated with this response.
  \return The value.
  */
  inline int32_t value() const { return _value; }

  inline uint8_t echoCode() const { return _echoCode; }
  inline uint8_t sequenceCode() const { return _sequenceCode; }
  inline bool valid() const { return _valid; }

 public:
  /*!
  Gets whether a recently-requested command is still executing.
  For example, KangarooChannel::setPos() is busy while the channel is getting
  into position, and KangarooChannel::home() is busy while the channel is
  homing. \return True if the channel is busy.
  */
  bool busy() const { return 0 != (flags() & KANGAROO_STATUS_BUSY); }

  /*!
  If the response is an error, gets the error.
  \return The error, or KANGAROO_NO_ERROR if the response is not an error.
  */
  KangarooError error() const {
    return (KangarooError)(0 != (flags() & KANGAROO_STATUS_ERROR) ? value()
                                                                  : 0);
  }

  /*!
  Gets whether a recently-requested command has completed execution.
  For example, KangarooChannel::setPos() is done when the channel is in
  position. \return True if the command is done.
  */
  bool done() const { return !busy(); }

  /*!
  Gets whether the response is not an error and KangarooStatus::value() is the
  requested value. \return True if the response is not an error.
  */
  bool ok() const { return !error(); }

  /*!
  Gets whether a timeout has occurred.
  \return True if the KangarooStatus::error() equals KANGARO_TIMED_OUT.
  */
  bool timedOut() const { return error() == KANGAROO_TIMED_OUT; }

 private:
  static KangarooStatus createFromError(KangarooError error);

 private:
  void init();
  bool parse(const uint8_t* data, size_t length);

 private:
  bool _valid;
  uint8_t _channel, _flags, _echoCode, _sequenceCode, _type;
  int32_t _value;
};

#endif  // KANGAROO_STATUS_HPP