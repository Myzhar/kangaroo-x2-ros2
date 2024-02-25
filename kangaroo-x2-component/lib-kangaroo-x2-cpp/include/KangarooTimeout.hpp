#ifndef KANGAROO_TIMEOUT_HPP
#define KANGAROO_TIMEOUT_HPP

#include "types.hpp"

/*!
\class KangarooTimeout
\brief Encapsulates a starting time and duration.

The same KangarooTimeout can be reused between different calls.
*/
class KangarooTimeout
{
public:
  /*!
  Constructs a KangarooTimeout object. The starting time is captured.
  \param timeoutMS The timeout duration, in milliseconds.
  */
  KangarooTimeout(uint64_t timeoutMS);

public:
  /*!
  Gets whether the timeout can expire.
  If the KangarooTimeout was created with a timeout of
  KANGAROO_INFINITE_TIMEOUT, it cannot expire. \return True if the timeout can
  expire.
  */
  bool canExpire() const;

  /*!
  Gets whether the timeout has expired.
  \return True if the timeout has expired.
  */
  bool expired() const;

  /*!
  Causes the timeout to expire immediately.
  */
  void expire();

  /*!
  Captures the current time and uses it as the new starting time for the
  timeout.
  */
  void reset();

private:
  int64_t _start;
  int64_t _timeoutMS;
};

#endif  // KANGAROO_TIMEOUT_HPP
