#ifndef KANGAROO_MONITOR_HPP
#define KANGAROO_MONITOR_HPP

#include "types.hpp"

class KangarooChannel;
class KangarooStatus;
class KangarooTimeout;

/*!
\class KangarooMonitor
\brief Makes it easy to track the status of position commands, speed commands,
and any other commands that take time to complete.
*/
class KangarooMonitor
{
  friend class KangarooChannel;

public:
  /*!
  Constructs a KangarooMonitor object.
  This must be assigned before it will be valid.
  */
  KangarooMonitor();

public:
  /*!
  Gets the most recently received status.
  \return A KangarooStatus.
  */
  KangarooStatus status() const;

  /*!
  Gets whether the status is valid.
  If you keep an old KangarooMonitor object around and issue a new command on
  the same channel, the old KangarooMonitor will become invalid. \return True if
  the status is valid.
  */
  bool valid() const;

  /*!
  Updates the status. The channel's command timeout is used.
  \return The same KangarooMonitor.
  */
  KangarooMonitor update();

  /*!
  Updates the status.
  \param timeoutMS The timeout to use, in milliseconds.
  \return The same KangarooMonitor.
  */
  KangarooMonitor update(int32_t timeoutMS);

  /*!
  Updates the status.
  \param timeout The timeout object to use.
  \return The same KangarooMonitor.
  */
  KangarooMonitor update(const KangarooTimeout & timeout);

  /*!
  Waits until KangarooStatus::done() returns true or the timeout expires.
  \param timeoutMS The timeout to use, in milliseconds.
  \return The same KangarooMonitor.
  */
  KangarooMonitor wait(int32_t timeoutMS = KANGAROO_INFINITE_TIMEOUT);

  /*!
  Waits until KangarooStatus::done() returns true or the timeout object expires.
  \param timeout The timeout object to use.
  \return The same KangarooMonitor.
  */
  KangarooMonitor wait(const KangarooTimeout & timeout);

private:
  KangarooMonitor(KangarooChannel * channel, uint32_t monitorCode);

private:
  struct State
  {
    KangarooChannel * channel;
    uint32_t monitorCode;
  } _state;
};

/*!
Waits until all of the monitors are done or the timeout expires.
\param count     The number of monitors.
\param monitors  The monitors to wait for.
\param timeoutMS The timeout to use, in milliseconds.
\return True if all of the monitors are done, or false if the timeout expired.
\remark If a monitor is null, it is ignored.
*/
bool waitAll(
  size_t count, KangarooMonitor * monitors[],
  int32_t timeoutMS = KANGAROO_INFINITE_TIMEOUT);

/*!
Waits until all of the monitors are done or the timeout object expires.
\param count    The number of monitors.
\param monitors The monitors to wait for.
\param timeout  The timeout object to use.
\return True if all of the monitors are done, or false if the timeout expired.
\remark If a monitor is null, it is ignored.
*/
bool waitAll(
  size_t count, KangarooMonitor * monitors[],
  const KangarooTimeout & timeout);

/*!
Waits until at least one of the monitors is done or the timeout expires.
\param count     The number of monitors.
\param monitors  The monitors to wait for.
\param timeoutMS The timeout to use, in milliseconds.
\return The array index of the monitor that is done, or -1 if the timeout
expired. \remark If a monitor is null, it is ignored.
*/
int waitAny(
  size_t count, KangarooMonitor * monitors[],
  int32_t timeoutMS = KANGAROO_INFINITE_TIMEOUT);

/*!
Waits until at least one of the monitors is done or the timeout expires.
\param count    The number of monitors.
\param monitors The monitors to wait for.
\param timeout  The timeout object to use.
\return The array index of the monitor that is done, or -1 if the timeout
expired. \remark If a monitor is null, it is ignored.
*/
int waitAny(
  size_t count, KangarooMonitor * monitors[],
  const KangarooTimeout & timeout);

#endif  // KANGAROO_MONITOR_HPP
