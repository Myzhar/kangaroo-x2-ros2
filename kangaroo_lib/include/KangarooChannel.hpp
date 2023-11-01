#ifndef KANGAROO_CHANNEL_HPP
#define KANGAROO_CHANNEL_HPP

#include "types.hpp"
#include "KangarooSerial.hpp"
#include "KangarooMonitor.hpp"

/*!
\class KangarooChannel
\brief Lets you send commands to and get position, speed, etc. for a specific channel of the Kangaroo.

Any number of KangarooChannel objects can be associated with a single KangarooSerial object.
*/
class KangarooChannel
{
  friend class Kangaroo;
  friend class KangarooMonitor;
  
public:
  /*!
  Constructs a KangarooChannel object.
  \param serial The KangarooSerial whose serial port the controller is on.
  \param name The name of the channel (a single character).
              By default, mixed mode channels are 'D' (Drive) and 'T' (Turn). Independent mode channels are '1' and '2'.
              You can change these in DEScribe. Mainly this is useful if you have multiple Kangaroos daisy-chained and have
              'Enable multi-Kangaroo mode (shared signal lines).' checked, or if your particular application would be clearer
              if for instance the channels were named 'X', 'Y', and 'Z'.
  \param address The Packet Serial address of the controller.
                 Normally this can be left at its default. If you daisy-chain the TX/S1 line between your Kangaroo and a 
                 Sabertooth or SyRen motor driver set in Packet Serial, however, make sure they aren't both using the same address.
  */
   KangarooChannel(KangarooSerial& serial, char name, byte address = KANGAROO_DEFAULT_ADDRESS);
  ~KangarooChannel();
  
public:
  /*!
  Gets the channel name.
  \return The channel name.
  */
  inline char name() const { return _name; }
  
  /*!
  Gets the Packet Serial address of the controller.
  \return The packet serial address of the controller.
  */
  inline byte address() const { return _address; }

public:
  /*!
  Starts the channel. Also, the Kangaroo LED will shine brightly for a third of a second.
  \param onlyIfNecessary Whether the channel should only be started if necessary. If true, and it is already started, it will not be restarted.
                         This option requires 2014-11-13 or newer firmware.
  \return A KangarooError.
          Most commonly, this will be KANGAROO_NO_ERROR if the channel does not require homing, or KANGAROO_NOT_HOMED if it does.
  */
  KangarooError start(bool onlyIfNecessary = false);
  
  /*!
  Sets custom units for the channel.
  This command may be called after you start the channel but before you home it.
  If you do not set custom units, the units you have set up in DEScribe will be used.
  If you haven't set any, machine units will be used.
  \param desiredUnits The amount in your units that correspond to the specified amount of machine units.
  \param machineUnits The amount of machine units (millivolts or lines) corresponding to the specified amount in your units.
  \return A KangarooError.
          Most commonly, this will be KANGAROO_NO_ERROR if the channel does not require homing, or KANGAROO_NOT_HOMED if it does.
  */
  KangarooError units(int32_t desiredUnits, int32_t machineUnits);
  
  /*!
  Homes the channel.
  \param onlyIfNecessary Whether or not the channel should only be homed if necessary. If true, and it has already been homed, it will not be rehomed.
                         This option requires 2014-11-13 or newer firmware.
  \return A KangarooMonitor for tracking the homing request.
          The easiest way to use the KangarooMonitor here is to call KangarooMonitor::wait().
  */
  KangarooMonitor home(bool onlyIfNecessary = false);
   
  /*!
  Moves to the specified absolute position.
  This command is most useful for absolutely-positioned systems.
  \param position          The position to move to.
  \param speedLimit        The speed limit for the move.
  \param flags             Modifiers for the move command.
  \return A KangarooMonitor for tracking the move request.
  */
  KangarooMonitor p(int32_t position,
                    int32_t speedLimit = KANGAROO_UNSPECIFIED_LIMIT,
                    KangarooMoveFlags flags = KANGAROO_MOVE_DEFAULT);
  
  /*!
  Makes an incremental move, relative to where you are right now.
  Rovers are a case where this is almost always what you want.
  \param positionIncrement The amount to increment the current position by.
  \param speedLimit        The speed limit for the move.
  \param flags             Modifiers for the move command.
  \return A KangarooMonitor for tracking the move request.
  */
  KangarooMonitor pi(int32_t positionIncrement,
                     int32_t speedLimit = KANGAROO_UNSPECIFIED_LIMIT,
                     KangarooMoveFlags flags = KANGAROO_MOVE_DEFAULT);
  
  /*!
  Makes an incremental move, relative to the current position setpoint.
  This is useful for conveyors.
  2014-11-13 or newer firmware is required for this command.
  \param positionIncrement The amount to increment the current position by.
  \param speedLimit        The speed limit for the move.
  \param flags             Modifiers for the move command.
  \return A KangarooMonitor for tracking the move request.
  */
  KangarooMonitor psi(int32_t positionIncrement,
                      int32_t speedLimit = KANGAROO_UNSPECIFIED_LIMIT,
                      KangarooMoveFlags flags = KANGAROO_MOVE_DEFAULT);
                     
  /*!
  Moves at a particular speed.
  \param speed             The speed to move at.
  \param speedRampingLimit The speed ramping limit for the move.
  \param flags             Modifiers for the move command.
  \return A KangarooMonitor for tracking the move request.
  */
  KangarooMonitor s(int32_t speed,
                    int32_t speedRampingLimit = KANGAROO_UNSPECIFIED_LIMIT,
                    KangarooMoveFlags flags = KANGAROO_MOVE_DEFAULT);

  /*!
  Moves at a particular speed, incremental from the current speed.
  \param speedIncrement    The amount to increment the current speed by.
  \param speedRampingLimit The speed ramping limit for the move.
  \param flags             Modifiers for the move command.
  \return A KangarooMonitor for tracking the move request.
  */
  KangarooMonitor si(int32_t speedIncrement,
                     int32_t speedRampingLimit = KANGAROO_UNSPECIFIED_LIMIT,
                     KangarooMoveFlags flags = KANGAROO_MOVE_DEFAULT);

  /*!
  Moves at a particular speed, incremental from the current speed setpoint.
  This is useful for cruise control: a negative increment will slow down,
                                     a positive increment will speed up,
                                     and a zero increment will hold the current speed.
  2014-11-13 or newer firmware is required for this command.
  \param speedIncrement    The amount to increment the current speed by.
  \param speedRampingLimit The speed ramping limit for the move.
  \param flags             Modifiers for the move command.
  \return A KangarooMonitor for tracking the move request.
  */
  KangarooMonitor ssi(int32_t speedIncrement,
                      int32_t speedRampingLimit = KANGAROO_UNSPECIFIED_LIMIT,
                      KangarooMoveFlags flags = KANGAROO_MOVE_DEFAULT);
                     
public:
  /*!
  Issues a 'get' request.
  \param type The type of the 'get' request.
  \param flags Flags modifying the 'get' request.
  \return A KangarooStatus object describing the response.
  */
  KangarooStatus get(KangarooGetType type, KangarooGetFlags flags = KANGAROO_GET_DEFAULT);
  
  /*!
  Gets the absolute position.
  \param flags Flags modifying the 'get' request.
  \return A KangarooStatus object describing the position.
  */
  inline KangarooStatus getP(KangarooGetFlags flags = KANGAROO_GET_DEFAULT)
  {
    return get(KANGAROO_GETP, flags);
  }

  /*!
  Gets the incremental position (relative to the position when the last command was issued).
  \param flags Flags modifying the 'get' request.
  \return A KangarooStatus object describing the position.
  */  
  inline KangarooStatus getPI(KangarooGetFlags flags = KANGAROO_GET_DEFAULT)
  {
    return get(KANGAROO_GETPI, flags);
  }
  
  /*!
  Gets the setpoint position.
  2014-11-13 or newer firmware is required for this request.
  \param flags Flags modifying the 'get' request.
  \return A KangarooStatus object describing the position.
  */  
  inline KangarooStatus getPS(KangarooGetFlags flags = KANGAROO_GET_DEFAULT)
  {
    return get(KANGAROO_GETPS, flags);
  }
  
  /*!
  Gets the absolute speed (positive or negative depending on direction).
  \param flags Flags modifying the 'get' request.
  \return A KangarooStatus object describing the speed.
  */
  inline KangarooStatus getS(KangarooGetFlags flags = KANGAROO_GET_DEFAULT)
  {
    return get(KANGAROO_GETS, flags);
  }
  
  /*!
  Gets the incremental speed (relative to the speed when the last command was issued).
  \param flags Flags modifying the 'get' request.
  \return A KangarooStatus object describing the speed.
  */
  inline KangarooStatus getSI(KangarooGetFlags flags = KANGAROO_GET_DEFAULT)
  {
    return get(KANGAROO_GETSI, flags);
  }
  
  /*!
  Gets the setpoint speed.
  2014-11-13 or newer firmware is required for this request.
  \param flags Flags modifying the 'get' request.
  \return A KangarooStatus object describing the speed.
  */
  inline KangarooStatus getSS(KangarooGetFlags flags = KANGAROO_GET_DEFAULT)
  {
    return get(KANGAROO_GETSS, flags);
  }
  
  /*!
  Gets the minimum position. This corresponds to DEScribe's Nominal Travel minimum.
  \param flags Flags modifying the 'get' request.
  \return A KangarooStatus object describing the limit.
  */
  inline KangarooStatus getMin(KangarooGetFlags flags = KANGAROO_GET_DEFAULT)
  {
    return get(KANGAROO_GETMIN, flags);
  }

  /*!
  Gets the maximum position. This corresponds to DEScribe's Nominal Travel maximum.
  \param flags Flags modifying the 'get' request.
  \return A KangarooStatus object describing the limit.
  */
  inline KangarooStatus getMax(KangarooGetFlags flags = KANGAROO_GET_DEFAULT)
  {
    return get(KANGAROO_GETMAX, flags);
  }
  
public:  
  // While the following calls violate the standard Arduino casing, they sure are nicer to type.
  // The contrast between 'p' and 'getP' may confuse as well, so we include these to make life easier for everyone.
  inline KangarooStatus getp  (KangarooGetFlags flags = KANGAROO_GET_DEFAULT) { return getP  (flags); }
  inline KangarooStatus getpi (KangarooGetFlags flags = KANGAROO_GET_DEFAULT) { return getPI (flags); }
  inline KangarooStatus getps (KangarooGetFlags flags = KANGAROO_GET_DEFAULT) { return getPS (flags); }
  inline KangarooStatus gets  (KangarooGetFlags flags = KANGAROO_GET_DEFAULT) { return getS  (flags); }  
  inline KangarooStatus getsi (KangarooGetFlags flags = KANGAROO_GET_DEFAULT) { return getSI (flags); }
  inline KangarooStatus getss (KangarooGetFlags flags = KANGAROO_GET_DEFAULT) { return getSS (flags); }
  inline KangarooStatus getmin(KangarooGetFlags flags = KANGAROO_GET_DEFAULT) { return getMin(flags); }
  inline KangarooStatus getmax(KangarooGetFlags flags = KANGAROO_GET_DEFAULT) { return getMax(flags); }

public:
  /*! Gets the command retry interval.
  \return The command retry interval, in milliseconds.
  */
  inline int32_t commandRetryInterval() const { return _commandRetryInterval; }
  
  /*! Sets the command retry interval.
  \param intervalMS The command retry interval, in milliseconds.
  */
  inline void commandRetryInterval(int32_t intervalMS) { _commandRetryInterval = intervalMS; }
  
  /*! Gets the command timeout.
  \return The command timeout, in milliseconds.
  */
  inline int32_t commandTimeout() const { return _commandTimeout; }
  
  /*! Sets the command timeout.
  \param timeoutMS The command timeout, in milliseconds.
  */
  inline void commandTimeout(int32_t timeoutMS) { _commandTimeout = timeoutMS; }
  
  /*! Gets whether streaming is enabled.
  \return True if streaming is enabled.
  */
  inline boolean streaming() const { return _streaming; }
  
  /*!
  Enables or disables streaming.
  When streaming is disabled (the default), commands are delivered reliably:
  a request is sent to the Kangaroo after every command to make sure the command was received.
  When streaming is enabled, commands are delivered unreliably:
  no effort is made to verify that they are received.

  If you are constantly sending position or speed commands, you may not care if an intermediate one is lost.
  Enabling streaming can more than double your command rate, at the cost of reduced reliability.
  If you only have a transmit line and no way to receive data back from the Kangaroo, you should enable streaming.
  KangarooMonitor::wait() cannot be meaningfully called for a command that was streamed.
  */
  inline void streaming(boolean enabled) { _streaming = enabled; }
  
  /*!
  Sets the baud rate.
  This affects all channels of the controller that receives the command.
  \param baudRate The baud rate to use. Supported rates are 9600, 19200, 38400, and 115200.
  */
  void baudRate(int32_t baudRate);
  
  /*!
  Powers down the channel.
  \return A KangarooError, usually KANGAROO_NO_ERROR.
  */
  KangarooError powerDown();
  
  /*!
  Powers down all channels of the controller that receives the command.
  \return A KangarooError, usually KANGAROO_NO_ERROR.
  */
  KangarooError powerDownAll();
  
  /*!
  Sets the serial timeout.
  This affects all channels of the controller that receives the command.
  \param milliseconds The timeout, in milliseconds. A value of zero uses the DEScribe setting. KANGAROO_INFINITE_TIMEOUT disables the timeout.
  \return A KangarooError, usually KANGAROO_NO_ERROR.
  */
  KangarooError serialTimeout(int32_t milliseconds);
  
  /*!
  Sends a system command. These are mostly for advanced use cases.
  \param systemCommand The command to set.
  \param expectReply   Whether or not to expect a reply.
                       If a command has an unusual effect on the state of the Kangaroo, expecting a reply may
                       cause the command to retry until it times out.
  \param values        The parameters of the command.
  \param valueCount    The number of parameters.
  \return A KangarooError.
  */
  KangarooError systemCommand(KangarooSystemCommand systemCommand, boolean expectReply, int32_t values[], size_t valueCount);

private:
  KangarooStatus  getSpecial(KangarooGetType type, KangarooGetFlags flags, const KangarooTimeout& timeout);
  KangarooMonitor motion    (byte motionType, int32_t motionValue,
                             byte limit1Type, int32_t limit1Value,
                             byte limit2Type, int32_t limit2Value,
                             KangarooMoveFlags flags);
  KangarooMonitor set       (KangarooCommand        command,
                             KangarooCommandWriter& contents,
                             KangarooMoveFlags      moveFlags = KANGAROO_MOVE_DEFAULT,
                             KangarooGetType        getType   = KANGAROO_GETP);
  void setNoReply(KangarooCommand command, KangarooCommandWriter& contents,
                  KangarooMoveFlags moveFlags = KANGAROO_MOVE_DEFAULT);
                             
  boolean getInitialSequenceCodeIfNecessary(const KangarooTimeout& timeout, KangarooStatus& status);
  boolean updateMonitoredResult(const KangarooTimeout& timeout, boolean acceptRepliesWithStartupSequenceCode);

private:
  KangarooChannel (KangarooChannel& channel); // no copy
  void operator = (KangarooChannel& channel);
  
private:
  KangarooSerial&  _serial;
  char             _name;
  byte             _address;
  int32_t          _commandRetryInterval;
  int32_t          _commandTimeout;
  byte             _echoCode;
  uint32_t         _monitorCode;
  KangarooGetType  _monitoredGetType;
  KangarooGetFlags _monitoredGetFlags;
  KangarooStatus   _monitoredGetResult;
  byte             _monitoredSequenceCode;
  boolean          _monitoredSequenceCodeIsReady;
  boolean          _streaming;
};

#endif // KANGAROO_CHANNEL_HPP