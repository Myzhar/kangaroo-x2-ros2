#ifndef TYPES_HPP
#define TYPES_HPP

#include <stddef.h>
#include <stdint.h>

#define KANGAROO_BIT_PACKED_MAX_VALUE 536870911
#define KANGAROO_COMMAND_MAX_DATA_LENGTH 27
#define KANGAROO_COMMAND_MAX_BUFFER_LENGTH 32
#define KANGAROO_CRC_GOOD_VALUE 0x307e
#define KANGAROO_DEFAULT_ADDRESS \
  128  //!< The default Packet Serial address for Kangaroo is 128.
#define KANGAROO_DEFAULT_COMMAND_RETRY_INTERVAL \
  100  //!< By default, commands retry after 100 milliseconds if no response is
       //!< received.
#define KANGAROO_DEFAULT_COMMAND_TIMEOUT \
  KANGAROO_INFINITE_TIMEOUT  //!< By default, commands are retried forever.
#define KANGAROO_INFINITE_TIMEOUT \
  -1  //!< This value, passed as a timeout, disables the timeout.
#define KANGAROO_UNSPECIFIED_LIMIT \
  -1  //!< This value, passed as a limit, leaves the limit unspecified.

/*!
\enum KangarooError
Errors that can occur in response to status queries.
*/
enum KangarooError {
  // Negative values are responses from the library, though they may
  // *correspond* to remote states. Positive values are responses from the
  // Kangaroo.
  KANGAROO_NO_ERROR = 0x00,     //!< No error occurred.
  KANGAROO_NOT_STARTED = 0x01,  //!< The channel is not started. Call
                                //!< KangarooChannel::start() on the channel.
  KANGAROO_NOT_HOMED = 0x02,    //!< The channel needs to be homed. Call
                                //!< KangarooChannel::home() on the channel.
  KANGAROO_CONTROL_ERROR =
      0x03,  //!< A control error has occurred. Call KangarooChannel::start() on
             //!< the channel to clear the control error.
  KANGAROO_WRONG_MODE =
      0x04,  //!< The controller is in the wrong mode. For example, the DIP
             //!< switches may be in Mixed Mode while the tune was done in
             //!< Independent Mode.
  KANGAROO_UNRECOGNIZED_CODE = 0x05,
  KANGAROO_SERIAL_TIMEOUT =
      0x06,  //!< A serial timeout occurred, or the TX line was disconnected.
             //!< Call KangarooChannel::start() on the channel to clear the
             //!< serial timeout.
  KANGAROO_INVALID_STATUS = -0x01,
  KANGAROO_TIMED_OUT = -0x02,     //!< The requested timeout expired.
  KANGAROO_PORT_NOT_OPEN = -0x03  // The serial port is not open. (Not returned
                                  // by the Arduino library at present.)
};

/*!
\enum KangarooGetType
The possible 'get' request types.
*/
enum KangarooGetType {
  KANGAROO_GETP = 0x01,   //!< Absolute position.
  KANGAROO_GETPI = 0x41,  //!< Incremental position (relative to the position
                          //!< when the last command was issued).
  KANGAROO_GETS =
      0x02,  //!< Absolute speed (positive or negative depending on direction).
  KANGAROO_GETSI = 0x42,  //!< Incremental speed (relative to the speed when the
                          //!< last command was issued).
  KANGAROO_GETMIN = 0x08,  //!< The minimum position. This corresponds to
                           //!< DEScribe's Nominal Travel minimum.
  KANGAROO_GETMAX = 0x09,  //!< The maximum position. This corresponds to
                           //!< DEScribe's Nominal Travel maximum.
  KANGAROO_GETPS = 0x05,   //!< Setpoint position. 2014-11-13 or newer firmware
                           //!< is required for this request.
  KANGAROO_GETSS = 0x06,   //!< Setpoint speed. 2014-11-13 or newer firmware is
                           //!< required for this request.
};

/*!
\enum KangarooGetFlags
Flags that modify the behavior of a 'get' request.
*/
enum KangarooGetFlags {
  KANGAROO_GET_DEFAULT = 0x00,  //!< Normal behavior.
  KANGAROO_GET_ECHO_CODE = 0x10,
  KANGAROO_GET_RAW_UNITS =
      0x20,  //!< Use raw units. For analog, raw units are millivolts. For
             //!< quadrature, 4 raw units equal 1 line.
  KANGAROO_GET_SEQUENCE_CODE = 0x40
};

/*!
\enum KangarooMoveFlags
Flags that modify the behavior of a motion request.
*/
enum KangarooMoveFlags {
  KANGAROO_MOVE_DEFAULT = 0x00,  //!< Normal behavior.
  KANGAROO_MOVE_ONLY_IF_NECESSARY = 0x04,
  KANGAROO_MOVE_NO_DEFAULT_LIMITS =
      0x08,  //!< Do not apply the speed limit and acceleration limit source
             //!< settings. By default, the speed limit comes from Kangaroo's
             //!< potentiometers.
  KANGAROO_MOVE_RAW_UNITS =
      0x20,  //!< Use raw units. For analog, raw units are millivolts. For
             //!< quadrature, 4 raw units equal 1 line.
  KANGAROO_MOVE_SEQUENCE_CODE = 0x40
};

/*!
\enum KangarooStatusFlags
Flags that provide details about a query response.
*/
enum KangarooStatusFlags {
  KANGAROO_STATUS_NONE = 0x00,   //!< The response is normal.
  KANGAROO_STATUS_ERROR = 0x01,  //!< The response is an error.
  KANGAROO_STATUS_BUSY =
      0x02,  //!< For a motion, this means the motion is not finished. For an
             //!< error, this means the error (such as KANGAROO_NOT_HOMED)
             //!< should self-clear. This corresponds to a lowercase letter in
             //!< Simplified Serial.
  KANGAROO_STATUS_ECHO_CODE = 0x10,
  KANGAROO_STATUS_RAW_UNITS =
      0x20,  //!< The value in the response is in raw units. For analog, raw
             //!< units are millivolts. For quadrature, 4 raw units equal 1
             //!< line.
  KANGAROO_STATUS_SEQUENCE_CODE = 0x40
};

enum KangarooCommand {
  KANGAROO_CMD_START = 0x20,
  KANGAROO_CMD_UNITS = 0x21,
  KANGAROO_CMD_HOME = 0x22,
  KANGAROO_CMD_STATUS = 0x23,
  KANGAROO_CMD_MOVE = 0x24,
  KANGAROO_CMD_SYSTEM = 0x25
};

/*!
\enum KangarooSystemCommand
Advanced commands that have special effects on the Kangaroo motion controller.
*/
enum KangarooSystemCommand {
  KANGAROO_SYS_POWER_DOWN = 0x00,  //!< Powers down the channel.
  KANGAROO_SYS_POWER_DOWN_ALL =
      0x01,  //!< Powers down all channels of the controller that receives the
             //!< command.
  KANGAROO_SYS_TUNE_ENTER_MODE =
      0x03, /*!< Enters tune mode 1 (Teach), 2 (Limit Switches), or 3
               (Mechanical Stops). This corresponds to pressing the tune button
               to get to the mode you want.

                 Do not expect a reply from this command. */
  KANGAROO_SYS_TUNE_SET_DISABLED_CHANNELS =
      0x08, /*!< Initially all channels are disabled for safety reasons after
               entering a tune mode. You must send this bitmask before beginning
               the tune. 0 enables all channels.

                 Do not expect a reply from this command. */
  KANGAROO_SYS_TUNE_CONTROL_OPEN_LOOP =
      0x06, /*!< Sets the open loop power. This can be used to position for a
               Teach tune. The range is -(2^28-1) to 2^28-1.

                 Do not expect a reply from this command. */
  KANGAROO_SYS_TUNE_GO =
      0x04, /*!< Begins the tune.
                 This corresponds to pressing the tune button after you are in
               the desired mode.

                 Tuning has an automatic serial timeout for safety reasons.
                 You must continually send packets or it will abort.
               KangarooChannel::getPos() in a loop does the job.

                 Do not expect a reply from this command. */
  KANGAROO_SYS_TUNE_ABORT =
      0x05, /*!< Aborts the tune.

                 Do not expect a reply from this command. */
  KANGAROO_SYS_SET_BAUD_RATE =
      0x20,                               /*!< Sets the baud rate.
                              
                                               Do not expect a reply from this command. */
  KANGAROO_SYS_SET_SERIAL_TIMEOUT = 0x21  //!< Sets the serial timeout.
};

enum KangarooReplyCode { KANGAROO_RC_STATUS = 0x43 };

#endif  // TYPES_HPP