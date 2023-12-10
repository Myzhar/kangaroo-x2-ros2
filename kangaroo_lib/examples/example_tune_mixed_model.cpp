#include <stdlib.h>

#include <iostream>
#include <thread>  // std::this_thread::sleep_for

#include "Kangaroo.hpp"

#define TUNE_MODE_NUMBER 1

int main(int argc, char *argv[]) {
  std::string ser_port_name = "/dev/ttyUSB0";
  Stream stream;
  if (!stream.openSerialPort(ser_port_name, LibSerial::BaudRate::BAUD_115200)) {
    std::cerr << "Error opening serial port '" << ser_port_name << "'!"
              << std::endl;
    return EXIT_FAILURE;
  }

  // Give the Kangaroo some time to power up
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  KangarooSerial ser(stream);
  KangarooChannel drive(ser, 'D', 128);
  KangarooChannel turn(ser, 'T', 128);

  KangarooError err;

    // Enter the desired tune mode.
  int32_t enterModeParams[1] = { TUNE_MODE_NUMBER };
  err = drive.systemCommand(KANGAROO_SYS_TUNE_ENTER_MODE, false, enterModeParams, 1);

  if(err!=KANGAROO_NO_ERROR) {
    std::cerr << "KANGAROO_SYS_TUNE_ENTER_MODE error: " << toString(err) << std::endl;
    return EXIT_FAILURE;
  }

  // Set the disabled channel bitmask to 0 (tune all channels).
  int32_t disableChannelsParams[1] = { 0 };
  err = drive.systemCommand(KANGAROO_SYS_TUNE_SET_DISABLED_CHANNELS, false, disableChannelsParams, 1);
  if(err!=KANGAROO_NO_ERROR) {
    std::cerr << "KANGAROO_SYS_TUNE_ENTER_MODE error: " << toString(err) << std::endl;
    return EXIT_FAILURE;
  }

  // Begin the tune.
  int32_t goParams[0];
  err = drive.systemCommand(KANGAROO_SYS_TUNE_GO, false, goParams, 0);
  if(err!=KANGAROO_NO_ERROR) {
    std::cerr << "KANGAROO_SYS_TUNE_ENTER_MODE error: " << toString(err) << std::endl;
    return EXIT_FAILURE;
  }

  while(1)
  {
     std::cout << "Speed: " << drive.getS().value() << " " << turn.getS().value() << std::endl << std::flush;
     std::cout << "Pos: " << drive.getP().value() << " " << turn.getP().value() << std::endl << std::flush;
     std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return EXIT_SUCCESS;
}