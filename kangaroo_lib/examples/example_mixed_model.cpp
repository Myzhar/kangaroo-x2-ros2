#include <stdlib.h>

#include <iostream>
#include <thread>  // std::this_thread::sleep_for

#include "Kangaroo.hpp"

int main(int argc, char *argv[]) {
  std::string ser_port_name = "/dev/ttyUSB0";
  Stream stream;
  if (!stream.openSerialPort(ser_port_name, LibSerial::BaudRate::BAUD_115200)) {
    std::cerr << "Error opening serial port '" << ser_port_name << "'!"
              << std::endl;
    return EXIT_FAILURE;
  }

  KangarooSerial ser(stream);
  KangarooChannel drive(ser, 'D', 128);
  KangarooChannel turn(ser, 'T', 128);

  KangarooError err;
  err = drive.start();
  if (err != KANGAROO_NO_ERROR) {
    std::cerr << "Error starting the drive channel'" << static_cast<int>(err)
              << "' !" << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Channel 'D' started" << std::endl;

  err = turn.start();
  if (err != KANGAROO_NO_ERROR) {
    std::cerr << "Error starting the turn channel'" << static_cast<int>(err)
              << "' !" << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Channel 'T' started" << std::endl;

  drive.pi(0);
  turn.pi(0);

  for (int i = 0; i < 10; i++) {
    drive.pi(500);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    turn.pi(500);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  for (int i = 0; i < 10; i++) {
    drive.pi(-500);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    turn.pi(-500);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  return EXIT_SUCCESS;
}