#include <stdlib.h>

#include <iostream>
#include <thread>  // std::this_thread::sleep_for

#include "Kangaroo.hpp"

int main(int argc, char *argv[]) {
  (void)argc;
  (void)argv;

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
    std::cerr << "Error starting the drive channel: " << toString(err)
              << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Channel 'D' started" << std::endl;

  err = turn.start();
  if (err != KANGAROO_NO_ERROR) {
    std::cerr << "Error starting the turn channel: " << toString(err)
              << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Channel 'T' started" << std::endl;

  // err = drive.units(565, 4583);
  // if (err != KANGAROO_NO_ERROR) {
  //   std::cerr << "Error setting drive units: " << toString(err)
  //             << std::endl;
  //   return EXIT_FAILURE;
  // }
  // std::cout << "Channel 'D' unit set" << std::endl;

  // err = turn.units(360, 8147);
  // if (err != KANGAROO_NO_ERROR) {
  //   std::cerr << "Error setting turn units: " << toString(err) << std::endl;
  //   return EXIT_FAILURE;
  // }
  // std::cout << "Channel 'T' unit set" << std::endl;

  long min_d = drive.getMin().value();
  long max_d = drive.getMax().value();

  std::cout << "Drive - Min: " << min_d << " / Max: " << max_d << std::endl;

  long min_t = turn.getMin().value();
  long max_t = turn.getMax().value();
  std::cout << "Turn - Min: " << min_t << " / Max: " << max_t << std::endl;

  drive.setSpeed(0,1000);
  turn.setSpeed(0,360);

  int32_t speed = 2200;

  err = drive.setSpeed(speed, 1000, KANGAROO_MOVE_DEFAULT).status().error();
  if (err != KANGAROO_NO_ERROR) {
    std::cerr << "Error setting forward speed: " << toString(err) << std::endl;
    err = drive.powerDown();
    if (err != KANGAROO_NO_ERROR) {
      std::cerr << "Error shutting down Drive channel: " << toString(err)
                << std::endl;
    }
    err = turn.powerDown();
    if (err != KANGAROO_NO_ERROR) {
      std::cerr << "Error shutting down Drive channel: " << toString(err)
                << std::endl;
    }
    return EXIT_FAILURE;
  }

  for (int i = 0; i < 100; i++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "D speed: " << drive.getSpeed().value() << "/" << drive.getSetPointSpeed().value()<< std::endl
              << std::flush;
    std::cout << "T speed: " << turn.getSpeed().value() << "/"
              << turn.getSetPointSpeed().value() << std::endl
              << std::flush;
  }

  speed = -2200;

  err = drive.setSpeed(speed, 1000, KANGAROO_MOVE_DEFAULT).status().error();
  if (err != KANGAROO_NO_ERROR) {
    std::cerr << "Error setting forward speed: " << toString(err) << std::endl;
    err = drive.powerDown();
    if (err != KANGAROO_NO_ERROR) {
      std::cerr << "Error shutting down Drive channel: " << toString(err)
                << std::endl;
    }
    err = turn.powerDown();
    if (err != KANGAROO_NO_ERROR) {
      std::cerr << "Error shutting down Drive channel: " << toString(err)
                << std::endl;
    }
    return EXIT_FAILURE;
  }

  for (int i = 0; i < 100; i++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "D speed: " << drive.getSpeed().value() << "/"
              << drive.getSetPointSpeed().value() << std::endl
              << std::flush;
    std::cout << "T speed: " << turn.getSpeed().value() << "/"
              << turn.getSetPointSpeed().value() << std::endl
              << std::flush;
  }

  drive.setSpeed(0, 1000);
  turn.setSpeed(0, 360);

  speed = 720;

  err = turn.setSpeed(speed, 360, KANGAROO_MOVE_DEFAULT).status().error();
  if (err != KANGAROO_NO_ERROR) {
    std::cerr << "Error setting forward speed: " << toString(err) << std::endl;
    err = drive.powerDown();
    if (err != KANGAROO_NO_ERROR) {
      std::cerr << "Error shutting down Drive channel: " << toString(err)
                << std::endl;
    }
    err = turn.powerDown();
    if (err != KANGAROO_NO_ERROR) {
      std::cerr << "Error shutting down Drive channel: " << toString(err)
                << std::endl;
    }
    return EXIT_FAILURE;
  }

  for (int i = 0; i < 100; i++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "D speed: " << drive.getSpeed().value() << "/"
              << drive.getSetPointSpeed().value() << std::endl
              << std::flush;
    std::cout << "T speed: " << turn.getSpeed().value() << "/"
              << turn.getSetPointSpeed().value() << std::endl
              << std::flush;
  }

  speed = -720;

  err = turn.setSpeed(speed, 360, KANGAROO_MOVE_DEFAULT).status().error();
  if (err != KANGAROO_NO_ERROR) {
    std::cerr << "Error setting forward speed: " << toString(err) << std::endl;
    err = drive.powerDown();
    if (err != KANGAROO_NO_ERROR) {
      std::cerr << "Error shutting down Drive channel: " << toString(err)
                << std::endl;
    }
    err = turn.powerDown();
    if (err != KANGAROO_NO_ERROR) {
      std::cerr << "Error shutting down Drive channel: " << toString(err)
                << std::endl;
    }
    return EXIT_FAILURE;
  }

  for (int i = 0; i < 100; i++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "D speed: " << drive.getSpeed().value() << "/"
              << drive.getSetPointSpeed().value() << std::endl
              << std::flush;
    std::cout << "T speed: " << turn.getSpeed().value() << "/"
              << turn.getSetPointSpeed().value() << std::endl
              << std::flush;
  }

  drive.setSpeed(0, 1000);
  turn.setSpeed(0, 360);

  return EXIT_SUCCESS;
}