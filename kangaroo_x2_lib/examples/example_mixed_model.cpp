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

  err =
      drive.units(565, 1833);  // 100 CPR, 180 mm wheel radius, 320 mm wheelbase
  if (err != KANGAROO_NO_ERROR) {
    std::cerr << "Error setting drive units: " << toString(err) << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Channel 'D' unit set" << std::endl;

  err =
      turn.units(360, 3256);  // 100 CPR, 180 mm wheel radius, 320 mm wheelbase
  if (err != KANGAROO_NO_ERROR) {
    std::cerr << "Error setting turn units: " << toString(err) << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Channel 'T' unit set" << std::endl;

  long min_d = drive.getMin().value();
  long max_d = drive.getMax().value();

  std::cout << "Drive - Min: " << min_d << " / Max: " << max_d << std::endl;

  long min_t = turn.getMin().value();
  long max_t = turn.getMax().value();
  std::cout << "Turn - Min: " << min_t << " / Max: " << max_t << std::endl;

  double fw_avg = 0.0;
  double bw_avg = 0.0;
  double cw_avg = 0.0;
  double ccw_avg = 0.0;

  int d_acc = 2000;
  int t_acc = 720;

  int32_t speed = 2200;

  int read_count = 150;
  uint16_t real_count = 0;

  drive.setSpeed(0, d_acc);
  turn.setSpeed(0, t_acc);

  err = drive.setSpeed(speed, d_acc, KANGAROO_MOVE_DEFAULT).status().error();
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

  for (int i = 0; i < read_count; i++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    double d = drive.getSpeed().value();
    double t = turn.getSpeed().value();
    if (i >= 100) {
      real_count++;
      fw_avg += d;
    }
    std::cout << "D speed: " << d << "/" << drive.getSetPointSpeed().value()
              << std::endl
              << std::flush;
    std::cout << "T speed: " << t << "/" << turn.getSetPointSpeed().value()
              << std::endl
              << std::flush;
  }

  fw_avg /= real_count;

  drive.setSpeed(0, d_acc);
  turn.setSpeed(0, t_acc);

  speed = -2200;

  err = drive.setSpeed(speed, d_acc, KANGAROO_MOVE_DEFAULT).status().error();
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

  real_count = 0;

  for (int i = 0; i < read_count; i++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    double d = drive.getSpeed().value();
    double t = turn.getSpeed().value();
    if (i >= 100) {
      real_count++;
      bw_avg += d;
    }
    std::cout << "D speed: " << d << "/" << drive.getSetPointSpeed().value()
              << std::endl
              << std::flush;
    std::cout << "T speed: " << t << "/" << turn.getSetPointSpeed().value()
              << std::endl
              << std::flush;
  }

  bw_avg /= real_count;

  drive.setSpeed(0, d_acc);
  turn.setSpeed(0, t_acc);

  speed = 720;

  err = turn.setSpeed(speed, t_acc, KANGAROO_MOVE_DEFAULT).status().error();
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

  real_count = 0;

  for (int i = 0; i < read_count; i++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    double d = drive.getSpeed().value();
    double t = turn.getSpeed().value();
    if (i >= 100) {
      real_count++;
      ccw_avg += t;
    }
    std::cout << "D speed: " << d << "/" << drive.getSetPointSpeed().value()
              << std::endl
              << std::flush;
    std::cout << "T speed: " << t << "/" << turn.getSetPointSpeed().value()
              << std::endl
              << std::flush;
  }

  ccw_avg /= real_count;

  drive.setSpeed(0, d_acc);
  turn.setSpeed(0, t_acc);

  speed = -720;

  err = turn.setSpeed(speed, t_acc, KANGAROO_MOVE_DEFAULT).status().error();
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

  real_count = 0;

  for (int i = 0; i < read_count; i++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    double d = drive.getSpeed().value();
    double t = turn.getSpeed().value();
    if (i >= 100) {
      real_count++;
      cw_avg += t;
    }
    std::cout << "D speed: " << d << "/" << drive.getSetPointSpeed().value()
              << std::endl
              << std::flush;
    std::cout << "T speed: " << t << "/" << turn.getSetPointSpeed().value()
              << std::endl
              << std::flush;
  }

  cw_avg /= real_count;

  drive.setSpeed(0, d_acc);
  turn.setSpeed(0, t_acc);

  std::cout << std::endl;
  std::cout << " * Avg forward speed:\t\t" << fw_avg / 1000. << " m/sec"
            << std::endl;
  std::cout << " * Avg backward speed:\t\t" << bw_avg / 1000. << " m/sec"
            << std::endl;
  std::cout << " * Avg counterclockwise speed:\t" << ccw_avg << " °/sec"
            << std::endl;
  std::cout << " * Avg clockwise speed:\t\t" << cw_avg << " °/sec" << std::endl;

  return EXIT_SUCCESS;
}