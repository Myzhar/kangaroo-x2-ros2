#include <stdlib.h>

#include <iostream>
#include <thread>  // std::this_thread::sleep_for

#include "Kangaroo.hpp"
#include "tools.hpp"

int main(int argc, char *argv[]) {
  (void)argc;
  (void)argv;

  std::cout << std::endl << "*** Kangaroo x2 differential control test ***" << std::endl;
  std::cout << std::endl;

  // ----> Units setup calculation
  float radius = 90.0f;
  float baseline = 320.0f;
  uint32_t enc_lines = 100;
  float gear_ratio = 18.33333333333333333333f;
  uint32_t out_d_dist, out_d_lines, out_t_lines;
  calculateDiffDriveUnits(radius, baseline, enc_lines, gear_ratio, out_d_dist,
                          out_d_lines, out_t_lines);

  std::cout << "Robot Configuration: " << std::endl;
  std::cout << " * Wheel radius: " << radius << " mm" << std::endl;
  std::cout << " * Wheel distance: " << baseline << " mm" << std::endl;
  std::cout << " * Encoder lines: " << enc_lines << std::endl;
  std::cout << " * Gear ratio: " << gear_ratio << ":1" << std::endl;

  std::cout << std::endl;

  std::cout << "Kangaroo x2 Configuration: " << std::endl;
  std::cout << " * D, UNITS: " << out_d_dist << " mm = " << out_d_lines
            << " lines" << std::endl;
  std::cout << " * T, UNITS: " << 360 << "° = " << out_t_lines << " lines"
            << std::endl;
  // <---- Units setup calculation

  std::string ser_port_name = "/dev/ttyUSB0";

  std::cout << std::endl << "Opening Kangaroo x2 on port '" << ser_port_name << "..." << std::endl;

  Stream stream;
  if (!stream.openSerialPort(ser_port_name, LibSerial::BaudRate::BAUD_115200)) {
    std::cerr << "Error opening serial port '" << ser_port_name << "'!"
              << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "serial connection OK." << ser_port_name << std::endl;

  // ----> Setup drive channels: Drive 'D', Turn 'T'
  KangarooSerial ser(stream);
  KangarooChannel drive(ser, 'D', 128);
  KangarooChannel turn(ser, 'T', 128);
  // <---- Setup drive channels: Drive 'D', Turn 'T'

  KangarooError err;

  // ----> Start control channels
  err = drive.start();
  if (err != KANGAROO_NO_ERROR) {
    std::cerr << "Error starting the drive channel: " << toString(err)
              << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << " * Channel 'D' started" << std::endl;

  err = turn.start();
  if (err != KANGAROO_NO_ERROR) {
    std::cerr << "Error starting the turn channel: " << toString(err)
              << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << " * Channel 'T' started" << std::endl;
  std::cout << std::endl;
  // <---- Start control channels

  // ----> Set drive units
  err = drive.units(out_d_dist, out_d_lines);  // 100 CPR, 180 mm wheel
                                 // radius, 320 mm wheelbase
  if (err != KANGAROO_NO_ERROR) {
    std::cerr << "Error setting drive units: " << toString(err) << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << " * Channel 'D' unit set" << std::endl;

  err =
      turn.units(360, out_t_lines);  // 100 CPR, 180 mm wheel radius, 320 mm wheelbase
  if (err != KANGAROO_NO_ERROR) {
    std::cerr << "Error setting turn units: " << toString(err) << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << " * Channel 'T' unit set" << std::endl;
  std::cout << std::endl;
  // <---- Set drive units

  long min_d = drive.getMin().value();
  long max_d = drive.getMax().value();

  std::cout << " * Drive - Min: " << min_d << " / Max: " << max_d << " mm/sec"
            << std::endl;

  long min_t = turn.getMin().value();
  long max_t = turn.getMax().value();
  std::cout << " * Turn - Min: " << min_t << " / Max: " << max_t << " °/sec"
            << std::endl;
  std::cout << std::endl;

  double fw_avg = 0.0;
  double bw_avg = 0.0;
  double cw_avg = 0.0;
  double ccw_avg = 0.0;

  int d_acc = 2000;
  int t_acc = 720;

  int read_count = 150;  

  drive.setSpeed(0, d_acc);
  turn.setSpeed(0, t_acc);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::cout << std::endl;

  for (int cycle = 0; cycle < 4; cycle++) {
    int32_t d_speed = 0;
    int32_t t_speed = 0;

    double t_avg = 0.0;
    double d_avg = 0.0;

    uint16_t real_count = 0;

    switch (cycle) {
      case 0:
        std::cout << std::endl << "+++ Test forward speed +++" << std::endl;
        d_speed = max_d;
        t_speed = 0;
        break;
      case 1:
        std::cout << std::endl << "+++ Test backward speed +++" << std::endl;
        d_speed = -max_d;
        t_speed = 0;
        break;
      case 2:
        std::cout << std::endl << "+++ Test CCW turn speed +++" << std::endl;
        d_speed = 0;
        t_speed = max_t;
        break;
      case 3:
        std::cout << std::endl << "+++ Test CW turn speed +++" << std::endl;
        d_speed = 0;
        t_speed = -max_t;
        break;
    }

    err =
        drive.setSpeed(d_speed, d_acc, KANGAROO_MOVE_DEFAULT).status().error();
    if (err != KANGAROO_NO_ERROR) {
      std::cerr << "Error setting forward speed: " << toString(err)
                << std::endl;
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
    err = turn.setSpeed(t_speed, d_acc, KANGAROO_MOVE_DEFAULT).status().error();
    if (err != KANGAROO_NO_ERROR) {
      std::cerr << "Error setting turn speed: " << toString(err) << std::endl;
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
      if (i >= 50) {
        real_count++;
        d_avg += d;
        t_avg += t;
      }
      std::cout << "+ " << i + 1 << "/" << read_count;
      std::cout << " * D: " << d << "/" << drive.getSetPointSpeed().value();
      std::cout << " * T: " << t << "/" << turn.getSetPointSpeed().value()
                << "\r" << std::flush;
    }

    drive.setSpeed(0, d_acc);
    turn.setSpeed(0, t_acc);

    d_avg /= real_count;
    t_avg /= real_count;

    std::cout << std::endl;
    std::cout << "- Avg drive: " << d_avg / 1000. << " m/sec ";
    std::cout << "- Avg turn: " << t_avg << " °/sec ";

    switch (cycle) {
      case 0:
        fw_avg = d_avg / 1000.;
        break;
      case 1:
        bw_avg = d_avg / 1000.;
        break;
      case 2:
        ccw_avg = t_avg;
        break;
      case 3:
        cw_avg = t_avg;
        break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

  std::cout << std::endl;
  std::cout << " * Avg forward speed:\t\t" << fw_avg << " m/sec" << std::endl;
  std::cout << " * Avg backward speed:\t\t" << bw_avg << " m/sec" << std::endl;
  std::cout << " * Avg counterclockwise speed:\t" << ccw_avg << " °/sec"
            << std::endl;
  std::cout << " * Avg clockwise speed:\t\t" << cw_avg << " °/sec" << std::endl;

  return EXIT_SUCCESS;
}