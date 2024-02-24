#include <stdlib.h>

#include <iostream>
#include <thread>  // std::this_thread::sleep_for

#include "Kangaroo.hpp"

#define TUNE_MODE_NUMBER 1

int main(int argc, char *argv[]) {
  (void)argc;
  (void)argv;

  std::cout << std::endl
            << "*** Kangaroo x2 tuning ***" << std::endl;
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

  std::cout << std::endl
            << "Opening Kangaroo x2 on port '" << ser_port_name << "..."
            << std::endl;

  Stream stream;
  if (!stream.openSerialPort(ser_port_name, LibSerial::BaudRate::BAUD_115200)) {
    std::cerr << "Error opening serial port '" << ser_port_name << "'!"
              << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "serial connection OK." << ser_port_name << std::endl;

  // Give the Kangaroo some time to power up
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  // ----> Setup drive channels: Drive 'D', Turn 'T'
  KangarooSerial ser(stream);
  KangarooChannel drive(ser, 'D', 128);
  KangarooChannel turn(ser, 'T', 128);
  // <---- Setup drive channels: Drive 'D', Turn 'T'

  KangarooError err;

  // ----> Set drive units
  err = drive.units(out_d_dist, out_d_lines);  // 100 CPR, 180 mm wheel
                                               // radius, 320 mm wheelbase
  if (err != KANGAROO_NO_ERROR) {
    std::cerr << "Error setting drive units: " << toString(err) << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << " * Channel 'D' unit set" << std::endl;

  err = turn.units(
      360, out_t_lines);  // 100 CPR, 180 mm wheel radius, 320 mm wheelbase
  if (err != KANGAROO_NO_ERROR) {
    std::cerr << "Error setting turn units: " << toString(err) << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << " * Channel 'T' unit set" << std::endl;
  std::cout << std::endl;
  // <---- Set drive units

  // ---> Enter the desired tune mode.
  int32_t enterModeParams[1] = {TUNE_MODE_NUMBER};
  err = drive.systemCommand(KANGAROO_SYS_TUNE_ENTER_MODE, false,
                            enterModeParams, 1);

  if (err != KANGAROO_NO_ERROR) {
    std::cerr << "KANGAROO_SYS_TUNE_ENTER_MODE error: " << toString(err)
              << std::endl;
    return EXIT_FAILURE;
  }
  // <--- Enter the desired tune mode.

  // ----> Set the disabled channel bitmask to 0 (tune all channels)
  int32_t disableChannelsParams[1] = {0};
  err = drive.systemCommand(KANGAROO_SYS_TUNE_SET_DISABLED_CHANNELS, false,
                            disableChannelsParams, 1);
  if (err != KANGAROO_NO_ERROR) {
    std::cerr << "KANGAROO_SYS_TUNE_ENTER_MODE error: " << toString(err)
              << std::endl;
    return EXIT_FAILURE;
  }
  // <---- Set the disabled channel bitmask to 0 (tune all channels).

  // Begin the tune.
  err = drive.systemCommand(KANGAROO_SYS_TUNE_GO, false, nullptr, 0);
  if (err != KANGAROO_NO_ERROR) {
    std::cerr << "KANGAROO_SYS_TUNE_ENTER_MODE error: " << toString(err)
              << std::endl;
    return EXIT_FAILURE;
  }

  while (1) {
    std::cout << "Speed: " << drive.getSpeed().value() << " "
              << turn.getSpeed().value() << std::endl
              << std::flush;
    std::cout << "Pos: " << drive.getPos().value() << " "
              << turn.getPos().value() << std::endl
              << std::flush;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return EXIT_SUCCESS;
}