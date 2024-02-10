#ifndef STREAM_HPP
#define STREAM_HPP

#include "SerialPort.h"

#include <atomic>
#include <string>

#include "types.hpp"

class Stream {
 private:
  /* data */
 public:
  Stream();
  ~Stream();

  bool openSerialPort(
      std::string& port_name,
      const LibSerial::BaudRate& baudrate = LibSerial::BaudRate::BAUD_1152000);

  ssize_t writeBuffer(const uint8_t* buffer, size_t lengthOfBuffer);
  bool readByte(uint8_t& data);

 protected:
  void close();

 private:
  std::shared_ptr<LibSerial::SerialPort> _serial;
};

#endif  // STREAM_HPP