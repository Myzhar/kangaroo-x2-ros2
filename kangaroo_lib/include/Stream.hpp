#ifndef STREAM_HPP
#define STREAM_HPP

#include "types.hpp"
#include <string>
#include <atomic>
#include <termios.h>

class Stream {
 private:
  /* data */
 public:
  Stream(/* args */);
  ~Stream();

  bool openSerialPort(std::string& port_name, int baudrate=B115200);

  ssize_t write(const byte* buffer, size_t lengthOfBuffer);
  bool read(byte& data);

 protected:
  
  void close();

  private:
  int32_t _comHandle;

  std::atomic<bool> _isOpen;
};

#endif  // STREAM_HPP