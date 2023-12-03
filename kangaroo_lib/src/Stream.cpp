#include "Stream.hpp"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>

Stream::Stream() : _comHandle(-1), _isOpen(false) {}

Stream::~Stream() { close(); }

  replace termios with libserial calls


bool Stream::openSerialPort(std::string& port_name, speed_t baudrate) {
  int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

  _comHandle = ::open(port_name.c_str(), flags);
  if (-1 == _comHandle) {
    std::cerr << "openSerialPort::Open open error!" << std::endl;
    return false;
  }

  _isOpen = true;

  // get port options
  struct termios options;
  if (-1 == ::tcgetattr(_comHandle, &options)) {
    std::cerr << "Stream::openSerialPort::Open tcgetattr error!" << std::endl;
    close();
    return false;
  }

  options.c_cflag |= (tcflag_t)(CLOCAL | CREAD | CS8 | CRTSCTS);
   options.c_cflag &= (tcflag_t) ~(CSTOPB | PARENB | PARODD);
  options.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL |
                                  ISIG | IEXTEN);  // |ECHOPRT
  options.c_oflag &= (tcflag_t) ~(OPOST);
  options.c_iflag &=
      (tcflag_t) ~(IXON | IXOFF | INLCR | IGNCR | ICRNL | IGNBRK);

  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  if (::cfsetispeed(&options, baudrate) == -1) {
    std::cerr << "Stream::openSerialPort::Open cfsetispeed error!" << std::endl;
    close();
    return false;
  }

  if (tcsetattr(_comHandle, TCSANOW, &options) < 0) {
    std::cerr << "Stream::openSerialPort::Open tcsetattr error!" << std::endl;
    close();
    return false;
  }

  if (::tcflush(_comHandle, TCIFLUSH) == -1) {
    std::cerr << "Stream::openSerialPort::Open tcflush error!" << std::endl;
    close();
    return false;
  }

  _isOpen = true;

  return true;
}

void Stream::close() {
  if (_isOpen == false) {
    return;
  }

  if (_comHandle != -1) {
    close();
    _comHandle = -1;
  }

  _isOpen = false;
}

ssize_t Stream::write(const byte* buffer, size_t lengthOfBuffer) {
  ssize_t len = -1;

  if (_isOpen) {
    len = ::write(_comHandle, buffer, lengthOfBuffer);
  }
  return len;
}

bool Stream::read(byte& data) {
  static timespec timeout = {0, static_cast<int>(100 * 1e6)};
  ssize_t len = -1;

  if (_isOpen) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(_comHandle, &read_fds);
    int r = ::pselect(_comHandle + 1, &read_fds, NULL, NULL, &timeout, NULL);
    if (r < 0) {
      // Select was interrupted
      if (errno == EINTR) {
        return false;
      }
    } else if (r == 0) {  // timeout
      return false;
    }

    if (FD_ISSET(_comHandle, &read_fds)) {
      len = ::read(_comHandle, &data, 1);
    }
  }

  return len == 1;
}