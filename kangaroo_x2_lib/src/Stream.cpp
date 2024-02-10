#include "Stream.hpp"

#include <string.h>

#include <iostream>

Stream::Stream() {}

Stream::~Stream() { close(); }

bool Stream::openSerialPort(std::string& port_name,
                            const LibSerial::BaudRate& baudrate) {
  try {
    _serial = std::make_shared<LibSerial::SerialPort>(
        port_name, baudrate, LibSerial::CharacterSize::CHAR_SIZE_8,
        LibSerial::FlowControl::FLOW_CONTROL_NONE,
        LibSerial::Parity::PARITY_NONE, LibSerial::StopBits::STOP_BITS_1);

    if (!_serial->IsOpen()) _serial->Open(port_name);
  } catch (LibSerial::OpenFailed& ex) {
    std::cerr << "LibSerial open failed: " << ex.what() << std::endl;
    return false;
  }

  return _serial->IsOpen();
}

void Stream::close() {
  if (!_serial || !_serial->IsOpen()) {
    return;
  }

  _serial->Close();
}

ssize_t Stream::writeBuffer(const uint8_t* buffer, size_t lengthOfBuffer) {
  ssize_t len = -1;

  if (_serial->IsOpen()) {
    LibSerial::DataBuffer dataBuffer(buffer, buffer + lengthOfBuffer);

    _serial->Write(dataBuffer);
  }
  return len;
}

bool Stream::readByte(uint8_t& data) {
  if (!_serial->IsOpen()) {
    return false;
  }

  try {
    _serial->ReadByte(data, 25);
  } catch (LibSerial::ReadTimeout& ex) {
    std::cerr << "LibSerial read timeout: " << ex.what() << std::endl;
    return false;
  }

  return true;
}