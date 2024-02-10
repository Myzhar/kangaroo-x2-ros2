/*
Arduino Library for Kangaroo
Copyright (c) 2013 Dimension Engineering LLC
http://www.dimensionengineering.com/kangaroo

Permission to use, copy, modify, and/or distribute this software for any
purpose with or without fee is hereby granted, provided that the above
copyright notice and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER
RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE
USE OR PERFORMANCE OF THIS SOFTWARE.
*/

#include "KangarooCommandWriter.hpp"

#include <string.h>

#include "KangarooCRC.hpp"
#include "tools.hpp"

KangarooCommandWriter::KangarooCommandWriter()
    : /*_address(0), _command(0),*/ _length(0) {
  memset(_data, 0, sizeof(_data));
}

void KangarooCommandWriter::write(uint8_t data) { _data[_length++] = data; }

void KangarooCommandWriter::write(const uint8_t* data, size_t length) {
  for (size_t i = 0; i < length; i++) {
    write(data[i]);
  }
}

void KangarooCommandWriter::writeBitPackedNumber(int32_t number) {
  number = constrain(number, -KANGAROO_BIT_PACKED_MAX_VALUE,
                     KANGAROO_BIT_PACKED_MAX_VALUE);

  uint32_t encodedNumber;
  if (number < 0) {
    encodedNumber = (uint32_t)(-number);
    encodedNumber <<= 1;
    encodedNumber |= 1;
  } else {
    encodedNumber = (uint32_t)number;
    encodedNumber <<= 1;
  }

  do {
    write((encodedNumber & 0x3f) | (encodedNumber >= 0x40 ? 0x40 : 0x00));
    encodedNumber >>= 6;
  } while (encodedNumber);
}

size_t KangarooCommandWriter::writeToBuffer(uint8_t* buffer, uint8_t address,
                                            KangarooCommand command) const {
  return writeToBuffer(buffer, address, command, data(), length());
}

size_t KangarooCommandWriter::writeToBuffer(uint8_t* buffer, uint8_t address,
                                            KangarooCommand command,
                                            const uint8_t* data,
                                            size_t lengthOfData) {
  size_t i = 0;

  buffer[i++] = address;
  buffer[i++] = (uint8_t)command;
  buffer[i++] = lengthOfData;
  for (size_t j = 0; j < lengthOfData; j++) {
    buffer[i++] = data[j];
  }

  uint16_t crc = KangarooCRC::value(buffer, i);
  buffer[i++] = crc & 0x7f;
  buffer[i++] = (crc >> 7) & 0x7f;

  return i;
}

void KangarooCommandWriter::writeToStream(Stream& port, uint8_t address,
                                          KangarooCommand command) const {
  writeToStream(port, address, command, data(), length());
}

void KangarooCommandWriter::writeToStream(Stream& port, uint8_t address,
                                          KangarooCommand command,
                                          const uint8_t* data,
                                          size_t lengthOfData) {
  uint8_t buffer[KANGAROO_COMMAND_MAX_BUFFER_LENGTH];
  size_t lengthOfBuffer =
      writeToBuffer(buffer, address, command, data, lengthOfData);
  port.writeBuffer(buffer, lengthOfBuffer);
}
