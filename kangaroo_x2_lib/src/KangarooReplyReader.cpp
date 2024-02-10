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

#include "KangarooReplyReader.hpp"

KangarooReplyReader::KangarooReplyReader(const uint8_t* data, size_t length)
    : _data(data), _dataEnd(data + length) {}

bool KangarooReplyReader::canRead() const { return _data != _dataEnd; }

bool KangarooReplyReader::tryRead(uint8_t* value) {
  if (canRead()) {
    *value = read();
    return true;
  } else {
    return false;
  }
}

uint8_t KangarooReplyReader::read() {
  uint8_t data = *_data;
  _data++;
  return data;
}

int32_t KangarooReplyReader::readBitPackedNumber() {
  uint32_t encodedNumber = 0;
  uint8_t shift = 0;

  for (uint8_t i = 0; i < 5; i++)  // 30 bits
  {
    uint8_t uint8_t;
    if (!tryRead(&uint8_t)) {
      break;
    }

    encodedNumber |= (uint32_t)(uint8_t & 0x3f) << shift;
    shift += 6;
    if (!(uint8_t & 0x40)) {
      break;
    }
  }

  return (encodedNumber & 1) ? -(int32_t)(encodedNumber >> 1)
                             : (int32_t)(encodedNumber >> 1);
}
