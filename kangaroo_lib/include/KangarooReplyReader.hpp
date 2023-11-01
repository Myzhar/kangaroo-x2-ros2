#ifndef KANGAROO_REPLY_READER_HPP
#define KANGAROO_REPLY_READER_HPP

#include "types.hpp"

class KangarooReplyReader {
 public:
  KangarooReplyReader(const byte* data, size_t length);

 public:
  boolean canRead() const;
  boolean tryRead(byte* value);
  byte read();
  int32_t readBitPackedNumber();

 private:
  const byte* _data;
  const byte* _dataEnd;
};

#endif  // KANGAROO_REPLY_READER_HPP