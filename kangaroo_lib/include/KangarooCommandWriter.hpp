#ifndef KANGAROO_COMMAND_WRITER_HPP
#define KANGAROO_COMMAND_WRITER_HPP

#include "types.hpp"
#include "Stream.hpp"

class KangarooCommandWriter {
 public:
  KangarooCommandWriter();

 public:
  inline const byte* data() const { return _data; }
  inline size_t length() const { return _length; }

 public:
  void write(byte data);
  void write(const byte* data, size_t length);
  void writeBitPackedNumber(int32_t number);

 public:
  size_t writeToBuffer(byte* buffer, byte address,
                       KangarooCommand command) const;
  static size_t writeToBuffer(byte* buffer, byte address,
                              KangarooCommand command, const byte* data,
                              size_t lengthOfData);
  void writeToStream(Stream& port, byte address, KangarooCommand command) const;
  static void writeToStream(Stream& port, byte address, KangarooCommand command,
                            const byte* data, size_t lengthOfData);

 private:
  byte _address, _command;
  size_t _length;
  byte _data[KANGAROO_COMMAND_MAX_DATA_LENGTH];
};

#endif  // KANGAROO_COMMAND_WRITER_HPP