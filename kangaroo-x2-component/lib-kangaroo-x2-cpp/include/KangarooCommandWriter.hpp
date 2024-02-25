#ifndef KANGAROO_COMMAND_WRITER_HPP
#define KANGAROO_COMMAND_WRITER_HPP

#include "Stream.hpp"
#include "types.hpp"

class KangarooCommandWriter
{
public:
  KangarooCommandWriter();

public:
  inline const uint8_t * data() const {return _data;}
  inline size_t length() const {return _length;}

public:
  void write(uint8_t data);
  void write(const uint8_t * data, size_t length);
  void writeBitPackedNumber(int32_t number);

public:
  size_t writeToBuffer(
    uint8_t * buffer, uint8_t address,
    KangarooCommand command) const;
  static size_t writeToBuffer(
    uint8_t * buffer, uint8_t address,
    KangarooCommand command, const uint8_t * data,
    size_t lengthOfData);
  void writeToStream(
    Stream & port, uint8_t address,
    KangarooCommand command) const;
  static void writeToStream(
    Stream & port, uint8_t address,
    KangarooCommand command, const uint8_t * data,
    size_t lengthOfData);

private:
  //uint8_t _address, _command;
  size_t _length;
  uint8_t _data[KANGAROO_COMMAND_MAX_DATA_LENGTH];
};

#endif  // KANGAROO_COMMAND_WRITER_HPP
