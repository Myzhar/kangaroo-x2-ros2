#ifndef KANGAROO_REPLY_RECEIVER_HPP
#define KANGAROO_REPLY_RECEIVER_HPP

#include "types.hpp"

class KangarooReplyReceiver
{
public:
  KangarooReplyReceiver();

public:
  inline uint8_t address() const {return _data[0];}
  inline KangarooReplyCode command() const
  {
    return (KangarooReplyCode)_data[1];
  }
  inline const uint8_t * data() const {return &_data[3];}
  inline size_t length() const {return _length - 5;}

public:
  inline bool ready() const {return _ready;}
  void read_byte(uint8_t data);
  void reset();

private:
  size_t _length;
  bool _ready;
  uint8_t _data[KANGAROO_COMMAND_MAX_BUFFER_LENGTH];
};

#endif  // KANGAROO_REPLY_RECEIVER_HPP
