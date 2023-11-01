#ifndef KANGAROO_REPLY_RECEIVER_HPP
#define KANGAROO_REPLY_RECEIVER_HPP

#include "types.hpp"

class KangarooReplyReceiver {
 public:
  KangarooReplyReceiver();

 public:
  inline byte address() const { return _data[0]; }
  inline KangarooReplyCode command() const {
    return (KangarooReplyCode)_data[1];
  }
  inline const byte* data() const { return &_data[3]; }
  inline size_t length() const { return _length - 5; }

 public:
  inline boolean ready() const { return _ready; }
  void read(byte data);
  void reset();

 private:
  size_t _length;
  boolean _ready;
  byte _data[KANGAROO_COMMAND_MAX_BUFFER_LENGTH];
};

#endif  // KANGAROO_REPLY_RECEIVER_HPP