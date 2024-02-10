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

#include "KangarooTimeout.hpp"

#include "tools.hpp"

KangarooTimeout::KangarooTimeout(uint64_t timeoutMS) : _timeoutMS(timeoutMS) {
  reset();
}

bool KangarooTimeout::canExpire() const { return _timeoutMS >= 0; }

bool KangarooTimeout::expired() const {
  auto now = millis();
  return canExpire() && (static_cast<int64_t>(now) - _start >= _timeoutMS);
}

void KangarooTimeout::expire() {
  if (!canExpire()) {
    return;
  }
  _start = millis() - _timeoutMS;
}

void KangarooTimeout::reset() { _start = millis(); }
