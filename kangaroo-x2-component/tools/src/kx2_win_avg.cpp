////////////////////////////////////////////////////////////////////////////////
//  Copyright 2024 Walter Lucetti
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
////////////////////////////////////////////////////////////////////////////////

#include "kx2_win_avg.hpp"

namespace kx2
{
namespace tools
{

WinAvg::WinAvg(size_t win_size)
{
  _winSize = win_size;
  _sumVals = 0.0;
}

WinAvg::~WinAvg() {}

double WinAvg::setNewSize(size_t win_size)
{
  std::lock_guard<std::mutex> guard(mQueueMux);

  _winSize = win_size;
  while (_vals.size() > _winSize) {
    double val = _vals.back();
    _vals.pop_back();
    _sumVals -= val;
  }

  return _sumVals / _vals.size();
}

double WinAvg::addValue(double val)
{
  std::lock_guard<std::mutex> guard(mQueueMux);
  if (_vals.size() == _winSize) {
    double older = _vals.back();
    _vals.pop_back();
    _sumVals -= older;
  }

  _vals.push_front(val);
  _sumVals += val;

  auto avg = _sumVals / _vals.size();

  // std::cout << "New val: " << val << " - Size: " << _vals.size()
  // << " - Sum: " << _sumVals << " - Avg: " << avg << std::endl;

  return avg;
}

double WinAvg::getAvg()
{
  std::lock_guard<std::mutex> guard(mQueueMux);

  double avg = _sumVals / _vals.size();

  return avg;
}

}  // namespace tools

}  // namespace kx2
