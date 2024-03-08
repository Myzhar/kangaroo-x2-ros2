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
