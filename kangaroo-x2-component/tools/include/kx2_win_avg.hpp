#ifndef KX2_WIN_AVG_HPP
#define KX2_WIN_AVG_HPP

#include <cstddef>  // size_t
#include <deque>    // std::dequeue
#include <mutex>

namespace kx2
{
namespace tools
{

class WinAvg
{
public:
  explicit WinAvg(size_t win_size = 15);
  ~WinAvg();

  double setNewSize(size_t win_size);
  double addValue(double val);

  /// @brief Get the current average of the stored values
  /// @return average of the stored values
  double getAvg();

  inline size_t size() {return _vals.size();}

private:
  size_t _winSize;

  std::deque<double> _vals;  // The values in the queue used to calculate the windowed average
  double _sumVals = 0.0;  // The updated sum of the values in the queue

  std::mutex mQueueMux;
};

}  // namespace tools
}  // namespace kx2

#endif  // SL_WIN_AVG_HPP
