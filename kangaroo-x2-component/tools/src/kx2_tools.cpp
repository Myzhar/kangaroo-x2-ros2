#include "kx2_tools.hpp"
#include <iostream>

namespace kx2
{
namespace tools
{

StopWatch::StopWatch(rclcpp::Clock::SharedPtr clock)
: mStartTime(0, 0, RCL_ROS_TIME),
  mClockPtr(clock)
{
  tic();  // Start the timer at creation
}

void StopWatch::tic()
{
  mStartTime = mClockPtr->now();  // Reset the start time point
}

double StopWatch::toc(std::string func_name)
{
  auto now = mClockPtr->now();

  double elapsed_nsec = (now - mStartTime).nanoseconds();
  if (!func_name.empty()) {
    std::cerr << func_name << " -> toc elapsed_sec: " << elapsed_nsec / 1e9 << std::endl <<
      std::flush;
  }

  return elapsed_nsec / 1e9;  // Returns elapsed time in seconds
}

} // namespace tools
} // namespace kx2
