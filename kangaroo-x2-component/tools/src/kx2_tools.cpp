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
