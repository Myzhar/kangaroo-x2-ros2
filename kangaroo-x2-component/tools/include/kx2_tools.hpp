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

#ifndef KX2_TOOLS_HPP
#define KX2_TOOLS_HPP

#include <rclcpp/clock.hpp>

#include "kx2_win_avg.hpp"

namespace kx2
{
namespace tools
{

/**
 * @brief Stop Timer used to measure time intervals
 *
 */
class StopWatch
{
public:
  explicit StopWatch(rclcpp::Clock::SharedPtr clock);
  ~StopWatch() {}

  void tic();    //!< Set the reference time point to the current time
  double toc(std::string func_name = std::string() );  //!< Returns the seconds elapsed from the last tic in ROS clock reference (it works also in simulation)

private:
  rclcpp::Time mStartTime;  // Reference time point
  rclcpp::Clock::SharedPtr mClockPtr;  // Node clock interface
};

} // namespace tools
} // namespace kx2
#endif  // KX2_TOOLS_HPP
