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

#include "odometry.hpp"
namespace kx2
{

Odometry::Odometry(Mode mode)
{
  _mode = mode;
}

Odometry::~Odometry() {}

void Odometry::update(
  const double & d_speed, const double & t_speed,
  const std::uint64_t & ts)
{
  std::lock_guard<std::mutex> lock(_muxOdom);

  if (_mode == EXACT_INTEGRATION) {
    integrateExact(d_speed, t_speed, ts);
  } else if (_mode == RUNGE_KUTTA_INTEGRATION) {
    integrateRungeKutta(d_speed, t_speed, ts);
  }
}

void Odometry::integrateRungeKutta(
  const double & d_speed, const double & t_speed,
  const std::uint64_t & ts)
{
  // See https://github.com/mdrwiega/md_drive_ros/blob/master/src/odometry.cpp
  if (!_initialized) {

  }
}
void Odometry::integrateExact(
  const double & d_speed, const double & t_speed,
  const std::uint64_t & ts) {}

void Odometry::getOdometry(double & x_m, double & y_m, double & theta_rad, std::uint64_t & ts)
{
  std::lock_guard<std::mutex> lock(_muxOdom);

  x_m = _x;
  y_m = _y;
  theta_rad = _theta;
  ts = _ts;
}

}  // namespace kx2
