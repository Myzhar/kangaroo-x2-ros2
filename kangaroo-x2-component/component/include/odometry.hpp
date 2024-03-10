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

#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <cstdint>
#include <mutex>

namespace kx2
{

typedef enum _mode { EXACT_INTEGRATION = 0, RUNGE_KUTTA_INTEGRATION = 1 } Mode;

class Odometry
{
public:
  explicit Odometry(Mode mode = EXACT_INTEGRATION);
  virtual ~Odometry();

  void update(const double & d_speed, const double & t_speed, const std::uint64_t & ts);

  void getOdometry(
    double & x_m, double & y_m, double & theta_rad,
    std::uint64_t & ts);

protected:
  void integrateRungeKutta(
    const double & d_speed, const double & t_speed,
    const std::uint64_t & ts);
  void integrateExact(
    const double & d_speed, const double & t_speed,
    const std::uint64_t & ts);

private:
  std::mutex _muxOdom;

  Mode _mode;

  double _x_prec;
  double _y_prec;
  double _theta_prec;
  std::uint64_t _ts_prec;

  double _x;
  double _y;
  double _theta;
  std::uint64_t _ts;

  bool _initialized = false;
};

}  // namespace kx2

#endif  // ODOMETRY_HPP
