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

namespace kx2
{

typedef enum _mode { EXACT_INTEGRATION = 0, RUNGE_KUTTA_INTEGRATION = 1 } Mode;

class Odometry
{
public:
  explicit Odometry(Mode mode = EXACT_INTEGRATION);
  virtual ~Odometry();

private:
}

}  // namespace kx2

#endif  // ODOMETRY_HPP
