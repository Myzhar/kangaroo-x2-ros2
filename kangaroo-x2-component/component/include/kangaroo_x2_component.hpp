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

#ifndef KANGAROO_X2_COMPONENT
#define KANGAROO_X2_COMPONENT

#include <rcutils/logging_macros.h>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <string>

#include "Kangaroo.hpp"
#include "visibility_control.hpp"

namespace lc = rclcpp_lifecycle;

namespace kx2
{
class KangarooX2Component : public nav2_util::LifecycleNode
{
public:
  /// \brief Default constructor
  KANGAROO_X2_COMPONENTS_EXPORT
  explicit KangarooX2Component(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~KangarooX2Component();

  /// \brief Callback from transition to "configuring" state.
  /// \param[in] state The current state that the node is in.
  nav2_util::CallbackReturn on_configure(const lc::State & prev_state) override;

  /// \brief Callback from transition to "activating" state.
  /// \param[in] state The current state that the node is in.
  nav2_util::CallbackReturn on_activate(const lc::State & prev_state) override;

  /// \brief Callback from transition to "deactivating" state.
  /// \param[in] state The current state that the node is in.
  nav2_util::CallbackReturn on_deactivate(const lc::State & prev_state) override;

  /// \brief Callback from transition to "unconfigured" state.
  /// \param[in] state The current state that the node is in.
  nav2_util::CallbackReturn on_cleanup(const lc::State & prev_state) override;

  /// \brief Callback from transition to "shutdown" state.
  /// \param[in] state The current state that the node is in.
  nav2_util::CallbackReturn on_shutdown(const lc::State & prev_state) override;

  /// \brief Callback from transition to "error" state.
  /// \param[in] state The current state that the node is in.
  nav2_util::CallbackReturn on_error(const lc::State & prev_state) override;

  /// \brief Callback for diagnostic updater
  /// \param[in] stat The current diagnostic status
  void callback_updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper & stat);

protected:
  // ----> Node Parameters
  template<typename T>
  void getParam(
    std::string paramName, T defValue, T & outVal,
    const std::string & description = "", bool read_only = true,
    std::string log_info = std::string());

  void getParam(
    std::string paramName, int defValue, int & outVal,
    const nav2_util::LifecycleNode::integer_range & range,
    const std::string & description = "", bool read_only = true,
    std::string log_info = std::string());

  void getParam(
    std::string paramName, float defValue, float & outVal,
    const nav2_util::LifecycleNode::floating_point_range & range,
    const std::string & description = "", bool read_only = true,
    std::string log_info = std::string());

  void getParameters();
  void getDebugParams();
  void getCommParams();
  void getControlParams();
  // <---- Node Parameters

private:
  // Diagnostic updater
  diagnostic_updater::Updater _diagUpdater;

  // ----> Parameters
  bool _debugMode = false;

  std::string _serialPort = "/dev/tty0";  //!< Serial port name
  int _baudrate = 115200;                 //!< Serial baudrate
  int _readTimeOut_msec = 1000;           //!< Serial read timeout in msec

  float _wheelRad_mm = 0.0;  //!< Radius of the wheels [mm]
  float _trackWidth_mm =
    0.0;    //!< Distance between the middle of the wheels [mm]
  int _encLines =
    0;    //!< The counting feature of the encoder [Pulse per Round
          //!< (PPR) or lines]. That is CPR/4 [Counts per Round]
  float _gearRatio = 1.0;  //!< Motor gear ration -> _gearRation:1
  // <---- Parameters

  // ----> Diff Driver values
  uint32_t _d_dist;  //!<  Encoder Forward distance
  uint32_t _d_lines;  //!<  Encoder Forward lines
  uint32_t _t_lines;  //!<  Encoder Turn lines
  // <---- Diff Driver values
};

}  // namespace kx2

#endif  // KANGAROO_X2_COMPONENT
