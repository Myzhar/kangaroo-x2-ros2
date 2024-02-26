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

#include "kangaroo_x2_component.hpp"

namespace kx2
{

KangarooX2Component::KangarooX2Component(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("lidar_node", "", options), _diagUpdater(this)
{
  RCLCPP_INFO(get_logger(), "***********************************************");
  RCLCPP_INFO(get_logger(), " Kangaroo X2 Motor Control Lifecycle Component ");
  RCLCPP_INFO(
    get_logger(),
    " * **********************************************");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(
    get_logger(),
    " * **********************************************");
  RCLCPP_INFO(
    get_logger(),
    " + State: 'unconfigured [1]'. Use lifecycle commands "
    "to configure [1] or shutdown [5]");

  // ----> Diagnostic
  _diagUpdater.add(
    "Kangaroo X2 Motor Control Diagnostic", this,
    &KangarooX2Component::callback_updateDiagnostic);
  _diagUpdater.setHardwareID("Kangaroo X2 Motor Control");
  // <---- Diagnostic
}

KangarooX2Component::~KangarooX2Component() {}

nav2_util::CallbackReturn KangarooX2Component::on_configure(
  const lc::State & prev_state)
{
  return nav2_util::CallbackReturn::ERROR;
}

nav2_util::CallbackReturn KangarooX2Component::on_activate(
  const lc::State & prev_state)
{
  return nav2_util::CallbackReturn::ERROR;
}

nav2_util::CallbackReturn KangarooX2Component::on_deactivate(
  const lc::State & prev_state)
{
  return nav2_util::CallbackReturn::ERROR;
}

nav2_util::CallbackReturn KangarooX2Component::on_cleanup(
  const lc::State & prev_state)
{
  return nav2_util::CallbackReturn::ERROR;
}

nav2_util::CallbackReturn KangarooX2Component::on_shutdown(
  const lc::State & prev_state)
{
  return nav2_util::CallbackReturn::ERROR;
}

nav2_util::CallbackReturn KangarooX2Component::on_error(
  const lc::State & prev_state)
{
  return nav2_util::CallbackReturn::ERROR;
}

void KangarooX2Component::callback_updateDiagnostic(
  diagnostic_updater::DiagnosticStatusWrapper & stat) {}

void KangarooX2Component::getParameters()
{
  RCLCPP_INFO_STREAM(get_logger(), "****** NODE PARAMETERS ******");

  // DEBUG parameters
  getDebugParams();

  // COMMUNICATION parameters
  getCommParams();

  // MOTOR CONTROL parameters
  getControlParams();
}

template<typename T>
void KangarooX2Component::getParam(
  std::string paramName, T defValue, T & outVal,
  const std::string & description,
  bool read_only, std::string log_info)
{
  try {
    add_parameter(
      paramName, rclcpp::ParameterValue(defValue), description, "",
      read_only);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) {
    RCLCPP_DEBUG_STREAM(get_logger(), "Exception: " << ex.what());
  }

  if (!get_parameter(paramName, outVal)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The parameter '"
        << paramName
        << "' is not available or is not valid, using the default value: "
        << defValue);
  }

  if (!log_info.empty()) {
    RCLCPP_INFO_STREAM(get_logger(), log_info << outVal);
  }
}

void KangarooX2Component::getParam(
  std::string paramName, int defValue, int & outVal,
  const nav2_util::LifecycleNode::integer_range & range,
  const std::string & description, bool read_only, std::string log_info)
{
  try {
    add_parameter(
      paramName, rclcpp::ParameterValue(defValue), range,
      description, "", read_only);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) {
    RCLCPP_DEBUG_STREAM(get_logger(), "Exception: " << ex.what());
  }

  if (!get_parameter(paramName, outVal)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The parameter '"
        << paramName
        << "' is not available or is not valid, using the default value: "
        << defValue);
  }

  if (!log_info.empty()) {
    RCLCPP_INFO_STREAM(get_logger(), log_info << outVal);
  }
}

void KangarooX2Component::getParam(
  std::string paramName, float defValue, float & outVal,
  const nav2_util::LifecycleNode::floating_point_range & range,
  const std::string & description, bool read_only, std::string log_info)
{
  try {
    add_parameter(
      paramName, rclcpp::ParameterValue(defValue), range,
      description, "", read_only);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) {
    RCLCPP_DEBUG_STREAM(get_logger(), "Exception: " << ex.what());
  }

  if (!get_parameter(paramName, outVal)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The parameter '"
        << paramName
        << "' is not available or is not valid, using the default value: "
        << defValue);
  }

  if (!log_info.empty()) {
    RCLCPP_INFO_STREAM(get_logger(), log_info << outVal);
  }
}

void KangarooX2Component::getDebugParams() {}
void KangarooX2Component::getCommParams() {}
void KangarooX2Component::getControlParams() {}
}  // namespace kx2
