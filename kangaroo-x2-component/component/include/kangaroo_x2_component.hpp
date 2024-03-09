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
#include <thread>

#include "Kangaroo.hpp"
#include "kx2_tools.hpp"
#include "visibility_control.hpp"

namespace lc = rclcpp_lifecycle;

namespace kx2
{
/*!
 * @brief ROS 2 Lifecycle Component for Kangaroo x2 control
 */
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
  void startMainThread();
  void stopMainThread();
  void mainThreadFunc();

  // ----> Kangaroo x2
  bool initKangarooX2();
  bool activateKangarooX2();
  // <---- Kangaroo x2

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
    std::string paramName, double defValue, double & outVal,
    const nav2_util::LifecycleNode::floating_point_range & range,
    const std::string & description = "", bool read_only = true,
    std::string log_info = std::string());

  void getParameters();
  void getDebugParams();
  void getCommParams();
  void getControlParams();
  // <---- Node Parameters

private:
  // ----> Threads
  std::thread _mainThread;  //!< Main thread
  bool _threadStop = false;
  // <---- Threads

  // Diagnostic updater
  diagnostic_updater::Updater _diagUpdater;

  // ----> Parameters
  bool _debugMode = false;

  std::string _serialPort = "/dev/tty0";  //!< Serial port name
  int _baudrate = 115200;                 //!< Serial baudrate
  int _readTimeOut_msec = 1000;           //!< Serial read timeout in msec

  // TODO(Walt) Add these parameters
  int _kx2Address = 128;       //!< Board address. Use "DEScribe" to configure
  char _kxDriveChannel = 'D';  //!< Drive channel. Use "DEScribe" to configure
  char _kxTurnChannel = 'T';   //!< Drive channel. Use "DEScribe" to configure

  double _wheelRad_mm = 0.0f;  //!< Radius of the wheels [mm]
  double _trackWidth_mm =
    0.0;              //!< Distance between the middle of the wheels [mm]
  int _encLines = 0;  //!< The counting feature of the encoder [Pulse per Round
                      //!< (PPR) or lines]. That is CPR/4 [Counts per Round]
  double _gearRatio = 1.0f;  //!< Motor gear ration -> _gearRation:1

  double _controlFreq = 20.0f;  //!< Main thread frequency
  // <---- Parameters

  // ----> Diff Driver values
  uint32_t _d_dist;   //!<  Encoder Drive distance
  uint32_t _d_lines;  //!<  Encoder Drive lines
  uint32_t _t_lines;  //!<  Encoder Turn lines
  // <---- Diff Driver values

  // ----> Diagnostic
  std::unique_ptr<tools::WinAvg> _avgMainThreadPeriod_sec;
  // <---- Diagnostic

  // ----> Kangaroo x2
  Stream _stream;  // Data stream
  std::unique_ptr<KangarooSerial> _kx2Serial;
  std::unique_ptr<KangarooChannel> _kx2ChDrive;
  std::unique_ptr<KangarooChannel> _kx2ChTurn;
  // <---- Kangaroo x2

  // ----> Motor control status
  double _d_speed;
  double _t_speed;
  // <---- Motor control status
};

}  // namespace kx2

#endif  // KANGAROO_X2_COMPONENT
