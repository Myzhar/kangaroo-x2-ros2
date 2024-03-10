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

#include <sstream>

using std::placeholders::_1;

namespace kx2
{

const int QOS_QUEUE_SIZE = 10;
const double RAD2DEG = 180. / M_PI;

KangarooX2Component::KangarooX2Component(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("lidar_node", "", options), _diagUpdater(this)
{
  RCLCPP_INFO(get_logger(), "***********************************************");
  RCLCPP_INFO(get_logger(), " Kangaroo X2 Motor Control Lifecycle Component ");
  RCLCPP_INFO(get_logger(), "**********************************************");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(get_logger(), "**********************************************");
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

KangarooX2Component::~KangarooX2Component()
{
  // Stop Main THread
  stopMainThread();
}

bool KangarooX2Component::initKangarooX2()
{
  RCLCPP_INFO(get_logger(), "*** Initialize Kangaroo x2 ***");

  // ----> Calculate diff drive coefficients
  kx2::calculateDiffDriveUnits(
    _wheelRad_mm, _trackWidth_mm, _encLines,
    _gearRatio, _d_dist, _d_lines, _t_lines);

  RCLCPP_INFO(
    get_logger(),
    " * Kangaroo x2 Configuration [Differential drive]: ");
  RCLCPP_INFO_STREAM(
    get_logger(),
    "   - [Drive channel] '" << _kxDriveChannel << "', UNITS : " << _d_dist << " mm = " << _d_lines <<
      " lines");
  RCLCPP_INFO_STREAM(
    get_logger(),
    "   - [Turn channel] '" << _kxTurnChannel << "', UNITS: " << 360 << "° = " << _t_lines <<
      " lines");
  // <---- Calculate diff drive coefficients

  return true;
}

nav2_util::CallbackReturn KangarooX2Component::on_configure(
  const lc::State & prev_state)
{
  RCLCPP_DEBUG_STREAM(
    get_logger(),
    "on_configure: " << prev_state.label() << " ["
                     << static_cast<int>(prev_state.id())
                     << "] -> Inactive");

  // Initialize parameters from yaml file
  getParameters();

  // Initialize Kangaroo x2
  if (!initKangarooX2()) {
    return nav2_util::CallbackReturn::ERROR;
  }

  // ----> Initialize Avg Mean
  _avgMainThreadPeriod_sec =
    std::make_unique<tools::WinAvg>(static_cast<size_t>(_controlFreq));
  // <---- Initialize Avg Mean

  RCLCPP_INFO(
    get_logger(),
    " + State: 'inactive [2]'. Use lifecycle commands to "
    "activate [3], cleanup [2] or shutdown "
    "[6]");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool KangarooX2Component::activateKangarooX2()
{
  RCLCPP_INFO(get_logger(), "*** Activate Kangaroo x2 ***");

  // ----> Setup Communication
  LibSerial::BaudRate baud;
  switch (_baudrate) {
    case 9600:
      baud = LibSerial::BaudRate::BAUD_9600;
      break;
    case 19200:
      baud = LibSerial::BaudRate::BAUD_19200;
      break;
    case 38400:
      baud = LibSerial::BaudRate::BAUD_38400;
      break;
    case 57600:
      baud = LibSerial::BaudRate::BAUD_57600;
      break;
    case 115200:
      baud = LibSerial::BaudRate::BAUD_115200;
      break;
    default:
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Baudrate " << _baudrate
                    << " not valid. Please set a correct parameter value.");
      return false;
  }
  if (!_stream.openSerialPort(_serialPort, baud)) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Error opening serial port '" << _serialPort << "'!");
    return false;
  }
  // <---- Setup Communication

  // ----> Setup drive channels: Drive 'D', Turn 'T'
  _kx2Serial = std::make_unique<KangarooSerial>(_stream);
  _kx2ChDrive = std::make_unique<KangarooChannel>(
    *_kx2Serial, _kxDriveChannel,
    _kx2Address);
  _kx2ChTurn = std::make_unique<KangarooChannel>(
    *_kx2Serial, _kxTurnChannel,
    _kx2Address);
  // <---- Setup drive channels: Drive 'D', Turn 'T'

  KangarooError err;

  // ----> Start control channels
  err = _kx2ChDrive->start();
  if (err != KANGAROO_NO_ERROR) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Error starting the DRIVE channel: " << toString(err));
    return false;
  }
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Channel '" << _kxDriveChannel << "' started");

  err = _kx2ChTurn->start();
  if (err != KANGAROO_NO_ERROR) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Error starting the TURN channel: " << toString(err));
    return false;
  }
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Channel '" << _kxTurnChannel << "' started");
  // <---- Start control channels

  // ----> Set drive units
  err = _kx2ChDrive->units(_d_dist, _d_lines);
  if (err != KANGAROO_NO_ERROR) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Error setting DRIVE units: " << toString(err));
    return false;
  }
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Channel '" << _kxDriveChannel << "' units set");

  err = _kx2ChTurn->units(360, _t_lines);
  if (err != KANGAROO_NO_ERROR) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Error setting TURN units: " << toString(err));
    return false;
  }
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Channel '" << _kxTurnChannel << "' units set");
  // <---- Set drive units

  return true;
}

nav2_util::CallbackReturn KangarooX2Component::on_activate(
  const lc::State & prev_state)
{
  RCLCPP_DEBUG_STREAM(
    get_logger(),
    "on_activate: " << prev_state.label() << " ["
                    << static_cast<int>(prev_state.id())
                    << "] -> Active");

  _diagUpdater.force_update();

  // create bond connection
  createBond();

  // TODO Activate publishers
  //_pub->on_activate();

  // Activate Kangaroo X2
  if (!activateKangarooX2()) {
    return nav2_util::CallbackReturn::ERROR;
  }

  // Start main thread
  startMainThread();

  // ----> Create subscribers
  rclcpp::QoS qos(QOS_QUEUE_SIZE);
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();

  _cmdVelSub = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", qos,
    std::bind(&KangarooX2Component::cmd_vel_callback, this, _1), sub_opt);
  // <---- Create subscribers

  RCLCPP_INFO(
    get_logger(),
    " + State: 'active [3]'. Use lifecycle commands to "
    "deactivate [4] or shutdown [7]");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn KangarooX2Component::on_deactivate(
  const lc::State & prev_state)
{
  RCLCPP_DEBUG_STREAM(
    get_logger(),
    "on_deactivate: " << prev_state.label() << " ["
                      << static_cast<int>(prev_state.id())
                      << "] -> Inactive");

  _diagUpdater.force_update();

  // ----> Reset subscribers
  _cmdVelSub.reset();
  // <---- Reset subscribers

  // Stop Main THread
  stopMainThread();

  // Dectivate publisher
  //_scanPub->on_deactivate();

  // ----> Shutdown motors
  KangarooError d_err = _kx2ChDrive->powerDown();
  if (d_err != KANGAROO_NO_ERROR) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      " Drive channel powerdown error: " << toString(d_err));
  }
  KangarooError t_err = _kx2ChTurn->powerDown();
  if (t_err != KANGAROO_NO_ERROR) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      " Turn channel powerdown error: " << toString(t_err));
  }
  // <---- Shutdown motors

  // ----> Close connection
  _kx2ChDrive.reset();
  _kx2ChTurn.reset();
  _kx2Serial.reset();
  _stream.close();
  // <---- Close connection

  // destroy bond connection
  destroyBond();

  RCLCPP_INFO(
    get_logger(),
    " + State: 'inactive [2]'. Use lifecycle commands to "
    "activate [3], cleanup [2] or shutdown "
    "[6]");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn KangarooX2Component::on_cleanup(
  const lc::State & prev_state)
{
  RCLCPP_DEBUG_STREAM(
    get_logger(),
    "on_cleanup: " << prev_state.label() << " ["
                   << static_cast<int>(prev_state.id())
                   << "] -> Unconfigured");

  _diagUpdater.force_update();

  // ----> Destroy subscribers
  _cmdVelSub.reset();
  // <---- Destroy subscribers

  RCLCPP_INFO(
    get_logger(),
    " + State: 'unconfigured [1]'. Use lifecycle commands "
    "to configure [1] or shutdown [5]");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn KangarooX2Component::on_shutdown(
  const lc::State & prev_state)
{
  RCLCPP_DEBUG_STREAM(
    get_logger(),
    "on_shutdown: " << prev_state.label() << " ["
                    << static_cast<int>(prev_state.id())
                    << "] -> Finalized");

  _diagUpdater.force_update();

  //_scanPub.reset();

  // Stop Main Thread
  stopMainThread();

  RCLCPP_INFO_STREAM(
    get_logger(),
    " + State: 'finalized [4]'. Press Ctrl+C to kill...");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn KangarooX2Component::on_error(
  const lc::State & prev_state)
{
  RCLCPP_DEBUG_STREAM(
    get_logger(),
    "on_error: " << prev_state.label() << " ["
                 << static_cast<int>(prev_state.id())
                 << "] -> Finalized");

  _diagUpdater.force_update();

  RCLCPP_INFO_STREAM(
    get_logger(),
    " + State: 'finalized [4]'. Press Ctrl+C to kill...");
  return nav2_util::CallbackReturn::FAILURE;
}

void KangarooX2Component::callback_updateDiagnostic(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // RCLCPP_DEBUG(get_logger(), "callback_updateDiagnostic");

  // ----> Lifecycle state
  auto state = this->get_current_state();

  std::stringstream ss;
  ss << "Lifecycle state: " << state.label() << " ["
     << static_cast<int>(state.id()) << "]";
  if (state.id() != 3) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, ss.str());
    return;
  }

  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, ss.str());
  // <---- Lifecycle state

  // ----> Main thread frequency
  if (_avgMainThreadPeriod_sec) {
    double freq = 1.0 / _avgMainThreadPeriod_sec->getAvg();
    double freq_perc = 100. * freq / _controlFreq;
    if (freq < 0.8 * _controlFreq) {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Control frequency too low. Consider lowering the value of "
        "'general.control_freq'");
    }
    stat.addf(
      "Control thread", "Mean Frequency: %.1f Hz (%.1f%%)", freq,
      freq_perc);
  }
  // <---- Main thread frequency

  // ----> Motor control
  ss = std::stringstream();
  ss << "Current: " << _d_speed
     << " m/sec - SetPoint: " << (static_cast<double>(_d_speedSetpoint) / 1000.)
     << " m/sec";
  stat.addf("Drive speed", ss.str().c_str());
  ss = std::stringstream();
  ss << "Current: " << _t_speed << " °/sec - SetPoint: " << _t_speedSetpoint
     << " °/sec";
  stat.addf("Turn speed", ss.str().c_str());
  // <---- Motor control
}

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
  std::string paramName, double defValue, double & outVal,
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

void KangarooX2Component::getDebugParams()
{
  // ----> Debug mode initialization from parameters
  getParam(
    "general.debug_mode", _debugMode, _debugMode,
    "Enable debug messages");
  if (_debugMode) {
    rcutils_ret_t res = rcutils_logging_set_logger_level(
      get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    if (res != RCUTILS_RET_OK) {
      RCLCPP_INFO(get_logger(), "Error setting DEBUG level for logger");
    } else {
      RCLCPP_INFO(get_logger(), "*** Debug Mode enabled ***");
    }
  } else {
    rcutils_ret_t res = rcutils_logging_set_logger_level(
      get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

    if (res != RCUTILS_RET_OK) {
      RCLCPP_INFO(get_logger(), "Error setting INFO level for logger");
    }
  }

  RCLCPP_DEBUG(
    get_logger(), "[ROS2] Using RMW_IMPLEMENTATION = %s",
    rmw_get_implementation_identifier());
  // <---- Debug mode initialization from parameters
}

void KangarooX2Component::getCommParams()
{
  RCLCPP_INFO(get_logger(), "+++ COMMUNICATION PARAMETERS +++");

  // ----> Communication
  getParam(
    "comm.serial_port", _serialPort, _serialPort,
    "Communication serial port path", true, " * Serial port: ");

  getParam(
    "comm.baudrate", _baudrate, _baudrate,
    "Communication serial port path", true, " * Baudrate: ");

  getParam(
    "comm.timeout_msec", _readTimeOut_msec, _readTimeOut_msec,
    "Data reading timeout in msec", false, " * Timeout [msec]: ");

  getParam(
    "comm.kx2_address", _kx2Address, _kx2Address,
    "Kangaroo x2 address. Use 'DEScribe' software to configure", true,
    " * Board address: ");

  std::string par_val;
  getParam(
    "comm.kx2_drive_channel", par_val, par_val,
    "Kangaroo x2 name of the drive channel. Use the 'DEScribe' sw to "
    "configure",
    true, " * Drive channel: ");
  _kxDriveChannel = par_val[0];

  getParam(
    "comm.kx2_turn_channel", par_val, par_val,
    "Kangaroo x2 name of the turn channel. Use the 'DEScribe' sw to "
    "configure",
    true, " * Turn channel: ");
  _kxTurnChannel = par_val[0];
  // <---- Communication
}

void KangarooX2Component::getControlParams()
{
  RCLCPP_INFO(get_logger(), "+++ MOTOR CONTROL PARAMETERS +++");

  // ----> Differential Drive
  getParam(
    "diff_drive.wheel_radius_mm", _wheelRad_mm, _wheelRad_mm,
    "Wheel radius in millimeters", true, " * Wheel radius [mm]: ");

  getParam(
    "diff_drive.track_width_mm", _trackWidth_mm, _trackWidth_mm,
    "Distance between the middle of the wheels in millimeters", true,
    " * Track Width [mm]: ");

  getParam(
    "diff_drive.max_drive_accel", _driveRamp, _driveRamp,
    "Maximum drive acceleration [mm/s²]", false,
    " * Max Drice Accel. [mm/s²]: ");

  getParam(
    "diff_drive.max_turn_accel", _turnRamp, _turnRamp,
    "Maximum turn acceleration [°/s²]", false,
    " * Max Turn Accel. [°/s²]: ");
  // <---- Differential Drive

  // ----> Encoders and reduction
  getParam(
    "encoder.lines", _encLines, _encLines,
    "The counting feature of the encoder [Pulse per Round (PPR) or "
    "lines]. That is CPR/4 [Counts per Round]",
    true, " * Encoder lines [PPR]: ");

  getParam(
    "encoder.gear_ratio", _gearRatio, _gearRatio,
    "The gear ratio of the motor, according to where the encoder is "
    "placed.",
    true);
  RCLCPP_INFO_STREAM(get_logger(), " * Gear ratio: " << _gearRatio << ":1");
  // <---- Encoders and reduction
}

void KangarooX2Component::startMainThread()
{
  _mainThread = std::thread(&KangarooX2Component::mainThreadFunc, this);
}

void KangarooX2Component::stopMainThread()
{
  if (!_threadStop) {
    RCLCPP_DEBUG(get_logger(), "Stopping main thread...");
    _threadStop = true;
    try {
      RCLCPP_DEBUG(get_logger(), "...");
      if (_mainThread.joinable()) {
        _mainThread.join();
      }
      RCLCPP_DEBUG(get_logger(), "... stopped");
    } catch (std::system_error & e) {
      RCLCPP_WARN(get_logger(), "Main thread joining exception: %s", e.what());
    }
  }
}

void KangarooX2Component::mainThreadFunc()
{
  RCLCPP_DEBUG(get_logger(), "*** Main thread begin ***");

  _threadStop = false;

  double thread_period_sec = 1.0 / _controlFreq;

  tools::StopWatch freq_meas(get_clock());

  while (1) {
    tools::StopWatch duration_meas(get_clock());

    // ----> Interruption check
    if (!rclcpp::ok()) {
      RCLCPP_DEBUG(get_logger(), "Ctrl+C received: stopping grab thread");
      _threadStop = true;
    }

    if (_threadStop) {
      RCLCPP_DEBUG(get_logger(), "Main thread stopped");
      break;
    }
    // <---- Interruption check

    KangarooError err;

    // ----> Set velocities
    err = _kx2ChDrive
      ->setSpeed(_d_speedSetpoint, _driveRamp, KANGAROO_MOVE_DEFAULT)
      .status()
      .error();
    if (err != KANGAROO_NO_ERROR) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "Error setting drive speed: " << toString(err));
    }

    err =
      _kx2ChTurn->setSpeed(_t_speedSetpoint, _turnRamp, KANGAROO_MOVE_DEFAULT)
      .status()
      .error();
    if (err != KANGAROO_NO_ERROR) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "Error setting turn speed: " << toString(err));
    }
    // <---- Set velocities

    // ----> Read velocities
    kx2::KangarooStatus d_status;
    _driveMux.lock();
    d_status = _kx2ChDrive->getSpeed();
    _driveMux.unlock();
    if (d_status.valid()) {
      _d_speed = static_cast<double>(d_status.value()) / 1000.;
    }

    kx2::KangarooStatus t_status;
    _turnMux.lock();
    t_status = _kx2ChTurn->getSpeed();
    _turnMux.unlock();
    if (t_status.valid()) {
      _t_speed = static_cast<double>(t_status.value());
    }

    RCLCPP_DEBUG_STREAM(
      get_logger(),
      " * Drive speed: " << _d_speed / 1000. << " m/sec");
    RCLCPP_DEBUG_STREAM(
      get_logger(),
      " * Turn speed: " << _t_speed << " °/sec");
    // <---- Read velocities

    // ----> Thread sleep
    double elapsed_sec = duration_meas.toc();
    int sleep_usec = 100;
    if (elapsed_sec < thread_period_sec) {
      sleep_usec = static_cast<int>(1e6 * (thread_period_sec - elapsed_sec));
    }

    RCLCPP_DEBUG_STREAM(get_logger(), " * Duration: " << elapsed_sec << " sec");
    RCLCPP_DEBUG_STREAM(get_logger(), " * Sleep: " << sleep_usec << " usec");
    rclcpp::sleep_for(std::chrono::microseconds(sleep_usec));
    // <---- Thread sleep

    // ----> Thread frequency statistics
    if (_avgMainThreadPeriod_sec) {
      double freq_elapsed_sec = freq_meas.toc();
      _avgMainThreadPeriod_sec->addValue(freq_elapsed_sec);
      RCLCPP_DEBUG_STREAM(
        get_logger(), " * Thread frequency:"
          << 1.0 / freq_elapsed_sec << " Hz");
      freq_meas.tic();
    }
    // <---- Thread frequency statistics
  }

  RCLCPP_DEBUG(get_logger(), "*** Main thread end ***");
}

void KangarooX2Component::cmd_vel_callback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "*** cmd_vel_callback ***");
  _d_speedSetpoint = static_cast<int32_t>(std::round(msg->linear.x * 1000.));
  _t_speedSetpoint = static_cast<int32_t>(std::round(msg->angular.z * RAD2DEG));

  RCLCPP_DEBUG_STREAM(
    get_logger(), " * New Setpoint: " << _d_speedSetpoint
                                      << " mm/sec - "
                                      << _t_speedSetpoint << "deg/sec");
}

}  // namespace kx2

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(kx2::KangarooX2Component)
