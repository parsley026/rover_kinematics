#include "rover_kinematics/ros/kinematics_node.hpp"

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>


KinematicsNode::KinematicsNode(const rclcpp::NodeOptions &options)
    : Node("rover_kinematics", options),
      publish_period_(rclcpp::Duration::from_seconds(0.05)), config_(),
      param_manager_(this, config_), kinematics_solver_(),
      kinematics_estimator(), hardware_interface_(),
      control_mode_(ControlMode::DRIVE),
      communication_state_(CommunicationState::CREATED) {

  const std::string pkg_share = ament_index_cpp::get_package_share_directory("rover_kinematics");
  config_file_path_ = pkg_share + "/config/rover_config.yaml";

  if (!config_.loadFromFile(config_file_path_)) {
    RCLCPP_WARN(this->get_logger(), "failed to load config from '%s', using built-in defaults", config_file_path_.c_str());
  }

  // ─ ─
  param_manager_.initialize();

  // ─ ─
  kinematics_solver_.setConfig(config_);
  kinematics_estimator.setConfig(config_);
  hardware_interface_.setConfig(config_);

  // ─ ─
  strategies_.emplace(DriveMode::SYM_ACKERMANN, std::make_unique<ManualSymAckermannStrategy>());
  strategies_.emplace(DriveMode::FWD_ACKERMANN, std::make_unique<ManualFwdAckermannStrategy>());
  strategies_.emplace(DriveMode::RWD_ACKERMANN, std::make_unique<ManualRwdAckermannStrategy>());
  strategies_.emplace(DriveMode::CRAB,          std::make_unique<ManualCrabStrategy>());
  strategies_.emplace(DriveMode::SYM_SPIN,      std::make_unique<ManualSymSpinStrategy>());
  
  // ─ ─
  strategies_.emplace(ControlMode::DRIVE_AUTONOMY, std::make_unique<AutonomyDriveStrategy>());

  // ─ ─
  param_cb_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params) {
        return param_manager_.onSetParameters(
            params, kinematics_solver_, kinematics_estimator,
            hardware_interface_,
            [this](double rate) { updateTimerRate(rate); });
      });

  // ─ ─
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / config_.publish_rate());
  timer_ = this->create_wall_timer(publish_period_.to_chrono<std::chrono::milliseconds>(), std::bind(&KinematicsNode::onUpdate, this));

  initCmdVelBuffer();
  initFeedbackBuffer();

  // ─ ─
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  //
  // // ─ ─ 
  //

  status_sub_ = this->create_subscription<rex_interfaces::msg::RoverStatus>(
    "/MQTT/RoverStatus", qos,
    std::bind(&KinematicsNode::statusCallback, this, std::placeholders::_1));

  cmd_vel_sub_manual_ = this->create_subscription<rex_interfaces::msg::RoverControl>(
    "/MQTT/RoverControl", qos,
    std::bind(&KinematicsNode::cmdVelManualCallback, this, std::placeholders::_1));

  cmd_vel_sub_autonomy_ = this->create_subscription<geometry_msgs::msg::Twist>(
    config_.cmd_vel_autonomy_topic(), qos,
    std::bind(&KinematicsNode::cmdVelAutonomyCallback, this, std::placeholders::_1));

  wheels_vel_feedback_sub_ = this->create_subscription<rex_interfaces::msg::VescStatus>(
    "/CAN/RX/vesc_status", rclcpp::QoS(1000).reliable(),
    std::bind(&KinematicsNode::feedbackCallback, this, std::placeholders::_1));

  wheels_vel_pub_ = this->create_publisher<rex_interfaces::msg::Wheels>(
    "/CAN/TX/set_motor_vel", qos);
  
  tf_odom_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", qos);
     odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/kinematics/odom", qos);

  reset_kinematics_srv_ = this->create_service<std_srvs::srv::Empty>(
    "reset_kinematics",
    std::bind(&KinematicsNode::resetKinematicsCallback, this, std::placeholders::_1, std::placeholders::_2));

  reset_odometry_srv_ = this->create_service<std_srvs::srv::Empty>(
    "reset_odometry",
    std::bind(&KinematicsNode::resetOdometryCallback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(),
              "KinematicsNode initialized"
              "config file: '%s'. ", config_file_path_.c_str());
}

//
// // ─ ─ Init ─ ─
//

/**
 * @brief Zero-initialise the command velocity realtime buffer.
 */
void KinematicsNode::initCmdVelBuffer() {
  rex_interfaces::msg::RoverControl cmd_vel;
  cmd_vel.header.stamp = this->get_clock()->now();
  cmd_vel.x_axis = 0.0;
  cmd_vel.y_axis = 0.0;
  cmd_vel.vel = 0.0;
  rover_cmd_velocity_buffer_.writeFromNonRT(cmd_vel);
}

/**
 * @brief Zero-initialise the wheel feedback realtime buffer.
 */
void KinematicsNode::initFeedbackBuffer() {
  rex_interfaces::msg::Wheels feedback;
  feedback.header.stamp = this->get_clock()->now();

  feedback.front_left.drive.set_value = 0.0;
  feedback.front_left.turn.set_value = 0.0;
  feedback.front_right.drive.set_value = 0.0;
  feedback.front_right.turn.set_value = 0.0;
  feedback.rear_left.drive.set_value = 0.0;
  feedback.rear_left.turn.set_value = 0.0;
  feedback.rear_right.drive.set_value = 0.0;
  feedback.rear_right.turn.set_value = 0.0;

  rover_wheels_velocity_feedback_buffer_.writeFromNonRT(feedback);

  std::lock_guard<std::mutex> lock(feedback_mutex_);
  wheel_quality_weights_.fill(1.0);
  drive_quality_weights_.fill(1.0);
  steer_quality_weights_.fill(1.0);
  kinematics_estimator.setWheelQuality(wheel_quality_weights_);
}

//
// // ─ ─ 
//

void KinematicsNode::updateTimerRate(double new_rate_hz) {
  timer_->cancel();
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / new_rate_hz);
  timer_ = this->create_wall_timer(
      publish_period_.to_chrono<std::chrono::milliseconds>(),
      std::bind(&KinematicsNode::onUpdate, this)); RCLCPP_INFO(this->get_logger(), "publish_rate updated to %.1f Hz.", new_rate_hz);
}

//
// // ─ ─ 
//

double KinematicsNode::computeWheelQualityScore(
    const rex_interfaces::msg::VescStatus &status) const {
  if (!config_.enable_dynamic_wheel_weighting()) {
    return 1.0;
  }

  const double erpm = std::abs(static_cast<double>(status.erpm));
  const double current = std::abs(static_cast<double>(status.current));
  const double duty_cycle = std::abs(static_cast<double>(status.duty_cycle));

  if (erpm >= config_.wheel_quality_low_erpm_threshold()) {
    return 1.0;
  }

  const double current_threshold = std::max(config_.wheel_quality_high_current_threshold(), 1e-6);
  const double duty_threshold = std::clamp(config_.wheel_quality_high_duty_threshold(), 1e-6, 0.999999);

  const double current_penalty = std::max(0.0, (current - current_threshold) / current_threshold);
  const double duty_penalty = std::max(0.0, (duty_cycle - duty_threshold) / (1.0 - duty_threshold));

  const double quality = 1.0 / (1.0 + current_penalty + duty_penalty);
  return std::clamp(quality, config_.wheel_quality_min_weight(), 1.0);
}

void KinematicsNode::updateWheelQuality(
    std::size_t wheel_index, const rex_interfaces::msg::VescStatus &status) {
  if (wheel_index >= wheel_quality_weights_.size()) {
    return;
  }

  const double quality = computeWheelQualityScore(status);

  if (status.vesc_id >= 0x50 && status.vesc_id <= 0x53) {
    drive_quality_weights_[wheel_index] = quality;
  } else {
    steer_quality_weights_[wheel_index] = quality;
  }

  wheel_quality_weights_[wheel_index] = std::min(
      drive_quality_weights_[wheel_index], steer_quality_weights_[wheel_index]);
  kinematics_estimator.setWheelQuality(wheel_quality_weights_);
}

//
// // ─ ─ Assembly ─ ─
//

rex_interfaces::msg::Wheels
KinematicsNode::assembleWheelsFromCommand(const WheelCommand &command, const rclcpp::Time &time) {
  rex_interfaces::msg::Wheels msg;
  msg.header.stamp = time;

  rex_interfaces::msg::Wheel* wheels[] = {
      &msg.front_left, &msg.front_right, &msg.rear_left, &msg.rear_right
  };

  for (std::size_t i = 0; i < 4; ++i) {
    auto *w = wheels[i];

    w->drive.command_id = VescCommand::SET_RPM;
    w->drive.set_value  = hardware_interface_.driveSetFromMetersPerSecond(command.drive_velocity_mps[i], i);

    w->turn.command_id  = VescCommand::SET_POS;
    w->turn.set_value   = hardware_interface_.steeringSetFromRadians(command.steering_angle_rad[i], i);
  }

  return msg;
}

rex_interfaces::msg::Wheels
KinematicsNode::assembleSetOriginMessage(const rclcpp::Time &time) {
  rex_interfaces::msg::Wheels msg;
  msg.header.stamp = time;

  for (auto *w : {&msg.front_left, &msg.front_right, &msg.rear_left, &msg.rear_right}) {
    w->drive.command_id = VescCommand::SET_CURRENT;
    w->drive.set_value  = 0.0;

    w->turn.command_id  = VescCommand::SET_ORIGIN;
    w->turn.set_value   = 0.0;

    w->turn.set_origin_data = 0;
  }
  
  return msg;
}

rex_interfaces::msg::Wheels
KinematicsNode::assembleStopMessage(const rclcpp::Time &time) {
  rex_interfaces::msg::Wheels msg;
  msg.header.stamp = time;

  for (auto *w : {&msg.front_left, &msg.front_right, &msg.rear_left, &msg.rear_right}) {
    w->drive.command_id = VescCommand::SET_CURRENT;
    w->drive.set_value  = 0.0;

    w->turn.command_id  = VescCommand::SET_POS;
    w->turn.set_value   = 0.0;
  }
  return msg;
}

rex_interfaces::msg::Wheels
KinematicsNode::assembleBrakeMessage(const rclcpp::Time &time) {
  rex_interfaces::msg::Wheels msg;
  msg.header.stamp = time;

  for (auto *w : {&msg.front_left, &msg.front_right, &msg.rear_left, &msg.rear_right}) {
    w->drive.command_id = VescCommand::SET_CURRENT_BRAKE;
    w->drive.set_value  = CURRENT_BRAKE_VALUE;

    w->turn.command_id  = VescCommand::SET_POS;
    w->turn.set_value   = 0.0;
  }
  return msg;
}

rex_interfaces::msg::Wheels
KinematicsNode::assembleHandBrakeMessage(const rclcpp::Time &time) {
  rex_interfaces::msg::Wheels msg;
  msg.header.stamp = time;

  for (auto *w : {&msg.front_left, &msg.front_right, &msg.rear_left, &msg.rear_right}) {
    w->drive.command_id = VescCommand::SET_CURRENT_HANDBRAKE;
    w->drive.set_value  = CURRENT_HANDBRAKE_VALUE;

    w->turn.command_id  = VescCommand::SET_POS;
    w->turn.set_value   = 0.0;
  }
  return msg;
}

//
// // ─ ─ Watchdog ─ ─
//

void KinematicsNode::updateWatchdog(const rclcpp::Time &current_time) {
  const int64_t last_feedback_ns = last_feedback_time_ns_.load(std::memory_order_acquire);
  
  const bool feedback_never_received = (last_feedback_ns == 0);
  const int64_t elapsed_ns = current_time.nanoseconds() - last_feedback_ns;

  const bool is_feedback_stale = feedback_never_received || (elapsed_ns > config_.feedback_timeout_ns());
  
  const bool was_feedback_stale = feedback_stale_.exchange(is_feedback_stale, std::memory_order_acq_rel);

  if (is_feedback_stale && !was_feedback_stale) {
    RCLCPP_ERROR(get_logger(), "feedback stale,   timeout: %.3f sec. Disabling.", config_.feedback_timeout_sec());
    communication_state_.store(CommunicationState::CREATED, std::memory_order_release);
  }
  if (!is_feedback_stale && was_feedback_stale) {
    RCLCPP_INFO(get_logger(), "feedback restored, timeout: %.3f sec. Enabling.",  config_.feedback_timeout_sec());
    communication_state_.store(CommunicationState::OPENED, std::memory_order_release);
  }
}

//
// // ─ ─ Loop ─ ─
//

void KinematicsNode::onUpdate() {
  const rclcpp::Time current_time = this->get_clock()->now();

  updateWatchdog(current_time);

  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    kinematics_estimator.setWheelQuality(wheel_quality_weights_);
  }

  if (!feedback_stale_.load(std::memory_order_acquire)) {
    auto feedback_msg =
        *(rover_wheels_velocity_feedback_buffer_.readFromNonRT());
    const auto estimate =
        kinematics_estimator.update(feedback_msg, current_time);

    if (estimate.valid) {
      odom_pub_->publish(estimate.odometry);
      if (config_.enable_odom_tf()) {
        tf_odom_pub_->publish(estimate.transform);
      }
    }
  }

  auto cmd = *(rover_cmd_velocity_buffer_.readFromNonRT());
  rex_interfaces::msg::Wheels target_wheels_msg;

  // ─ ─ 
  
  int64_t initialization_ns = initialization_time_ns_.load(std::memory_order_acquire);
  if (initialization_ns == 0) {
    initialization_ns = current_time.nanoseconds();
    initialization_time_ns_.store(initialization_ns, std::memory_order_release);
  }
  
  if (current_time.nanoseconds() - initialization_ns <= 5LL * 1'000'000'000LL) {
    target_wheels_msg = assembleSetOriginMessage(current_time);
    rover_wheels_velocity_ = target_wheels_msg;
    wheels_vel_pub_->publish(rover_wheels_velocity_);
    return;
  }

  // ─ ─ 

  if (communication_state_.load(std::memory_order_acquire) == CommunicationState::OPENED) {
    WheelCommand target_ik{};
    const int control_mode = control_mode_.load(std::memory_order_acquire);
    int strategy_key = -1;
    bool message_already_assembled = false;

    // ─ ─ 

    const int kinematics_control_mode = control_mode & KinematicsControlModeMask;

    // ─ ─
    
    const int active_kinematics_control_mode = [&]() -> int {
      for (const int mode : KinematicsControlMode) {
        if (kinematics_control_mode & mode) {
          return mode;
        }
      }
      return ControlMode::NONE;
    }();

    // ─ ─
 
    if (active_kinematics_control_mode != ControlMode::NONE) {
      last_kinematics_active_time_ns_.store(current_time.nanoseconds(), std::memory_order_release);
    } else {
      const int64_t last_ns = last_kinematics_active_time_ns_.load(std::memory_order_acquire);
      if (current_time.nanoseconds() - last_ns <= 5LL * 1'000'000'000LL) {
        target_wheels_msg = assembleStopMessage(current_time);
        message_already_assembled = true;
      } else {
        rex_interfaces::msg::RoverControl cmd_vel;
        cmd_vel.header.stamp = current_time;
        
        cmd_vel.vel = 0.0;
        cmd_vel.x_axis = 0.0;
        cmd_vel.y_axis = 0.0;
        
        rover_cmd_velocity_buffer_.writeFromNonRT(cmd_vel);
        return;
      }
    }

    // ─ ─

    switch (active_kinematics_control_mode) {
    // case ControlMode::ESTOP:
    //   break;
    case ControlMode::STOP:
      target_wheels_msg = assembleStopMessage(current_time);
      message_already_assembled = true;
      break;
    // case ControlMode::CONFIG:
    //   break;
    case ControlMode::DRIVE:
      switch (cmd.mode) {
      case DriveMode::BRAKE:
        target_wheels_msg = assembleBrakeMessage(current_time);
        message_already_assembled = true;
        break;
      case DriveMode::HANDBRAKE:
        target_wheels_msg = assembleHandBrakeMessage(current_time);
        message_already_assembled = true;
        break;
      default:
        strategy_key = static_cast<int>(cmd.mode);
        break;
      }
      break;
    // case ControlMode::ROBOTIC_ARM:
    //   break;
    case ControlMode::DEEP_SAMPLER:
      target_ik = kinematics_solver_.computeXConfiguration();
      break;
    case ControlMode::SURFACE_SAMPLER:
      target_ik = kinematics_solver_.computeXConfiguration();
      break;
    case ControlMode::DRIVE_AUTONOMY:
      strategy_key = ControlMode::DRIVE_AUTONOMY;
      break;
    // case ControlMode::ROBOTIC_ARM_AUTONOMY:
    //   break;
    case ControlMode::DEEP_SAMPLER_AUTONOMY:
      target_ik = kinematics_solver_.computeXConfiguration();
      break;
    case ControlMode::SURFACE_SAMPLER_AUTONOMY:
      target_ik = kinematics_solver_.computeXConfiguration();
      break;
      
    default:
      break;
    }

    if (!message_already_assembled) {
      if (strategy_key != -1) {
        const auto it = strategies_.find(strategy_key);
        if (it != strategies_.end()) {
          target_ik = it->second->compute(cmd, kinematics_solver_, config_);
        }
      }

      target_wheels_msg = assembleWheelsFromCommand(target_ik, current_time);
    }
  } else {
    rex_interfaces::msg::RoverControl cmd_vel;

    cmd_vel.header.stamp = current_time;

    cmd_vel.vel = 0.0;
    cmd_vel.x_axis = 0.0;
    cmd_vel.y_axis = 0.0;

    rover_cmd_velocity_buffer_.writeFromNonRT(cmd_vel);

    target_wheels_msg = assembleStopMessage(current_time);
  }

  rover_wheels_velocity_ = target_wheels_msg;
  wheels_vel_pub_->publish(rover_wheels_velocity_);
}

//
// // ─ ─ Topic Callbacks ─ ─
//

/**
 * @brief Update control mode and communication state from the RoverStatus Message.
 */
void KinematicsNode::statusCallback(
    const rex_interfaces::msg::RoverStatus::SharedPtr msg) {
  communication_state_.store(msg->communication_state, std::memory_order_release);
  control_mode_.store(msg->control_mode, std::memory_order_release);
}

/**
 * @brief Accept manual Custom Messege when in ControlMode::DRIVE.
 */
void KinematicsNode::cmdVelManualCallback(
    const rex_interfaces::msg::RoverControl::SharedPtr msg) {
  if (control_mode_.load(std::memory_order_acquire) & ControlMode::DRIVE) {
    rex_interfaces::msg::RoverControl cmd_vel;

    cmd_vel.mode = msg->mode;

    cmd_vel.vel    = msg->vel;
    cmd_vel.y_axis = msg->y_axis;
    cmd_vel.x_axis = msg->x_axis;

    rover_cmd_velocity_buffer_.writeFromNonRT(cmd_vel);
  }
}

/**
 * @brief Accept autonomy Twist Messege when in ControlMode::DRIVE_AUTONOMY.
 */
void KinematicsNode::cmdVelAutonomyCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (control_mode_.load(std::memory_order_acquire) & ControlMode::DRIVE_AUTONOMY) {
    rex_interfaces::msg::RoverControl cmd_vel;

    cmd_vel.mode = DriveMode::SYM_ACKERMANN;

    cmd_vel.vel    = msg->linear.x;
    cmd_vel.y_axis = msg->linear.y;
    cmd_vel.x_axis = msg->angular.z;

    rover_cmd_velocity_buffer_.writeFromNonRT(cmd_vel);
  }
}

/**
 * @brief Receive per-wheel VESC status feedback and update the odometry input buffer.
 */
void KinematicsNode::feedbackCallback(
    const rex_interfaces::msg::VescStatus::SharedPtr msg) {

  std::lock_guard<std::mutex> lock(feedback_mutex_);

  rex_interfaces::msg::Wheels current_feedback =
      *(rover_wheels_velocity_feedback_buffer_.readFromNonRT());

  const rclcpp::Time receive_time = this->get_clock()->now();

  rclcpp::Time measurement_time;
  if (config_.use_measurement_timestamp() &&
      (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0)) {
    measurement_time = rclcpp::Time(msg->header.stamp);
  } else {
    measurement_time = receive_time;
  }

  current_feedback.header.stamp = measurement_time;

  rex_interfaces::msg::Wheel* wheels[4] = {
      &current_feedback.front_left, &current_feedback.front_right,
      &current_feedback.rear_left,  &current_feedback.rear_right
  };

  const uint8_t id = msg->vesc_id;

  if (id >= VescID::DRIVE_FL && id <= VescID::DRIVE_RR) {
    const std::size_t idx = HardwareInterface::vescIdToDriveWheelIndex(id);
    wheels[idx]->drive.set_value = hardware_interface_.metersPerSecondFromErpm(msg->erpm, idx);
    updateWheelQuality(idx, *msg);
  } 
  if (id >= VescID::STEER_FL && id <= VescID::STEER_RL) {
    const std::size_t idx = HardwareInterface::vescIdToTurnWheelIndex(id);
    wheels[idx]->turn.set_value = hardware_interface_.steeringSetFromPrecisePos(msg->precise_pos, idx);
    updateWheelQuality(idx, *msg);
  }

  rover_wheels_velocity_feedback_buffer_.writeFromNonRT(current_feedback);

  last_feedback_time_ns_.store(measurement_time.nanoseconds(), std::memory_order_release);
  feedback_stale_.store(false, std::memory_order_release);
  communication_state_.store(CommunicationState::OPENED, std::memory_order_release);
}

//
// // ─ ─  Service Callbacks ─ ─
//

void KinematicsNode::resetOdometryCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
  RCLCPP_INFO(this->get_logger(), "resetting odometry pose to origin.");
  kinematics_estimator.reset();
}

void KinematicsNode::resetKinematicsCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
  RCLCPP_INFO(this->get_logger(), "Reloading config from '%s'...",
              config_file_path_.c_str());

  if (config_.loadFromFile(config_file_path_)) {
    param_manager_.applyAll();

    kinematics_solver_.setConfig(config_);
    kinematics_estimator.setConfig(config_);
    hardware_interface_.setConfig(config_);

    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      wheel_quality_weights_.fill(1.0);
      drive_quality_weights_.fill(1.0);
      steer_quality_weights_.fill(1.0);
      kinematics_estimator.setWheelQuality(wheel_quality_weights_);
    }

    kinematics_estimator.reset();

    timer_->cancel();
    publish_period_ =
        rclcpp::Duration::from_seconds(1.0 / config_.publish_rate());
    timer_ = this->create_wall_timer(
        publish_period_.to_chrono<std::chrono::milliseconds>(),
        std::bind(&KinematicsNode::onUpdate, this));

    RCLCPP_INFO(this->get_logger(), "Config reloaded successfully.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to reload config file: '%s'",
                 config_file_path_.c_str());
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(KinematicsNode)
