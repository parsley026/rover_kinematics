#include "rover_kinematics/ros/kinematics_node.hpp"

#include <Eigen/Dense>
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
    w->turn.set_value   = hardware_interface_.steeringSetFromRadians(command.steering_angle_rad[i] * STEER_PRESCALE, i);
  }

  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "Assembled Wheels Message:");
  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "Front Left: Drive RPM = %f, Turn Pos = %f",
                       msg.front_left.drive.set_value,
                       msg.front_left.turn.set_value);
  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "Front Right: Drive RPM = %f, Turn Pos = %f",
                       msg.front_right.drive.set_value,
                       msg.front_right.turn.set_value);
  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "Rear Left: Drive RPM = %f, Turn Pos = %f",
                       msg.rear_left.drive.set_value,
                       msg.rear_left.turn.set_value);
  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "Rear Right: Drive RPM = %f, Turn Pos = %f",
                       msg.rear_right.drive.set_value,
                       msg.rear_right.turn.set_value);

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

double KinematicsNode::getPhysicalAngleRad(double measured_value, std::size_t wheel_index) const {
  (void)wheel_index; // Unused, hardware interface already applied polarity
  double val = measured_value;
  // Hardware polarity inversion is already removed by HardwareInterface::steeringSetFromPrecisePos,
  // so measured_value is already the true physical angle in degrees.
  // Convert degrees to radians
  return val * (M_PI / 180.0);
}

bool KinematicsNode::isMechanicallySafe(const rex_interfaces::msg::Wheels &feedback) const {
  const auto is_outside_limits = [&](double measured_value, std::size_t wheel_index) {
    const double physical_angle_rad = getPhysicalAngleRad(measured_value, wheel_index);
    const double epsilon = 1e-3; // small margin for floating point errors
    return physical_angle_rad < (config_.min_mechanical_angle(wheel_index) - epsilon) ||
           physical_angle_rad > (config_.max_mechanical_angle(wheel_index) + epsilon);
  };

  const bool out_of_bounds = is_outside_limits(feedback.front_left.turn.set_value, 0) ||
                             is_outside_limits(feedback.front_right.turn.set_value, 1) ||
                             is_outside_limits(feedback.rear_left.turn.set_value, 2) ||
                             is_outside_limits(feedback.rear_right.turn.set_value, 3);

  if (out_of_bounds) {
    const auto& mn = config_.min_mechanical_angles();
    const auto& mx = config_.max_mechanical_angles();
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Mechanical limit safety active: drive disabled. "
                         "Limits [FL:%.2f..%.2f  FR:%.2f..%.2f  RL:%.2f..%.2f  RR:%.2f..%.2f] rad",
                         mn[0], mx[0], mn[1], mx[1], mn[2], mx[2], mn[3], mx[3]);
    return false;
  }
  return true;
}

bool KinematicsNode::isSteeringCoherent(const rex_interfaces::msg::Wheels &target,
                                        const rex_interfaces::msg::Wheels &feedback) const {
  if (!config_.enable_coherence_safety()) {
    return true;
  }

  // Only run the coherence check when at least one wheel is commanding drive velocity.
  const bool any_drive_commanded =
    std::fabs(target.front_left.drive.set_value)  > 1e-6 ||
    std::fabs(target.front_right.drive.set_value) > 1e-6 ||
    std::fabs(target.rear_left.drive.set_value)   > 1e-6 ||
    std::fabs(target.rear_right.drive.set_value)  > 1e-6;

  if (!any_drive_commanded) {
    return true;
  }

  const std::array<double, 4> steer_rad = {
    getPhysicalAngleRad(feedback.front_left.turn.set_value,  0),
    getPhysicalAngleRad(feedback.front_right.turn.set_value, 1),
    getPhysicalAngleRad(feedback.rear_left.turn.set_value,   2),
    getPhysicalAngleRad(feedback.rear_right.turn.set_value,  3),
  };

  const std::array<double, 4> drive_speeds = {
    target.front_left.drive.set_value,
    target.front_right.drive.set_value,
    target.rear_left.drive.set_value,
    target.rear_right.drive.set_value,
  };

  const double coherence_error = computeSteeringCoherence(steer_rad, drive_speeds);

  if (coherence_error > config_.steering_coherence_threshold()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "Coherence safety active: steering incoherence %.3f > threshold %.3f. "
                         "Drive disabled until wheels align. angles=[%.2f, %.2f, %.2f, %.2f] rad",
                         coherence_error, config_.steering_coherence_threshold(),
                         steer_rad[0], steer_rad[1], steer_rad[2], steer_rad[3]);
    return false;
  }
  return true;
}

void KinematicsNode::applySteeringFirstSafety(
    rex_interfaces::msg::Wheels &target,
    const rex_interfaces::msg::Wheels &feedback,
    double /* angle_tolerance_deg */) {

  // 1. Mechanical Limits Check
  // Ensures steering actuators haven't rotated past physical boundaries.
  if (!isMechanicallySafe(feedback)) {
    target.front_left.drive.set_value = 0.0;
    target.front_right.drive.set_value = 0.0;
    target.rear_left.drive.set_value = 0.0;
    target.rear_right.drive.set_value = 0.0;
    return; // Takes priority — don't run coherence check if mechanically jammed
  }

  // 2. Steering Coherence Check (Instantaneous Velocity Agreement)
  // Ensures the wheels are pointing in consistent directions before driving.
  if (!isSteeringCoherent(target, feedback)) {
    target.front_left.drive.set_value = 0.0;
    target.front_right.drive.set_value = 0.0;
    target.rear_left.drive.set_value = 0.0;
    target.rear_right.drive.set_value = 0.0;
    return;
  }
}

//
// // ─ ─ Loop ─ ─
//

/**
 * @brief Fits a rigid-body twist (Vx, Vy, ω) to the four wheel unit-direction
 *        vectors derived from @p steer_angles_rad, using the closed-form 3×3
 *        normal equations instead of an iterative solver.
 *
 * Each wheel contributes two kinematic constraints:
 *   cos(θᵢ) = Vx − ω·yᵢ
 *   sin(θᵢ) = Vy + ω·xᵢ
 *
 * Stacking these into  A·x = b  (8×3 system, x = [Vx, Vy, ω]ᵀ) and solving
 * via normal equations  (AᵀA)·x = Aᵀb  gives the least-squares best-fit
 * rigid body motion.  The mean squared residual per wheel (normalised to [0,1]
 * by the worst-case residual of 2.0) is then returned as the coherence score.
 *
 * @return Normalised coherence error in [0, 1].  0 = perfect agreement;
 *         values above config_.steering_coherence_threshold() block drive.
 */
double KinematicsNode::computeSteeringCoherence(
    const std::array<double, 4> &steer_angles_rad,
    const std::array<double, 4> &drive_speeds) const {

  // Wheel positions in the robot body frame [FL, FR, RL, RR].
  // x: positive = forward (front axle),  y: positive = left.
  const double half_wb = config_.wheelbase()  / 2.0;
  const double half_tw = config_.track_width() / 2.0;
  const double wx[4] = { half_wb,  half_wb, -half_wb, -half_wb };
  const double wy[4] = { half_tw, -half_tw,  half_tw, -half_tw };

  // Build the 8×3 matrix A and the 8-vector b.
  // Row 2i:   [1, 0, -yᵢ]  →  cos(θᵢ)
  // Row 2i+1: [0, 1,  xᵢ]  →  sin(θᵢ)
  Eigen::Matrix<double, 8, 3> A;
  Eigen::Matrix<double, 8, 1> b;

  // Find the maximum speed to normalize the velocity vectors.
  // This prevents the residual from scaling with absolute speed, while preserving the
  // relative velocities which are required for a valid rigid body twist (e.g. Ackerman inner vs outer).
  double max_speed = 1e-6;
  for (int i = 0; i < 4; ++i) {
    max_speed = std::max(max_speed, std::abs(drive_speeds[i]));
  }

  for (int i = 0; i < 4; ++i) {
    A(2*i,     0) =  1.0;  A(2*i,     1) = 0.0;  A(2*i,     2) = -wy[i];
    A(2*i + 1, 0) =  0.0;  A(2*i + 1, 1) = 1.0;  A(2*i + 1, 2) =  wx[i];
    
    // Scale the direction vector by the normalized target speed.
    // This perfectly reconstructs the commanded rigid body velocity field.
    const double v = drive_speeds[i] / max_speed;
    b(2*i)     = v * std::cos(steer_angles_rad[i]);
    b(2*i + 1) = v * std::sin(steer_angles_rad[i]);
  }

  // Solve the 3×3 normal equations: (AᵀA) x = Aᵀb.
  // ldlt() is numerically stable for small positive-semidefinite systems.
  const Eigen::Matrix<double, 3, 1> x =
      (A.transpose() * A).ldlt().solve(A.transpose() * b);

  // Compute the mean squared residual per wheel, normalised by the theoretical
  // worst-case value of 2.0 (unit vectors completely anti-aligned).
  const Eigen::Matrix<double, 8, 1> residual = b - A * x;
  const double mean_sq_residual = residual.squaredNorm() / 4.0; // 8 rows / 2 per wheel
  constexpr double WORST_CASE = 2.0;
  return std::min(mean_sq_residual / WORST_CASE, 1.0);
}

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
      if (config_.publish_tf()) {
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
  
  if (current_time.nanoseconds() - initialization_ns <= static_cast<int64_t>(config_.initialization_timeout_sec() * 1e9)) {
    target_wheels_msg = assembleSetOriginMessage(current_time);
    rover_wheels_velocity_ = target_wheels_msg;
    wheels_vel_pub_->publish(rover_wheels_velocity_);
    return;
  }

  // ─ ─ 

  const int64_t last_cmd_ns = last_cmd_vel_time_ns_.load(std::memory_order_acquire);
  const double elapsed_cmd_sec = (current_time.nanoseconds() - last_cmd_ns) / 1e9;

  if (last_cmd_ns != 0 && elapsed_cmd_sec > config_.cmd_vel_timeout_sec()) {
    cmd.vel    = 0.0;
    cmd.x_axis = 0.0;
    cmd.y_axis = 0.0;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "cmd_vel timeout! No command received for %.2fs (limit: %.2fs). Braking.",
                         elapsed_cmd_sec, config_.cmd_vel_timeout_sec());
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
      if (current_time.nanoseconds() - last_ns <= static_cast<int64_t>(config_.stop_timeout_sec() * 1e9)) {
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
      target_ik = kinematics_solver_.computeSamplerConfiguration();
      break;
    case ControlMode::SURFACE_SAMPLER:
      target_ik = kinematics_solver_.computeSamplerConfiguration();
      break;
    case ControlMode::DRIVE_AUTONOMY:
      strategy_key = ControlMode::DRIVE_AUTONOMY;
      break;
    // case ControlMode::ROBOTIC_ARM_AUTONOMY:
    //   break;
    case ControlMode::DEEP_SAMPLER_AUTONOMY:
      target_ik = kinematics_solver_.computeSamplerConfiguration();
      break;
    case ControlMode::SURFACE_SAMPLER_AUTONOMY:
      target_ik = kinematics_solver_.computeSamplerConfiguration();
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

      // Clamp target steering commands to per-wheel mechanical limits
      for (int i = 0; i < 4; ++i) {
        target_ik.steering_angle_rad[i] = std::clamp(
            target_ik.steering_angle_rad[i],
            config_.min_mechanical_angle(i),
            config_.max_mechanical_angle(i));
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

  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "Publishing Wheels Message:");
  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "Front Left: Drive RPM = %f, Turn Pos = %f",
                       target_wheels_msg.front_left.drive.set_value,
                       target_wheels_msg.front_left.turn.set_value);
  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "Front Right: Drive RPM = %f, Turn Pos = %f",
                       target_wheels_msg.front_right.drive.set_value,
                       target_wheels_msg.front_right.turn.set_value);
  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "Rear Left: Drive RPM = %f, Turn Pos = %f",
                       target_wheels_msg.rear_left.drive.set_value,
                       target_wheels_msg.rear_left.turn.set_value);
  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "Rear Right: Drive RPM = %f, Turn Pos = %f",
                       target_wheels_msg.rear_right.drive.set_value,
                       target_wheels_msg.rear_right.turn.set_value);

  // Keep the drive torque off until the steering has reached the commanded
  // angle for any drive mode. This prevents the rover from applying wheel
  // velocity while the steering is still moving between the previous and next
  // target orientation, which can damage the robot when switching back to an
  // Ackermann or crab/spin mode.
  // We want this safety to apply whenever any sort of drive command is active
  const bool is_drive_mode = (control_mode_.load(std::memory_order_acquire) & (ControlMode::DRIVE | ControlMode::DRIVE_AUTONOMY)) != 0;
  const bool requires_steering_alignment = is_drive_mode &&
      cmd.mode != DriveMode::BRAKE &&
      cmd.mode != DriveMode::HANDBRAKE &&
      cmd.mode != 0;

  if (requires_steering_alignment) {
    const auto feedback_msg = *(rover_wheels_velocity_feedback_buffer_.readFromNonRT());
    applySteeringFirstSafety(target_wheels_msg, feedback_msg, 0.0);
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
  last_cmd_vel_time_ns_.store(this->get_clock()->now().nanoseconds(), std::memory_order_release);

  if (control_mode_.load(std::memory_order_acquire) & ControlMode::DRIVE) {
    rex_interfaces::msg::RoverControl cmd_vel;

    cmd_vel.mode = msg->mode;

    cmd_vel.vel    = msg->vel;
    cmd_vel.y_axis = msg->y_axis;
    cmd_vel.x_axis = msg->x_axis;

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
               "Received Manual Command: Mode = %d, Vel = %f, Y Axis = %f, X Axis = %f",
               static_cast<int>(cmd_vel.mode), cmd_vel.vel,
               cmd_vel.y_axis, cmd_vel.x_axis);

    rover_cmd_velocity_buffer_.writeFromNonRT(cmd_vel);
  }
}

/**
 * @brief Accept autonomy Twist Messege when in ControlMode::DRIVE_AUTONOMY.
 */
void KinematicsNode::cmdVelAutonomyCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  last_cmd_vel_time_ns_.store(this->get_clock()->now().nanoseconds(), std::memory_order_release);

  if (control_mode_.load(std::memory_order_acquire) & ControlMode::DRIVE_AUTONOMY) {
    rex_interfaces::msg::RoverControl cmd_vel;

    cmd_vel.mode = DriveMode::SYM_ACKERMANN;

    cmd_vel.vel    = msg->linear.x;
    cmd_vel.y_axis = msg->linear.y;
    cmd_vel.x_axis = msg->angular.z;

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
               "Received Autonomy Command: Mode = %d, Vel = %f, Y Axis = %f, X Axis = %f",
               static_cast<int>(cmd_vel.mode), cmd_vel.vel,
               cmd_vel.y_axis, cmd_vel.x_axis);

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
  // feedback_stale_.store(false, std::memory_order_release);
  // communication_state_.store(CommunicationState::OPENED, std::memory_order_release);

  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "Received Feedback: VESC ID = %d, ERPM = %d, Precise Pos = %f, Current = %f, Duty Cycle = %f",
                       static_cast<int>(id), msg->erpm, msg->precise_pos,
                       msg->current, msg->duty_cycle);
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
