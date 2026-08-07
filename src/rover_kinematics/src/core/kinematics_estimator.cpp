#include "rover_kinematics/core/kinematics_estimator.hpp"

#include <algorithm>

namespace {
/// @brief Row-major 3×3 matrix multiply: out = a * b
inline void mat3_mul(const double a[9], const double b[9], double out[9]) {
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      out[row * 3 + col] = 0.0;
      for (int k = 0; k < 3; ++k)
        out[row * 3 + col] += a[row * 3 + k] * b[k * 3 + col];
    }
  }
}
/// @brief Row-major 3×3 matrix transpose: out = a^T
inline void mat3_transpose(const double a[9], double out[9]) {
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      out[col * 3 + row] = a[row * 3 + col];
}
} // namespace

KinematicsEstimator::KinematicsEstimator(const KinematicsConfig &config) {
  setConfig(config);
}

/**
 * @brief Applies a rover configuration and refreshes derived parameters and
 * output message settings.
 */
void KinematicsEstimator::setConfig(const KinematicsConfig &config) {
  config_ = config;
  pose_covariance_ = {config_.pose_covariance_diagonal()[0], 0.0, 0.0, 0.0,
                      config_.pose_covariance_diagonal()[1], 0.0, 0.0, 0.0,
                      config_.pose_covariance_diagonal()[5]};
  wheel_quality_.fill(1.0);

  initOdomMessage();
  initTransformMessage();

  // Force Ceres to rebuild the problem blocks using the new geometry parameters
  problem_initialized_ = false;
}

void KinematicsEstimator::setWheelQuality(
    const std::array<double, 4> &wheel_quality) {
  for (std::size_t wheel_index = 0; wheel_index < wheel_quality.size();
       ++wheel_index) {
    const double clamped_quality = std::clamp(
        wheel_quality[wheel_index], config_.wheel_quality_min_weight(), 1.0);
    wheel_quality_[wheel_index] =
        config_.enable_dynamic_wheel_weighting() ? clamped_quality : 1.0;
  }
}

void KinematicsEstimator::initOdomMessage() {
  // odometry_.header.stamp.sec = 0;
  // odometry_.header.stamp.nanosec = 0;
  odometry_.header.frame_id = config_.odom_frame_id();
  odometry_.child_frame_id = config_.base_frame_id();

  // odometry_.pose.pose.position.x = 0.0;
  // odometry_.pose.pose.position.y = 0.0;
  odometry_.pose.pose.position.z = 0.0;

  // odometry_.pose.pose.orientation.x = 0.0;
  // odometry_.pose.pose.orientation.y = 0.0;
  // odometry_.pose.pose.orientation.z = 0.0;

  odometry_.pose.pose.orientation.w = 1.0;

  odometry_.pose.covariance = {pose_covariance_[0],
                               pose_covariance_[1],
                               0.,
                               0.,
                               0.,
                               pose_covariance_[2],
                               pose_covariance_[3],
                               pose_covariance_[4],
                               0.,
                               0.,
                               0.,
                               pose_covariance_[5],
                               0.,
                               0.,
                               config_.pose_covariance_diagonal()[2],
                               0.,
                               0.,
                               0.,
                               0.,
                               0.,
                               0.,
                               config_.pose_covariance_diagonal()[3],
                               0.,
                               0.,
                               0.,
                               0.,
                               0.,
                               0.,
                               config_.pose_covariance_diagonal()[4],
                               0.,
                               pose_covariance_[6],
                               pose_covariance_[7],
                               0.,
                               0.,
                               0.,
                               pose_covariance_[8]};

  // odometry_.twist.twist.linear.x = 0.0;
  // odometry_.twist.twist.linear.y = 0.0;
  odometry_.twist.twist.linear.z = 0.0;

  odometry_.twist.twist.angular.x = 0.0;
  odometry_.twist.twist.angular.y = 0.0;
  // odometry_.twist.twist.angular.z = 0.0;

  odometry_.twist.covariance = {
      config_.twist_covariance_diagonal()[0], 0., 0., 0., 0., 0., 0.,
      config_.twist_covariance_diagonal()[1], 0., 0., 0., 0., 0., 0.,
      config_.twist_covariance_diagonal()[2], 0., 0., 0., 0., 0., 0.,
      config_.twist_covariance_diagonal()[3], 0., 0., 0., 0., 0., 0.,
      config_.twist_covariance_diagonal()[4], 0., 0., 0., 0., 0., 0.,
      config_.twist_covariance_diagonal()[5]};
}

void KinematicsEstimator::initTransformMessage() {
  transformation_.transforms.resize(1);

  // transformation_.transforms[0].header.stamp.sec = 0;
  // transformation_.transforms[0].header.stamp.nanosec = 0;
  transformation_.transforms[0].header.frame_id = config_.odom_frame_id();
  transformation_.transforms[0].child_frame_id = config_.base_frame_id();

  // transformation_.transforms[0].transform.translation.x = 0.0;
  // transformation_.transforms[0].transform.translation.y = 0.0;
  transformation_.transforms[0].transform.translation.z = 0.0;

  // transformation_.transforms[0].transform.rotation.x = 0.0;
  // transformation_.transforms[0].transform.rotation.y = 0.0;
  // transformation_.transforms[0].transform.rotation.z = 0.0;

  transformation_.transforms[0].transform.rotation.w = 1.0;
}

OdometryEstimate
KinematicsEstimator::update(const rex_interfaces::msg::Wheels &feedback,
                            const rclcpp::Time &timestamp) {
  OdometryEstimate estimate;
  estimate.valid = false;

  // 1. Time Management (Safety Check)
  double dt = updateAndGetTimeDelta(timestamp);
  // REPLACE the hardcoded 0.5 with the config value
  if (dt <= 0.0 || dt > config_.max_allowed_dt()) {
    return estimate;
  }

  // 2. Hardware to Math Extraction
  auto wheel_vectors = extractWheelVectors(feedback);

  // 3. Ceres Optimization (Pure Math)
  SolverResult result = solveBodyTwist(wheel_vectors);

  // 4. Safety Fallback: Did the solver fail?
  // BUG FIX: single static variable so recovery correctly resets the throttle.
  static int solver_failure_counter = 0;
  if (!result.is_usable) {
    if (solver_failure_counter++ % 20 == 0) { // Throttle to avoid log spam
      RCLCPP_WARN(rclcpp::get_logger("KinematicsEstimator"),
                  "Ceres solver failed! Falling back to dead reckoning.");
    }

    integratePose(guess_velocity_x_mps_, guess_velocity_y_mps_,
                  guess_velocity_omega_rad_s_, dt);
    return populateRosMessages(timestamp);
  } else {
    solver_failure_counter = 0; // Reset the same counter once solver recovers
  }

  // 5. Apply valid math to state
  updateCovariance(result, dt);
  integratePose(result.velocity_x_mps, result.velocity_y_mps,
                result.velocity_omega_rad_s, dt);

  return populateRosMessages(timestamp);
}

// -----------------------------------------------------------------------------------------
// Pipeline Helper Implementations
// -----------------------------------------------------------------------------------------

double
KinematicsEstimator::updateAndGetTimeDelta(const rclcpp::Time &timestamp) {
  const double dt = (rclcpp::Time(timestamp.nanoseconds(), RCL_ROS_TIME) -
                     rclcpp::Time(timestamp_.nanoseconds(), RCL_ROS_TIME))
                        .seconds();
  timestamp_ = timestamp;
  return dt;
}

std::array<WheelVelocity, 4> KinematicsEstimator::extractWheelVectors(
    const rex_interfaces::msg::Wheels &feedback) const {
  // 1. Explicit Unit Conversion
  auto degreesToRadians = [](double deg) { return deg * (M_PI / 180.0); };

  // 2. Separate Deadband Logic (Optional)
  auto applyDeadband = [](double value, double threshold) {
    return std::abs(value) > threshold ? value : 0.0;
  };

  // 3. Velocity Vector Calculation
  auto wheelVel = [&](double speed, double steer_rad) -> WheelVelocity {
    return {speed * std::cos(steer_rad), speed * std::sin(steer_rad)};
  };

  // Note: Array initialization now matches standard layout: [FL, FR, RL, RR]
  return {wheelVel(feedback.front_left.drive.set_value,
                   degreesToRadians(feedback.front_left.turn.set_value)),
          wheelVel(feedback.front_right.drive.set_value,
                   degreesToRadians(feedback.front_right.turn.set_value)),
          wheelVel(feedback.rear_left.drive.set_value,
                   degreesToRadians(feedback.rear_left.turn.set_value)),
          wheelVel(feedback.rear_right.drive.set_value,
                   degreesToRadians(feedback.rear_right.turn.set_value))};
}

// std::array<WheelVelocity, 4> KinematicsEstimator::extractWheelVectors(
//     const rex_interfaces::msg::Wheels& feedback) const
// {
//     // 1. Explicit Unit Conversion
//     auto degreesToRadians = [](double deg) {
//         return deg * (M_PI / 180.0);
//     };

//     // 2. Separate Deadband Logic
//     auto applyDeadband = [](double value, double threshold) {
//         return std::abs(value) > threshold ? value : 0.0;
//     };

//     // 3. Velocity Vector Calculation
//     auto wheelVel = [&](double speed, double steer_rad) -> WheelVelocity {
//         return {speed * std::cos(steer_rad), speed * std::sin(steer_rad)};
//     };

//     // Set deadband to ~0.03 m/s (filters out the micro-slip from Gazebo
//     suspension) const double DEADBAND_MPS = 0.03;

//     // Note: Array initialization now matches standard layout: [FL, FR, RL,
//     RR] return {
//         wheelVel(applyDeadband(feedback.front_left.drive.set_value,
//         DEADBAND_MPS), degreesToRadians(feedback.front_left.turn.set_value)),
//         wheelVel(applyDeadband(feedback.front_right.drive.set_value,
//         DEADBAND_MPS),
//         degreesToRadians(feedback.front_right.turn.set_value)),
//         wheelVel(applyDeadband(feedback.rear_left.drive.set_value,
//         DEADBAND_MPS), degreesToRadians(feedback.rear_left.turn.set_value)),
//         wheelVel(applyDeadband(feedback.rear_right.drive.set_value,
//         DEADBAND_MPS),  degreesToRadians(feedback.rear_right.turn.set_value))
//     };
// }

SolverResult KinematicsEstimator::solveBodyTwist(
    const std::array<WheelVelocity, 4> &wheel_vectors) {
  if (!problem_initialized_) {
    // Destroy the old problem (and safely auto-delete all old Ceres-owned
    // pointers)
    problem_ = std::make_unique<ceres::Problem>();

    const double half_wb = config_.wheelbase() / 2.0;
    const double half_tw = config_.track_width() / 2.0;

    // Define wheel positions matching standard layout: [FL, FR, RL, RR]
    const double wheel_x[4] = {half_wb, half_wb, -half_wb, -half_wb};
    const double wheel_y[4] = {half_tw, -half_tw, half_tw,
                               -half_tw}; // FL is positive Y, FR is negative Y

    // Elegantly create and assign residuals inside a loop
    for (int i = 0; i < 4; ++i) {
      auto *residual =
          new WheelResidual(&measured_vx_[i], &measured_vy_[i],
                            &wheel_quality_[i], wheel_x[i], wheel_y[i]);
      auto *cost_function =
          new ceres::AutoDiffCostFunction<WheelResidual, 2, 1, 1, 1>(residual);

      // Ceres takes ownership of cost_function and the HuberLoss pointer
      problem_->AddResidualBlock(
          cost_function, new ceres::HuberLoss(config_.huber_loss_threshold()),
          &solver_velocity_x_mps_, &solver_velocity_y_mps_,
          &solver_velocity_omega_rad_s_);
    }

    problem_initialized_ = true;
  }

  // Apply new measured data using the explicit properties
  for (int i = 0; i < 4; ++i) {
    measured_vx_[i] = wheel_vectors[i].vx_mps;
    measured_vy_[i] = wheel_vectors[i].vy_mps;
  }

  solver_velocity_x_mps_ = guess_velocity_x_mps_;
  solver_velocity_y_mps_ = guess_velocity_y_mps_;
  solver_velocity_omega_rad_s_ = guess_velocity_omega_rad_s_;

  ceres::Solver::Options options;
  options.max_num_iterations = config_.solver_max_num_iterations();
  options.function_tolerance = config_.solver_function_tolerance();
  options.gradient_tolerance = config_.solver_gradient_tolerance();
  options.parameter_tolerance = config_.solver_parameter_tolerance();
  options.linear_solver_type = config_.solver_linear_solver_type();
  options.trust_region_strategy_type =
      config_.solver_trust_region_strategy_type();
  options.minimizer_progress_to_stdout =
      config_.solver_minimizer_progress_to_stdout();

  ceres::Solver::Summary summary;
  // Dereference the unique_ptr to pass the raw pointer to Solve
  ceres::Solve(options, problem_.get(), &summary);

  if (summary.IsSolutionUsable()) {
    guess_velocity_x_mps_ = solver_velocity_x_mps_;
    guess_velocity_y_mps_ = solver_velocity_y_mps_;
    guess_velocity_omega_rad_s_ = solver_velocity_omega_rad_s_;
  }

  return {summary.IsSolutionUsable(), solver_velocity_x_mps_,
          solver_velocity_y_mps_, solver_velocity_omega_rad_s_,
          summary.final_cost};
}

void KinematicsEstimator::updateCovariance(const SolverResult &result,
                                           double dt) {
  const int num_residuals = 8;
  const int num_params = 3;
  double sigma2 = (num_residuals - num_params > 0)
                      ? (2.0 * result.final_cost) /
                            static_cast<double>(num_residuals - num_params)
                      : 1e-6;

  ceres::Covariance::Options cov_options;
  ceres::Covariance covariance(cov_options);
  std::vector<std::pair<const double *, const double *>> cov_blocks = {
      {&solver_velocity_x_mps_, &solver_velocity_y_mps_},
      {&solver_velocity_x_mps_, &solver_velocity_omega_rad_s_},
      {&solver_velocity_x_mps_, &solver_velocity_x_mps_},
      {&solver_velocity_y_mps_, &solver_velocity_omega_rad_s_},
      {&solver_velocity_y_mps_, &solver_velocity_y_mps_},
      {&solver_velocity_omega_rad_s_, &solver_velocity_omega_rad_s_},
  };

  // Safety: Check if covariance computation succeeded, using problem_.get()
  if (covariance.Compute(cov_blocks, problem_.get())) {
    double twist_cov[9] = {0.0};

    auto getCov = [&](const double *a, const double *b) {
      double value[1];
      covariance.GetCovarianceBlock(a, b, value);
      return value[0] * sigma2;
    };

    const double cov_vx_vx =
        getCov(&solver_velocity_x_mps_, &solver_velocity_x_mps_);
    const double cov_vx_vy =
        getCov(&solver_velocity_x_mps_, &solver_velocity_y_mps_);
    const double cov_vx_om =
        getCov(&solver_velocity_x_mps_, &solver_velocity_omega_rad_s_);
    const double cov_vy_vy =
        getCov(&solver_velocity_y_mps_, &solver_velocity_y_mps_);
    const double cov_vy_om =
        getCov(&solver_velocity_y_mps_, &solver_velocity_omega_rad_s_);
    const double cov_om_om =
        getCov(&solver_velocity_omega_rad_s_, &solver_velocity_omega_rad_s_);

    twist_cov[0] = cov_vx_vx;
    twist_cov[1] = cov_vx_vy;
    twist_cov[2] = cov_vx_om;
    twist_cov[3] = cov_vx_vy;
    twist_cov[4] = cov_vy_vy;
    twist_cov[5] = cov_vy_om;
    twist_cov[6] = cov_vx_om;
    twist_cov[7] = cov_vy_om;
    twist_cov[8] = cov_om_om;

    const double yaw = orientation_yaw_rad_;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    double delta_x_local_m = 0.0;
    double delta_y_local_m = 0.0;
    double d_dx_d_vx = 0.0;
    double d_dx_d_vy = 0.0;
    double d_dx_d_om = 0.0;
    double d_dy_d_vx = 0.0;
    double d_dy_d_vy = 0.0;
    double d_dy_d_om = 0.0;
    double d_dtheta_d_om = dt;

    const double velocity_x_mps = solver_velocity_x_mps_;
    const double velocity_y_mps = solver_velocity_y_mps_;
    const double angular_velocity_rad_s = solver_velocity_omega_rad_s_;

    if (std::abs(angular_velocity_rad_s) <
        config_.se2_integration_omega_threshold()) {
      delta_x_local_m = velocity_x_mps * dt;
      delta_y_local_m = velocity_y_mps * dt;

      d_dx_d_vx = dt;
      d_dy_d_vy = dt;
    } else {
      const double wdt = angular_velocity_rad_s * dt;
      const double sin_wdt = std::sin(wdt);
      const double cos_wdt = std::cos(wdt);

      delta_x_local_m =
          (velocity_x_mps * sin_wdt + velocity_y_mps * (cos_wdt - 1.0)) /
          angular_velocity_rad_s;
      delta_y_local_m =
          (velocity_x_mps * (1.0 - cos_wdt) + velocity_y_mps * sin_wdt) /
          angular_velocity_rad_s;

      const double inv_omega = 1.0 / angular_velocity_rad_s;
      const double inv_omega_sq = inv_omega * inv_omega;

      d_dx_d_vx = sin_wdt * inv_omega;
      d_dx_d_vy = (cos_wdt - 1.0) * inv_omega;
      d_dy_d_vx = (1.0 - cos_wdt) * inv_omega;
      d_dy_d_vy = sin_wdt * inv_omega;

      const double dnx_d_om =
          dt * (velocity_x_mps * cos_wdt - velocity_y_mps * sin_wdt);
      const double dny_d_om =
          dt * (velocity_x_mps * sin_wdt + velocity_y_mps * cos_wdt);

      const double nx =
          velocity_x_mps * sin_wdt + velocity_y_mps * (cos_wdt - 1.0);
      const double ny =
          velocity_x_mps * (1.0 - cos_wdt) + velocity_y_mps * sin_wdt;

      d_dx_d_om = (dnx_d_om * angular_velocity_rad_s - nx) * inv_omega_sq;
      d_dy_d_om = (dny_d_om * angular_velocity_rad_s - ny) * inv_omega_sq;
    }

    const double j_pose[9] = {
        1.0, 0.0, -sin_yaw * delta_x_local_m - cos_yaw * delta_y_local_m,
        0.0, 1.0, cos_yaw * delta_x_local_m - sin_yaw * delta_y_local_m,
        0.0, 0.0, 1.0};

    const double g[9] = {cos_yaw * d_dx_d_vx - sin_yaw * d_dy_d_vx,
                         cos_yaw * d_dx_d_vy - sin_yaw * d_dy_d_vy,
                         cos_yaw * d_dx_d_om - sin_yaw * d_dy_d_om,
                         sin_yaw * d_dx_d_vx + cos_yaw * d_dy_d_vx,
                         sin_yaw * d_dx_d_vy + cos_yaw * d_dy_d_vy,
                         sin_yaw * d_dx_d_om + cos_yaw * d_dy_d_om,
                         0.0,
                         0.0,
                         d_dtheta_d_om};

    // mat3_mul / mat3_transpose are free functions defined in the anonymous
    // namespace at the top of this file — no longer re-created every tick.
    double jp[9];
    double jpjt[9];
    double gt[9];
    double q_pose[9];
    double propagated_pose_cov[9];

    mat3_mul(j_pose, pose_covariance_.data(), jp);
    mat3_transpose(j_pose, gt);
    mat3_mul(jp, gt, jpjt);

    double gq[9];
    double gtg[9];
    mat3_mul(g, twist_cov, gq);
    mat3_transpose(g, gt);
    mat3_mul(gq, gt, gtg);

    for (int i = 0; i < 9; ++i) {
      q_pose[i] = gtg[i];
      propagated_pose_cov[i] = jpjt[i] + q_pose[i];
    }

    pose_covariance_ = {
        propagated_pose_cov[0], propagated_pose_cov[1], propagated_pose_cov[2],
        propagated_pose_cov[3], propagated_pose_cov[4], propagated_pose_cov[5],
        propagated_pose_cov[6], propagated_pose_cov[7], propagated_pose_cov[8]};

    odometry_.pose.covariance = {pose_covariance_[0],
                                 pose_covariance_[1],
                                 0.,
                                 0.,
                                 0.,
                                 pose_covariance_[2],
                                 pose_covariance_[3],
                                 pose_covariance_[4],
                                 0.,
                                 0.,
                                 0.,
                                 pose_covariance_[5],
                                 0.,
                                 0.,
                                 config_.pose_covariance_diagonal()[2],
                                 0.,
                                 0.,
                                 0.,
                                 0.,
                                 0.,
                                 0.,
                                 config_.pose_covariance_diagonal()[3],
                                 0.,
                                 0.,
                                 0.,
                                 0.,
                                 0.,
                                 0.,
                                 config_.pose_covariance_diagonal()[4],
                                 0.,
                                 pose_covariance_[6],
                                 pose_covariance_[7],
                                 0.,
                                 0.,
                                 0.,
                                 pose_covariance_[8]};

    // BUG FIX: preserve configured diagonal entries for the unmeasured DOFs
    // (vz, omega_x, omega_y) that were set in initOdomMessage.  Without this
    // they are silently zeroed out after the first successful Ceres solve,
    // which produces a non-positive-definite covariance matrix for downstream
    // EKF/UKF filters.
    odometry_.twist.covariance = {
        twist_cov[0], twist_cov[1], 0., 0., 0., twist_cov[2],
        twist_cov[3], twist_cov[4], 0., 0., 0., twist_cov[5],
        0., 0., config_.twist_covariance_diagonal()[2], 0., 0., 0.,
        0., 0., 0., config_.twist_covariance_diagonal()[3], 0., 0.,
        0., 0., 0., 0., config_.twist_covariance_diagonal()[4], 0.,
        twist_cov[6], twist_cov[7], 0., 0., 0., twist_cov[8]};
  }
}

void KinematicsEstimator::integratePose(double velocity_x_mps,
                                        double velocity_y_mps,
                                        double angular_velocity_rad_s,
                                        double dt_s) {

  // Apply Low-Pass EMA Filter
  // alpha = 1.0 means no filtering (raw data). alpha = 0.1 means heavily
  // smoothed.
  double filtered_velocity_x_mps =
      (config_.twist_ema_alpha() * velocity_x_mps) +
      ((1.0 - config_.twist_ema_alpha()) * velocity_x_mps_);
  double filtered_velocity_y_mps =
      (config_.twist_ema_alpha() * velocity_y_mps) +
      ((1.0 - config_.twist_ema_alpha()) * velocity_y_mps_);
  double filtered_omega_rad_s =
      (config_.twist_ema_alpha() * angular_velocity_rad_s) +
      ((1.0 - config_.twist_ema_alpha()) * velocity_omega_rad_s_);

  // 1. Calculate local frame translation using SE(2) exact integration
  double delta_x_local_m = 0.0;
  double delta_y_local_m = 0.0;

  // Use the configured threshold to switch integration modes
  if (std::abs(filtered_omega_rad_s) <
      config_.se2_integration_omega_threshold()) {
    // Straight-line Euler
    delta_x_local_m = filtered_velocity_x_mps * dt_s;
    delta_y_local_m = filtered_velocity_y_mps * dt_s;
  } else {
    // Ackermann or curved motion (Exact Arc Integration)
    const double sin_wdt = std::sin(filtered_omega_rad_s * dt_s);
    const double cos_wdt = std::cos(filtered_omega_rad_s * dt_s);

    delta_x_local_m = (filtered_velocity_x_mps * sin_wdt +
                       filtered_velocity_y_mps * (cos_wdt - 1.0)) /
                      filtered_omega_rad_s;
    delta_y_local_m = (filtered_velocity_x_mps * (1.0 - cos_wdt) +
                       filtered_velocity_y_mps * sin_wdt) /
                      filtered_omega_rad_s;
  }

  // 2. Rotate the local translation into the global Odometry frame
  const double cos_yaw = std::cos(orientation_yaw_rad_);
  const double sin_yaw = std::sin(orientation_yaw_rad_);

  position_x_m_ += delta_x_local_m * cos_yaw - delta_y_local_m * sin_yaw;
  position_y_m_ += delta_x_local_m * sin_yaw + delta_y_local_m * cos_yaw;

  // 3. Update global heading
  orientation_yaw_rad_ += filtered_omega_rad_s * dt_s;

  // Normalize heading to keep it between -PI and PI (prevents floating point
  // drift)
  orientation_yaw_rad_ = std::atan2(std::sin(orientation_yaw_rad_),
                                    std::cos(orientation_yaw_rad_));

  // Update internal state trackers for the Twist message
  velocity_x_mps_ = filtered_velocity_x_mps;
  velocity_y_mps_ = filtered_velocity_y_mps;
  velocity_omega_rad_s_ = filtered_omega_rad_s;
}

OdometryEstimate
KinematicsEstimator::populateRosMessages(const rclcpp::Time &timestamp) {
  OdometryEstimate estimate;

  // Odometry Message
  odometry_.header.stamp = timestamp;
  odometry_.pose.pose.position.x = position_x_m_;
  odometry_.pose.pose.position.y = position_y_m_;
  odometry_.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, orientation_yaw_rad_);
  odometry_.pose.pose.orientation = tf2::toMsg(q);

  odometry_.twist.twist.linear.x = velocity_x_mps_;
  odometry_.twist.twist.linear.y = velocity_y_mps_;
  odometry_.twist.twist.angular.z = velocity_omega_rad_s_;

  // Transform Message
  transformation_.transforms[0].header.stamp = timestamp;
  transformation_.transforms[0].transform.translation.x = position_x_m_;
  transformation_.transforms[0].transform.translation.y = position_y_m_;
  transformation_.transforms[0].transform.rotation = tf2::toMsg(q);

  // Finalize Estimate Struct
  estimate.odometry = odometry_;
  estimate.transform = transformation_;
  estimate.valid = true;

  return estimate;
}

/**
 * @brief Reset estimator internal state: pose, velocities and timestamps.
 */
void KinematicsEstimator::reset() {
  position_x_m_ = 0.0;
  position_y_m_ = 0.0;
  orientation_yaw_rad_ = 0.0;

  velocity_x_mps_ = 0.0;
  velocity_y_mps_ = 0.0;
  velocity_omega_rad_s_ = 0.0;

  guess_velocity_x_mps_ = 0.0;
  guess_velocity_y_mps_ = 0.0;
  guess_velocity_omega_rad_s_ = 0.0;

  pose_covariance_ = {config_.pose_covariance_diagonal()[0], 0.0, 0.0, 0.0,
                      config_.pose_covariance_diagonal()[1], 0.0, 0.0, 0.0,
                      config_.pose_covariance_diagonal()[5]};

  wheel_quality_.fill(1.0);

  // solver_velocity_x_mps_ = 0.0;
  // solver_velocity_y_mps_ = 0.0;
  // solver_velocity_omega_rad_s_ = 0.0;

  timestamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  initOdomMessage();
  initTransformMessage();
}