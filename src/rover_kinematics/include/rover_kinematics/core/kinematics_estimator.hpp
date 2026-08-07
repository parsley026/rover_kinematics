#pragma once

#include <algorithm>
#include <array>
#include <ceres/ceres.h>
#include <ceres/covariance.h>
#include <cmath>
#include <cstddef>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rex_interfaces/msg/wheels.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <utility>
#include <vector>

#include "rover_kinematics/core/kinematics_config.hpp"

namespace {
enum WheelIndex {
  FRONT_RIGHT = 0,
  FRONT_LEFT = 1,
  REAR_LEFT = 2,
  REAR_RIGHT = 3
};

namespace CovarianceIndex {
constexpr int POSE_X = 0;
constexpr int POSE_Y = 7;
constexpr int YAW = 35;
} // namespace CovarianceIndex
} // namespace

struct WheelVelocity {
  double vx_mps;
  double vy_mps;
};

struct SolverResult {
  bool is_usable;

  double velocity_x_mps;
  double velocity_y_mps;
  double velocity_omega_rad_s;

  double final_cost;
};

struct OdometryEstimate {
  nav_msgs::msg::Odometry odometry;
  tf2_msgs::msg::TFMessage transform;
  bool valid{false};
};

class KinematicsEstimator {
public:
  explicit KinematicsEstimator(
      const KinematicsConfig &config = KinematicsConfig{});

  void setConfig(const KinematicsConfig &config);

  void initOdomMessage();
  void initTransformMessage();

  void setWheelQuality(const std::array<double, 4> &wheel_quality);

  void reset();

  OdometryEstimate update(const rex_interfaces::msg::Wheels &feedback,
                          const rclcpp::Time &timestamp);

private:
  std::unique_ptr<ceres::Problem> problem_;

  double updateAndGetTimeDelta(const rclcpp::Time &timestamp);

  std::array<WheelVelocity, 4>
  extractWheelVectors(const rex_interfaces::msg::Wheels &feedback) const;

  SolverResult
  solveBodyTwist(const std::array<WheelVelocity, 4> &wheel_vectors);

  void updateCovariance(const SolverResult &result, double dt);

  // void integratePose(double v_x, double v_y, double omega, double dt);

  void integratePose(double velocity_x_mps, double velocity_y_mps,
                     double angular_velocity_rad_s, double dt_s);

  OdometryEstimate populateRosMessages(const rclcpp::Time &timestamp);

  // ----------------------------------------

  struct WheelResidual {
  public:
    WheelResidual(const double *v_xi_ptr, const double *v_yi_ptr,
                  const double *weight_ptr, double x_i, double y_i)
        : v_xi_ptr_(v_xi_ptr), v_yi_ptr_(v_yi_ptr), weight_ptr_(weight_ptr),
          x_i_(x_i), y_i_(y_i) {}

    template <typename T>
    bool operator()(const T *const v_x, const T *const v_y,
                    const T *const omega, T *residual) const {
      const T measured_vx = T(*v_xi_ptr_);
      const T measured_vy = T(*v_yi_ptr_);
      const T weight = T(std::sqrt(std::max(*weight_ptr_, 0.0)));
      residual[0] = weight * (measured_vx - (*v_x - *omega * T(y_i_)));
      residual[1] = weight * (measured_vy - (*v_y + *omega * T(x_i_)));
      return true;
    }

  private:
    const double *v_xi_ptr_;
    const double *v_yi_ptr_;
    const double *weight_ptr_;
    const double x_i_;
    const double y_i_;
  };

  KinematicsConfig config_;

  nav_msgs::msg::Odometry odometry_;
  tf2_msgs::msg::TFMessage transformation_;

  rclcpp::Time timestamp_;

  double position_x_m_{0.0};
  double position_y_m_{0.0};
  double orientation_yaw_rad_{0.0};

  double velocity_x_mps_{0.0};
  double velocity_y_mps_{0.0};
  double velocity_omega_rad_s_{0.0};

  double guess_velocity_x_mps_{0.0};
  double guess_velocity_y_mps_{0.0};
  double guess_velocity_omega_rad_s_{0.0};

  double solver_velocity_x_mps_{0.0};
  double solver_velocity_y_mps_{0.0};
  double solver_velocity_omega_rad_s_{0.0};

  std::array<double, 9> pose_covariance_{{0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001}};
  std::array<double, 4> wheel_quality_{1.0, 1.0, 1.0, 1.0};
  double measured_vx_[4] = {0.0, 0.0, 0.0, 0.0};
  double measured_vy_[4] = {0.0, 0.0, 0.0, 0.0};

  bool problem_initialized_{false};
};