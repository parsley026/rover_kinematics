#pragma once

#include "rex_interfaces/msg/rover_control.hpp"
#include "rex_interfaces/msg/rover_status.hpp"
#include "rex_interfaces/msg/vesc_status.hpp"
#include "rex_interfaces/msg/wheels.hpp"
#include "rover_kinematics/core/kinematics_config.hpp"
#include "rover_kinematics/core/kinematics_estimator.hpp"
#include "rover_kinematics/core/kinematics_solver.hpp"
#include "rover_kinematics/ros/hardware_interface.hpp"

#include "rover_kinematics/ros/drive_strategies.hpp"
#include "rover_kinematics/ros/kinematics_parameter_manager.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

// // ─ ─ 

namespace CommunicationState {
  constexpr int32_t CREATED = 0;
  constexpr int32_t OPENING = 1;
  constexpr int32_t OPENED  = 2;
  constexpr int32_t CLOSING = 3;
  constexpr int32_t CLOSED  = 4;
  constexpr int32_t FAULTED = 5;
}

namespace ControlMode {
  constexpr int32_t NONE                     = 0;
  constexpr int32_t ESTOP                    = 1 << 0;
  constexpr int32_t STOP                     = 1 << 1;
  constexpr int32_t CONFIG                   = 1 << 2;
  constexpr int32_t DRIVE                    = 1 << 3;
  constexpr int32_t ROBOTIC_ARM              = 1 << 4;
  constexpr int32_t DEEP_SAMPLER             = 1 << 5;
  constexpr int32_t SURFACE_SAMPLER          = 1 << 6;
  constexpr int32_t DRIVE_AUTONOMY           = 1 << 7;
  constexpr int32_t ROBOTIC_ARM_AUTONOMY     = 1 << 8;
  constexpr int32_t DEEP_SAMPLER_AUTONOMY    = 1 << 9;
  constexpr int32_t SURFACE_SAMPLER_AUTONOMY = 1 << 10;
}

// // ─ ─ 

namespace VescCommand {
  constexpr uint8_t SET_DUTY                  = 0;
  constexpr uint8_t SET_CURRENT               = 1;
  constexpr uint8_t SET_CURRENT_BRAKE         = 2;
  constexpr uint8_t SET_RPM                   = 3;
  constexpr uint8_t SET_POS                   = 4;
  constexpr uint8_t SET_ORIGIN                = 5;
  constexpr uint8_t SET_POS_SPEED_LOOP        = 6;
  constexpr uint8_t SET_CURRENT_REL           = 10;
  constexpr uint8_t SET_CURRENT_BRAKE_REL     = 11;
  constexpr uint8_t SET_CURRENT_HANDBRAKE     = 12;
  constexpr uint8_t SET_CURRENT_HANDBRAKE_REL = 13;
}

// // ─ ─ 

namespace DriveMode {
  constexpr uint8_t BRAKE     = 44;
  constexpr uint8_t HANDBRAKE = 4;
  
  constexpr uint8_t SYM_ACKERMANN = 1;
  constexpr uint8_t FWD_ACKERMANN = 11;
  constexpr uint8_t RWD_ACKERMANN = 111;
  constexpr uint8_t CRAB          = 2;
  constexpr uint8_t SYM_SPIN      = 3;
}

// // ─ ─ 

template <typename T, std::size_t N>
constexpr T build_mask(const std::array<T, N>& flags) {
    T mask = 0;
    for (T flag : flags) {
        mask |= flag;
    }
    return mask;
}

constexpr std::array KinematicsControlMode = {
    // ControlMode::NONE                    ,
    // ControlMode::ESTOP                   ,
    ControlMode::STOP                    ,
    // ControlMode::CONFIG                  ,
    
    // ─
    ControlMode::DRIVE                   ,
    // ControlMode::ROBOTIC_ARM             ,
    ControlMode::DEEP_SAMPLER            ,
    ControlMode::SURFACE_SAMPLER         ,
    
    // ─
    ControlMode::DRIVE_AUTONOMY          ,
    // ControlMode::ROBOTIC_ARM_AUTONOMY    ,
    ControlMode::DEEP_SAMPLER_AUTONOMY   ,
    ControlMode::SURFACE_SAMPLER_AUTONOMY,
};

constexpr int KinematicsControlModeMask = build_mask(KinematicsControlMode);

// // ─ ─ 

constexpr uint8_t CURRENT_BRAKE_VALUE     = 5;
constexpr uint8_t CURRENT_HANDBRAKE_VALUE = 10;

#define STEER_PRESCALE 0.01

class KinematicsNode : public rclcpp::Node {
public:
  explicit KinematicsNode(const rclcpp::NodeOptions &options);

  void applySteeringFirstSafety(rex_interfaces::msg::Wheels &target,
                                const rex_interfaces::msg::Wheels &feedback,
                                double angle_tolerance_deg = 2.5);

private:
  // ── Layered Safety Checks ────────────────────────────────────────────────
  bool isMechanicallySafe(const rex_interfaces::msg::Wheels &feedback) const;
  bool isSteeringCoherent(const rex_interfaces::msg::Wheels &target,
                          const rex_interfaces::msg::Wheels &feedback) const;
  double getPhysicalAngleRad(double measured_value, std::size_t wheel_index) const;

  void initCmdVelBuffer();
  void initFeedbackBuffer();

  void updateTimerRate(double new_rate_hz);

  void onUpdate();

  void updateWatchdog(const rclcpp::Time &current_time);

  void statusCallback(const rex_interfaces::msg::RoverStatus::SharedPtr msg);
  
  void cmdVelManualCallback(const rex_interfaces::msg::RoverControl::SharedPtr msg);
  void cmdVelAutonomyCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  
  void feedbackCallback(const rex_interfaces::msg::VescStatus::SharedPtr msg);

  void resetOdometryCallback(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void resetKinematicsCallback(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);
  
  // // ─ ─ 

  double computeWheelQualityScore(const rex_interfaces::msg::VescStatus &status) const;
  void updateWheelQuality(std::size_t wheel_index, const rex_interfaces::msg::VescStatus &status);

  /// @brief Fits a rigid-body twist (Vx, Vy, ω) to the four wheel steering
  ///        directions and returns the mean normalised residual per wheel.
  ///        A value near 0 means all wheels agree on a coherent motion; a value
  ///        near 1 (or above the configured threshold) means the chassis is in a
  ///        destructively incoherent configuration.
  double computeSteeringCoherence(const std::array<double, 4> &steer_angles_rad) const;

  // // ─ ─ 

  rex_interfaces::msg::Wheels assembleWheelsFromCommand(const WheelCommand &command, const rclcpp::Time &time);

  rex_interfaces::msg::Wheels assembleSetOriginMessage(const rclcpp::Time &time);

  rex_interfaces::msg::Wheels assembleStopMessage(const rclcpp::Time &time);

  rex_interfaces::msg::Wheels assembleBrakeMessage(const rclcpp::Time &time);

  rex_interfaces::msg::Wheels assembleHandBrakeMessage(const rclcpp::Time &time);

  // // ─ ─ 

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Duration publish_period_;

  // // ─ ─ 

  rclcpp::Subscription<rex_interfaces::msg::RoverStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<rex_interfaces::msg::RoverControl>::SharedPtr cmd_vel_sub_manual_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_autonomy_;
  rclcpp::Subscription<rex_interfaces::msg::VescStatus>::SharedPtr wheels_vel_feedback_sub_;


  rclcpp::Publisher<rex_interfaces::msg::Wheels>::SharedPtr wheels_vel_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;


  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_kinematics_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_odometry_srv_;


  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      param_cb_handle_;


  rex_interfaces::msg::Wheels rover_wheels_velocity_;
  realtime_tools::RealtimeBuffer<rex_interfaces::msg::RoverControl>
      rover_cmd_velocity_buffer_;
  realtime_tools::RealtimeBuffer<rex_interfaces::msg::Wheels>
      rover_wheels_velocity_feedback_buffer_;


  KinematicsConfig config_;
  KinematicsParameterManager param_manager_;
  KinematicsSolver kinematics_solver_;
  KinematicsEstimator kinematics_estimator;
  HardwareInterface hardware_interface_;

  std::string config_file_path_;


  std::unordered_map<int, std::unique_ptr<IDriveStrategy>> strategies_;


  std::mutex feedback_mutex_;

  std::array<double, 4> wheel_quality_weights_{{1.0, 1.0, 1.0, 1.0}};
  std::array<double, 4> drive_quality_weights_{{1.0, 1.0, 1.0, 1.0}};
  std::array<double, 4> steer_quality_weights_{{1.0, 1.0, 1.0, 1.0}};

  // // ─ ─ 

  std::atomic<int> control_mode_{ControlMode::DRIVE};
  std::atomic<int> communication_state_{CommunicationState::CREATED};

  std::atomic<int64_t> initialization_time_ns_{0};
  std::atomic<int64_t> last_feedback_time_ns_{0};
  std::atomic<int64_t> last_cmd_vel_time_ns_{0};
  std::atomic<int64_t> last_kinematics_active_time_ns_{0};
  
  std::atomic<bool>      feedback_stale_{false};
  std::atomic<bool> prev_feedback_stale_{false};
};
