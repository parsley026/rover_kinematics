#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>

#include "rover_kinematics/core/kinematics_config.hpp"

struct WheelCommand {
  std::array<double, 4> steering_angle_rad{};
  std::array<double, 4> drive_velocity_mps{};
};

class KinematicsSolver {
public:
  explicit KinematicsSolver(
      const KinematicsConfig &config = KinematicsConfig{});

  void setConfig(const KinematicsConfig &config);

  // - - -

  WheelCommand computeSymmetricAckermann(double velocity_x_mps,
                                         double velocity_omega_rad_s_) const;
  WheelCommand computeFrontOnlyAckermann(double velocity_x_mps,
                                         double velocity_omega_rad_s) const;
  WheelCommand computeRearOnlyAckermann(double velocity_x_mps,
                                        double velocity_omega_rad_s) const;

  // - - -

  WheelCommand computeOmnidirectional(double velocity_x_mps, double velocity_y_mps, double velocity_omega_rad_s) const;

  // - - -

  WheelCommand computeSkidSteer(double velocity_x_mps, double velocity_omega_rad_s) const;

  // - - -

  WheelCommand computeCrab(double velocity_x_mps, double velocity_y_mps) const;

  // - - -

  WheelCommand computeSymmetricSpin(double velocity_omega_rad_s) const;

  // - - -

  WheelCommand computeXConfiguration() const;

  // - - -

  WheelCommand computeTConfiguration() const;
  WheelCommand computeReverseTConfiguration() const;

  // - - -

  WheelCommand computeYConfiguration() const;
  WheelCommand computeReverseYConfiguration() const;

  // - - -

  WheelCommand computeSamplerConfiguration() const;

private:
  KinematicsConfig config_;

  // - - -

  void enforceDriveVelocityLimit(WheelCommand &command) const;

  // - - -

  void normalizeSteerAngle(WheelCommand &command) const;

  void optimizeSteerAngle(WheelCommand &command) const;

  // - - -

  void clampDriveVelocity(WheelCommand &command, double max_velocity_m_s) const;
  void clampSteerAngle(WheelCommand &command, double max_angle_rad) const;

  // - - -

  void enforceMinimumTurningRadius(double velocity_x_mps, double &velocity_omega_rad_s) const;
};
