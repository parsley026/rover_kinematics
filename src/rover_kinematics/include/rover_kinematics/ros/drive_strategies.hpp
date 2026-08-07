#pragma once

#include <cmath>
#include <memory>
#include <unordered_map>

#include "rex_interfaces/msg/rover_control.hpp"
#include "rover_kinematics/core/kinematics_config.hpp"
#include "rover_kinematics/core/kinematics_solver.hpp"

/**
 * @class IDriveStrategy
 * @brief Abstract base for all drive IK strategies.
 *
 * Each concrete strategy encapsulates the command → WheelCommand logic for one
 * drive mode (Ackermann, Crab, Spin, Autonomy).  Strategies are stateless;
 * they receive the solver and config by const-ref and perform no I/O.
 *
 * Dispatch key convention (matches KinematicsNode constants):
 *   SYM_ACKERMANN_MODE   (1) → SymmetricAckermannStrategy
 *   FWD_ACKERMANN_MODE   (2) → FwdAckermannStrategy
 *   RWD_ACKERMANN_MODE   (3) → RwdAckermannStrategy
 *   CRAB_MODE            (4) → CrabStrategy
 *   SYM_SPIN_MODE        (5) → SymmetricSpinStrategy
 *   CONTROL_MODE_DRIVE_AUTONOMY(6) → AutonomyDriveStrategy
 *
 * The manual drive-mode values (1-5) and the autonomy control-mode value (6)
 * do not overlap, so a single flat std::unordered_map<int, IDriveStrategy>
 * covers all cases.
 */
class IDriveStrategy {
public:
  virtual ~IDriveStrategy() = default;

  /**
   * @brief Compute per-wheel targets from a rover control command.
   *
   * @param cmd     Incoming rover control (vel, x_axis, y_axis, mode).
   * @param solver  IK solver — holds geometry and constraint logic.
   * @param config  Rover config — provides steering radii and limits.
   * @return        WheelCommand with per-wheel steering angles and drive
   * speeds.
   */
  virtual WheelCommand compute(const rex_interfaces::msg::RoverControl &cmd,
                               const KinematicsSolver &solver,
                               const KinematicsConfig &config) const = 0;
};

// ── Template Method Base for Manual Drive ────────────────────────────────────

/**
 * @class ManualDriveStrategy
 * @brief Template method base class for manual joystick modes.
 *
 * Centralises the "Ghost Velocity" and "Drive Muting" logic.
 * Ensures the physical drive motors are muted when the user's joystick
 * input is zero, whilst delegating the specific gamepad-to-kinematics
 * math to the concrete sub-modes.
 */
class ManualDriveStrategy : public IDriveStrategy {
public:
  WheelCommand compute(const rex_interfaces::msg::RoverControl &cmd,
                       const KinematicsSolver &solver,
                       const KinematicsConfig &config) const override final {
    // 1. Calculate ghost velocity for dry-steering
    const double ghost_vel = (std::fabs(cmd.vel) < 1e-9) ? 1.0 : cmd.vel;

    // 2. Delegate specific gamepad-to-kinematics parsing
    WheelCommand target = computeSubMode(cmd, ghost_vel, solver, config);

    // 3. Mute drive motors if actual joystick input is zero
    if (std::fabs(cmd.vel) < 1e-9) {
      target.drive_velocity_mps.fill(0.0);
    }

    return target;
  }

protected:
  virtual WheelCommand
  computeSubMode(const rex_interfaces::msg::RoverControl &cmd, double ghost_vel,
                 const KinematicsSolver &solver,
                 const KinematicsConfig &config) const = 0;
};

// ── Concrete Strategies
// ────────────────────────────────────────────────────────

/**
 * @class ManualSymAckermannStrategy
 * @brief All four wheels steer; ICR at robot centre.
 */
class ManualSymAckermannStrategy final : public ManualDriveStrategy {
protected:
  WheelCommand computeSubMode(const rex_interfaces::msg::RoverControl &cmd,
                              double ghost_vel, const KinematicsSolver &solver,
                              const KinematicsConfig &config) const override {
    double angular_rad_s = 0.0;
    if (std::fabs(cmd.x_axis) > 1e-9) {
      const double slope =
          (config.min_steering_radius() - config.max_steering_radius()) / 0.99;
      const double offset = config.min_steering_radius() - slope;
      const double radius = slope * std::fabs(cmd.x_axis) + offset;
      angular_rad_s = (ghost_vel / radius) * ((cmd.x_axis >= 0.0) ? 1.0 : -1.0);
    }
    return solver.computeSymmetricAckermann(ghost_vel, angular_rad_s);
  }
};

/**
 * @class ManualFwdAckermannStrategy
 * @brief Front wheels steer; ICR locked to rear axle.
 */
class ManualFwdAckermannStrategy final : public ManualDriveStrategy {
protected:
  WheelCommand computeSubMode(const rex_interfaces::msg::RoverControl &cmd,
                              double ghost_vel, const KinematicsSolver &solver,
                              const KinematicsConfig &config) const override {
    double angular_rad_s = 0.0;
    if (std::fabs(cmd.x_axis) > 1e-9) {
      const double slope =
          (config.min_steering_radius() - config.max_steering_radius()) / 0.99;
      const double offset = config.min_steering_radius() - slope;
      const double radius = slope * std::fabs(cmd.x_axis) + offset;
      angular_rad_s = (ghost_vel / radius) * ((cmd.x_axis >= 0.0) ? 1.0 : -1.0);
    }
    return solver.computeFrontOnlyAckermann(ghost_vel, angular_rad_s);
  }
};

/**
 * @class ManualRwdAckermannStrategy
 * @brief Rear wheels steer; ICR locked to front axle.
 */
class ManualRwdAckermannStrategy final : public ManualDriveStrategy {
protected:
  WheelCommand computeSubMode(const rex_interfaces::msg::RoverControl &cmd,
                              double ghost_vel, const KinematicsSolver &solver,
                              const KinematicsConfig &config) const override {
    double angular_rad_s = 0.0;
    if (std::fabs(cmd.x_axis) > 1e-9) {
      const double slope =
          (config.min_steering_radius() - config.max_steering_radius()) / 0.99;
      const double offset = config.min_steering_radius() - slope;
      const double radius = slope * std::fabs(cmd.x_axis) + offset;
      angular_rad_s = (ghost_vel / radius) * ((cmd.x_axis >= 0.0) ? 1.0 : -1.0);
    }
    return solver.computeRearOnlyAckermann(ghost_vel, angular_rad_s);
  }
};

/**
 * @class ManualCrabStrategy
 * @brief Holonomic crab translation — all wheels point in the same direction.
 */
class ManualCrabStrategy final : public ManualDriveStrategy {
protected:
  WheelCommand computeSubMode(const rex_interfaces::msg::RoverControl &cmd,
                              double ghost_vel, const KinematicsSolver &solver,
                              const KinematicsConfig &) const override {
    const double axis_y = std::max(0.0, static_cast<double>(cmd.y_axis));
    double theta = 0.0;

    if (std::fabs(axis_y) >= 1e-9 || std::fabs(cmd.x_axis) >= 1e-9) {
      theta = std::atan2(cmd.x_axis, axis_y);
    }

    return solver.computeCrab(ghost_vel * std::cos(theta),
                              ghost_vel * std::sin(theta));
  }
};

/**
 * @class ManualSymSpinStrategy
 * @brief In-place symmetric spin — ICR at the geometric centre of the robot.
 */
class ManualSymSpinStrategy final : public ManualDriveStrategy {
protected:
  WheelCommand computeSubMode(const rex_interfaces::msg::RoverControl & /*cmd*/,
                              double ghost_vel, const KinematicsSolver &solver,
                              const KinematicsConfig &) const override {
    return solver.computeSymmetricSpin(ghost_vel);
  }
};

/**
 * @class AutonomyDriveStrategy
 * @brief Autonomy / MPPI path-following — bypasses the manual drive-mode
 * multiplexer.
 */
class AutonomyDriveStrategy final : public IDriveStrategy {
public:
  WheelCommand compute(const rex_interfaces::msg::RoverControl &cmd,
                       const KinematicsSolver &solver,
                       const KinematicsConfig &) const override {
    return solver.computeSymmetricAckermann(cmd.vel, cmd.x_axis);
  }
};
