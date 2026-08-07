#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "rover_kinematics/core/kinematics_config.hpp"

namespace VescID {
  constexpr uint8_t DRIVE_FL = 0x50;
  constexpr uint8_t DRIVE_FR = 0x51;
  constexpr uint8_t DRIVE_RL = 0x52;
  constexpr uint8_t DRIVE_RR = 0x53;

  constexpr uint8_t STEER_FL  = 0x60;
  constexpr uint8_t STEER_FR  = 0x61;
  constexpr uint8_t STEER_RR  = 0x62; 
  constexpr uint8_t STEER_RL  = 0x63;
}

/**
 * @class HardwareInterface
 * @brief Convert between physical units (m, rad, m/s) and hardware units (ERPM,
 * degrees, precise_pos).
 *
 * @details
 * HardwareInterface encapsulates hardware-specific conversions and polarity
 * handling required to send and interpret messages to/from the VESC motor
 * controllers. It uses `KinematicsConfig` fields WheelCommand as
 * `wheel_radius_`, `poles_pairs_number_` and `motor_gear_ratio_` to perform
 * conversions.
 *
 * Conventions and units:
 * - Input/outputs to the public API use physical units: meters (m), radians
 * (rad), meters/sec (m/s).
 * - Drive set values produced by `driveSetFromMetersPerSecond()` map to the
 *   legacy ERPM-based command space used by the rover's CAN bridge.
 * - Steering values are represented as degrees or hardware-specific
 *   `precise_pos` depending on the source; this class preserves polarity
 *   inversions configured in `KinematicsConfig`.
 *
 * Thread-safety: stateless conversion functions are safe to call from multiple
 * threads as long as the underlying `config_` is not mutated concurrently.
 */
class HardwareInterface {
public:
  HardwareInterface() = default;
  /**
   * @brief Construct and set configuration.
   * @param cfg Rover configuration describing geometry and polarity.
   */
  explicit HardwareInterface(const KinematicsConfig &cfg) { setConfig(cfg); }

  /**
   * @brief Stores configuration used by subsequent unit conversions.
   *
   * @param cfg Configuration containing conversion parameters and polarity
   * settings.
   */
  void setConfig(const KinematicsConfig &cfg) { config_ = cfg; }

  /**
   * @brief Convert linear speed (m/s) to hardware drive set value (ERPM-like).
   *
   * Formula (preserved from legacy code):
   * set_value = (mps * 30 / (pi * wheel_radius)) * poles_pairs_number *
   * motor_gear_ratio + sign_offset
   *
   * @param mps Linear speed in meters/sec.
   * @param wheel_index Index of the wheel (0..3) used to apply polarity.
   * @return Hardware set value corresponding to the requested speed (ERPM
   * units).
   */
  double driveSetFromMetersPerSecond(double mps, std::size_t wheel_index) const;

  /**
   * @brief Convert hardware ERPM-like value -> linear speed (m/s) for feedback.
   *
   * This inverts the formula used for `driveSetFromMetersPerSecond` and
   * applies configured drive polarity corrections.
   *
   * @param erpm Motor speed reported by the controller (ERPM-like scale).
   * @param wheel_index Wheel index used to determine polarity.
   * @return Linear speed in meters/sec.
   */
  double metersPerSecondFromErpm(double erpm, std::size_t wheel_index) const;

  /**cout
   * @brief Convert steering angle in radians to hardware steering units.
   *
   * The legacy implementation used degrees; polarity inversions configured
   * in `KinematicsConfig` are applied.
   *
   * @param rad Steering angle in radians.
   * @param wheel_index Wheel index used to determine steering polarity.
   * @return Steering set value in hardware units (degrees by convention).
   */
  double steeringSetFromRadians(double rad, std::size_t wheel_index) const;

  /**
   * @brief Convert hardware precise_pos to steering set value while applying
   * polarity.
   *
   * Some controllers provide a `precise_pos` measurement; this function
   * preserves the original mapping while allowing configured polarity.
   *
   * @param precise_pos Raw hardware position value.
   * @param wheel_index Wheel index used to determine steering polarity.
   * @return Steering set value adjusted for polarity.
   */
  double steeringSetFromPrecisePos(double precise_pos,
                                   std::size_t wheel_index) const;

  /**
   * @brief Map VESC ID from CAN messages to the drive wheel index used by the
   * stack.
   * @param vesc_id Raw VESC ID from incoming messages.
   * @return Wheel index in [0..3] corresponding to front_left, front_right,
   * rear_left, rear_right.
   */
  static std::size_t vescIdToDriveWheelIndex(uint8_t vesc_id);

  /**
   * @brief Map VESC ID from CAN messages to the steering wheel index used by
   * the stack.
   * @param vesc_id Raw VESC ID from incoming messages.
   * @return Wheel index in [0..3] corresponding to steering controllers.
   */
  static std::size_t vescIdToTurnWheelIndex(uint8_t vesc_id);

private:
  KinematicsConfig config_;
};
