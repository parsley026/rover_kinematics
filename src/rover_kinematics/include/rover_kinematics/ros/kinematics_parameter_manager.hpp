#pragma once

#include <functional>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rover_kinematics/core/kinematics_config.hpp"

// Forward declarations — avoids including ROS-heavy subsystem headers here.
class KinematicsSolver;
class KinematicsEstimator;
class HardwareInterface;

/**
 * @class KinematicsParameterManager
 * @brief Adapter/Facade that bridges the ROS 2 parameter server and KinematicsConfig.
 *
 * This class owns all parameter-server logic that previously lived in
 * KinematicsNode::declareParameters(), applyParameters(), and onSetParameters().
 *
 * Responsibilities:
 *  - Declare all KinematicsConfig fields on the ROS 2 parameter server.
 *  - Read back the (potentially overridden) values into config_ on startup.
 *  - Validate, apply, and propagate live parameter changes at runtime.
 *
 * Design: **table-driven** — every parameter is described exactly once as a
 * row in the static kParams table.  Adding a new tuneable parameter requires
 * only a single new table row; no other code changes are needed.
 *
 * Core/ROS boundary:
 *  KinematicsConfig (core/) remains 100% ROS-free.  This class (ros/) holds
 *  the only rclcpp-aware parameter logic.
 */
class KinematicsParameterManager {
public:
    /**
     * @param node    Non-owning pointer to the parent ROS 2 node.
     *                Lifetime guaranteed by KinematicsNode.
     * @param config  Non-owning reference to the node's config object.
     *                Lifetime guaranteed by KinematicsNode.
     */
    KinematicsParameterManager(rclcpp::Node* node, KinematicsConfig& config);

    /**
     * @brief Declare all parameters on the parameter server, then apply current values.
     *
     * Must be called once from the node constructor after the YAML config is loaded.
     * YAML values become the declared defaults; any --ros-args -p overrides are
     * applied immediately via get_parameter().
     */
    void initialize();

    /**
     * @brief Re-read and apply all parameters from the parameter server.
     *
     * Used during resetKinematicsCallback after the YAML config is reloaded.
     */
    void applyAll();

    /**
     * @brief Validate, apply, and propagate a batch of parameter changes.
     *
     * Called from the node's add_on_set_parameters_callback lambda.
     *
     * @param params           Incoming parameters from ros2 param set.
     * @param solver           IK solver — receives setConfig() if geometry changes.
     * @param estimator        Odometry estimator — receives setConfig() if its params change.
     * @param hw_interface     Hardware interface — receives setConfig() if motor params change.
     * @param update_timer_fn  Callback to recreate the update timer (called if publish_rate changes).
     * @return                 Result with successful=true, or failure + reason string.
     */
    rcl_interfaces::msg::SetParametersResult
    onSetParameters(const std::vector<rclcpp::Parameter>& params,
                    KinematicsSolver&                      solver,
                    KinematicsEstimator&                   estimator,
                    HardwareInterface&                     hw_interface,
                    std::function<void(double)>            update_timer_fn);

private:
    // ── Parameter descriptor ─────────────────────────────────────────────────

    struct ParamEntry {
        std::string name;
        uint8_t     flags;  // bitmask of UpdateFlags

        /// Returns the current field value wrapped in a ParameterValue for declare_parameter.
        std::function<rclcpp::ParameterValue(const KinematicsConfig&)> get_default;

        /// Writes the incoming parameter value into config.
        std::function<void(const rclcpp::Parameter&, KinematicsConfig&)> apply;

        /// Returns "" if the value is acceptable, or a human-readable error string.
        /// May be nullptr (no validation needed).
        std::function<std::string(const rclcpp::Parameter&)> validate;
    };

    // ── Subsystem propagation bitmask ────────────────────────────────────────

    enum UpdateFlags : uint8_t {
        kNone      = 0,
        kSolver    = 1 << 0,   ///< solver.setConfig(config_)
        kEstimator = 1 << 1,   ///< estimator.setConfig(config_)
        kHardware  = 1 << 2,   ///< hw_interface.setConfig(config_)
        kDerived   = 1 << 3,   ///< config_.updateDerivedParameters()
        kTimer     = 1 << 4,   ///< update_timer_fn(config_.publish_rate())
    };

    // ── Table ────────────────────────────────────────────────────────────────

    /// The single source of truth for all 36 tunable parameters.
    static const std::vector<ParamEntry> kParams;

    /// Build a name → index map for O(1) lookup in onSetParameters.
    static std::unordered_map<std::string, std::size_t>
    buildNameIndex(const std::vector<ParamEntry>& entries);

    // ── State ────────────────────────────────────────────────────────────────

    rclcpp::Node*     node_;    ///< non-owning
    KinematicsConfig& config_;  ///< non-owning
};
