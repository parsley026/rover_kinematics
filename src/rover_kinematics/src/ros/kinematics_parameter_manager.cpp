#include "rover_kinematics/ros/kinematics_parameter_manager.hpp"

#include "rover_kinematics/core/kinematics_estimator.hpp"
#include "rover_kinematics/core/kinematics_solver.hpp"
#include "rover_kinematics/ros/hardware_interface.hpp"

#include <unordered_map>

// ── Constructor ───────────────────────────────────────────────────────────────

KinematicsParameterManager::KinematicsParameterManager(rclcpp::Node*    node,
                                                       KinematicsConfig& config)
    : node_(node), config_(config) {}

// ── Parameter table ───────────────────────────────────────────────────────────
//
// Each row describes one tuneable parameter completely.
// The parameter name is written ONCE — impossible to drift between
// declare / apply / validate / propagate.
//
// Column layout:
//   name | flags | get_default(config) | apply(param, config) | validate(param)
//
// validate may be nullptr when no range check is required.

const std::vector<KinematicsParameterManager::ParamEntry>
KinematicsParameterManager::kParams = {

  // ── Physical geometry ───────────────────────────────────────────────────────
  { "wheelbase", kSolver | kEstimator,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.wheelbase()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.wheelbase_ = p.as_double(); },
    [](const rclcpp::Parameter& p) -> std::string {
        return p.as_double() > 0.0 ? "" : "wheelbase must be > 0";
    }
  },
  { "track_width", kSolver | kEstimator,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.track_width()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.track_width_ = p.as_double(); },
    [](const rclcpp::Parameter& p) -> std::string {
        return p.as_double() > 0.0 ? "" : "track_width must be > 0";
    }
  },
  { "wheel_diameter", kDerived | kHardware,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.wheel_diameter()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.wheel_diameter_ = p.as_double(); },
    [](const rclcpp::Parameter& p) -> std::string {
        return p.as_double() > 0.0 ? "" : "wheel_diameter must be > 0";
    }
  },
  { "poles_pair_number", kDerived | kHardware,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(static_cast<int64_t>(c.poles_pairs_number())); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.poles_pairs_number_ = static_cast<int>(p.as_int()); },
    nullptr
  },
  { "motor_gear_ratio", kDerived | kHardware,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.motor_gear_ratio()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.motor_gear_ratio_ = p.as_double(); },
    nullptr
  },
  { "min_erpm", kHardware,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.min_erpm()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.min_erpm_ = p.as_double(); },
    nullptr
  },
  { "max_erpm", kDerived | kHardware,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.max_erpm()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.max_erpm_ = p.as_double(); },
    nullptr
  },

  // ── Steering geometry ────────────────────────────────────────────────────────
  // Per-wheel mechanical angle limits [fl=0, fr=1, rl=2, rr=3]
  { "min_mechanical_angle_fl", kSolver,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.min_mechanical_angle(0)); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.min_mechanical_angle_[0] = p.as_double(); },
    nullptr
  },
  { "min_mechanical_angle_fr", kSolver,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.min_mechanical_angle(1)); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.min_mechanical_angle_[1] = p.as_double(); },
    nullptr
  },
  { "min_mechanical_angle_rl", kSolver,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.min_mechanical_angle(2)); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.min_mechanical_angle_[2] = p.as_double(); },
    nullptr
  },
  { "min_mechanical_angle_rr", kSolver,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.min_mechanical_angle(3)); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.min_mechanical_angle_[3] = p.as_double(); },
    nullptr
  },
  { "max_mechanical_angle_fl", kSolver,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.max_mechanical_angle(0)); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.max_mechanical_angle_[0] = p.as_double(); },
    nullptr
  },
  { "max_mechanical_angle_fr", kSolver,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.max_mechanical_angle(1)); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.max_mechanical_angle_[1] = p.as_double(); },
    nullptr
  },
  { "max_mechanical_angle_rl", kSolver,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.max_mechanical_angle(2)); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.max_mechanical_angle_[2] = p.as_double(); },
    nullptr
  },
  { "max_mechanical_angle_rr", kSolver,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.max_mechanical_angle(3)); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.max_mechanical_angle_[3] = p.as_double(); },
    nullptr
  },
  { "min_steering_radius", kSolver,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.min_steering_radius()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.min_steering_radius_ = p.as_double(); },
    [](const rclcpp::Parameter& p) -> std::string {
        return p.as_double() > 0.0 ? "" : "min_steering_radius must be > 0";
    }
  },
  { "max_steering_radius", kSolver,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.max_steering_radius()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.max_steering_radius_ = p.as_double(); },
    nullptr
  },

  // ── Estimator tuning ─────────────────────────────────────────────────────────
  { "huber_loss_threshold", kEstimator,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.huber_loss_threshold()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.huber_loss_threshold_ = p.as_double(); },
    nullptr
  },
  { "twist_ema_alpha", kEstimator,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.twist_ema_alpha()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.twist_ema_alpha_ = p.as_double(); },
    [](const rclcpp::Parameter& p) -> std::string {
        const double a = p.as_double();
        return (a >= 0.0 && a <= 1.0) ? "" : "twist_ema_alpha must be in [0, 1]";
    }
  },
  { "se2_integration_omega_threshold", kEstimator,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.se2_integration_omega_threshold()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.se2_integration_omega_threshold_ = p.as_double(); },
    nullptr
  },
  { "max_allowed_dt", kEstimator,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.max_allowed_dt()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.max_allowed_dt_ = p.as_double(); },
    nullptr
  },

  // ── Ceres solver ─────────────────────────────────────────────────────────────
  // Solver params only update config_; the estimator reads them fresh on each solve.
  { "solver_max_num_iterations", kNone,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(static_cast<int64_t>(c.solver_max_num_iterations())); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.solver_max_num_iterations_ = static_cast<int>(p.as_int()); },
    nullptr
  },
  { "solver_function_tolerance", kNone,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.solver_function_tolerance()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.solver_function_tolerance_ = p.as_double(); },
    nullptr
  },
  { "solver_gradient_tolerance", kNone,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.solver_gradient_tolerance()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.solver_gradient_tolerance_ = p.as_double(); },
    nullptr
  },
  { "solver_parameter_tolerance", kNone,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.solver_parameter_tolerance()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.solver_parameter_tolerance_ = p.as_double(); },
    nullptr
  },
  { "solver_minimizer_progress_to_stdout", kNone,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.solver_minimizer_progress_to_stdout()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.solver_minimizer_progress_to_stdout_ = p.as_bool(); },
    nullptr
  },

  // ── Publishing / timing ───────────────────────────────────────────────────────
  { "publish_rate", kTimer,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.publish_rate()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.publish_rate_ = p.as_double(); },
    [](const rclcpp::Parameter& p) -> std::string {
        return p.as_double() > 0.0 ? "" : "publish_rate must be > 0";
    }
  },
  { "feedback_timeout_sec", kNone,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.feedback_timeout_sec()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.feedback_timeout_sec_ = p.as_double(); },
    nullptr
  },

  // ── Wheel quality ─────────────────────────────────────────────────────────────
  { "enable_dynamic_wheel_weighting", kEstimator,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.enable_dynamic_wheel_weighting()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.enable_dynamic_wheel_weighting_ = p.as_bool(); },
    nullptr
  },
  { "wheel_quality_low_erpm_threshold", kNone,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.wheel_quality_low_erpm_threshold()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.wheel_quality_low_erpm_threshold_ = p.as_double(); },
    nullptr
  },
  { "wheel_quality_high_current_threshold", kNone,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.wheel_quality_high_current_threshold()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.wheel_quality_high_current_threshold_ = p.as_double(); },
    nullptr
  },
  { "wheel_quality_high_duty_threshold", kNone,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.wheel_quality_high_duty_threshold()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.wheel_quality_high_duty_threshold_ = p.as_double(); },
    nullptr
  },
  { "wheel_quality_min_weight", kEstimator,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.wheel_quality_min_weight()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.wheel_quality_min_weight_ = p.as_double(); },
    nullptr
  },

  // ── Coherence safety ──────────────────────────────────────────────────────────
  { "enable_coherence_safety", kNone,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.enable_coherence_safety()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.enable_coherence_safety_ = p.as_bool(); },
    nullptr
  },
  { "steering_coherence_threshold", kNone,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.steering_coherence_threshold()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.steering_coherence_threshold_ = p.as_double(); },
    [](const rclcpp::Parameter& p) -> std::string {
        return (p.as_double() > 0.0 && p.as_double() <= 1.0) ? "" : "steering_coherence_threshold must be in (0, 1]";
    }
  },

  // ── Frames, topics & flags ────────────────────────────────────────────────────
  // base/odom frame IDs propagate to the estimator (which reinitialises its odom msg).
  { "base_frame_id", kEstimator,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.base_frame_id()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.base_frame_id_ = p.as_string(); },
    nullptr
  },
  { "odom_frame_id", kEstimator,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.odom_frame_id()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.odom_frame_id_ = p.as_string(); },
    nullptr
  },
  // publish_tf, use_measurement_timestamp: node reads via friend access each cycle.
  { "publish_tf", kNone,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.publish_tf()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.publish_tf_ = p.as_bool(); },
    nullptr
  },
  { "use_measurement_timestamp", kNone,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.use_measurement_timestamp()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.use_measurement_timestamp_ = p.as_bool(); },
    nullptr
  },
  { "cmd_vel_timeout_sec", kNone,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.cmd_vel_timeout_sec()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.cmd_vel_timeout_sec_ = p.as_double(); },
    [](const rclcpp::Parameter& p) -> std::string {
        return p.as_double() > 0.0 ? "" : "cmd_vel_timeout_sec must be > 0";
    }
  },
  { "initialization_timeout_sec", kNone,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.initialization_timeout_sec()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.initialization_timeout_sec_ = p.as_double(); },
    [](const rclcpp::Parameter& p) -> std::string {
        return p.as_double() > 0.0 ? "" : "initialization_timeout_sec must be > 0";
    }
  },
  { "stop_timeout_sec", kNone,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.stop_timeout_sec()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.stop_timeout_sec_ = p.as_double(); },
    [](const rclcpp::Parameter& p) -> std::string {
        return p.as_double() > 0.0 ? "" : "stop_timeout_sec must be > 0";
    }
  },
  // cmd_vel_autonomy_topic: subscription is created at startup; live re-subscription
  // not supported — update takes effect only after node restart.
  { "cmd_vel_autonomy_topic", kNone,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.cmd_vel_autonomy_topic()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.cmd_vel_autonomy_topic_ = p.as_string(); },
    nullptr
  },

  // ── Motor polarity (per-wheel: FL=0, FR=1, RL=2, RR=3) ─────────────────────
  { "invert_drive_fl", kHardware,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.invert_drive(0)); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.invert_drive_[0] = p.as_bool(); },
    nullptr
  },
  { "invert_drive_fr", kHardware,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.invert_drive(1)); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.invert_drive_[1] = p.as_bool(); },
    nullptr
  },
  { "invert_drive_rl", kHardware,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.invert_drive(2)); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.invert_drive_[2] = p.as_bool(); },
    nullptr
  },
  { "invert_drive_rr", kHardware,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.invert_drive(3)); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.invert_drive_[3] = p.as_bool(); },
    nullptr
  },
  { "invert_steering_fl", kHardware,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.invert_steering(0)); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.invert_steering_[0] = p.as_bool(); },
    nullptr
  },
  { "invert_steering_fr", kHardware,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.invert_steering(1)); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.invert_steering_[1] = p.as_bool(); },
    nullptr
  },
  { "invert_steering_rl", kHardware,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.invert_steering(2)); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.invert_steering_[2] = p.as_bool(); },
    nullptr
  },
  { "invert_steering_rr", kHardware,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.invert_steering(3)); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) { c.invert_steering_[3] = p.as_bool(); },
    nullptr
  },

  // ── Covariance diagonals ──────────────────────────────────────────────────────
  { "pose_covariance_diagonal", kEstimator,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.pose_covariance_diagonal()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) {
        const auto v = p.as_double_array();
        if (v.size() == 6) c.pose_covariance_diagonal_ = v;
    },
    [](const rclcpp::Parameter& p) -> std::string {
        return p.as_double_array().size() == 6
               ? "" : "pose_covariance_diagonal must have exactly 6 elements";
    }
  },
  { "twist_covariance_diagonal", kEstimator,
    [](const KinematicsConfig& c) { return rclcpp::ParameterValue(c.twist_covariance_diagonal()); },
    [](const rclcpp::Parameter& p, KinematicsConfig& c) {
        const auto v = p.as_double_array();
        if (v.size() == 6) c.twist_covariance_diagonal_ = v;
    },
    [](const rclcpp::Parameter& p) -> std::string {
        return p.as_double_array().size() == 6
               ? "" : "twist_covariance_diagonal must have exactly 6 elements";
    }
  },
};

// ── Index builder ─────────────────────────────────────────────────────────────

std::unordered_map<std::string, std::size_t>
KinematicsParameterManager::buildNameIndex(const std::vector<ParamEntry>& entries) {
    std::unordered_map<std::string, std::size_t> idx;
    idx.reserve(entries.size());
    for (std::size_t i = 0; i < entries.size(); ++i) {
        idx.emplace(entries[i].name, i);
    }
    return idx;
}

// ── initialize() ─────────────────────────────────────────────────────────────

void KinematicsParameterManager::initialize() {
    // Pass 1: declare every parameter using the current config value as default.
    // YAML-loaded values serve as defaults; --ros-args -p overrides come back in pass 2.
    for (const auto& e : kParams) {
        node_->declare_parameter(e.name, e.get_default(config_));
    }

    applyAll();
}

void KinematicsParameterManager::applyAll() {
    // Pass 2: read back the (potentially command-line-overridden) values.
    for (const auto& e : kParams) {
        e.apply(node_->get_parameter(e.name), config_);
    }

    config_.updateDerivedParameters();
}

// ── onSetParameters() ─────────────────────────────────────────────────────────

rcl_interfaces::msg::SetParametersResult
KinematicsParameterManager::onSetParameters(
    const std::vector<rclcpp::Parameter>& params,
    KinematicsSolver&                      solver,
    KinematicsEstimator&                   estimator,
    HardwareInterface&                     hw_interface,
    std::function<void(double)>            update_timer_fn)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // Build name→index map once on first call; safe since kParams is static const.
    static const auto index = buildNameIndex(kParams);

    uint8_t dirty = kNone;

    for (const auto& p : params) {
        auto it = index.find(p.get_name());
        if (it == index.end()) continue;   // unknown param — ignore

        const ParamEntry& e = kParams[it->second];

        // Validate before writing
        if (e.validate) {
            const auto err = e.validate(p);
            if (!err.empty()) {
                result.successful = false;
                result.reason     = err;
                return result;
            }
        }

        e.apply(p, config_);
        dirty |= e.flags;
    }

    // Propagate changes to subsystems based on accumulated dirty flags
    if (dirty & kDerived)   config_.updateDerivedParameters();
    if (dirty & kSolver)    solver.setConfig(config_);
    if (dirty & kEstimator) estimator.setConfig(config_);
    if (dirty & kHardware)  hw_interface.setConfig(config_);
    if (dirty & kTimer)     update_timer_fn(config_.publish_rate());

    return result;
}
