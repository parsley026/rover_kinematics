#pragma once

#include <cmath>
#include <string>
#include <vector>
#include <ceres/ceres.h>
#include <yaml-cpp/yaml.h>



/**
 * @class KinematicsConfig
 * @brief Encapsulates all rover kinematics configuration parameters.
 *
 * Fields are private; read access is provided through const getters.
 * The SOLE class permitted to mutate private fields after construction is
 * KinematicsParameterManager — via its table-driven apply-lambdas.  This
 * guarantees that every runtime change is validated, derived parameters
 * (wheel_radius_, max_wheel_speed_mps_, …) are kept consistent, and all
 * downstream subsystems (Solver, Estimator, HardwareInterface) are notified
 * automatically through the dirty-flag bitmask.
 *
 * Construction / initial load (one-time, before the node is live):
 *   KinematicsConfig cfg;                          // all built-in defaults
 *   auto cfg = KinematicsConfig::fromFile(path);   // load from YAML file
 *   auto cfg = KinematicsConfig::fromYaml(node);   // from parsed YAML node
 *
 * In-place reload (used by the reset_kinematics service in KinematicsNode):
 *   bool ok = cfg.loadFromFile(path);   // followed by param_manager_.applyAll()
 */
class KinematicsConfig {
public:
    /// Default-constructs a config with all built-in defaults.
    /// Member initialisers are the single source of truth for default values.
    KinematicsConfig() = default;

    // ── Factory methods ───────────────────────────────────────────────────────

    /// Returns a config with all built-in defaults.
    static KinematicsConfig withDefaults() { return KinematicsConfig{}; }

    /// Parses @p path and returns a fully-constructed config.
    /// Returns defaults silently if the file cannot be read or parsed.
    static KinematicsConfig fromFile(const std::string& path) {
        KinematicsConfig cfg;
        cfg.loadFromFile(path);   // error silently falls back to defaults
        return cfg;
    }

    /// Returns a config built from an already-parsed YAML node.
    /// Fields absent from the node retain their default values.
    static KinematicsConfig fromYaml(const YAML::Node& node) {
        KinematicsConfig cfg;
        cfg.loadFromYaml(node);
        return cfg;
    }

    // ── In-place reload ───────────────────────────────────────────────────────

    /// Resets to defaults, then loads from @p path.
    /// Returns true if the file parsed and the resulting config validated.
    bool loadFromFile(const std::string& path) {
        try {
            YAML::Node node = YAML::LoadFile(path);
            return loadFromYaml(node);
        } catch (const YAML::Exception&) {
            return false;
        }
    }

    // ── Validation & derived-parameter update ─────────────────────────────────

    bool validate() const {
        if (wheelbase_    <= 0.0) return false;
        if (track_width_  <= 0.0) return false;
        if (wheel_radius_ <= 0.0) return false;
        if (pose_covariance_diagonal_.size()  != 6) return false;
        if (twist_covariance_diagonal_.size() != 6) return false;
        if (publish_rate_         <= 0.0) return false;
        if (feedback_timeout_sec_ <= 0.0) return false;
        return true;
    }

    /// Recomputes wheel_radius_ and max_wheel_speed_mps_ from the primary
    /// geometry parameters.  Called automatically by KinematicsParameterManager
    /// whenever a kDerived-flagged parameter changes.  Do not call directly.
    void updateDerivedParameters() {
        wheel_radius_ = wheel_diameter_ / 2.0;
        const double max_mech_rpm =
            max_erpm_ / (static_cast<double>(poles_pairs_number_) * motor_gear_ratio_);
        const double max_rad_s = max_mech_rpm * (M_PI / 30.0);
        max_wheel_speed_mps_ = max_rad_s * wheel_radius_;
        feedback_timeout_ns_ = static_cast<int64_t>(feedback_timeout_sec_ * 1e9);
    }

    // ── Getters — Physical geometry ───────────────────────────────────────────
    double wheelbase()          const { return wheelbase_; }
    double track_width()        const { return track_width_; }
    double wheel_radius()       const { return wheel_radius_; }
    double wheel_diameter()     const { return wheel_diameter_; }

    int    poles_pairs_number() const { return poles_pairs_number_; }
    double motor_gear_ratio()   const { return motor_gear_ratio_; }
    double min_erpm()           const { return min_erpm_; }
    double max_erpm()           const { return max_erpm_; }
    double max_wheel_speed_mps()const { return max_wheel_speed_mps_; }

    // ── Getters — Steering geometry ───────────────────────────────────────────
    /// Per-wheel mechanical angle limits [FL=0, FR=1, RL=2, RR=3] (radians).
    double min_mechanical_angle(std::size_t wheel = 0) const { return min_mechanical_angle_[wheel]; }
    double max_mechanical_angle(std::size_t wheel = 0) const { return max_mechanical_angle_[wheel]; }
    const std::array<double, 4>& min_mechanical_angles() const { return min_mechanical_angle_; }
    const std::array<double, 4>& max_mechanical_angles() const { return max_mechanical_angle_; }
    double max_steering_radius()    const { return max_steering_radius_; }
    double min_steering_radius()    const { return min_steering_radius_; }
    double max_steering_angle_deg() const { return max_steering_angle_deg_; }
    double max_steering_angle_rad() const { return max_steering_angle_rad_; }

    // ── Getters — Motor polarity (per-wheel: FL=0, FR=1, RL=2, RR=3) ───────────────
    bool invert_drive(std::size_t wheel)    const { return invert_drive_[wheel]; }
    bool invert_steering(std::size_t wheel) const { return invert_steering_[wheel]; }
    const std::array<bool, 4>& invert_drive_arr()    const { return invert_drive_; }
    const std::array<bool, 4>& invert_steering_arr() const { return invert_steering_; }

    // ── Getters — Covariance ──────────────────────────────────────────────────
    const std::vector<double>& pose_covariance_diagonal()  const { return pose_covariance_diagonal_; }
    const std::vector<double>& twist_covariance_diagonal() const { return twist_covariance_diagonal_; }

    // ── Getters — Timing & publishing ─────────────────────────────────────────
    double publish_rate()         const { return publish_rate_; }
    double feedback_timeout_sec() const { return feedback_timeout_sec_; }
    int64_t feedback_timeout_ns() const { return feedback_timeout_ns_; }
    double max_allowed_dt()       const { return max_allowed_dt_; }

    // ── Getters — Frames, topics & flags ─────────────────────────────────────
    bool               use_sim_time()              const { return use_sim_time_; }
    const std::string& base_frame_id()             const { return base_frame_id_; }
    const std::string& odom_frame_id()             const { return odom_frame_id_; }
    bool               publish_tf()            const { return publish_tf_; }
    bool               use_measurement_timestamp() const { return use_measurement_timestamp_; }
    const std::string& cmd_vel_autonomy_topic()    const { return cmd_vel_autonomy_topic_; }
    double             cmd_vel_timeout_sec()       const { return cmd_vel_timeout_sec_; }
    double             initialization_timeout_sec()const { return initialization_timeout_sec_; }
    double             stop_timeout_sec()          const { return stop_timeout_sec_; }

    // ── Getters — Estimator tuning ────────────────────────────────────────────
    double huber_loss_threshold()            const { return huber_loss_threshold_; }
    double twist_ema_alpha()                 const { return twist_ema_alpha_; }
    double se2_integration_omega_threshold() const { return se2_integration_omega_threshold_; }
    double steering_angle_deadband_deg()     const { return steering_angle_deadband_deg_; }

    // ── Getters — Wheel quality ───────────────────────────────────────────────
    bool   enable_dynamic_wheel_weighting()       const { return enable_dynamic_wheel_weighting_; }
    double wheel_quality_low_erpm_threshold()     const { return wheel_quality_low_erpm_threshold_; }
    double wheel_quality_high_current_threshold() const { return wheel_quality_high_current_threshold_; }
    double wheel_quality_high_duty_threshold()    const { return wheel_quality_high_duty_threshold_; }
    double wheel_quality_min_weight()             const { return wheel_quality_min_weight_; }

    // ── Getters — Coherence safety ──────────────────────────────────────
    bool   enable_coherence_safety()          const { return enable_coherence_safety_; }
    double steering_coherence_threshold()     const { return steering_coherence_threshold_; }

    // ── Getters — Ceres solver ────────────────────────────────────────────────
    int    solver_max_num_iterations()           const { return solver_max_num_iterations_; }
    double solver_function_tolerance()           const { return solver_function_tolerance_; }
    double solver_gradient_tolerance()           const { return solver_gradient_tolerance_; }
    double solver_parameter_tolerance()          const { return solver_parameter_tolerance_; }
    bool   solver_minimizer_progress_to_stdout() const { return solver_minimizer_progress_to_stdout_; }
    ceres::LinearSolverType        solver_linear_solver_type()          const { return solver_linear_solver_type_; }
    ceres::TrustRegionStrategyType solver_trust_region_strategy_type()  const { return solver_trust_region_strategy_type_; }

    // ── Mutation authority ────────────────────────────────────────────────────
    /// KinematicsParameterManager is the SOLE class allowed to mutate private
    /// fields after construction.  All runtime parameter changes (ros2 param
    /// set / hot-reload) must go through its table-driven apply-lambdas so
    /// that validation, derived-parameter updates, and subsystem notifications
    /// are always applied together and never forgotten.
    friend class KinematicsParameterManager;

private:
    // ── YAML loading ─────────────────────────────────────────────────────────
    /// Resets to defaults, then overwrites fields present in @p config.
    /// Returns validate() after loading; called by loadFromFile and fromYaml.
    bool loadFromYaml(const YAML::Node& config) {
        if (!config) return false;
        *this = KinematicsConfig{};   // reset — member initialisers are the sole defaults

        tryLoad(config, "wheels_distance_length", wheelbase_);
        tryLoad(config, "wheels_distance_width",  track_width_);

        // wheel_diameter drives wheel_radius — update both together
        if (config["wheel_diameter"]) {
            wheel_diameter_ = config["wheel_diameter"].as<double>();
            wheel_radius_   = wheel_diameter_ / 2.0;
        }

        tryLoad(config, "poles_pair_number",  poles_pairs_number_);
        tryLoad(config, "motor_gear_ratio",   motor_gear_ratio_);
        tryLoad(config, "min_erpm",           min_erpm_);
        tryLoad(config, "max_erpm",           max_erpm_);

        // Per-wheel mechanical angle limits [fl=0, fr=1, rl=2, rr=3].
        // Defaults (±1.57 rad) are set in the field initialiser.
        tryLoad(config, "min_mechanical_angle_fl", min_mechanical_angle_[0]);
        tryLoad(config, "min_mechanical_angle_fr", min_mechanical_angle_[1]);
        tryLoad(config, "min_mechanical_angle_rl", min_mechanical_angle_[2]);
        tryLoad(config, "min_mechanical_angle_rr", min_mechanical_angle_[3]);

        tryLoad(config, "max_mechanical_angle_fl", max_mechanical_angle_[0]);
        tryLoad(config, "max_mechanical_angle_fr", max_mechanical_angle_[1]);
        tryLoad(config, "max_mechanical_angle_rl", max_mechanical_angle_[2]);
        tryLoad(config, "max_mechanical_angle_rr", max_mechanical_angle_[3]);
        
        tryLoad(config, "max_steering_radius",  max_steering_radius_);
        tryLoad(config, "min_steering_radius",  min_steering_radius_);

        // Per-wheel polarity — [FL=0, FR=1, RL=2, RR=3]
        // drive defaults: all false  |  steering defaults: all true
        tryLoad(config, "invert_drive_fl",    invert_drive_[0]);
        tryLoad(config, "invert_drive_fr",    invert_drive_[1]);
        tryLoad(config, "invert_drive_rl",    invert_drive_[2]);
        tryLoad(config, "invert_drive_rr",    invert_drive_[3]);
        tryLoad(config, "invert_steering_fl", invert_steering_[0]);
        tryLoad(config, "invert_steering_fr", invert_steering_[1]);
        tryLoad(config, "invert_steering_rl", invert_steering_[2]);
        tryLoad(config, "invert_steering_rr", invert_steering_[3]);

        tryLoad(config, "pose_covariance_diagonal",  pose_covariance_diagonal_);
        tryLoad(config, "twist_covariance_diagonal", twist_covariance_diagonal_);

        tryLoad(config, "publish_rate",             publish_rate_);
        tryLoad(config, "feedback_timeout_sec",     feedback_timeout_sec_);
        tryLoad(config, "cmd_vel_timeout_sec",      cmd_vel_timeout_sec_);
        tryLoad(config, "initialization_timeout_sec", initialization_timeout_sec_);
        tryLoad(config, "stop_timeout_sec",         stop_timeout_sec_);
        tryLoad(config, "use_sim_time",             use_sim_time_);
        tryLoad(config, "base_frame_id",            base_frame_id_);
        tryLoad(config, "odom_frame_id",            odom_frame_id_);
        tryLoad(config, "publish_tf",           publish_tf_);
        tryLoad(config, "use_measurement_timestamp", use_measurement_timestamp_);

        tryLoad(config, "huber_loss_threshold",            huber_loss_threshold_);
        tryLoad(config, "twist_ema_alpha",                 twist_ema_alpha_);
        tryLoad(config, "se2_integration_omega_threshold", se2_integration_omega_threshold_);

        tryLoad(config, "solver_max_num_iterations",           solver_max_num_iterations_);
        tryLoad(config, "solver_function_tolerance",           solver_function_tolerance_);
        tryLoad(config, "solver_gradient_tolerance",           solver_gradient_tolerance_);
        tryLoad(config, "solver_parameter_tolerance",          solver_parameter_tolerance_);
        tryLoad(config, "solver_minimizer_progress_to_stdout", solver_minimizer_progress_to_stdout_);

        tryLoad(config, "enable_dynamic_wheel_weighting",       enable_dynamic_wheel_weighting_);
        tryLoad(config, "wheel_quality_low_erpm_threshold",     wheel_quality_low_erpm_threshold_);
        tryLoad(config, "wheel_quality_high_current_threshold", wheel_quality_high_current_threshold_);
        tryLoad(config, "wheel_quality_high_duty_threshold",    wheel_quality_high_duty_threshold_);
        tryLoad(config, "wheel_quality_min_weight",             wheel_quality_min_weight_);

        tryLoad(config, "enable_coherence_safety",        enable_coherence_safety_);
        tryLoad(config, "steering_coherence_threshold",   steering_coherence_threshold_);

        // Canonical key wins; trailing-underscore key is a legacy fallback only.
        if (config["cmd_vel_autonomy_topic"])
            cmd_vel_autonomy_topic_ = config["cmd_vel_autonomy_topic"].as<std::string>();
        else if (config["cmd_vel_autonomy_topic_"])
            cmd_vel_autonomy_topic_ = config["cmd_vel_autonomy_topic_"].as<std::string>();

        if (config["solver_linear_solver_type"])
            solver_linear_solver_type_ = parseLinearSolverType(
                config["solver_linear_solver_type"].as<std::string>());
        
        if (config["solver_trust_region_strategy_type"])
            solver_trust_region_strategy_type_ = parseTrustRegionStrategyType(
                config["solver_trust_region_strategy_type"].as<std::string>());

        updateDerivedParameters();
        return validate();
    }

    // ── YAML parsing helpers ──────────────────────────────────────────────────

    /// Reads @p key from @p cfg into @p out if the key is present;
    /// leaves @p out unchanged otherwise.
    template<typename T>
    static void tryLoad(const YAML::Node& cfg, const char* key, T& out) {
        if (cfg[key]) out = cfg[key].as<T>();
    }

    static ceres::LinearSolverType parseLinearSolverType(const std::string& value) {
        if (value == "DENSE_QR")              return ceres::DENSE_QR;
        if (value == "DENSE_NORMAL_CHOLESKY") return ceres::DENSE_NORMAL_CHOLESKY;
        if (value == "SPARSE_NORMAL_CHOLESKY")return ceres::SPARSE_NORMAL_CHOLESKY;
        if (value == "ITERATIVE_SCHUR")       return ceres::ITERATIVE_SCHUR;
        return ceres::DENSE_QR;
    }

    static ceres::TrustRegionStrategyType parseTrustRegionStrategyType(const std::string& value) {
        if (value == "LEVENBERG_MARQUARDT") return ceres::LEVENBERG_MARQUARDT;
        if (value == "DOGLEG")              return ceres::DOGLEG;
        return ceres::DOGLEG;
    }

    // ── Fields — Physical geometry ────────────────────────────────────────────
    double wheelbase_       {1.0};
    double track_width_     {1.0};
    double wheel_radius_    {0.125};
    double wheel_diameter_  {0.25};

    int    poles_pairs_number_ {15};
    double motor_gear_ratio_   {1.0};
    double min_erpm_           {0.0000001};
    double max_erpm_           {1000000.0};
    double max_wheel_speed_mps_{0.0};

    // ── Fields — Steering geometry ────────────────────────────────────────────
    // Per-wheel arrays: index [FL=0, FR=1, RL=2, RR=3]
    std::array<double, 4> min_mechanical_angle_ {-1.57, -1.57, -1.57, -1.57};
    std::array<double, 4> max_mechanical_angle_ { 1.57,  1.57,  1.57,  1.57};
    double max_steering_radius_    {5.0};
    double min_steering_radius_    {1.0};
    double max_steering_angle_deg_ {90.0};
    double max_steering_angle_rad_ {1.57};

    // ── Fields — Motor polarity (per-wheel arrays: FL=0, FR=1, RL=2, RR=3) ───────────
    std::array<bool, 4> invert_drive_    {false, false, false, false};
    std::array<bool, 4> invert_steering_ {true,  true,  true,  true};

    // ── Fields — Covariance ───────────────────────────────────────────────────
    std::vector<double> pose_covariance_diagonal_  {0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
    std::vector<double> twist_covariance_diagonal_ {0.001, 0.001, 0.001, 0.001, 0.001, 0.001};

    // ── Fields — Timing & publishing ─────────────────────────────────────────
    double publish_rate_         {10.0};
    double feedback_timeout_sec_ {0.5};
    int64_t feedback_timeout_ns_ {static_cast<int64_t>(feedback_timeout_sec_ * 1e9)};
    double max_allowed_dt_       {0.5};

    // ── Fields — Frames, topics & flags ──────────────────────────────────────
    bool        use_sim_time_              {false};
    std::string base_frame_id_             {"/base_footprint"};
    std::string odom_frame_id_             {"/odom"};
    bool        publish_tf_            {false};
    bool        use_measurement_timestamp_ {false};
    std::string cmd_vel_autonomy_topic_    {"/cmd_vel"};
    double      cmd_vel_timeout_sec_        {0.5};
    double      initialization_timeout_sec_ {5.0};
    double      stop_timeout_sec_           {5.0};

    // ── Fields — Estimator tuning ─────────────────────────────────────────────
    double steering_angle_deadband_deg_     {0.5};
    double huber_loss_threshold_            {0.5};
    double se2_integration_omega_threshold_ {1e-4};
    double twist_ema_alpha_                 {0.8};

    // ── Fields — Wheel quality ────────────────────────────────────────────────
    bool   enable_dynamic_wheel_weighting_       {false};
    double wheel_quality_low_erpm_threshold_     {500.0};
    double wheel_quality_high_current_threshold_ {20.0};
    double wheel_quality_high_duty_threshold_    {0.85};
    double wheel_quality_min_weight_             {0.0};

    // ── Fields — Coherence safety ────────────────────────────────────────
    bool   enable_coherence_safety_        {true};
    double steering_coherence_threshold_   {0.15};

    // ── Fields — Ceres solver ─────────────────────────────────────────────────
    int    solver_max_num_iterations_          {200};
    double solver_function_tolerance_          {1e-8};
    double solver_gradient_tolerance_          {1e-10};
    double solver_parameter_tolerance_         {1e-9};
    bool   solver_minimizer_progress_to_stdout_{false};
    ceres::LinearSolverType        solver_linear_solver_type_         {ceres::DENSE_QR};
    ceres::TrustRegionStrategyType solver_trust_region_strategy_type_ {ceres::DOGLEG};
};
