#pragma once
#include <QObject>
#include <QString>
#include <vector>
#include <string>
#include <memory>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include "simulator/robot.hpp"

class SimulatorNode;

// ---------------------------------------------------------------------------
// Configuration structs
// ---------------------------------------------------------------------------
struct ParamDef {
    std::string node_name;   // ROS2 node name to set param on
    std::string param_name;  // parameter name
    double start = 0.0;
    double end   = 1.0;
    double step  = 0.1;
};

struct Waypoint { double x, y; };

struct ScenarioConfig {
    double timeout_sec  = 30.0;  // trial duration
    double settle_sec   = 0.5;   // wait after param apply before measuring

    // Controller plugin name passed to the FollowPath action server
    std::string controller_id = "FollowPath";

    // Reference path for cross-track error evaluation.
    // Empty → tracking_error fields = -1 (N/A).
    std::vector<Waypoint> reference_path;
};

struct TrialResult {
    int    index        = 0;
    std::vector<double> param_values;
    double elapsed_sec      = 0.0;
    double path_length      = 0.0;
    double smoothness       = 0.0;  // RMS(Δv) — lower = smoother
    double tracking_err_rms = -1.0; // cross-track RMS (m), -1 = N/A
    double tracking_err_max = -1.0; // max cross-track error (m), -1 = N/A
};

// ---------------------------------------------------------------------------
// SweepRunner
//   Drives automated parameter grid-search trials.
//   tick() must be called from MainWindow::onPhysicsUpdate() at ~50 Hz.
// ---------------------------------------------------------------------------
class SweepRunner : public QObject {
    Q_OBJECT
public:
    explicit SweepRunner(std::shared_ptr<SimulatorNode> node,
                         QObject* parent = nullptr);

    void setConfig(const std::vector<ParamDef>&  params,
                   const ScenarioConfig&          scenario);
    void start();
    void stop();

    bool isRunning()   const { return state_ != State::IDLE; }
    int  totalTrials() const { return static_cast<int>(combos_.size()); }
    int  currentTrial()const { return trial_index_; }

    const std::vector<ParamDef>&    paramDefs() const { return params_; }
    const std::vector<TrialResult>& results()   const { return results_; }

    // real_dt: actual wall-clock delta (for ROS message-propagation waits)
    // sim_dt:  real_dt * sim_speed  (for settle/trial timing)
    void tick(double real_dt, double sim_dt, const RobotState& robot);

signals:
    void trialFinished(TrialResult result);
    void sweepFinished();
    void progressChanged(int current, int total);
    void statusMessage(const QString& msg);

private:
    void applyCombo(int index);
    void beginTrial(const RobotState& robot);
    void finalizeTrial(const RobotState& robot);

    std::shared_ptr<SimulatorNode>   node_;
    rclcpp::Node::SharedPtr          param_client_node_;
    std::vector<ParamDef>            params_;
    ScenarioConfig                   scenario_;
    std::vector<std::vector<double>> combos_;
    std::vector<TrialResult>         results_;

    // Starting pose captured when sweep begins (reset robot here each trial)
    double start_x_     = 0.0;
    double start_y_     = 0.0;
    double start_theta_ = 0.0;

    enum class State { IDLE, RESETTING, SETTLING, RUNNING };
    State state_       = State::IDLE;
    int   trial_index_ = 0;

    // Per-trial accumulators
    double reset_elapsed_  = 0.0;  // time waited after pose reset before sending goal
    double settle_elapsed_ = 0.0;
    double trial_elapsed_  = 0.0;
    double path_length_    = 0.0;
    double prev_x_ = 0.0, prev_y_ = 0.0;
    double prev_vx_ = 0.0, prev_vtheta_ = 0.0;
    double smooth_sum_     = 0.0;
    int    smooth_n_       = 0;
    double track_err_sum_  = 0.0;
    double track_err_max_  = 0.0;
    int    track_err_n_    = 0;
};
