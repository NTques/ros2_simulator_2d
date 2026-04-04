#include "simulator/sweep_runner.hpp"
#include "simulator/simulator_node.hpp"
#include <rclcpp/parameter_client.hpp>
#include <cmath>
#include <map>
#include <algorithm>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static double sqDistToSegment(double px, double py,
                               double ax, double ay,
                               double bx, double by)
{
    double dx = bx - ax, dy = by - ay;
    double len2 = dx*dx + dy*dy;
    if (len2 < 1e-12) {
        double ex = px-ax, ey = py-ay;
        return ex*ex + ey*ey;
    }
    double t = std::clamp(((px-ax)*dx + (py-ay)*dy) / len2, 0.0, 1.0);
    double cx = ax + t*dx - px;
    double cy = ay + t*dy - py;
    return cx*cx + cy*cy;
}

static double distToPath(double px, double py,
                          const std::vector<Waypoint>& path)
{
    if (path.empty()) return 0.0;
    if (path.size() == 1) {
        double ex = px - path[0].x, ey = py - path[0].y;
        return std::sqrt(ex*ex + ey*ey);
    }
    double min_sq = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        double sq = sqDistToSegment(px, py,
                                    path[i].x,   path[i].y,
                                    path[i+1].x, path[i+1].y);
        if (sq < min_sq) min_sq = sq;
    }
    return std::sqrt(min_sq);
}

static std::vector<std::vector<double>> cartesianProduct(
    const std::vector<std::vector<double>>& vecs)
{
    std::vector<std::vector<double>> result = {{}};
    for (const auto& v : vecs) {
        std::vector<std::vector<double>> tmp;
        tmp.reserve(result.size() * v.size());
        for (const auto& combo : result)
            for (double val : v) {
                tmp.push_back(combo);
                tmp.back().push_back(val);
            }
        result = std::move(tmp);
    }
    return result;
}

// ---------------------------------------------------------------------------
SweepRunner::SweepRunner(std::shared_ptr<SimulatorNode> node, QObject* parent)
: QObject(parent), node_(std::move(node))
{
    param_client_node_ = rclcpp::Node::make_shared(
        "__sweep_param_client__",
        rclcpp::NodeOptions()
            .start_parameter_services(false)
            .start_parameter_event_publisher(false));
}

void SweepRunner::setConfig(const std::vector<ParamDef>& params,
                             const ScenarioConfig& scenario)
{
    params_   = params;
    scenario_ = scenario;
}

// ---------------------------------------------------------------------------
void SweepRunner::start() {
    if (params_.empty()) {
        emit statusMessage("No parameters configured.");
        return;
    }

    // Pre-flight checks — give clear feedback before blocking on param services
    if (!node_->hasCachedPath()) {
        emit statusMessage("ERROR: No path set. Draw a path on the map first.");
        return;
    }
    if (!node_->isFollowPathReady()) {
        emit statusMessage(
            "ERROR: follow_path action server not ready. "
            "Is controller_server running? (ros2 launch simulator simulator_nav2.launch.py)");
        return;
    }

    // Capture current robot pose as the trial start pose
    const auto rs = node_->getRobotState();
    start_x_     = rs.x;
    start_y_     = rs.y;
    start_theta_ = rs.theta;

    // Build value list for each parameter
    std::vector<std::vector<double>> value_lists;
    for (const auto& p : params_) {
        std::vector<double> vals;
        double step = std::max(p.step, 1e-9);
        for (double v = p.start; v <= p.end + step * 1e-6; v += step) {
            double rounded = std::round(v / step) * step;
            vals.push_back(rounded);
        }
        if (vals.empty()) vals.push_back(p.start);
        value_lists.push_back(vals);
    }

    combos_ = cartesianProduct(value_lists);
    results_.clear();
    trial_index_ = 0;

    emit progressChanged(0, static_cast<int>(combos_.size()));
    emit statusMessage(
        QString("Sweep started: %1 trial(s)").arg(combos_.size()));

    applyCombo(0);
}

void SweepRunner::stop() {
    state_       = State::IDLE;
    trial_index_ = 0;
    emit statusMessage("Sweep stopped by user.");
}

// ---------------------------------------------------------------------------
void SweepRunner::applyCombo(int index) {
    const auto& combo = combos_[index];

    std::map<std::string, std::vector<rclcpp::Parameter>> by_node;
    for (size_t i = 0; i < params_.size(); ++i)
        by_node[params_[i].node_name].emplace_back(
            params_[i].param_name, combo[i]);

    for (auto& [node_name, rclparams] : by_node) {
        auto client = std::make_shared<rclcpp::SyncParametersClient>(
            param_client_node_, node_name);

        if (client->wait_for_service(std::chrono::milliseconds(500))) {
            auto set_results = client->set_parameters(rclparams);
            for (size_t i = 0; i < set_results.size(); ++i) {
                if (!set_results[i].successful) {
                    RCLCPP_WARN(node_->get_logger(),
                        "[sweep] Failed to set %s on %s: %s",
                        rclparams[i].get_name().c_str(),
                        node_name.c_str(),
                        set_results[i].reason.c_str());
                }
            }
        } else {
            RCLCPP_WARN(node_->get_logger(),
                "[sweep] Parameter service not available: %s", node_name.c_str());
        }
    }

    // Reset robot pose and cancel any active goal.
    // sendFollowPath is deferred until RESETTING elapses so that the controller
    // has time to process the cancel and the odometry settles at zero velocity
    // before it receives a new goal (prevents NaN from instantaneous pose jump).
    node_->setRobotPose(start_x_, start_y_, start_theta_);
    node_->cancelFollowPath();

    emit statusMessage(
        QString("Trial %1/%2 — applying params, resetting pose...")
        .arg(index + 1).arg(combos_.size()));

    state_         = State::RESETTING;
    reset_elapsed_ = 0.0;
}

// ---------------------------------------------------------------------------
void SweepRunner::beginTrial(const RobotState& robot) {
    // Discard any done flag that arrived during RESETTING/SETTLING
    // (e.g. a legitimate early abort before the trial even started).
    node_->takeFollowPathDone();

    trial_elapsed_ = 0.0;
    path_length_   = 0.0;
    prev_x_        = robot.x;
    prev_y_        = robot.y;
    prev_vx_       = robot.cmd_vx;
    prev_vtheta_   = robot.cmd_vtheta;
    smooth_sum_    = 0.0;
    smooth_n_      = 0;
    track_err_sum_ = 0.0;
    track_err_max_ = 0.0;
    track_err_n_   = 0;
    state_         = State::RUNNING;

    QString info;
    const auto& combo = combos_[trial_index_];
    for (size_t i = 0; i < params_.size(); ++i) {
        info += QString("%1=%2")
            .arg(QString::fromStdString(params_[i].param_name))
            .arg(combo[i]);
        if (i + 1 < params_.size()) info += "  ";
    }
    emit statusMessage(QString("Trial %1/%2 — %3")
        .arg(trial_index_ + 1).arg(combos_.size()).arg(info));
}

// ---------------------------------------------------------------------------
void SweepRunner::finalizeTrial(const RobotState& robot) {
    (void)robot;

    TrialResult res;
    res.index        = trial_index_;
    res.param_values = combos_[trial_index_];
    res.elapsed_sec  = trial_elapsed_;
    res.path_length  = path_length_;
    res.smoothness   = (smooth_n_ > 0)
                       ? std::sqrt(smooth_sum_ / smooth_n_) : 0.0;

    if (!scenario_.reference_path.empty() && track_err_n_ > 0) {
        res.tracking_err_rms = std::sqrt(track_err_sum_ / track_err_n_);
        res.tracking_err_max = track_err_max_;
    }

    results_.push_back(res);
    emit trialFinished(res);

    ++trial_index_;
    emit progressChanged(trial_index_, static_cast<int>(combos_.size()));

    if (trial_index_ >= static_cast<int>(combos_.size())) {
        state_ = State::IDLE;
        emit sweepFinished();
    } else {
        applyCombo(trial_index_);
    }
}

// ---------------------------------------------------------------------------
void SweepRunner::tick(double real_dt, double sim_dt, const RobotState& robot) {
    switch (state_) {
        case State::IDLE:
            return;

        case State::RESETTING:
            // Use real_dt: waiting for ROS cancel/TF propagation, not sim time.
            reset_elapsed_ += real_dt;
            if (reset_elapsed_ >= 0.30) {
                node_->sendFollowPath(scenario_.controller_id);
                emit statusMessage(
                    QString("Trial %1/%2 — path sent, settling...")
                    .arg(trial_index_ + 1).arg(combos_.size()));
                state_          = State::SETTLING;
                settle_elapsed_ = 0.0;
            }
            return;

        case State::SETTLING:
            // Use sim_dt so settle time scales with simulation speed.
            settle_elapsed_ += sim_dt;
            if (settle_elapsed_ >= scenario_.settle_sec)
                beginTrial(robot);
            return;

        case State::RUNNING: {
            // Use sim_dt so timeout and all metrics are in simulation time.
            trial_elapsed_ += sim_dt;

            // Accumulate path length
            double dx = robot.x - prev_x_, dy = robot.y - prev_y_;
            path_length_ += std::hypot(dx, dy);
            prev_x_ = robot.x;
            prev_y_ = robot.y;

            // Smoothness: RMS of velocity increments
            double dvx = robot.cmd_vx     - prev_vx_;
            double dvo = robot.cmd_vtheta - prev_vtheta_;
            smooth_sum_ += dvx * dvx + dvo * dvo;
            ++smooth_n_;
            prev_vx_     = robot.cmd_vx;
            prev_vtheta_ = robot.cmd_vtheta;

            // Cross-track error against reference path
            if (!scenario_.reference_path.empty()) {
                double e = distToPath(robot.x, robot.y, scenario_.reference_path);
                track_err_sum_ += e * e;
                if (e > track_err_max_) track_err_max_ = e;
                ++track_err_n_;
            }

            // End trial when controller finishes (success or abort) or timeout
            if (node_->takeFollowPathDone() ||
                trial_elapsed_ >= scenario_.timeout_sec)
                finalizeTrial(robot);
            return;
        }
    }
}
