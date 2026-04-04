#include "simulator/simulator_node.hpp"
#include "simulator/robot.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

// ---------------------------------------------------------------------------
SimulatorNode::SimulatorNode()
: Node("simulator",
       rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
{
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::QoS(10),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            // Drop stale commands that arrived after a pose reset
            const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            if (now_ns < cmd_vel_ignore_until_ns_.load()) return;

            std::lock_guard<std::mutex> lk(mutex_);
            robot_.cmd_vx     = msg->linear.x;
            robot_.cmd_vtheta = msg->angular.z;
        });

    // /clock must use QoS with high durability so late-joining use_sim_time
    // nodes receive the latest time immediately.
    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::QoS(1).best_effort());
    odom_pub_  = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    goal_pub_  = create_publisher<geometry_msgs::msg::PoseStamped>(
                     "goal_pose", rclcpp::QoS(1));
    path_pub_  = create_publisher<nav_msgs::msg::Path>(
                     "path", rclcpp::QoS(1).transient_local());
    map_pub_   = create_publisher<nav_msgs::msg::OccupancyGrid>(
                         "map", rclcpp::QoS(1).transient_local());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Action client for sending paths directly to the controller server
    follow_path_client_ = rclcpp_action::create_client<FollowPath>(
        this, "follow_path");

    // Publish map every 3 seconds (fires in the ROS spin thread)
    map_timer_ = create_wall_timer(std::chrono::seconds(3), [this]() {
        nav_msgs::msg::OccupancyGrid grid;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            if (!has_map_) return;
            grid = cached_grid_;
        }
        grid.header.stamp = get_clock()->now();
        map_pub_->publish(grid);
    });
}

SimulatorNode::~SimulatorNode() {
    running_ = false;
    if (ros_thread_.joinable()) ros_thread_.join();
}

void SimulatorNode::start() {
    // Parse footprint parameter if present (nav2 format: "[ [x,y], ... ]")
    if (has_parameter("footprint")) {
        try {
            const auto fp_str = get_parameter("footprint").as_string();
            auto fp = parseFootprintString(fp_str);
            if (!fp.empty()) {
                std::lock_guard<std::mutex> lk(mutex_);
                robot_.footprint = fp;
                robot_.radius    = footprintRadius(fp);
                RCLCPP_INFO(get_logger(),
                    "[simulator] Footprint loaded: %zu vertices, circumscribed r=%.3f m",
                    fp.size(), robot_.radius);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(),
                "[simulator] footprint parameter parse failed: %s", e.what());
        }
    }

    // Publish initial sim clock (t=0) immediately so that use_sim_time nodes
    // do not block waiting for the first clock message.
    {
        rosgraph_msgs::msg::Clock clock_msg;
        clock_msg.clock.sec = 0;
        clock_msg.clock.nanosec = 0;
        clock_pub_->publish(clock_msg);
    }

    // Publish initial odom + TF so the costmap does not time out.
    publishState();

    running_ = true;
    ros_thread_ = std::thread([this]() {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(shared_from_this());
        while (running_ && rclcpp::ok()) {
            exec.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

// ---------------------------------------------------------------------------
// Thread-safe setters / getters
// ---------------------------------------------------------------------------
void SimulatorNode::setMap(MapData map) {
    std::lock_guard<std::mutex> lk(mutex_);
    map_     = std::move(map);
    has_map_ = true;

    // Pre-build OccupancyGrid (expensive conversion done once here)
    auto& m = map_;
    cached_grid_ = nav_msgs::msg::OccupancyGrid{};
    cached_grid_.header.frame_id         = "map";
    cached_grid_.info.resolution         = static_cast<float>(m.resolution);
    cached_grid_.info.width              = static_cast<uint32_t>(m.width);
    cached_grid_.info.height             = static_cast<uint32_t>(m.height);
    cached_grid_.info.origin.position.x  = m.origin_x;
    cached_grid_.info.origin.position.y  = m.origin_y;
    cached_grid_.info.origin.orientation.w = 1.0;

    // OccupancyGrid row 0 = world origin_y (bottom); MapData row 0 = top → flip
    const int w = m.width, h = m.height;
    cached_grid_.data.resize(static_cast<size_t>(w * h));
    for (int grid_row = 0; grid_row < h; ++grid_row) {
        int map_row = h - 1 - grid_row;
        for (int col = 0; col < w; ++col) {
            cached_grid_.data[static_cast<size_t>(grid_row * w + col)] =
                m.occupied[static_cast<size_t>(map_row * w + col)] ? 100 : 0;
        }
    }
}

void SimulatorNode::setRobotRadius(double radius) {
    std::lock_guard<std::mutex> lk(mutex_);
    robot_.radius = radius;
}

void SimulatorNode::setRobotFootprint(
    const std::vector<std::pair<double,double>>& fp)
{
    std::lock_guard<std::mutex> lk(mutex_);
    robot_.footprint = fp;
    if (!fp.empty()) robot_.radius = footprintRadius(fp);
}

void SimulatorNode::setRobotPose(double x, double y, double theta) {
    // Ignore cmd_vel for 400 ms after teleport so that in-flight controller
    // commands computed for the old pose do not move the robot from the new pose.
    const int64_t until_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        (std::chrono::steady_clock::now() + std::chrono::milliseconds(400))
        .time_since_epoch()).count();
    cmd_vel_ignore_until_ns_.store(until_ns);

    std::lock_guard<std::mutex> lk(mutex_);
    robot_.x          = x;
    robot_.y          = y;
    robot_.theta      = theta;
    robot_.cmd_vx     = 0.0;
    robot_.cmd_vtheta = 0.0;
}

int SimulatorNode::addObstacle(const Obstacle& obs) {
    std::lock_guard<std::mutex> lk(mutex_);
    return obstacle_mgr_.addObstacle(obs);
}

void SimulatorNode::removeObstacle(int id) {
    std::lock_guard<std::mutex> lk(mutex_);
    obstacle_mgr_.removeObstacle(id);
}

void SimulatorNode::updateObstacle(const Obstacle& obs) {
    std::lock_guard<std::mutex> lk(mutex_);
    obstacle_mgr_.updateObstacle(obs);
}

MapData SimulatorNode::getMap() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return map_;
}

RobotState SimulatorNode::getRobotState() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return robot_;
}

std::vector<Obstacle> SimulatorNode::getObstacles() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return obstacle_mgr_.getAll();
}

// ---------------------------------------------------------------------------
// Path: user-drawn waypoints → interpolated nav_msgs/Path
// ---------------------------------------------------------------------------
static nav_msgs::msg::Path buildInterpolatedPath(
    const std::vector<std::pair<double,double>>& wps,
    double resolution)
{
    nav_msgs::msg::Path msg;
    msg.header.frame_id = "map";
    if (wps.size() < 2) {
        if (wps.size() == 1) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header.frame_id = "map";
            ps.pose.position.x = wps[0].first;
            ps.pose.position.y = wps[0].second;
            ps.pose.orientation.w = 1.0;
            msg.poses.push_back(ps);
        }
        return msg;
    }
    for (size_t i = 0; i + 1 < wps.size(); ++i) {
        double dx  = wps[i+1].first  - wps[i].first;
        double dy  = wps[i+1].second - wps[i].second;
        double len = std::hypot(dx, dy);
        if (len < 1e-6) continue;

        double yaw = std::atan2(dy, dx);
        tf2::Quaternion q; q.setRPY(0.0, 0.0, yaw);
        auto qmsg = tf2::toMsg(q);

        // number of points to insert for this segment (spacing = resolution)
        int n = std::max(1, static_cast<int>(std::ceil(len / resolution)));
        for (int j = 0; j < n; ++j) {
            double t = static_cast<double>(j) / n;
            geometry_msgs::msg::PoseStamped ps;
            ps.header.frame_id    = "map";
            ps.pose.position.x    = wps[i].first  + t * dx;
            ps.pose.position.y    = wps[i].second + t * dy;
            ps.pose.orientation   = qmsg;
            msg.poses.push_back(ps);
        }
    }
    // Always include the final waypoint
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id  = "map";
        ps.pose.position.x  = wps.back().first;
        ps.pose.position.y  = wps.back().second;
        ps.pose.orientation.w = 1.0;
        msg.poses.push_back(ps);
    }
    return msg;
}

void SimulatorNode::setPath(const std::vector<std::pair<double,double>>& waypoints,
                             double resolution)
{
    nav_msgs::msg::Path path = buildInterpolatedPath(waypoints, resolution);
    path.header.stamp = get_clock()->now();
    {
        std::lock_guard<std::mutex> lk(mutex_);
        path_waypoints_raw_ = waypoints;
        cached_path_        = path;
    }
    path_pub_->publish(path);
    RCLCPP_INFO(get_logger(), "[simulator] Path set: %zu waypoints → %zu poses (res=%.3f m)",
                waypoints.size(), path.poses.size(), resolution);
}

void SimulatorNode::clearPath() {
    {
        std::lock_guard<std::mutex> lk(mutex_);
        path_waypoints_raw_.clear();
        cached_path_ = nav_msgs::msg::Path{};
        cached_path_.header.frame_id = "map";
    }
    nav_msgs::msg::Path empty;
    empty.header.frame_id = "map";
    empty.header.stamp    = get_clock()->now();
    path_pub_->publish(empty);
}

std::vector<std::pair<double,double>> SimulatorNode::getPathWaypoints() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return path_waypoints_raw_;
}

void SimulatorNode::publishGoal(double x, double y, double theta) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp    = get_clock()->now();
    msg.header.frame_id = "map";
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    msg.pose.orientation = tf2::toMsg(q);
    goal_pub_->publish(msg);
}

bool SimulatorNode::hasMap() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return has_map_;
}

// ---------------------------------------------------------------------------
// Send the currently cached path directly to the controller server
// (bypasses bt_navigator / planner entirely)
// ---------------------------------------------------------------------------
void SimulatorNode::sendFollowPath(const std::string& controller_id) {
    nav_msgs::msg::Path path;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        path = cached_path_;
    }

    if (path.poses.empty()) {
        RCLCPP_WARN(get_logger(), "[simulator] sendFollowPath: no path set, ignoring.");
        return;
    }

    if (!follow_path_client_->action_server_is_ready()) {
        RCLCPP_WARN(get_logger(), "[simulator] follow_path action server not ready, skipping.");
        return;
    }

    follow_path_done_.store(false);

    // Cancel any in-flight goal first
    cancelFollowPath();

    path.header.stamp = get_clock()->now();

    FollowPath::Goal goal;
    goal.path          = path;
    goal.controller_id = controller_id;
    goal.goal_checker_id = "general_goal_checker";

    auto send_options = rclcpp_action::Client<FollowPath>::SendGoalOptions{};
    send_options.goal_response_callback =
        [this](GoalHandleFP::SharedPtr handle) {
            if (!handle) {
                RCLCPP_WARN(get_logger(), "[simulator] FollowPath goal rejected.");
                current_goal_handle_ = nullptr;
            } else {
                current_goal_handle_ = handle;
                RCLCPP_INFO(get_logger(), "[simulator] FollowPath goal accepted.");
            }
        };
    send_options.result_callback =
        [this](const GoalHandleFP::WrappedResult& result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(get_logger(), "[simulator] FollowPath succeeded.");
            } else {
                RCLCPP_INFO(get_logger(), "[simulator] FollowPath finished (code=%d).",
                            static_cast<int>(result.code));
            }
            current_goal_handle_ = nullptr;
            // Do not signal done for CANCELED — that result comes from our own
            // cancelFollowPath() call and must not be mistaken for the current
            // trial finishing.
            if (result.code != rclcpp_action::ResultCode::CANCELED)
                follow_path_done_.store(true);
        };

    follow_path_client_->async_send_goal(goal, send_options);
}

void SimulatorNode::cancelFollowPath() {
    if (current_goal_handle_) {
        follow_path_client_->async_cancel_goal(current_goal_handle_);
        current_goal_handle_ = nullptr;
    }
}

bool SimulatorNode::isFollowPathReady() const {
    return follow_path_client_ && follow_path_client_->action_server_is_ready();
}

bool SimulatorNode::hasCachedPath() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return !cached_path_.poses.empty();
}

bool SimulatorNode::takeFollowPathDone() {
    return follow_path_done_.exchange(false);
}

// ---------------------------------------------------------------------------
// Physics step — called from Qt timer (main thread)
// sim_dt = real_dt * sim_speed_multiplier
// ---------------------------------------------------------------------------
rclcpp::Time SimulatorNode::simNow() const {
    return rclcpp::Time(sim_time_ns_.load(), RCL_ROS_TIME);
}

void SimulatorNode::physicsStep(double sim_dt) {
    // Advance simulation clock and publish /clock so that use_sim_time nodes
    // (controller_server, local_costmap) run at the correct scaled rate.
    const int64_t dt_ns = static_cast<int64_t>(sim_dt * 1e9);
    const int64_t t_ns  = sim_time_ns_.fetch_add(dt_ns) + dt_ns;

    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock.sec    = static_cast<int32_t>(t_ns / 1'000'000'000LL);
    clock_msg.clock.nanosec = static_cast<uint32_t>(t_ns % 1'000'000'000LL);
    clock_pub_->publish(clock_msg);

    std::lock_guard<std::mutex> lk(mutex_);

    // Update dynamic obstacles
    obstacle_mgr_.updateDynamic(sim_dt);

    // Differential-drive integration (midpoint method for accuracy)
    double half_dtheta = robot_.cmd_vtheta * sim_dt * 0.5;
    double mid_theta   = robot_.theta + half_dtheta;
    double new_theta   = robot_.theta + robot_.cmd_vtheta * sim_dt;
    double new_x       = robot_.x + robot_.cmd_vx * std::cos(mid_theta) * sim_dt;
    double new_y       = robot_.y + robot_.cmd_vx * std::sin(mid_theta) * sim_dt;

    // Rotation is always applied
    robot_.theta = new_theta;

    // Position with collision response
    auto obstacles = obstacle_mgr_.getAll();
    if (!robot_.footprint.empty()) {
        // Polygon footprint: accurate map collision, circle approx for obstacles
        physics_.move(robot_.x, robot_.y, robot_.theta,
                      new_x, new_y,
                      robot_.footprint, robot_.radius,
                      map_, obstacles);
    } else {
        physics_.move(robot_.x, robot_.y, new_x, new_y,
                      robot_.radius, map_, obstacles);
    }
}

// ---------------------------------------------------------------------------
// Publish odom + TF — called from Qt timer after physicsStep
// ---------------------------------------------------------------------------
void SimulatorNode::publishState() {
    RobotState s;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        s = robot_;
    }

    auto now = simNow();
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, s.theta);

    // Odometry message
    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link";
    odom.pose.pose.position.x  = s.x;
    odom.pose.pose.position.y  = s.y;
    odom.pose.pose.orientation = tf2::toMsg(q);
    odom.twist.twist.linear.x  = s.cmd_vx;
    odom.twist.twist.angular.z = s.cmd_vtheta;
    odom_pub_->publish(odom);

    // TF: odom → base_link
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp            = now;
    tf.header.frame_id         = "odom";
    tf.child_frame_id          = "base_link";
    tf.transform.translation.x = s.x;
    tf.transform.translation.y = s.y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation      = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(tf);

    // Optional: map → odom identity TF (enable when not using AMCL/localization)
    if (publish_map_odom_tf.load()) {
        geometry_msgs::msg::TransformStamped map_tf;
        map_tf.header.stamp         = now;
        map_tf.header.frame_id      = "map";
        map_tf.child_frame_id       = "odom";
        map_tf.transform.rotation.w = 1.0;   // identity
        tf_broadcaster_->sendTransform(map_tf);
    }
}
