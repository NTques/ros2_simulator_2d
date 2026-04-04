#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>

#include "map_loader.hpp"
#include "robot.hpp"
#include "obstacle_manager.hpp"
#include "physics_engine.hpp"

class SimulatorNode : public rclcpp::Node {
public:
    explicit SimulatorNode();
    ~SimulatorNode() override;

    // Call once after make_shared<SimulatorNode>() to start the ROS spin thread.
    void start();

    // ---- Thread-safe interface for Qt thread --------------------------------

    void setMap(MapData map);
    void setRobotRadius(double radius);
    void setRobotPose(double x, double y, double theta);
    // Set polygon footprint (robot-local frame).  Also recomputes radius.
    void setRobotFootprint(const std::vector<std::pair<double,double>>& fp);

    int  addObstacle(const Obstacle& obs);
    void removeObstacle(int id);
    void updateObstacle(const Obstacle& obs);

    MapData              getMap()         const;
    RobotState           getRobotState()  const;
    std::vector<Obstacle> getObstacles()  const;
    bool                 hasMap()         const;

    // Publish a goal pose to /goal_pose (for manual testing via bt_navigator)
    void publishGoal(double x, double y, double theta);

    // Send cached path directly to controller via FollowPath action (bypasses bt_navigator)
    // controller_id: name of the controller plugin (e.g. "FollowPath")
    void sendFollowPath(const std::string& controller_id = "FollowPath");
    void cancelFollowPath();
    bool isFollowPathReady() const;  // non-blocking DDS readiness check
    bool hasCachedPath()     const;

    // Returns true (and clears the flag) if the last FollowPath goal finished
    // (succeeded, aborted, or cancelled) since the last sendFollowPath() call.
    // Safe to call from the Qt thread while the ROS thread runs the callback.
    bool takeFollowPathDone();

    // Set user-drawn path, interpolate at given resolution and publish /path
    // waypoints: (world_x, world_y) pairs
    void setPath(const std::vector<std::pair<double,double>>& waypoints,
                 double resolution);
    void clearPath();
    std::vector<std::pair<double,double>> getPathWaypoints() const;

    // ---- Physics step (called from Qt timer at ~50 Hz) ---------------------
    // sim_dt = real_dt * sim_speed
    void physicsStep(double sim_dt);
    void publishState();

    // Current simulation time in nanoseconds (advanced by physicsStep)
    rclcpp::Time simNow() const;


    // Simulation speed multiplier (accessible from Qt thread, atomic is enough)
    std::atomic<double> sim_speed{1.0};

    // When true, publishState() also sends the map -> odom identity TF.
    // Use this instead of a localization node (e.g. AMCL) when the robot
    // is assumed to start at the map origin.
    std::atomic<bool> publish_map_odom_tf{false};

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    mutable std::mutex mutex_;
    MapData            map_;
    RobotState         robot_;
    ObstacleManager    obstacle_mgr_;
    PhysicsEngine      physics_;
    bool               has_map_ = false;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr      cmd_vel_sub_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr         clock_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr           odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr      map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr   goal_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr               path_pub_;

    using FollowPath      = nav2_msgs::action::FollowPath;
    using GoalHandleFP    = rclcpp_action::ClientGoalHandle<FollowPath>;
    rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
    GoalHandleFP::SharedPtr                      current_goal_handle_;

    nav_msgs::msg::Path                       cached_path_;
    std::vector<std::pair<double,double>>     path_waypoints_raw_;
    rclcpp::TimerBase::SharedPtr                                map_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>              tf_broadcaster_;

    nav_msgs::msg::OccupancyGrid cached_grid_;   // rebuilt on setMap()

    std::atomic<int64_t> cmd_vel_ignore_until_ns_{0};
    std::atomic<bool>    follow_path_done_{false};
    std::atomic<int64_t> sim_time_ns_{0};  // advances by sim_dt each physicsStep

    std::thread       ros_thread_;
    std::atomic<bool> running_{false};
};
