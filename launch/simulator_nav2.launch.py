"""
simulator_nav2.launch.py

Launches:
  1. simulator_node       — 2-D physics simulator + Qt GUI
  2. controller_server    — nav2 MPPI controller (includes local_costmap)
  3. lifecycle_manager    — activates controller_server

Path flow:
  simulator → /follow_path action → controller_server → /cmd_vel → simulator

Prerequisites:
  • Load a map in the simulator GUI and enable "map→odom TF" toggle so
    the costmap static_layer can locate obstacles in the odom frame.
  • Draw a path in the simulator and press "▶ Run Sweep" to start trials.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('simulator')

    params_file = LaunchConfiguration('params_file')
    log_level    = LaunchConfiguration('log_level')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'simulator_nav2_params.yaml'),
        description='Path to the nav2 + simulator parameters YAML file',
    )

    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='ROS2 logging level (debug, info, warn, error)',
    )

    # ------------------------------------------------------------------
    # 1. Simulator node
    #    • Publishes /map, /odom, /tf (odom→base_link, map→odom)
    #    • Subscribes to /cmd_vel
    #    • Exposes /follow_path action client (→ controller_server)
    # ------------------------------------------------------------------
    simulator_node = Node(
        package='simulator',
        executable='simulator_node',
        name='simulator',
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # ------------------------------------------------------------------
    # 2. Controller server  (nav2_mppi_controller + local_costmap)
    #    • Subscribes to /map (static_layer), /tf
    #    • Subscribes to /path via FollowPath action
    #    • Publishes /cmd_vel
    # ------------------------------------------------------------------
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', log_level],
        # No cmd_vel remap: controller outputs directly to /cmd_vel
        # (no velocity_smoother in this minimal stack)
    )

    # ------------------------------------------------------------------
    # 3. Lifecycle manager — configures and activates controller_server
    # ------------------------------------------------------------------
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_controller',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['controller_server'],
        }],
    )

    return LaunchDescription([
        declare_params_file,
        declare_log_level,
        simulator_node,
        controller_server,
        lifecycle_manager,
    ])
