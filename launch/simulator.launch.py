from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value='',
            description='Path to map YAML file (optional; can also be loaded from GUI)'
        ),
        DeclareLaunchArgument(
            'urdf_file',
            default_value='',
            description='Path to robot URDF file (optional; can also be loaded from GUI)'
        ),

        Node(
            package='simulator',
            executable='simulator_node',
            name='simulator',
            output='screen',
            parameters=[{
                'map_file':  LaunchConfiguration('map_file'),
                'urdf_file': LaunchConfiguration('urdf_file'),
            }],
        ),
    ])
