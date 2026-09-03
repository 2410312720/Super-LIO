import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('super_lio')
    default_config = os.path.join(package_share, 'config', 'livox_360.yaml')
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Super-LIO ROS parameter YAML file',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        Node(
            package='super_lio',
            executable='super_lio_node',
            name='super_lio_node',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
