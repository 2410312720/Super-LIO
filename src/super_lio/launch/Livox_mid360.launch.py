import os
import launch.logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_super_lio = get_package_share_directory('super_lio')
    config_yaml = os.path.join(pkg_super_lio, 'config', 'livox_360.yaml')
    rviz_config_file = os.path.join(pkg_super_lio, 'rviz', 'lio.rviz')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to start RVIZ2'
    )
    rviz_flag = LaunchConfiguration('rviz')

    declare_config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_yaml,
        description='Super-LIO ROS parameter YAML file'
    )
    config_file = LaunchConfiguration('config_file')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    super_lio_component = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_super_lio, 'launch', 'super_lio_component.launch.py')
        ),
        launch_arguments={
            'config_file': config_file,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='super_lio',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'warn'],
        condition=IfCondition(rviz_flag)
    )

    ld = LaunchDescription()
    ld.add_action(declare_rviz_arg)
    ld.add_action(declare_config_arg)
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(super_lio_component)
    ld.add_action(rviz2_node)

    return ld
