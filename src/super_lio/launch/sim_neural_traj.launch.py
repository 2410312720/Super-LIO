import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_super_lio = get_package_share_directory('super_lio')
    pkg_ros2_px4_odometry = get_package_share_directory('ros2_px4_odometry')
    pkg_super_planner = get_package_share_directory('super_planner')

    default_rviz_config_path = os.path.join(pkg_super_planner, 'rviz', 'top_down.rviz')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path],
        output='screen'
    )

    micro_xrce_agent_process = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
    )

    generic_odometry_node = Node(
        package='ros2_px4_odometry',
        executable='odom_to_px4_node',
        parameters=[
            PathJoinSubstitution([pkg_ros2_px4_odometry, 'config', 'super_lio_config.yaml'])
        ],
        output='screen',
        emulate_tty=True,
    )

    gt_odometry_node = Node(
        package='px4_ros2_converter',
        executable='gt_odom_from_px4_node',
        parameters=[{
            'odom_topic': '/gazebo/gt_odom',
            'odom_frame': 'world',
            'sensor_frame': 'base_link',
            'sensor_translation': [0.0, 0.0, 0.0],
            'sensor_rotation_matrix': [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0,
            ],
        }],
        output='screen',
        emulate_tty=True,
    )

    super_lio_node = Node(
        package='super_lio',
        executable='super_lio_node',
        name='super_lio_node',
        # output='screen',
        parameters=[os.path.join(pkg_super_lio, 'config', 'gz_sim.yaml')],
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        rviz_node,
        micro_xrce_agent_process,
        generic_odometry_node,
        gt_odometry_node,
        super_lio_node,
    ])
