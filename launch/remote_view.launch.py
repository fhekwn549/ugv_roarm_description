"""
WSL Remote View Launch File

RPi에서 rosbridge_server + rasp_bringup 실행 후,
WSL에서 이 launch 파일을 실행하면 relay + robot_state_publisher + RViz가 함께 뜹니다.

Usage:
  ros2 launch ugv_roarm_description remote_view.launch.py
  ros2 launch ugv_roarm_description remote_view.launch.py host:=192.168.0.71
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('ugv_roarm_description')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'remote_view.rviz')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'rasp_roarm.xacro')

    return LaunchDescription([
        DeclareLaunchArgument(
            'host',
            default_value='192.168.0.71',
            description='RPi rosbridge server IP'),

        DeclareLaunchArgument(
            'port',
            default_value='9090',
            description='RPi rosbridge server port'),

        # Local Robot State Publisher (URDF model + static TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', urdf_file]),
                    value_type=str)
            }]
        ),

        # Static TF: base_lidar_link → base_laser
        # (RPi ldlidar uses frame_id 'base_laser', URDF has 'base_lidar_link')
        # yaw=π/2: LiDAR physical mounting is 90° rotated from URDF assumption
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_lidar_to_base_laser',
            arguments=['0', '0', '0', '1.5708', '0', '0',
                       'base_lidar_link', 'base_laser']
        ),

        # Rosbridge Relay (RPi topics → WSL local topics)
        Node(
            package='ugv_bringup',
            executable='rosbridge_relay',
            name='rosbridge_relay',
            output='screen',
            parameters=[{
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
            }]
        ),

        # RViz2 (software rendering for WSL2)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'}
        ),
    ])
