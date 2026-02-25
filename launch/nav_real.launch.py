"""
Real Robot Navigation Launch File

SLAM으로 생성한 맵을 기반으로 Nav2 자율주행을 실행합니다.
RPi에서 rasp_bringup.launch.py 실행 후,
WSL에서 이 launch 파일을 실행하면 rosbridge relay + Nav2 스택 + RViz가 함께 뜹니다.

rosbridge_relay가 /odom 데이터에서 odom→base_footprint TF를 발행하므로
teleop은 mode=rviz 없이 실행해야 합니다 (TF 충돌 방지).

Usage:
  ros2 launch ugv_roarm_description nav_real.launch.py map:=~/maps/map_20250101_120000.yaml
  ros2 launch ugv_roarm_description nav_real.launch.py map:=~/maps/map.yaml host:=192.168.0.71
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('ugv_roarm_description')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'rasp_roarm.xacro')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'nav_view.rviz')

    lifecycle_nodes = [
        'map_server',
        'amcl',
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'host',
            default_value='192.168.0.71',
            description='RPi rosbridge server IP'),

        DeclareLaunchArgument(
            'port',
            default_value='9090',
            description='RPi rosbridge server port'),

        DeclareLaunchArgument(
            'map',
            description='Full path to map YAML file (e.g. ~/maps/map.yaml)'),

        # --- Local Robot State Publisher (URDF model + static TF) ---
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

        # --- Static TF: base_lidar_link → base_laser ---
        # RPi ldlidar uses frame_id 'base_laser', URDF has 'base_lidar_link'
        # yaw=π/2: LiDAR physical mounting is 90° rotated from URDF assumption
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_lidar_to_base_laser',
            arguments=['0', '0', '0', '1.5708', '0', '0',
                       'base_lidar_link', 'base_laser']
        ),

        # --- Rosbridge Relay (RPi topics → WSL local topics) ---
        # pub_odom_tf=true: /odom 데이터에서 odom→base_footprint TF 발행
        Node(
            package='ugv_bringup',
            executable='rosbridge_relay',
            name='rosbridge_relay',
            output='screen',
            parameters=[{
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'pub_odom_tf': True,
            }]
        ),

        # --- Nav2: Map Server ---
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                nav2_params,
                {'yaml_filename': LaunchConfiguration('map')},
            ],
        ),

        # --- Nav2: AMCL ---
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params],
        ),

        # --- Nav2: Planner Server ---
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params],
        ),

        # --- Nav2: Controller Server ---
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params],
        ),

        # --- Nav2: Behavior Server ---
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params],
        ),

        # --- Nav2: BT Navigator ---
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params],
        ),

        # --- Nav2: Velocity Smoother ---
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_params],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel'),
            ],
        ),

        # --- Nav2: Lifecycle Manager ---
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': lifecycle_nodes,
            }],
        ),

        # --- RViz2 with Nav2 view (software rendering for WSL2) ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'}
        ),
    ])
