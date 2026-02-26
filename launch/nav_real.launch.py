"""
Real Robot Navigation Launch File

SLAM으로 생성한 맵을 기반으로 Nav2 자율주행을 실행합니다.
RPi에서 rasp_bringup.launch.py 실행 후,
WSL에서 이 launch 파일을 실행하면 Nav2 스택 + RViz가 함께 뜹니다.
CycloneDDS를 통해 RPi의 토픽이 자동으로 수신됩니다.

Usage:
  ros2 launch ugv_roarm_description nav_real.launch.py map:=~/maps/map_20250101_120000.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('ugv_roarm_description')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'nav_view.rviz')

    lifecycle_nodes = [
        'map_server',
        'amcl',
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator',
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            description='Full path to map YAML file (e.g. ~/maps/map.yaml)'),

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

        # velocity_smoother disabled — motor deadband too high for gradual ramp-up

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
