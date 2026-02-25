"""
Real Robot SLAM Launch File

RPi에서 rasp_bringup.launch.py 실행 후,
WSL에서 이 launch 파일을 실행하면 slam_toolbox + RViz가 함께 뜹니다.
CycloneDDS를 통해 RPi의 토픽(/scan, /odom, /tf 등)이 자동으로 수신됩니다.

Ctrl+C 종료 시 ~/maps/map_YYYYMMDD_HHMMSS.pgm/.yaml 자동 저장.

Usage:
  ros2 launch ugv_roarm_description slam_real.launch.py
"""

import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('ugv_roarm_description')
    slam_params = os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'slam_view.rviz')

    # Auto-save map on shutdown
    map_dir = os.path.expanduser('~/maps')
    os.makedirs(map_dir, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    map_path = os.path.join(map_dir, f'map_{timestamp}')

    return LaunchDescription([
        # slam_toolbox (online async SLAM)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params,
                {'use_sim_time': False},
            ],
        ),

        # RViz2 with SLAM view (software rendering for WSL2)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'}
        ),

        # Map auto-saver: traps SIGINT and saves map before other nodes exit
        ExecuteProcess(
            cmd=['bash', '-c',
                 f'trap "'
                 f'echo [map_saver] Saving map to {map_path}... ; '
                 f'ros2 run nav2_map_server map_saver_cli -f {map_path} 2>/dev/null ; '
                 f'echo [map_saver] Done. ; '
                 f'exit 0'
                 f'" INT TERM; '
                 f'while true; do sleep 1; done'],
            output='screen',
        ),
    ])
