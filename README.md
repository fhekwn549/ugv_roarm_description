# ugv_roarm_description

ROS 2 Humble package for the **UGV Rover + RoArm-M2** mobile manipulator.

Waveshare UGV Rover (4WD skid-steer) 위에 RoArm-M2 (4-DOF + gripper) 로봇팔을 통합한 URDF 모델, 시뮬레이션, 키보드 텔레옵 패키지입니다.

## Features

- **통합 URDF/Xacro** - UGV Rover 베이스 + RoArm-M2 매니퓰레이터 + 센서(2D LiDAR, Depth Camera, IMU)
- **Gazebo Classic 시뮬레이션** - ros2_control 기반 관절 제어, skid-steer 주행
- **통합 키보드 텔레옵** - 주행 + 팔 + 그리퍼 동시 제어, Joint/TCP 모드 지원
- **RViz 시각화** - 직접 TF 퍼블리싱을 통한 로봇 제어 (Gazebo 없이)
- **SLAM** - slam_toolbox를 이용한 실시간 맵 생성

## Dependencies

### 소스 패키지 (같은 워크스페이스에 필요)

이 패키지는 UGV Rover의 메시 파일(`package://ugv_description/meshes/...`)을 참조하므로,
[ugv_ws](https://github.com/waveshareteam/ugv_ws)의 `ugv_description` 패키지가 같은 워크스페이스에 있어야 합니다.

```bash
# ugv_ws를 이미 클론한 경우, 그 안에 이 패키지를 배치
cd ~/ugv_ws/src
git clone https://github.com/fhekwn549/ugv_roarm_description.git
```

Gazebo 시뮬레이션 사용 시, 소스 빌드 버전의 `gazebo_ros2_control`이 필요합니다:

```bash
cd ~/ugv_ws/src
git clone -b humble https://github.com/ros-controls/gazebo_ros2_control.git
```

### apt 패키지

```bash
sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-ros2-control \
  ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller \
  ros-humble-gripper-controllers ros-humble-xacro ros-humble-slam-toolbox \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui
```

## Build

```bash
cd ~/ugv_ws
colcon build --packages-select ugv_roarm_description --symlink-install
source install/setup.bash
```

## Usage

### 1. RViz 시각화

```bash
# GUI 슬라이더로 관절 조작
ros2 launch ugv_roarm_description display.launch.py

# 키보드 텔레옵으로 조작 (터미널 2개 사용)
ros2 launch ugv_roarm_description display.launch.py use_gui:=false use_jsp:=false
ros2 run ugv_roarm_description teleop_all.py --ros-args -p mode:=rviz
```

### 2. Gazebo 시뮬레이션

```bash
# Gazebo 실행 (GUI 포함)
ros2 launch ugv_roarm_description gazebo.launch.py

# 다른 터미널에서 텔레옵
ros2 run ugv_roarm_description teleop_all.py
```

### 3. SLAM 맵핑

```bash
# Gazebo + slam_toolbox + RViz 통합 실행
ros2 launch ugv_roarm_description slam.launch.py

# 다른 터미널에서 텔레옵으로 주행하며 맵 생성
ros2 run ugv_roarm_description teleop_all.py
```

## Keyboard Controls

주행과 팔 제어가 **동시에** 가능합니다. `M` 키로 팔 제어 모드를 전환합니다.

| Key | Function |
|-----|----------|
| **Drive** | |
| `W` / `S` | Forward / Backward |
| `A` / `D` | Turn Left / Right |
| `Q` / `E` | Increase / Decrease speed |
| `Space` | Emergency Stop |
| **Arm** | |
| `M` | Joint ↔ TCP 모드 전환 |
| `1` / `2` / `3` | Joint 모드: 관절 1~3 선택 / TCP 모드: X/Y/Z 축 선택 |
| `I` / `K` | Joint 모드: 관절 +/- / TCP 모드: 선택 축 +/- (mm) |
| `U` / `J` | Step size +/- |
| **Gripper** | |
| `O` / `P` | Gripper open(+) / close(-) |

### TCP 제어 모드

TCP(Tool Center Point) 모드에서는 로봇팔 끝단의 직교 좌표(X, Y, Z)를 직접 제어합니다.
내부적으로 `roarm_moveit_cmd/solver.hpp`의 `roarm_m2` IK/FK 솔버를 Python으로 포팅하여 사용합니다.

- **FK**: 현재 관절 각도 → TCP 위치(mm) 계산
- **IK**: 목표 TCP 위치 → 관절 각도 역변환
- 도달 불가능한 위치로의 이동 명령은 자동으로 무시됩니다

## Package Structure

```
ugv_roarm_description/
├── urdf/
│   ├── ugv_roarm.xacro            # Display용 URDF
│   └── ugv_roarm.gazebo.xacro     # Gazebo 플러그인 포함 URDF
├── launch/
│   ├── display.launch.py          # RViz 시각화
│   ├── gazebo.launch.py           # Gazebo 시뮬레이션
│   └── slam.launch.py             # SLAM 맵핑
├── config/
│   ├── ugv_arm_controllers.yaml   # ros2_control 컨트롤러 설정
│   ├── ugv_arm_ros2_control.xacro # ros2_control 하드웨어 인터페이스
│   └── slam_toolbox.yaml          # SLAM 파라미터
├── scripts/
│   └── teleop_all.py              # 통합 키보드 텔레옵 노드
├── rviz/
│   ├── view_ugv_roarm.rviz        # 기본 RViz 설정
│   └── slam_view.rviz             # SLAM용 RViz 설정
├── meshes/roarm_m2/               # RoArm-M2 메시 파일
└── worlds/
    └── ugv_roarm.world            # 기본 Gazebo 월드
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | 주행 속도 명령 |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 팔 관절 목표 |
| `/gripper_controller/gripper_cmd` | `control_msgs/GripperCommand` | 그리퍼 제어 |
| `/joint_states` | `sensor_msgs/JointState` | 관절 상태 |
| `/scan` | `sensor_msgs/LaserScan` | 2D LiDAR |
| `/odom` | `nav_msgs/Odometry` | 오도메트리 |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM 맵 |

## Notes

- **WSL2**: Gazebo GUI는 소프트웨어 렌더링(`LIBGL_ALWAYS_SOFTWARE=1`) 사용. 플러그인 초기화에 ~2분 소요.
- **RViz 모드**: `robot_state_publisher`를 통하지 않고 직접 TF를 퍼블리시하여 비동기 TF로 인한 진동 방지.
