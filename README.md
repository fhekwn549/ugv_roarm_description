# ugv_roarm_description

ROS 2 Humble package for the **UGV Rover + RoArm-M2** mobile manipulator.

Waveshare UGV Rover (4WD skid-steer) 위에 RoArm-M2 (4-DOF + gripper) 로봇팔을 통합한 URDF 모델, 시뮬레이션, 키보드 텔레옵 패키지입니다.

## 리포 구조

| 리포 | 역할 | 내용 |
|------|------|------|
| **이 리포 (`ugv_roarm_description`)** | 로봇 정의 + 실행 구성 | URDF, launch, Gazebo 시뮬레이션, 텔레옵 |
| [fhekwn549/ugv_ws](https://github.com/fhekwn549/ugv_ws) | 하드웨어 구동 | 시리얼 드라이버, 센서 처리, 오도메트리, rosbridge 중계 |

RPi에서는 두 리포 모두 필요합니다. 이 리포의 `rasp_bringup.launch.py`가 `ugv_ws`의 드라이버 노드들을 실행합니다.

---

## Quick Start: 실제 로봇 제어 (WSL + RPi)

> 로봇 하드웨어가 조립되어 있고, RPi에 Ubuntu 22.04 + ROS 2 Humble이 설치된 상태를 전제합니다.

### 하드웨어 시리얼 포트

| 포트 | 장치 | 드라이버 |
|------|------|---------|
| `/dev/ttyAMA0` | UGV 바퀴 ESP32 | `ugv_driver` |
| `/dev/ttyUSB0` | RoArm-M2 ESP32 | `roarm_driver` |
| `/dev/ttyUSB1` | LDLidar STL-19P | `ldlidar_ros2` |

### Step 1: RPi 세팅 (SSH 터미널)

```bash
# SSH 접속
ssh pi@192.168.0.71

# 1. 워크스페이스 클론
cd ~
git clone -b ros2-humble-develop https://github.com/fhekwn549/ugv_ws.git
cd ~/ugv_ws/src/ugv_main
git clone https://github.com/fhekwn549/ugv_roarm_description.git

# 2. Python 의존성 설치
pip3 install pyserial

# 3. apt 패키지 설치
sudo apt install ros-humble-rosbridge-server ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher

# 4. 빌드 (필요한 패키지만)
cd ~/ugv_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ugv_bringup ugv_roarm_description ugv_description \
  ugv_base_node ugv_interface ldlidar
source install/setup.bash

# 5. bashrc에 자동 source 추가 (최초 1회)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ugv_ws/install/setup.bash" >> ~/.bashrc
```

### Step 2: WSL 세팅

```bash
# 1. 워크스페이스 클론
cd ~
git clone -b ros2-humble-develop https://github.com/fhekwn549/ugv_ws.git
cd ~/ugv_ws/src/ugv_main
git clone https://github.com/fhekwn549/ugv_roarm_description.git

# 2. Python 의존성 설치
pip3 install roslibpy

# 3. apt 패키지 설치
sudo apt install ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 ros-humble-tf2-ros

# 4. 빌드
cd ~/ugv_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ugv_bringup ugv_roarm_description ugv_description
source install/setup.bash

# 5. bashrc에 자동 source 추가 (최초 1회)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ugv_ws/install/setup.bash" >> ~/.bashrc
```

### Step 3: 실행 (매번)

**총 3개의 터미널**이 필요합니다.

#### 터미널 1 — RPi: 하드웨어 드라이버 + rosbridge (SSH)

```bash
ssh pi@192.168.0.71
source ~/ugv_ws/install/setup.bash

# 모든 하드웨어 드라이버 + rosbridge_server 실행
ros2 launch ugv_roarm_description rasp_bringup.launch.py
```

> rosbridge_server가 포트 9090에서 대기합니다. WSL에서 접속할 준비 완료.

#### 터미널 2 — WSL: RViz + rosbridge 중계

```bash
source ~/ugv_ws/install/setup.bash

ros2 launch ugv_roarm_description remote_view.launch.py
```

> 기본 host는 `192.168.0.71`입니다. RPi IP가 다르면 `host:=<IP>` 추가.
> RViz가 열리고 RPi의 센서 데이터(LiDAR, IMU, 관절 상태)가 표시됩니다.

#### 터미널 3 — WSL: 키보드 텔레옵

```bash
source ~/ugv_ws/install/setup.bash

ros2 run ugv_roarm_description teleop_all.py --ros-args -p mode:=rviz
```

> 이 터미널에 포커스를 두고 키보드를 누르면 로봇이 움직입니다.
> `mode:=rviz`는 RViz TF를 직접 publish하면서 동시에 실제 로봇도 제어합니다.
> 기본 model은 `rasp_rover`입니다. UGV Rover 사용 시 `-p model:=ugv_rover` 추가.

### 실행 순서 요약

```
[RPi SSH]  rasp_bringup.launch.py        ← 먼저 실행 (하드웨어 준비)
    ↓ rosbridge (ws://192.168.0.71:9090)
[WSL 1]    remote_view.launch.py          ← RPi 연결 후 실행 (RViz 시각화)
[WSL 2]    teleop_all.py mode:=rviz       ← 마지막 실행 (키보드 조작)
```

### 종료 순서

```
[WSL 2]    Ctrl+C (텔레옵 종료)
[WSL 1]    Ctrl+C (RViz + 중계 종료)
[RPi SSH]  Ctrl+C (하드웨어 드라이버 종료)
```

---

## Quick Start: RViz 단독 시각화 (로봇 없이)

실제 로봇 없이 URDF 모델과 키보드 텔레옵을 테스트합니다.

### GUI 슬라이더로 관절 확인

```bash
source ~/ugv_ws/install/setup.bash
ros2 launch ugv_roarm_description display.launch.py
```

### 키보드 텔레옵 (터미널 2개)

**터미널 1** — RViz:
```bash
source ~/ugv_ws/install/setup.bash
ros2 launch ugv_roarm_description display.launch.py use_gui:=false use_jsp:=false
```

**터미널 2** — 텔레옵:
```bash
source ~/ugv_ws/install/setup.bash
ros2 run ugv_roarm_description teleop_all.py --ros-args -p mode:=rviz
```

---

## Quick Start: 실제 로봇 SLAM (WSL + RPi)

> LiDAR로 주변 환경을 스캔하며 2D 맵을 생성합니다. slam_toolbox 사용.

**총 3개의 터미널**이 필요합니다.

#### 터미널 1 — RPi: 하드웨어 드라이버 (SSH)

```bash
ssh pi@192.168.0.71
source ~/ugv_ws/install/setup.bash

ros2 launch ugv_roarm_description rasp_bringup.launch.py
```

#### 터미널 2 — WSL: SLAM + RViz

```bash
source ~/ugv_ws/install/setup.bash

ros2 launch ugv_roarm_description slam_real.launch.py
```

> RViz가 Top-Down 뷰로 열리며 LiDAR 스캔과 점진적으로 생성되는 맵이 표시됩니다.

#### 터미널 3 — WSL: 키보드 텔레옵

```bash
source ~/ugv_ws/install/setup.bash

# SLAM 시에는 mode:=rviz 없이 기본 모드로 실행 (odom TF 충돌 방지)
ros2 run ugv_roarm_description teleop_all.py
```

> 로봇을 천천히 움직이며 맵을 완성합니다. 속도를 낮게 유지하세요 (`e` 키).
> SLAM에서는 `mode:=rviz`를 사용하지 않습니다 (relay가 odom TF를 발행).

#### 맵 저장

맵이 완성되면 새 터미널에서:

```bash
source ~/ugv_ws/install/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

> `~/map.pgm`과 `~/map.yaml` 파일이 생성됩니다.

---

## Quick Start: Gazebo 시뮬레이션

**터미널 1** — Gazebo:
```bash
source ~/ugv_ws/install/setup.bash
ros2 launch ugv_roarm_description gazebo.launch.py
```

> WSL2에서는 Gazebo 플러그인 초기화에 약 2분 소요.

**터미널 2** — 텔레옵:
```bash
source ~/ugv_ws/install/setup.bash
ros2 run ugv_roarm_description teleop_all.py
```

---

## Keyboard Controls

주행과 팔 제어가 **동시에** 가능합니다.

| Key | Function |
|-----|----------|
| **Drive** | |
| `W` / `S` | Forward / Backward |
| `A` / `D` | Turn Left / Right |
| `Q` / `E` | Increase / Decrease speed |
| `Space` | Emergency Stop |
| **Arm** | |
| `M` | Joint ↔ TCP 모드 전환 |
| `1` / `2` / `3` | 관절 1~3 선택 (Joint) / X/Y/Z 축 선택 (TCP) |
| `I` / `K` | 관절 또는 TCP +/- |
| `U` / `J` | Step size +/- |
| **Gripper** | |
| `O` | Gripper open (+) |
| `P` | Gripper close (-) |

### TCP 제어 모드

`M` 키로 전환. 로봇팔 끝단의 직교 좌표(X, Y, Z)를 직접 제어합니다.
내부적으로 `roarm_m2` IK/FK 솔버를 사용하며, 도달 불가능한 위치로의 이동은 자동 무시됩니다.

---

## 아키텍처

### 통신 흐름 (WSL ↔ RPi)

```
[WSL]                                          [RPi]
teleop_all.py                                  rasp_bringup.launch.py
  ├─ /cmd_vel ──────┐                            ├─ ugv_driver (/dev/ttyAMA0)
  ├─ /arm_controller │    rosbridge_relay         ├─ roarm_driver (/dev/ttyUSB0)
  │   /joint_trajectory┤  ←── WebSocket ──→       ├─ base_node (odom)
  ├─ /roarm/gripper_cmd┘   (ws://RPi:9090)        ├─ ugv_bringup (IMU, encoder)
  │                                                └─ ldlidar_ros2 (/dev/ttyUSB1)
  │  ← /joint_states, /scan, /odom, /imu ←
  │
  └─ RViz (TF 시각화)
```

### 그리퍼 값 변환

| 단계 | 값 | 범위 |
|------|-----|------|
| `grip_level` (teleop 내부) | 양수 | 0 (닫힘) ~ 2.094 (열림) |
| URDF joint value | 음수 (`-grip_level`) | -2.094 ~ 0 |
| RViz 시각 회전 | 양수 (axis -1이 부호 반전) | 0° ~ +120° |
| ESP32 명령 | `(max - grip) × 1.5` | π (닫힘) ~ 0 (열림) |

---

## Package Structure

```
ugv_roarm_description/
├── urdf/
│   ├── ugv_roarm.xacro            # UGV Rover + RoArm URDF
│   ├── ugv_roarm.gazebo.xacro     # UGV Rover Gazebo 플러그인
│   ├── rasp_roarm.xacro           # Rasp Rover + RoArm URDF (RPi용)
│   └── rasp_roarm.gazebo.xacro    # Rasp Rover Gazebo 플러그인
├── launch/
│   ├── display.launch.py          # RViz 단독 시각화
│   ├── gazebo.launch.py           # Gazebo 시뮬레이션
│   ├── slam.launch.py             # SLAM (Gazebo 시뮬레이션)
│   ├── slam_real.launch.py        # SLAM (실제 로봇)
│   ├── rasp_bringup.launch.py     # RPi 하드웨어 + rosbridge_server
│   └── remote_view.launch.py      # WSL RViz + rosbridge 중계
├── scripts/
│   └── teleop_all.py              # 통합 키보드 텔레옵 노드
├── config/
│   ├── ugv_arm_controllers.yaml   # ros2_control 컨트롤러 설정
│   ├── ugv_arm_ros2_control.xacro # ros2_control 하드웨어 인터페이스
│   └── slam_toolbox.yaml          # SLAM 파라미터
├── rviz/
│   ├── view_ugv_roarm.rviz        # 기본 RViz 설정
│   ├── remote_view.rviz           # 원격 제어용 RViz 설정
│   └── slam_view.rviz             # SLAM용 RViz 설정 (Top-Down)
├── meshes/
│   ├── rasp_rover/                # Rasp Rover 메시 (STL)
│   └── roarm_m2/                  # RoArm-M2 메시 (STL)
└── worlds/
    └── ugv_roarm.world            # Gazebo 월드
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | 주행 속도 명령 |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 팔 관절 목표 |
| `/roarm/gripper_cmd` | `std_msgs/Float64` | 그리퍼 제어 (ESP32) |
| `/gripper_controller/gripper_cmd` | `control_msgs/GripperCommand` | 그리퍼 제어 (Gazebo) |
| `/joint_states` | `sensor_msgs/JointState` | 관절 상태 피드백 |
| `/scan` | `sensor_msgs/LaserScan` | 2D LiDAR |
| `/odom` | `nav_msgs/Odometry` | 오도메트리 |

## 배포 (코드 수정 후)

```bash
# WSL에서
cd ~/ugv_ws/src/ugv_main/ugv_roarm_description
git add -A && git commit -m "설명" && git push origin main

cd ~/ugv_ws
git add -A && git commit -m "설명" && git push origin ros2-humble-develop

# RPi에서 (SSH)
cd ~/ugv_ws
git pull origin ros2-humble-develop
cd src/ugv_main/ugv_roarm_description && git pull origin main
cd ~/ugv_ws
colcon build --packages-select ugv_bringup ugv_roarm_description
source install/setup.bash
```

## Troubleshooting

| 증상 | 원인 | 해결 |
|------|------|------|
| RViz에서 로봇이 안 보임 | `ugv_description` 미빌드 | `colcon build --packages-select ugv_description` 후 source |
| rosbridge 연결 안 됨 | RPi의 rosbridge_server 미실행 | RPi에서 `rasp_bringup.launch.py` 먼저 실행 |
| 로봇이 RViz에서 안 움직임 | teleop 미실행 또는 Fixed Frame 설정 | teleop 실행 + Fixed Frame을 `odom`으로 설정 |
| 팔 관절 움직이면 그리퍼 토크 풀림 | `roarm_driver` 미업데이트 | RPi에서 `ugv_bringup` 리빌드 |
| WSL2에서 RViz 크래시 | GPU 호환성 | `LIBGL_ALWAYS_SOFTWARE=1` 환경변수 설정 |
| 그리퍼 방향 반대 | roarm_driver 버전 불일치 | RPi에서 git pull + 리빌드 |
