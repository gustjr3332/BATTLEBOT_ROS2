# 배틀봇 시뮬레이션 (Battlebot Simulation)

ROS 2 Foxy와 Gazebo Classic을 이용한 2인용 배틀봇 시뮬레이션 프로젝트입니다.

## 주요 기능
- 2대의 배틀봇을 각각 키보드로 조종
- 충돌 시 데미지 계산 및 체력 시스템
- 동적인 장애물 및 아이템(힐링팩) 포함

## 의존성 (Dependencies)
이 패키지를 빌드하고 실행하기 전에 아래 패키지들이 설치되어 있어야 합니다.
- ROS 2 Foxy
- Gazebo 11
- `ros-foxy-gazebo-ros-pkgs`
- Python 라이브러리: `transforms3d`
  ```bash
  sudo apt-get install ros-foxy-gazebo-ros-pkgs
  pip install transforms3d


# 실행관련
* 패키지명 : battlebot_sim
* 런치파일명 : battlebot_sim.launch.py

```bash
cd ~/your_workspace
colcon build
source install/setup.bash
ros2 launch battlebot_sim battlebot_sim.launch.py
ros2 run battlebot_sim dual_teleop


