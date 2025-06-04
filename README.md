<<<<<<< HEAD
#워크스페이스에서 git clone https://github.com/gustjr3332/BATTLEBOT_ROS2.git
실행하면 BATTLEBOT_ROS2 패키지다운받아집니다
>>>>>>>
=======
# battlesim 워크스페이스 안에 battlebot_sim 패키지 기준

1. cd ~/battlesim
2. colcon build
3. source install/setup.bash
4. ros2 launch battlebot_sim battlebot_sim.launch.py
5. ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/battlebot/cmd_vel!
6. ros2 topic echo /battlesim/health 또는 /battlesim/contact 로 state 값에 변화 확인

7.ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/battlebot/cmd_vel
-----------------------6/1 데미지 노드 관련 사항

주요변경사항:



-파이썬 패키지 추가 설치 필요-

sudo apt update

sudo apt install ros-foxy-tf-transformations

sudo apt install python3-pip

sudo pip3 install transforms3d
