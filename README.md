<<<<<<< HEAD
#워크스페이스에서 git clone https://github.com/gustjr3332/BATTLEBOT_ROS2.git
실행하면 BATTLEBOT_ROS2 패키지다운받아집니다
>>>>>>>
=======
# 패키지명 : battlebot_sim
# 런치파일명 : battlebot_sim.launch.py

1. cd ~/battlesim
2. colcon build
3. source install/setup.bash
4. ros2 launch battlebot_sim battlebot_sim.launch.py
5. ros2 run battlebot_sim dual_teleop


-주요 파이썬 패키지 설치 필요-

sudo apt update

sudo apt install ros-foxy-tf-transformations

sudo apt install python3-pip

sudo pip3 install transforms3d
