# battlebot_ws
version : gazebo fortress
ubuntu 22.04


gazebo classic에서 실행 시 코드 변경 필요!!

실행 방법
1. cd ~/battlesim
2. colcon build
3. source install/setup.bash
4. ros2 launch battlebot_sim battlebot_sim.launch.py

-----------------------6/1 데미지 노드 관련 사항

주요변경사항:



-파이썬 패키지 추가 설치 필요-
sudo apt update
sudo apt install ros-foxy-tf-transformations
sudo apt install python3-pip
sudo pip3 install transforms3d
