<<<<<<< HEAD
# BATTLEBOT_ROS2
월드 파일 업로드
=======
# battlesim 워크스페이스 안에 battlebot_sim 패키지 기준

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
