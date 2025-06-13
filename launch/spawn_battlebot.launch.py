import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('battlebot_sim')
    world_file = os.path.join(pkg_share, 'worlds', 'arena.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'battlebot_1.urdf')
    urdf_file_2 = os.path.join(pkg_share, 'urdf', 'battlebot_2.urdf')   
    
    models_path = os.path.join(pkg_share, 'models')

    current_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if models_path not in current_model_path.split(':'): # 중복 추가 방지
        if current_model_path:
            os.environ['GAZEBO_MODEL_PATH'] = models_path + ':' + current_model_path
        else:
            os.environ['GAZEBO_MODEL_PATH'] = models_path
    print(f"DEBUG: GAZEBO_MODEL_PATH set to: {os.environ.get('GAZEBO_MODEL_PATH')}")

    params_file_path = os.path.join(pkg_share, 'config', 'params.yaml')

    # Gazebo 시뮬레이터를 직접 실행하는 ExecuteProcess
    gazebo_process = ExecuteProcess(
        cmd=[
            'gazebo', # 시스템 PATH에 있는 'gazebo' 실행 파일 사용
            '--verbose',
            world_file,
            '-s', 'libgazebo_ros_init.so', # Gazebo ROS 초기화 플러그인 로드
            '-s', 'libgazebo_ros_factory.so' # Gazebo ROS 인터페이스 플러그인 로드
        ],
        output='screen'
    )

    # battlebot_1과 battlebot_2의 URDF 파일을 읽어옴
    with open(urdf_file, 'r') as file:
        battlebot_1_urdf_content = file.read()
    with open(urdf_file_2, 'r') as file:
        battlebot_2_urdf_content = file.read()

    # battlebot_1의 robot_state_publisher
    battlebot_1_rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='battlebot_1_state_publisher', 
        namespace='battlebot_1',
        output='screen',
        parameters=[{'robot_description': battlebot_1_urdf_content}]
    )

    # battlebot_2의 robot_state_publisher
    battlebot_2_rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='battlebot2_state_publisher',
        namespace='battlebot_2',
        output='screen',
        parameters=[{'robot_description': battlebot_2_urdf_content}]
    )


    # spawn_entity (Gazebo 서비스가 완전히 준비될 때까지 딜레이)
    spawn_battlebot_1_entity = TimerAction(
        period=5.0, # Gazebo가 완전히 로드될 시간을 충분히 줌
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'battlebot_1', # URDF의 <robot name="battlebot_1">과 일치
                    '-file', urdf_file,
                    '-x', '0', '-y', '4', '-z', '0.3'
                ],
                output='screen'
            )
        ]
    )

    # battlebot_2를 Gazebo에 반대편 위치로 스폰
    spawn_battlebot_2_entity = TimerAction(
        period=6.0,  # 첫 로봇보다 약간 늦게 스폰
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'battlebot_2',
                    '-file', urdf_file_2,
                    '-x', '0', '-y', '-4', '-z', '0.3'
                ],
                output='screen'
            )
        ]
    )

    # moving_obstacle_controller 실행 (로봇 스폰 후 더 늦게 실행)
    moving_obstacle_controller_node = TimerAction(
        period=8.0, # battlebot 스폰 후 추가 딜레이
        actions=[
            Node(
                package='battlebot_sim',
                executable='moving_obstacle_controller',
                name='moving_obstacle_controller',
                output='screen'
            )
        ]
    )

    # damage_calculator_node: 로봇들이 스폰된 후 실행되도록 지연
    damage_calculator_node = TimerAction(
        period=7.0, # 로봇 스폰 후 실행
        actions=[
            Node(
                package='battlebot_sim',
                executable='damage_calculator_node',
                name='damage_calculator',
                output='screen',
                parameters=[params_file_path],
                prefix='gnome-terminal --' 
            )
        ]
    )
    
    # health_monitor_terminal: damage_calculator가 토픽을 발행할 준비가 된 후 실행
    health_monitor_terminal = TimerAction(
        period=9.0, # damage_calculator(7초)가 실행된 후
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c', """
# (내부 스크립트는 이전과 동일)
obstacle_controller_dead=0
teleop_dead=0
echo "헬스 모니터링 및 자동 노드 종료 시스템 시작 (구버전 ROS2 호환 모드)..."
# ... (이전의 긴 bash 스크립트 내용 전체) ...
while true; do
    bot1_dead=$(ros2 topic echo /battlebot_1/health std_msgs/msg/Float32 | head -n 3 | grep "data:" | awk '{if ($2 <= 0) print 1; else print 0}')
    bot2_dead=$(ros2 topic echo /battlebot_2/health std_msgs/msg/Float32 | head -n 3 | grep "data:" | awk '{if ($2 <= 0) print 1; else print 0}')
    obs_dead=$(ros2 topic echo /moving_obstacle/health std_msgs/msg/Float32 | head -n 3 | grep "data:" | awk '{if ($2 <= 0) print 1; else print 0}')
    health1=$(ros2 topic echo /battlebot_1/health std_msgs/msg/Float32 | head -n 3 | grep "data:" | awk '{print $2}')
    health2=$(ros2 topic echo /battlebot_2/health std_msgs/msg/Float32 | head -n 3 | grep "data:" | awk '{print $2}')
    health3=$(ros2 topic echo /moving_obstacle/health std_msgs/msg/Float32 | head -n 3 | grep "data:" | awk '{print $2}')
    clear
    echo "========== REAL-TIME ROBOT HEALTH =========="
    echo "BattleBot 1 (BLUE): $health1"
    echo "BattleBot 2 (RED):  $health2"
    echo "Moving Obstacle:    $health3"
    echo "============================================"
    echo "Monitoring... (Press Ctrl+C in this terminal to stop monitoring)"
    if [ "$obs_dead" -eq 1 ] && [ "$obstacle_controller_dead" -eq 0 ]; then
        echo "Moving Obstacle 파괴됨! 컨트롤러 노드를 종료합니다..."
        kill $(ros2 node info /moving_obstacle_controller | grep 'Pid:' | awk '{print $2}') &> /dev/null
        obstacle_controller_dead=1
    fi
    if { [ "$bot1_dead" -eq 1 ] || [ "$bot2_dead" -eq 1 ]; } && [ "$teleop_dead" -eq 0 ]; then
        echo "Battlebot 파괴됨! 조종 노드를 종료합니다..."
        kill $(ros2 node info /dual_teleop | grep 'Pid:' | awk '{print $2}') &> /dev/null
        teleop_dead=1
    fi
    sleep 1
done
"""],
                shell=True,
                output='screen',
                name='health_monitor_terminal',
                # 터미널에서 실행되도록 gnome-terminal로 감싸기
                prefix='gnome-terminal --'
            )
        ]
    )

    # dual_teleop 키보드 조작 노드 실행 (가장 마지막에 실행)
    dual_teleop_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='battlebot_sim',
                executable='dual_teleop',
                name='dual_teleop',
                output='screen',
                prefix='gnome-terminal --'
            )
        ]
    )
   # damage_calculator, health_monitor, dual_teleop을 하나의 터미널에서 모두 실행 (f-string 문법 수정)


    # --- LaunchDescription 반환 부분 ---
    return LaunchDescription([
        # ... 기존 Gazebo 및 다른 노드들 ...
        gazebo_process,
        battlebot_1_rsp_node,
        battlebot_2_rsp_node,
        spawn_battlebot_1_entity,
        spawn_battlebot_2_entity,
        damage_calculator_node,
        moving_obstacle_controller_node,
        health_monitor_terminal,
        dual_teleop_node

    ])
