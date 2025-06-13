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
    battlebot_rsp_node = Node(
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

    # damage_calculator_node
    damage_calculator_node = Node(
        package='battlebot_sim',
        executable='damage_calculator_node',
        name='damage_calculator',
        output='screen',
        parameters=[params_file_path]
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

    # dual_teleop 키보드 조작 노드 실행
    dual_teleop_node = TimerAction(
        period=10.0, #추가 딜레이
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


    return LaunchDescription([
        gazebo_process,
        battlebot_rsp_node,
        damage_calculator_node,
        spawn_battlebot_1_entity,
        spawn_battlebot_2_entity,
        moving_obstacle_controller_node,
        dual_teleop_node,
        # obstacle_rsp_node, # 두 번째 로봇이 있다면 활성화
        # spawn_obstacle_entity, # 두 번째 로봇이 있다면 활성화
    ])