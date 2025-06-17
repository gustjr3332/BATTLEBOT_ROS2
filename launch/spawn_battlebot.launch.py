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
    params_file_path = os.path.join(pkg_share, 'config', 'params.yaml')

    # GAZEBO_MODEL_PATH 설정
    models_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    package_models_path = os.path.join(pkg_share, 'models')
    if package_models_path not in models_path:
        os.environ['GAZEBO_MODEL_PATH'] = package_models_path + ':' + models_path
    
    print(f"DEBUG: GAZEBO_MODEL_PATH set to: {os.environ.get('GAZEBO_MODEL_PATH')}")

    # --- [수정] Gazebo 실행 시 반드시 ROS 연동 플러그인을 로드하도록 보장 ---
    gazebo_process = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so', 
            '-s', 'libgazebo_ros_factory.so',
            world_file
        ],
        output='screen'
    )

    # URDF 파일 내용 읽기
    with open(urdf_file, 'r') as file:
        battlebot_1_urdf_content = file.read()
    with open(urdf_file_2, 'r') as file:
        battlebot_2_urdf_content = file.read()

    # 각 로봇의 robot_state_publisher
    battlebot_1_rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='battlebot_1_state_publisher', 
        namespace='battlebot_1',
        output='screen',
        parameters=[{'robot_description': battlebot_1_urdf_content}]
    )
    battlebot_2_rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='battlebot_2_state_publisher',
        namespace='battlebot_2',
        output='screen',
        parameters=[{'robot_description': battlebot_2_urdf_content}]
    )

    # --- [수정] 스폰 전 대기 시간을 8초, 9초로 늘려 안정성 확보 ---
    spawn_battlebot_1_entity = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', 'battlebot_1', '-file', urdf_file, '-x', '0', '-y', '4', '-z', '0.3'],
                output='screen'
            )
        ]
    )
    spawn_battlebot_2_entity = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', 'battlebot_2', '-file', urdf_file_2, '-x', '0', '-y', '-4', '-z', '0.3'],
                output='screen'
            )
        ]
    )

    # 다른 노드들 (조종 노드는 여기서 실행하지 않음)
    moving_obstacle_controller_node = Node(
        package='battlebot_sim',
        executable='moving_obstacle_controller',
        name='moving_obstacle_controller',
        output='screen'
    )
    damage_calculator_node = Node(
        package='battlebot_sim',
        executable='damage_calculator_node',
        name='damage_calculator',
        output='screen',
        parameters=[params_file_path]
    )
   
    return LaunchDescription([
        gazebo_process,
        battlebot_1_rsp_node,
        battlebot_2_rsp_node,
        spawn_battlebot_1_entity,
        spawn_battlebot_2_entity,
        moving_obstacle_controller_node,
        damage_calculator_node,
    ])