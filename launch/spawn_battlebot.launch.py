from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('battlebot_sim')
    world_file = os.path.join(pkg_share, 'worlds', 'arena.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'battlebot.urdf')
    

    params_file_path = os.path.join(pkg_share, 'config', 'params.yaml')

    # Gazebo 시뮬레이터를 직접 실행하는 ExecuteProcess
    gazebo_process = ExecuteProcess(
        cmd=[
            'gazebo', # 시스템 PATH에 있는 'gazebo' 실행 파일 사용
            '--verbose',
            world_file,
            '-s', 'libgazebo_ros_factory.so' # Gazebo ROS 인터페이스 플러그인 로드
        ],
        output='screen'
    )

    # battlebot의 robot_state_publisher
    # URDF 파일을 읽어와 robot_description 파라미터로 설정
    with open(urdf_file, 'r') as file:
        battlebot_urdf_content = file.read()

    battlebot_rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='battlebot_state_publisher', # 이름 충돌 방지를 위해 변경
        output='screen',
        parameters=[{'robot_description': battlebot_urdf_content}]
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
    spawn_battlebot_entity = TimerAction(
        period=5.0, # Gazebo가 완전히 로드될 시간을 충분히 줌
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'battlebot', # URDF의 <robot name="battlebot">과 일치
                    '-file', urdf_file,
                    '-x', '0', '-y', '4', '-z', '0.3'
                ],
                output='screen'
            )
        ]
    )

    # moving_obstacle_controller 실행 (로봇 스폰 후 더 늦게 실행)
    moving_obstacle_controller_node = TimerAction(
        period=10.0, # battlebot 스폰 후 추가 딜레이
        actions=[
            Node(
                package='battlebot_sim',
                executable='moving_obstacle_controller',
                name='moving_obstacle_controller',
                output='screen'
            )
        ]
    )

    # 만약 두 번째 로봇 (예: moving_obstacle)도 Gazebo에 스폰해야 한다면 여기에 추가
    # 예를 들어, moving_obstacle도 URDF 모델이 있다면
    # with open(obstacle_urdf_file, 'r') as file:
    #     obstacle_urdf_content = file.read()
    #
    # obstacle_rsp_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='obstacle_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': obstacle_urdf_content}]
    # )
    #
    # spawn_obstacle_entity = TimerAction(
    #     period=6.0, # battlebot 스폰보다 조금 뒤
    #     actions=[
    #         Node(
    #             package='gazebo_ros',
    #             executable='spawn_entity.py',
    #             arguments=[
    #                 '-entity', 'moving_obstacle', # moving_obstacle 모델 이름
    #                 '-file', obstacle_urdf_file,
    #                 '-x', '0', '-y', '0', '-z', '0.3'
    #             ],
    #             output='screen'
    #         )
    #     ]
    # )


    return LaunchDescription([
        gazebo_process,
        battlebot_rsp_node,
        damage_calculator_node,
        spawn_battlebot_entity,
        moving_obstacle_controller_node,
        # obstacle_rsp_node, # 두 번째 로봇이 있다면 활성화
        # spawn_obstacle_entity, # 두 번째 로봇이 있다면 활성화
    ])