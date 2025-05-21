import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 패키지 디렉토리 경로 가져오기
    pkg_battlebot = get_package_share_directory('battlebot_spinner')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Gazebo 관련 변수 설정
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = os.path.join(pkg_battlebot, 'worlds', 'battlebot_arena.world')
    
    # URDF 파일 경로
    urdf_file = os.path.join(pkg_battlebot, 'urdf', 'battlebot.urdf.xacro')
    
    # Gazebo 클라이언트 실행
    gzclient = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # Battlebot 스폰
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'battlebot', '-file', urdf_file],
        output='screen'
    )
    
    # 로봇 상태 퍼블리셔
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': urdf_file}]
    )
    
    # 무기 컨트롤러
    weapon_controller = Node(
        package='battlebot_spinner',
        executable='weapon_controller',
        name='weapon_controller',
        output='screen'
    )
    
    # 텔레오퍼레이션 노드
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen'
    )
    
    # 전체 런처 구성
    return LaunchDescription([
        gzclient,
        spawn_entity,
        robot_state_publisher,
        #weapon_controller,
        teleop_node
    ])
