# spawn_spinner_bot.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    gazebo_ros_pkg_path = get_package_share_directory('gazebo_ros')
    gazebo_launch_file = os.path.join(gazebo_ros_pkg_path, 'launch', 'gazebo.launch.py')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
    )

    pkg_path = get_package_share_directory('battlebot_spinner')
    xacro_file = os.path.join(pkg_path, 'urdf', 'spinner_bot.xacro')

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}] # use_sim_time 추가
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'spinner_bot',
                   '-x', '0.0',  
                   '-y', '0.0',
                   '-z', '0.1'],
        output='screen'
    )

    return LaunchDescription([
        gazebo, 
        robot_state_publisher_node,
        spawn_entity_node,
    ])
