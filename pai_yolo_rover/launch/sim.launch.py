"""Gazebo (Classic 11) world + rover only. Use this to drive the rover manually:
    ros2 launch pai_yolo_rover sim.launch.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('pai_yolo_rover')
    gazebo_ros = get_package_share_directory('gazebo_ros')
    world = os.path.join(pkg, 'worlds', 'patrol_room.world')
    xacro_file = os.path.join(pkg, 'urdf', 'rover.urdf.xacro')
    model_path = os.path.join(pkg, 'models')
    existing = os.environ.get('GAZEBO_MODEL_PATH', '')

    gui = LaunchConfiguration('gui')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='start gzclient'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path + (':' + existing if existing else '')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world, 'verbose': 'false'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(gui)),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-topic', 'robot_description', '-entity', 'yolo_rover',
                        '-x', '-0.5', '-y', '-0.5', '-z', '0.02', '-Y', '0.0'],
             output='screen'),
    ])
