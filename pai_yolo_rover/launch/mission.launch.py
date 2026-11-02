"""Full demo: Gazebo world + rover + YOLO search node (+ optional video recorder).

    ros2 launch pai_yolo_rover mission.launch.py
    ros2 launch pai_yolo_rover mission.launch.py targets:="['person','stop sign']" record_video:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('pai_yolo_rover')
    cfg = os.path.join(pkg, 'config', 'yolo_search.yaml')
    L = LaunchConfiguration

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('show_window', default_value='true',
                              description='show the annotated YOLO image in an OpenCV window'),
        DeclareLaunchArgument('record_video', default_value='false',
                              description='write demo video (overhead + YOLO view) with demo_recorder'),
        DeclareLaunchArgument('video_path', default_value=os.path.expanduser('~/pai_demo.avi')),
        DeclareLaunchArgument('targets', default_value="['stop sign', 'person', 'fire hydrant']"),
        DeclareLaunchArgument('model', default_value='yolov8n.pt'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
            launch_arguments={'gui': L('gui')}.items()),
        Node(package='pai_yolo_rover', executable='demo_recorder', output='screen',
             condition=IfCondition(L('record_video')),
             parameters=[{'use_sim_time': True, 'video_path': L('video_path')}]),
        # give Gazebo a few seconds to spawn the rover before the AI starts driving
        TimerAction(period=6.0, actions=[
            Node(package='pai_yolo_rover', executable='yolo_search', output='screen',
                 parameters=[cfg, {
                     'use_sim_time': True,
                     'show_window': ParameterValue(L('show_window'), value_type=bool),
                     'targets': ParameterValue(L('targets')),
                     'model': L('model'),
                 }]),
        ]),
    ])
