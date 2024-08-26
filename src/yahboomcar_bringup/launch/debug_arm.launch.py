from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import TimerAction, OpaqueFunction
# from launch import EmitEvent
# from launch import Shutdown
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.event_handlers import OnStateTransition

import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    vision_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('dnn_node_example'), 'launch'),
            '/dnn_node_example.launch.py']),
            # condition=IfCondition(LaunchConfiguration('debug_arm'))
        # output='screen'
    )

    servo_node = Node(
        package='servo_control',
        executable='servo',
        # condition=IfCondition(LaunchConfiguration('debug_arm'))
    )

    voice_node = Node(
        package='voice_broadcast',
        executable='voice',
    )

    stp23_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ldlidar'), 'launch'),
            '/stp23.launch.py']),
    )

    stp23l_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ldlidar'), 'launch'),
            '/stp23l.launch.py']),
    )

    return LaunchDescription([
        vision_node,
        servo_node,
        voice_node,
        # stp23_node
        # stp23l_node
    ])
