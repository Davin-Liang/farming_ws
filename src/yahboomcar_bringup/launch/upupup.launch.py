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

print("---------------------robot_type = x1---------------------")

def generate_launch_description():
    urdf_tutorial_path = get_package_share_path('yahboomcar_description')
    default_model_path = urdf_tutorial_path / 'urdf/yahboomcar_X3.urdf'

    gui_arg = DeclareLaunchArgument(name='gui', 
                                    default_value='false', 
                                    choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', 
                                      default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    pub_odom_tf_arg = DeclareLaunchArgument('pub_odom_tf', 
                                            default_value='false',
                                            description='Whether to publish the tf from the original odom to the base_footprint')
    
    # debug_arm_arg = DeclareLaunchArgument(name='debug_arm', 
    #                                 default_value='false', 
    #                                 choices=['true', 'false'],
    #                                 description='Flag to enable debug arm.')


    driver_node = Node(
        package='yahboomcar_bringup',
        executable='Mcnamu_driver_x1',
        # condition=UnlessCondition(LaunchConfiguration('debug_arm'))
    )

    base_node = Node(
        package='yahboomcar_base_node',
        executable='base_node_x1',
        parameters=[{'pub_odom_tf': LaunchConfiguration('pub_odom_tf')}],
        # condition=UnlessCondition(LaunchConfiguration('debug_arm'))
    )

    yahboom_joy_node = Node(
        package='yahboomcar_ctrl',
        executable='yahboom_joy_X3',
        # condition=UnlessCondition(LaunchConfiguration('debug_arm'))
    )

    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher', 
        emulate_tty=True,
        arguments="0.0 0.0 0.05325 0.0 0.0 0.0 /base_footprint /base_link".split(' '),
        # condition=UnlessCondition(LaunchConfiguration('debug_arm'))
    )

    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher', 
        emulate_tty=True,
        arguments="0.0 0.0 0.0 0.0 0.0 0.0 /base_link /imu_link".split(' '),
        # condition=UnlessCondition(LaunchConfiguration('debug_arm'))
    )
    
    ekf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robot_localization'), 'launch'),
            '/ekf_x1_x3_launch.py']),
        # condition=UnlessCondition(LaunchConfiguration('debug_arm'))
    )

    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ldlidar'), 'launch'),
            '/stp23l.launch.py']),
        # condition=UnlessCondition(LaunchConfiguration('debug_arm'))
    )

    vision_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('dnn_node_example'), 'launch'),
            '/dnn_node_example.launch.py']),
            # condition=IfCondition(LaunchConfiguration('debug_arm'))
        # output='screen'
    )

    fdilink_ahrs_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('fdilink_ahrs'), 'launch'),
            '/ahrs_driver.launch.py']),
        # condition=UnlessCondition(LaunchConfiguration('debug_arm'))
    )

    servo_node = Node(
        package='yahboomcar_bringup',
        executable='servo',
        # condition=IfCondition(LaunchConfiguration('debug_arm'))
    )

    critical_nodes = [driver_node, base_node, lidar_node, vision_node]

    # event_handlers = [
    #     RegisterEventHandler(
    #         OnStateTransition(
    #             target_lifecycle_node=node,
    #             goal_state='inactive',
    #             entities=[
    #                 LogInfo(msg=f"A critical node [{node.executable}] has stopped. Shutting down..."),
    #                 EmitEvent(event=Shutdown())
    #             ]
    #         )
    #     )
    #     for node in critical_nodes
    # ]

    def launch_other_nodes(context):
        return [
            driver_node,
            base_node,
            ekf_node,
            yahboom_joy_node,
            base_footprint_tf,
            imu_tf,
            lidar_node,
            vision_node,
            servo_node,
            # *event_handlers,
        ]

    delayed_start = TimerAction(
        period=5.0,  # 延迟5秒启动其他节点
        actions=[OpaqueFunction(function=launch_other_nodes)]
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        pub_odom_tf_arg,
        fdilink_ahrs_node,
        # debug_arm_arg,
        
        delayed_start,
    ])
