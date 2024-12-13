from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # defining launch config variables
    rviz = LaunchConfiguration('rviz')

    # defining nav2 and slam package paths
    path_nav2 = get_package_share_directory('nav2_bringup')
    path_sdk = get_package_share_directory('go1_nav2_sdk')
    path_slam = get_package_share_directory('slam_toolbox')

    # defining launch arguments
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value = 'True',
        description = 'Whether to start RVIZ or not.'
    )

    # defining nav2 and slam launch includes
    include_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([path_nav2 + '/launch/bringup_launch.py']),
        launch_arguments = {
            'map': os.path.join(path_sdk, 'maps/map-test2.yaml'),
            'use_sim_time': 'True',
        }.items()
    )
    # include_rviz = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(path_nav2, 'launch', 'rviz_launch.py')),
    #     condition = IfCondition(rviz),
    #     launch_arguments = {
    #         'namespace': '',
    #         'use_namespace': 'false',
    #         'rviz_config': os.path.join(path_nav2, 'rviz', 'nav2_namespaced_view.rviz'),
    #     }.items()
    # )
    include_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([path_slam + '/launch/online_async_launch.py']),
        launch_arguments = {
            'use_sim_time': 'True'
        }.items()
    )

    # creating static transform node
    node_rviz = ExecuteProcess(
        cmd = [
            'ros2', 'run', 'rviz2', 'rviz2',
            '-d', '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz',
        ],
        output = 'screen',
        condition = IfCondition(rviz)
    )
    node_static_tf = ExecuteProcess(
        cmd = [
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '--frame-id', 'trunk',
            '--child-frame-id', 'base_link',
        ],
        output = 'screen'
    )
    # node_static_tf = Node(
    #     package = 'tf2_ros',
    #     namespace = '',
    #     executable = 'static_transform_publisher',
    #     name = 'tf_static_trunk_baselink',
    #     arguments = [
    #         '--frame-id trunk',
    #         '--child-frame-id base_link',
    #     ]
    # )

    # create launch description
    return LaunchDescription([
        declare_rviz,
        node_static_tf,
        node_rviz,
        include_nav2,
        # include_rviz,
        include_slam,
    ])