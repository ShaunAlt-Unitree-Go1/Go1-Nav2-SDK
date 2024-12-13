from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # defining nav2 and slam package paths
    path_nav2 = get_package_share_directory('nav2_bringup')
    path_sdk = get_package_share_directory('go1_nav2_sdk')
    path_slam = get_package_share_directory('slam_toolbox')

    # defining nav2 and slam launch includes
    include_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([path_nav2 + '/launch/bringup_launch.py']),
        launch_arguments = {
            'map': os.path.join(path_sdk, 'maps/map-test2.yaml'),
            'use_sim_time': 'true',
        }.items()
    )
    include_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([path_slam + '/launch/online_async_launch.py']),
        launch_arguments = {
            'use_sim_time': 'true'
        }.items()
    )

    # creating static transform node
    node_static_tf = Node(
        package = 'tf2_ros',
        namespace = '',
        executable = 'static_transform_publisher',
        name = 'tf_static_trunk_baselink',
        arguments = [
            '--frame-id trunk',
            '--child-frame-id base_link',
        ]
    )

    # create launch description
    return LaunchDescription([
        node_static_tf,
        include_nav2,
        include_slam,
    ])