from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
import os

def create_static_tf(context, namespace, *args, **kwargs):
    arg_namespace = context.perform_substitution(namespace)
    node_static_tf = ExecuteProcess(
        cmd = [
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '--frame-id', f'{arg_namespace}/trunk',
            '--child-frame-id', f'{arg_namespace}/base_link',
        ],
        output = 'screen'
    )
    return [node_static_tf]


def generate_launch_description():
    # defining launch config variables
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    rviz = LaunchConfiguration('rviz')

    # defining nav2 and slam package paths
    path_nav2 = get_package_share_directory('nav2_bringup')
    path_sdk = get_package_share_directory('go1_nav2_sdk')
    path_slam = get_package_share_directory('slam_toolbox')

    # defining launch arguments
    declare_name = DeclareLaunchArgument(
        'namespace',
        default_value = 'r1',
        description = 'Namespace of the Go1 Robot.'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value = os.path.join(path_nav2, 'params', 'nav2_multirobot_params_all.yaml'),
        description = 'Path to the parameters file for the Go1 Robot.'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value = 'True',
        description = 'Whether to start RVIZ or not.'
    )

    # defining launch includes
    include_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(path_nav2, 'launch', 'navigation_launch.py')),
        launch_arguments = {
            # 'map': os.path.join(path_sdk, 'maps/map-test2.yaml'),
            'namespace': namespace,
            'params_file': params_file,
            # 'slam': 'True',
            # 'use_namespace': 'True',
            'use_sim_time': 'True',
        }.items()
    )
    include_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(path_nav2, 'launch', 'rviz_launch.py')),
        condition = IfCondition(rviz),
        launch_arguments = {
            'namespace': namespace,
            'use_namespace': 'True',
            'rviz_config': os.path.join(path_nav2, 'rviz', 'nav2_namespaced_view.rviz'),
        }.items()
    )

    # node_static_tf = Node(
    #     package = 'tf2_ros',
    #     namespace = TextSubstitution(namespace),
    #     executable = 'static_transform_publisher',
    #     name = 'tf_static_' + TextSubstitution(namespace) + '_trunk_baselink',
    #     arguments = [
    #         '--frame-id ' + TextSubstitution(namespace) + '/trunk',
    #         '--child-frame-id ' + TextSubstitution(namespace) + '/base_link',
    #     ]
    # )
    include_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path_slam + '/launch/online_async_launch.py'),
        launch_arguments = {
            'use_sim_time': 'true'
        }.items()
    )

    # creating namespaced group action
    # group = GroupAction([
    #     include_nav2,
    #     include_rviz,
    #     # include_slam,
    # ])

    # create launch description
    return LaunchDescription([
        declare_name,
        declare_params_file,
        declare_rviz,
        OpaqueFunction(function = create_static_tf, args = [namespace]),
        include_nav2,
        include_rviz,
        include_slam,
    ])