from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap
import os

def create_tfs(context, namespace, *args, **kwargs):
    arg_namespace = context.perform_substitution(namespace)
    node_static_tf = ExecuteProcess(
        cmd = [
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '--frame-id', f'{arg_namespace}/trunk',
            '--child-frame-id', f'{arg_namespace}/base_link',
        ],
        output = 'screen'
    )
    node_tf_republish = Node(
        package = 'go1_nav2_sdk',
        executable = 'tf_republish',
        name = 'tf_republish_' + arg_namespace,
        parameters = [{
            'source': '',
            'target': arg_namespace,
        }]
    )
    return [
        node_tf_republish,
        node_static_tf,
    ]


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
        # default_value = os.path.join(path_nav2, 'params', 'nav2_multirobot_params_all.yaml'),
        # default_value = os.path.join(path_nav2, 'params', 'nav2_params.yaml'),
        # default_value = os.path.join(path_sdk, 'params', 'r1.yaml'),
        default_value = os.path.join(path_sdk, 'params', 'r1.yaml'),
        description = 'Path to the parameters file for the Go1 Robot.'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value = 'True',
        description = 'Whether to start RVIZ or not.'
    )

    # defining launch includes
    include_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(path_nav2, 'launch', 'bringup_launch.py')),
        launch_arguments = {
            'map': os.path.join(path_sdk, 'maps', 'map-test2.yaml'),
            'namespace': namespace,
            'params_file': params_file,
            # 'slam': 'True',
            'use_namespace': 'True',
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
    node_rviz = ExecuteProcess(
        cmd = [
            'ros2', 'run', 'rviz2', 'rviz2',
            '-d', '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz',
        ],
        output = 'screen',
        condition = IfCondition(rviz)
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
            'use_sim_time': 'True',
            'slam_params_file': params_file,
        }.items()
    )

    # creating namespaced group action
    group_slam = GroupAction([
        # PushRosNamespace(namespace = namespace),
        SetRemap(src='/map', dst='/r1/map'),
        SetRemap(src='/map_metadata', dst='/r1/map_metadata'),
        SetRemap(src='/map_updates', dst='/r1/map_updates'),
        SetRemap(src='/scan', dst='/r1/scan'),
        include_slam,
    ])

    # namespaced rviz group
    group_rviz = GroupAction([
        # PushRosNamespace(namespace = namespace),
        # SetRemap(src='/follow_waypoints', dst='/r1/follow_waypoints'),
        # SetRemap(src='/navigate_through_poses', dst='/r1/navigate_through_poses'),
        # SetRemap(src='/navigate_to_pose', dst='/r1/navigate_to_pose'),
        node_rviz,
    ])

    # create launch description
    return LaunchDescription([
        declare_name,
        declare_params_file,
        declare_rviz,
        OpaqueFunction(function = create_tfs, args = [namespace]),
        include_nav2,
        include_rviz,
        # node_rviz,
        group_rviz,
        group_slam,
    ])