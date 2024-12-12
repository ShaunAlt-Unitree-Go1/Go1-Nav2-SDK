from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
import os

def generate_launch_description():
    # defining launch config variables
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    rviz = LaunchConfiguration('rviz')

    # defining nav2 and slam package paths
    path_nav2 = get_package_share_directory('nav2_bringup')
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
        default_value = 'true',
        description = 'Whether to start RVIZ or not.'
    )

    # defining launch includes
    include_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(path_nav2, 'launch', 'bringup_launch.py')),
        launch_arguments = {
            'use_sim_time': 'true',
            'namespace': namespace,
            'params_file': params_file,
            'slam': 'true',
            'use_namespace': 'true',
        }.items()
    )
    include_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(path_nav2, 'launch', 'rviz_launch.py')),
        condition = IfCondition(rviz),
        launch_arguments = {
            'namespace': TextSubstitution(namespace),
            'use_namespace': 'true',
            'rviz_config': os.path.join(path_nav2, 'rviz', 'nav2_namespaced_view.rviz'),
        }.items()
    )
    # include_slam = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(path_slam + '/launch/online_async_launch.py'),
    #     launch_arguments = {
    #         'use_sim_time': 'true'
    #     }.items()
    # )

    # creating namespaced group action
    group = GroupAction([
        include_nav2,
        include_rviz,
        # include_slam,
    ])

    # create launch description
    return LaunchDescription([
        declare_name,
        declare_params_file,
        declare_rviz,
        group,
    ])