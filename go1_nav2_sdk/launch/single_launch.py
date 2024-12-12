from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # defining nav2 and slam package paths
    path_nav2 = get_package_share_directory('nav2_bringup')
    path_slam = get_package_share_directory('slam_toolbox')

    # defining nav2 and slam launch includes
    include_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([path_nav2 + '/launch/bringup_launch.py']),
        launch_arguments = {
            'use_sim_time': 'true'
        }.items()
    )
    include_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([path_slam + '/launch/online_async_launch.py']),
        launch_arguments = {
            'use_sim_time': 'true'
        }.items()
    )

    # create launch description
    return LaunchDescription([
        IncludeLaunchDescription([
            include_nav2,
            include_slam,
        ])
    ])