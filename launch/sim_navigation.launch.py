from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    galaxsea = get_package_share_directory('galaxsea')
    slam_toolbox = get_package_share_directory('slam_toolbox')
    slam_params = '/root/roboboat_ws/src/galaxsea/params/slam_params.yaml'

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(galaxsea, 'launch', 'nav2.launch.py')
            ),
            launch_arguments={
                'thruster_params_type': 'sim_thruster_controller',
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(galaxsea, 'launch', 'obstacle_type_map.launch.py')
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(galaxsea, 'launch', 'localization.launch.py')
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'slam_params_file': slam_params,
            }.items(),
        ),
    ])