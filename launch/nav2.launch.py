from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_bringup = get_package_share_directory('nav2_bringup')
    nav2_params = '/root/roboboat_ws/src/galaxsea/params/nav2_params.yaml'

    return LaunchDescription([
        Node(
            package='galaxsea',
            executable='lidar_preprocess',
            name='lidar_preprocess',
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': nav2_params,
            }.items(),
        ),
    ])