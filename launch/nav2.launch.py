from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_bringup = get_package_share_directory('nav2_bringup')
    nav2_params = '/root/roboboat_ws/src/galaxsea/params/nav2_params.yaml'
    thruster_params = '/root/roboboat_ws/src/galaxsea/params/auto_thruster_move_params.yaml'

    thruster_params_type = LaunchConfiguration('thruster_params_type')

    return LaunchDescription([

        DeclareLaunchArgument(
            'thruster_params_type',
            default_value='sim_thruster_controller'
        ),

        Node(
            package='galaxsea',
            executable='lidar_preprocess',
            name='lidar_preprocess',
            output='screen',
        ),

        Node(
            package='galaxsea',
            executable='auto_thruster_move',
            name=thruster_params_type,
            output='screen',
            parameters=[thruster_params],
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