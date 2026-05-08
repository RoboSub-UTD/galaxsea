from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    galaxsea = get_package_share_directory('galaxsea')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    slam_toolbox = get_package_share_directory('slam_toolbox')
    nav2_params = '/root/roboboat_ws/src/galaxsea/params/nav2_params.yaml'
    slam_params = '/root/roboboat_ws/src/galaxsea/params/slam_params.yaml'
    thruster_params = '/root/roboboat_ws/src/galaxsea/params/auto_thruster_move_params.yaml'

    return LaunchDescription([

        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{
                'fcu_url': 'udp://127.0.0.1:14550@14555',
            }],
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
            name='sim_thruster_controller',
            output='screen',
            parameters=[thruster_params],
        ),

        Node(
            package='galaxsea',
            executable='auto_thruster_move',
            name='mavros_thruster_controller',
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