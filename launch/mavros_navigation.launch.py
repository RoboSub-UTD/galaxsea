from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    galaxsea = get_package_share_directory('galaxsea')
    slam_toolbox = get_package_share_directory('slam_toolbox')
    slam_params = '/root/roboboat_ws/src/galaxsea/params/slam_params.yaml'

    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{
                'fcu_url': '/dev/ttyACM0:115200', #might change, just guessed
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(galaxsea, 'launch', 'nav2.launch.py')
            ),
            launch_arguments={
                'thruster_params_type': 'mavros_thruster_controller',
                'use_sim_time': 'false',
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