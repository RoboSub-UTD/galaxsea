# localization.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg = get_package_share_directory('galaxsea')
    ekf_config    = os.path.join(pkg, 'params', 'ekf.yaml')
    navsat_config = os.path.join(pkg, 'params', 'navsat.yaml')

    return LaunchDescription([


        # ─────────────────────────────────────────
        # 1. NavSat Transform Node
        #    Converts lat/lon → local x/y odometry
        #    Uses fixed GPS topic from node above
        # ─────────────────────────────────────────
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=[navsat_config],
            remappings=[
                ('imu',          '/wamv/sensors/imu/imu/data'),
                ('gps/fix',           '/wamv/sensors/gps/gps/fix'),
                ('odometry/filtered', '/odometry/filtered'),    # from EKF
            ]
        ),

        # ─────────────────────────────────────────
        # 2. EKF Node
        #    Fuses IMU + GPS odometry
        # ─────────────────────────────────────────
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config],
            remappings=[
                ('odometry/filtered', '/odometry/filtered'),
            ]
        ),
    ])