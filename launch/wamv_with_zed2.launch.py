from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import EnvironmentVariable
import os
import xacro

def generate_launch_description():
    vrx_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('vrx_gz'),
                'launch',
                'competition.launch.py'
            )
        )
    )

    model_path = os.path.join(
        get_package_share_directory('galaxsea26'),
        'sensors',
        'urdf'
    )
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[model_path, ':', EnvironmentVariable('GZ_SIM_RESOURCE_PATH')]    
    )
    wamv_zed2_xacro = os.path.join(
        get_package_share_directory('galaxsea26'),
        'sensors',
        'urdf',
        'wamv_with_zed2.xacro'
    )

    # vrx_gz_path = get_package_share_directory('vrx_gz')
    # os.environ['ROS_PACKAGE_PATH'] = f"{vrx_gz_path}:{os.environ.get('ROS_PACKAGE_PATH', '')}"

    # 🧩 Convert Xacro to SDF on the fly
    robot_description_config = xacro.process_file(wamv_zed2_xacro)
    robot_description = robot_description_config.toxml()
    # print("test print" + os.environ['GZ_SIM_RESOURCE_PATH'])
    # Write the expanded file to /tmp for Gazebo
    tmp_path = '/tmp/wamv_with_zed2.sdf'
    with open(tmp_path, 'w') as f:
        f.write(robot_description)

    spawn_combined = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-name', 'wamv_with_zed2',
                    # '-world', 'vrx_2023',
                    '-file', tmp_path,
                    '-x', '0', '-y', '0', '-z', '0.1'
                ]
            )
        ]
    )

    return LaunchDescription([
        set_resource_path,
        vrx_launch,
        spawn_combined
    ])
