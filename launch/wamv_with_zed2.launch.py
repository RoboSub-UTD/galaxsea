from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import EnvironmentVariable
import os

def generate_launch_description():
    model_path = os.path.join(
        get_package_share_directory('galaxsea'),
        'sensors',
        'urdf'
    )
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[model_path, ':', EnvironmentVariable('GZ_SIM_RESOURCE_PATH')]    
    )
    wamv_zed2_xacro = os.path.join(
        get_package_share_directory('galaxsea'),
        'sensors',
        'urdf',
        'wamv_with_zed2.xacro'
    )

    # Let VRX run xacro with its expected arguments; pass the custom xacro path directly.
    vrx_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('vrx_gz'),
                'launch',
                'competition.launch.py'
            )
        ),
        launch_arguments={
            'name': 'wamv',
            'urdf': wamv_zed2_xacro,
        }.items()
    )

    # Bridge ZED2 stereo, depth, point cloud, and IMU outputs into ROS 2 topics.
    zed2_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/zed2/left/image_rect_color@sensor_msgs/msg/Image@gz.msgs.Image',
            '/zed2/left/image_rect_color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/zed2/right/image_rect_color@sensor_msgs/msg/Image@gz.msgs.Image',
            '/zed2/right/image_rect_color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/zed2/depth/depth_registered@sensor_msgs/msg/Image@gz.msgs.Image',
            '/zed2/depth/depth_registered/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/zed2/point_cloud/cloud_registered/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/zed2/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ]
    )

    return LaunchDescription([
        set_resource_path,
        vrx_launch,
        zed2_camera_bridge
    ])
