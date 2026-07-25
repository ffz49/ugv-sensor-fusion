import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. ZED X Camera Launch
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'zedx',
            'publish_tf': 'false',        # Forces ZED to use our TF tree
            'publish_map_tf': 'false'     # Forces ZED to use our TF tree
        }.items()
    )

    # 2. Smartmicro Radar Launch
    umrr_driver_dir = get_package_share_directory('umrr_ros2_driver')
    radar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(umrr_driver_dir, 'launch', 'radar.launch.py')
        )
    )

    # 3. Static Transform (base_link to zed_camera_link)
    static_tf_zed = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_zed',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.81', 
            '--roll', '-0.0110',
            '--pitch', '-0.3309',  # Camera tilts downward
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'zed_camera_link'
        ]
    )

    # 4. Static Transform (base_link to umrr)
    static_tf_radar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_radar',
        arguments=[
            '--x', '0.0', 
            '--y', '0.0', 
            '--z', '0.66',         # Radar height is exactly 66 cm
            '--roll', '0.0', 
            '--pitch', '0.0',      # Radar is perfectly level
            '--yaw', '0.0',      
            '--frame-id', 'base_link',
            '--child-frame-id', 'umrr'
        ]
    )

    return LaunchDescription([
        zed_launch,
        radar_launch,
        static_tf_zed,
        static_tf_radar
    ])
