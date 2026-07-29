import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Replace 'ugv_fusion' with whatever you name your actual ROS 2 package
    package_name = 'ugv_fusion'

    return LaunchDescription([
        # 1. Visual BEV Node
        Node(
            package=package_name,
            executable='visual_bev_node',
            name='visual_bev_node',
            output='screen'
        ),
        # 2. Radar BEV Node
        Node(
            package=package_name,
            executable='radar_bev_node',
            name='radar_bev_node',
            output='screen'
        ),
        # 3. Radar Clarity Monitor
        Node(
            package=package_name,
            executable='radar_clarity_node',
            name='radar_clarity_node',
            output='screen'
        ),
        # 4. Visual Clarity Monitor
        Node(
            package=package_name,
            executable='visual_clarity_node',
            name='visual_clarity_node',
            output='screen'
        ),
        # 5. GPU-Accelerated Dempster-Shafer Fusion Node
        Node(
            package=package_name,
            executable='dempster_shafer_fusion_node',
            name='dempster_shafer_fusion_node',
            output='screen'
        ),
        # Highway 2: Dynamic Object Pipeline
        Node(
            package='ugv_fusion',
            executable='radar_object_node',
            name='radar_object_node',
            output='screen'
        ),
        Node(
            package='ugv_fusion',
            executable='spatio_temporal_association_node',
            name='spatio_temporal_association_node',
            output='screen'
        )
    ])
