import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'ugv_fusion'

    # Declare the toggle argument (Defaults to True)
    enable_dynamic_objects_arg = DeclareLaunchArgument(
        'enable_dynamic_objects',
        default_value='true',
        description='Set to false to disable the spatio-temporal object detection pipeline.'
    )

    enable_dynamic_objects = LaunchConfiguration('enable_dynamic_objects')

    return LaunchDescription([
        enable_dynamic_objects_arg,

        # Highway 1: Static BEV Evidential Pipeline (ALWAYS ON)
        Node(package=package_name, executable='visual_bev_node', name='visual_bev_node', output='screen'),
        Node(package=package_name, executable='radar_bev_node', name='radar_bev_node', output='screen',
             parameters=[{'vegetation_mode': False,
                          'hard_rcs_min': 10.0,
                          'soft_rcs_min': -5.0}]),
        Node(package=package_name, executable='radar_clarity_node', name='radar_clarity_node', output='screen'),
        Node(package=package_name, executable='visual_clarity_node', name='visual_clarity_node', output='screen'),
        Node(package=package_name, executable='dempster_shafer_fusion_node', name='dempster_shafer_fusion_node', output='screen'),

        # Highway 2: Dynamic Object Pipeline (CONDITIONALLY ON)
        Node(
            package=package_name,
            executable='radar_object_node',
            name='radar_object_node',
            output='screen',
            condition=IfCondition(enable_dynamic_objects)
        ),
        Node(
            package=package_name,
            executable='spatio_temporal_association_node',
            name='spatio_temporal_association_node',
            output='screen',
            condition=IfCondition(enable_dynamic_objects)
        )
    ])
