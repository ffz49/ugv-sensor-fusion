import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('ugv_fusion')
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    
    # AgileX Ranger/Bunker bringup
    bunker_bringup_dir = get_package_share_directory('bunker_base')
    bunker_launch_file = os.path.join(bunker_bringup_dir, 'launch', 'bunker_base.launch.py')
    
    # Official Nav2 bringup
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')

    return LaunchDescription([
        
        # 1. AgileX UGV Driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bunker_launch_file),
            launch_arguments={'port_name': 'can1'}.items()
        ),
        
        # 2. Nav2 Stack (Loads your MPPI Costmap YAML)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={'params_file': nav2_params_file}.items()
        ),
        
        # 3. Your Autonomous Experiment Node (The Commander)
        Node(
            package='ugv_fusion',
            executable='experiment_control_node',
            name='experiment_control',
            output='screen'
        )
    ])
