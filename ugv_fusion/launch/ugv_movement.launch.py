import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('your_package_name')
    twist_mux_config = os.path.join(pkg_dir, 'config', 'twist_mux.yaml')
    
    # Locate the AgileX Ranger bringup package
    # Note: Verify if your specific AgileX package is named 'ranger_bringup' or 'ranger_mini_v2'
    ranger_bringup_dir = get_package_share_directory('ranger_bringup')
    ranger_launch_file = os.path.join(ranger_bringup_dir, 'launch', 'ranger.launch.py')

    return LaunchDescription([
        
        # 1. AgileX UGV Driver (starts the physical hardware)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ranger_launch_file)
        ),
        
        # 2. Joystick Driver (reads raw inputs from the physical controller)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        
        # 3. Teleop Node (converts raw joystick inputs to velocity commands)
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            # Requires holding a deadman switch (usually 'RB' or 'LB') to send teleop commands
            parameters=[{'require_enable_button': True}], 
            remappings=[
                # Remap the default output to our teleop topic for twist_mux
                ('/cmd_vel', '/cmd_vel_teleop') 
            ]
        ),
        
        # 4. Command Multiplexer (handles priorities)
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[twist_mux_config],
            remappings=[
                # The winner of the priority battle gets sent directly to the AgileX driver
                ('/cmd_vel_out', '/cmd_vel') 
            ]
        ),
        
        # 5. Your Autonomous Experiment Control Node
        Node(
            package='your_package_name',
            executable='experiment_control_node',
            name='experiment_control',
            output='screen',
            remappings=[
                # Remap your node's default output to the lower-priority auto topic
                ('/cmd_vel', '/cmd_vel_auto') 
            ]
        )
    ])
