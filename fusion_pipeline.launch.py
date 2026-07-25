import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    workspace_src = os.path.expanduser('~/ros2_ws/src')
    
    return LaunchDescription([
        # 1. Visual BEV Node
        ExecuteProcess(
            cmd=['python3', os.path.join(workspace_src, 'visual_bev_node.py')],
            output='screen'
        ),
        # 2. Radar BEV Node
        ExecuteProcess(
            cmd=['python3', os.path.join(workspace_src, 'radar_bev_node.py')],
            output='screen'
        ),
        # 3. Radar Clarity Monitor (Beta)
        ExecuteProcess(
            cmd=['python3', os.path.join(workspace_src, 'radar_clarity_node.py')],
            output='screen'
        ),
        # 4. GPU-Accelerated Dempster-Shafer Fusion Node
        ExecuteProcess(
            cmd=['python3', os.path.join(workspace_src, 'dempster_shafer_fusion_node.py')],
            output='screen'
        )
    ])
