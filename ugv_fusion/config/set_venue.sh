#!/bin/bash
set -e
CFG=~/ros2_ws/src/ugv_fusion/config
NODE=~/ros2_ws/src/ugv_fusion/ugv_fusion/experiment_control_node.py
LAUNCH=~/ros2_ws/src/ugv_fusion/launch/fusion_pipeline.launch.py
case "$1" in
  indoor)  W1=8.0; W2=15.0; VEG=False ;;
  outdoor) W1=50.0; W2=90.0; VEG=True  ;;
  *) echo "usage: set_venue.sh indoor|outdoor"; exit 1 ;;
esac
cp "$CFG/nav2_params_$1.yaml" "$CFG/nav2_params.yaml"
sed -i "s|^.*# VENUE_WP1\$|            self.declare_parameter('waypoint_1_x', $W1)   # VENUE_WP1|" "$NODE"
sed -i "s|^.*# VENUE_WP2\$|            self.declare_parameter('waypoint_2_x', $W2)   # VENUE_WP2|" "$NODE"
sed -i "s|'vegetation_mode': [A-Za-z]*|'vegetation_mode': $VEG|" "$LAUNCH"
cd ~/ros2_ws && colcon build --packages-select ugv_fusion
echo ""
echo "=== Venue: $1 | waypoints ${W1}m, ${W2}m | vegetation_mode=$VEG ==="
echo "Now run: source ~/ros2_ws/install/setup.bash"
