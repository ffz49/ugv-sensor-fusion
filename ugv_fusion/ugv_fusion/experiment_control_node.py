import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist

class ExperimentControlNode(Node):
    def __init__(self):
        super().__init__('experiment_control_node')
        
        # 1. Listen to the fusion pipeline's 3D bounding boxes
        self.sub_fusion = self.create_subscription(MarkerArray, '/planning/diagnostic_fusion_markers', self.fusion_callback, 10)
        
        # 2. Publish to the AI movement topic (which twist_mux will read)
        self.pub_cmd = self.create_publisher(Twist, '/ai/cmd_vel', 10)
        
        # --- EXPERIMENT PARAMETERS (Easy to change here!) ---
        self.max_speed = 0.5       # Cruising speed (m/s)
        self.safe_distance = 2.0   # Start braking when obstacle is within 3 meters
        self.stop_distance = 0.8   # Full emergency stop at 1 meter
        self.robot_width = 0.6     # Only care about obstacles within +/- 0.6m of our center lane
        # ----------------------------------------------------
        
        self.get_logger().info("Experiment Control Brain Started. Awaiting obstacles...")

    def fusion_callback(self, msg):
        cmd = Twist()
        closest_obstacle_x = float('inf')

        # Parse the 3D coordinates of all fused objects
        for marker in msg.markers:
            if marker.type == marker.CUBE: # We only calculate distance using the boxes, not the floating text
                x = marker.pose.position.x # Distance straight ahead
                y = marker.pose.position.y # Distance left/right
                
                # Check if the object is IN FRONT of us and IN OUR LANE
                if x > 0 and abs(y) < self.robot_width:
                    if x < closest_obstacle_x:
                        closest_obstacle_x = x

        # Movement Logic State Machine
        if closest_obstacle_x == float('inf'):
            # STATE 1: Road is entirely clear
            cmd.linear.x = self.max_speed
            self.get_logger().info("Path clear. Cruising.", throttle_duration_sec=1.0)
            
        elif closest_obstacle_x <= self.stop_distance:
            # STATE 2: Obstacle is critically close
            cmd.linear.x = 0.0
            self.get_logger().warn(f"OBSTACLE AT {closest_obstacle_x:.2f}m! STOPPING.", throttle_duration_sec=0.5)
            
        elif closest_obstacle_x <= self.safe_distance:
            # STATE 3: Obstacle detected, apply proportional braking
            # Formula: v = v_max * (x - d_stop) / (d_safe - d_stop)
            speed_factor = (closest_obstacle_x - self.stop_distance) / (self.safe_distance - self.stop_distance)
            cmd.linear.x = self.max_speed * speed_factor
            self.get_logger().info(f"Obstacle at {closest_obstacle_x:.2f}m. Braking to {cmd.linear.x:.2f} m/s", throttle_duration_sec=0.5)

        # Keep steering straight for this initial test
        cmd.angular.z = 0.0
        
        # Publish the command
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ExperimentControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
