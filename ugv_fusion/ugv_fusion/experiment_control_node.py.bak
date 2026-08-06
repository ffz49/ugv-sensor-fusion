import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import tf2_ros
import sys
import math

class ExperimentControlNode(Node):
    def __init__(self):
        super().__init__('experiment_control_node')
        
        self.nav_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.countdown = 5
        self.get_logger().info("Waypoint Patrol Brain Initialized. Waiting for Nav2...")
        
        # This timer replaces the 'sleep' loop, allowing background TF data to stream in smoothly
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        if not self.nav_client.server_is_ready():
            self.get_logger().info("Still waiting for Nav2 action server to wake up...")
            return
            
        if self.countdown > 0:
            self.get_logger().info(f"Launching in {self.countdown}...")
            self.countdown -= 1
        else:
            self.timer.cancel()
            self.get_logger().info("GO! Executing Patrol Route.")
            self.send_patrol_route()

    def create_waypoint(self, x_offset, y_offset, yaw_offset, start_x, start_y, start_yaw):
        target_yaw = start_yaw + yaw_offset
        target_x = start_x + (x_offset * math.cos(start_yaw)) - (y_offset * math.sin(start_yaw))
        target_y = start_y + (x_offset * math.sin(start_yaw)) + (y_offset * math.cos(start_yaw))
        
        cy = math.cos(target_yaw * 0.5)
        sy = math.sin(target_yaw * 0.5)
        cp = 1.0
        sp = 0.0
        cr = 1.0
        sr = 0.0

        q_w = cr * cp * cy + sr * sp * sy
        q_x = sr * cp * cy - cr * sp * sy
        q_y = cr * sp * cy + sr * cp * sy
        q_z = cr * cp * sy - sr * sp * cy

        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = target_x
        pose.pose.position.y = target_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w
        
        return pose

    def send_patrol_route(self):
        self.get_logger().info("Locating UGV starting position in Odom frame...")
        try:
            # The node is fully spinning, so this will succeed instantly
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f"Failed to find UGV location: {e}")
            sys.exit(1)

        rx = trans.transform.translation.x
        ry = trans.transform.translation.y
        q = trans.transform.rotation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        ryaw = math.atan2(siny_cosp, cosy_cosp)

        # --- DEFINE YOUR PATROL ROUTE HERE ---
        # Format: create_waypoint(Forward_m, Left/Right_m, Turn_Radians, rx, ry, ryaw)
        
        wp1 = self.create_waypoint(5.0, 0.0, 0.0, rx, ry, ryaw)          # Go straight 10m
        #wp2 = self.create_waypoint(10.0, 5.0, math.pi/2, rx, ry, ryaw)    # Turn 90 deg left, go 5m
        wp2 = self.create_waypoint(8.0, 0.0, 0.0, rx, ry, ryaw)          # Continue straight to 8m

        # -------------------------------------

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = [wp1, wp2]

        self.get_logger().info(f"Sending {len(goal_msg.poses)} waypoints to Nav2.")
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 REJECTED the patrol route!")
            return

        self.get_logger().info("Patrol ACCEPTED. UGV is moving.")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("✅ Patrol Complete. All waypoints reached.")
        else:
            self.get_logger().error("🛑 NO WAY AROUND! The path is completely blocked. Stopping the UGV.")
            
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = ExperimentControlNode()
    
    # The timer handles the countdown entirely in the background while the node spins natively!
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
