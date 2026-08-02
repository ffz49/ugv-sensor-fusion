import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import sys

class ExperimentControlNode(Node):
    def __init__(self):
        super().__init__('experiment_control_node')
        
        # Create an Action Client to talk to the Nav2 Stack
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # --- EXPERIMENT PARAMETERS ---
        self.goal_distance_x = 10.0  # Tell the UGV to drive 10 meters forward
        # -----------------------------
        
        self.get_logger().info("Experiment Control Brain Initialized. Nav2 Action Client Ready.")

    def send_goal(self):
        self.get_logger().info("Waiting for Nav2 server to wake up...")
        self.nav_client.wait_for_server()
        
        # Define the target destination
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        
        # We use 'base_link' so the goal is always strictly relative to the UGV's current position
        goal_msg.pose.header.frame_id = 'base_link'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set the target 10 meters straight ahead
        goal_msg.pose.pose.position.x = self.goal_distance_x
        goal_msg.pose.pose.position.y = 0.0
        goal_msg.pose.pose.position.z = 0.0
        
        # Point the UGV perfectly straight forward (Quaternion representation of 0 yaw)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending command to drive {self.goal_distance_x}m forward and avoid obstacles.")
        
        # Send the goal and wait for the result asynchronously
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 REJECTED the goal. Check costmaps!")
            return

        self.get_logger().info("Nav2 ACCEPTED the goal. UGV is moving.")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Experiment Complete. Destination reached.")
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = ExperimentControlNode()
    
    node.get_logger().info("Experiment Control Node started. Waiting for Nav2...")
    
    # Wait until Nav2 is online
    node.nav_client.wait_for_server()
    node.get_logger().info("Nav2 is ONLINE. Starting 5-second countdown to launch...")
    
    # Non-blocking 5 second countdown!
    import time
    for i in range(5, 0, -1):
        node.get_logger().info(f"Launching in {i}...")
        time.sleep(1)
        
    node.get_logger().info("GO!")
    node.send_goal()
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
