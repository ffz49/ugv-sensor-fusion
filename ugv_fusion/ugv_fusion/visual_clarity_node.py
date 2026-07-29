import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import message_filters
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np

class VisualClarityNode(Node):
    def __init__(self):
        super().__init__('visual_clarity_node')
        self.sub_rgb = Subscriber(self, Image, '/zed/zed_node/rgb/color/rect/image')
        self.sub_depth = Subscriber(self, Image, '/zed/zed_node/depth/depth_registered')
        self.sync = ApproximateTimeSynchronizer([self.sub_rgb, self.sub_depth], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.sync_callback)
        self.pub_clarity = self.create_publisher(Float32, '/environment/visual_clarity', 10)
        
        self.get_logger().info("Blended Clarity Node (Entropy + Contrast) Started. CPU Optimized.")

    def ros_image_to_numpy(self, msg, stride=4):
        """
        Stride=8 processes only ~8,000 pixels instead of 2 million.
        This cures the 100% CPU bottleneck instantly.
        """
        if msg.encoding in ['bgra8', '8UC4', 'rgba8']:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 4))[::stride, ::stride]
            np_gray = np.dot(np_arr[..., :3].astype(np.float32), np.array([0.114, 0.587, 0.299], dtype=np.float32))
            return np_gray.astype(np.uint8)
        elif msg.encoding in ['bgr8', 'rgb8']:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))[::stride, ::stride]
            np_gray = np.dot(np_arr[..., :3].astype(np.float32), np.array([0.114, 0.587, 0.299], dtype=np.float32))
            return np_gray.astype(np.uint8)
        elif msg.encoding in ['32FC1']:
            return np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))[::stride, ::stride]
        else:
            raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    def sync_callback(self, msg_rgb, msg_depth):
        try:
            # Enforcing stride=8 for maximum CPU relief
            cpu_rgb = self.ros_image_to_numpy(msg_rgb, stride=8)
            cpu_depth = self.ros_image_to_numpy(msg_depth, stride=8)
        except Exception as e:
            self.get_logger().error(f"Buffer Error: {e}")
            return

        # 1. Depth Mask (0.2m to 5.0m)
        valid_depth_mask = (cpu_depth >= 0.2) & (cpu_depth <= 5.0)
        roi_pixels = cpu_rgb[valid_depth_mask]

        # 2. DVE Trap: If the depth map goes blind (or object is closer than 0.2m)
        if len(roi_pixels) < 200: 
            self.publish_alpha(0.0)
            return

        # 3. METRIC A: Histogram Entropy
        hist, _ = np.histogram(roi_pixels, bins=256, range=(0, 256))
        hist = hist.astype(np.float32) / hist.sum()
        non_zero_hist = hist[hist > 0] 
        entropy = -np.sum(non_zero_hist * np.log2(non_zero_hist))

        # 4. METRIC B: Pixel Contrast (StdDev)
        std_dev = np.std(roi_pixels)

        # 5. Calculate Smooth, Linear Confidence Weights
        # Widened the ranges significantly to guarantee smooth "in-between" values
        alpha_ent = np.clip((entropy - 4.0) / (7.5 - 4.0), 0.0, 1.0)
        alpha_std = np.clip((std_dev - 10.0) / (60.0 - 10.0), 0.0, 1.0)

        # 6. Blended Output
        alpha = float((alpha_ent + alpha_std) / 2.0)

        self.publish_alpha(alpha)
        # Check this debug output to see the smooth sliding scale in action!
        self.get_logger().debug(f"Entropy: {entropy:.2f} | StdDev: {std_dev:.2f} | Blended Alpha: {alpha:.2f}")

    def publish_alpha(self, alpha_value):
        msg = Float32()
        msg.data = alpha_value
        self.pub_clarity.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisualClarityNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
