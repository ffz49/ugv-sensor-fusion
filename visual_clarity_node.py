import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisualClarityNode(Node):
    def __init__(self):
        super().__init__('visual_clarity_node')
        
        # Subscribe to the raw RGB image from the ZED X
        self.sub_image = self.create_subscription(
            Image, '/zed/zed_node/rgb/color/rect/image', self.image_callback, 10)
        
        # Publish the final clarity weight (alpha)
        self.pub_clarity = self.create_publisher(Float32, '/environment/visual_clarity', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info("Visual Clarity (Entropy) Node Started.")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to an OpenCV grayscale matrix
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        # 1. Calculate the histogram of the grayscale image
        hist = cv2.calcHist([cv_image], [0], None, [256], [0, 256])
        
        # 2. Normalize the histogram to get probabilities (p_i)
        hist = hist.ravel() / hist.sum()

        # 3. Calculate Shannon Entropy: H = -sum(p * log2(p))
        non_zero_hist = hist[hist > 0] # Avoid log2(0)
        entropy = -np.sum(non_zero_hist * np.log2(non_zero_hist))

        # 4. Normalize the entropy to an alpha weight [0.0 to 1.0]
        # A perfectly sharp outdoor image usually has an entropy of ~7.5
        # A completely washed out/foggy image drops below 5.0
        max_expected_entropy = 7.5 
        min_expected_entropy = 5.0
        
        alpha = (entropy - min_expected_entropy) / (max_expected_entropy - min_expected_entropy)
        
        # Clip the result so it strictly stays between 0.0 and 1.0
        alpha = float(np.clip(alpha, 0.0, 1.0))

        # Publish the weight
        clarity_msg = Float32()
        clarity_msg.data = alpha
        self.pub_clarity.publish(clarity_msg)

        # Log it to the terminal so we can see it working
        self.get_logger().info(f"Entropy: {entropy:.2f} | Fusion Weight (Alpha): {alpha:.2f}")

def main():
    rclpy.init()
    node = VisualClarityNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
