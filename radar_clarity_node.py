import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class RadarClarityNode(Node):
    def __init__(self):
        super().__init__('radar_clarity_node')
        
        # Subscribe to the Smartmicro Radar point cloud
        self.sub_radar = self.create_subscription(
            PointCloud2, '/smart_radar/port_targets_0', self.radar_callback, 10)
            
        # Publish the Beta Weight (0.0 = Blind, 1.0 = Perfect Clarity)
        self.pub_beta = self.create_publisher(Float32, '/environment/radar_beta', 10)
        
        # Tuning parameters for radar health
        self.min_expected_targets = 5     # If we see fewer than this, radome might be blocked
        self.min_avg_intensity = 10.0     # Baseline SNR threshold for healthy reflections

    def radar_callback(self, msg):
        # Extract the intensity (SNR/RCS) field from the radar targets
        # The Smartmicro driver usually maps signal strength to the 'intensity' field
        pts_list = pc2.read_points_list(msg, field_names=("snr",), skip_nans=True)
        
        target_count = len(pts_list)
        
        if target_count == 0:
            # Complete signal loss (mud coverage or hardware failure)
            beta = 0.0
            avg_intensity = 0.0
        else:
            # Calculate the average signal strength of the current environment
            intensities = np.array(pts_list, dtype=np.float32)
            avg_intensity = np.mean(intensities)
            
            # Calculate Health Weight based on target volume and signal strength
            # This is a simplified linear model; you can expand this for your thesis
            target_health = min(target_count / self.min_expected_targets, 1.0)
            signal_health = min(avg_intensity / self.min_avg_intensity, 1.0)
            
            # The Beta weight requires both a healthy number of targets AND healthy signal strength
            beta = target_health * signal_health
            
        # Ensure beta stays strictly bounded between 0.0 and 1.0
        beta = float(np.clip(beta, 0.0, 1.0))

        # Publish the Beta weight for the Dempster-Shafer Fusion Node
        beta_msg = Float32()
        beta_msg.data = beta
        self.pub_beta.publish(beta_msg)
        
        self.get_logger().info(f"Targets: {target_count} | Avg SNR: {avg_intensity:.2f} | Fusion Beta Weight: {beta:.2f}")

def main():
    rclpy.init()
    node = RadarClarityNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
