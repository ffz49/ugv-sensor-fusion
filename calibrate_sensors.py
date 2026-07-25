import rclpy
from rclpy.node import Node
from zed_msgs.msg import ObjectsStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('sensor_calibrator')
        
        # Subscribe to ZED Person Tracking
        self.sub_zed = self.create_subscription(ObjectsStamped, '/zed/zed_node/obj_det/objects', self.zed_callback, 10)
            
        # Subscribe to Radar PointCloud
        self.sub_radar = self.create_subscription(PointCloud2, '/smart_radar/port_targets_0', self.radar_callback, 10)

        self.zed_current_pos = None
        self.offsets_x = []
        self.offsets_y = []

    def zed_callback(self, msg):
        for obj in msg.objects:
            if obj.label.upper() == 'PERSON':
                self.zed_current_pos = np.array([obj.position[0], obj.position[1]])

    def radar_callback(self, msg):
        if self.zed_current_pos is not None:
            # Extract X and Y coordinates from the PointCloud
            points = list(pc2.read_points(msg, field_names=("x", "y"), skip_nans=True))
            if len(points) > 0:
                # Assume the first/primary point is the tracked person
                radar_x = points[0][0]
                radar_y = points[0][1]
                
                self.offsets_x.append(self.zed_current_pos[0] - radar_x)
                self.offsets_y.append(self.zed_current_pos[1] - radar_y)
                
                avg_x = sum(self.offsets_x) / len(self.offsets_x)
                avg_y = sum(self.offsets_y) / len(self.offsets_y)
                
                self.get_logger().info(f"CALCULATING... Current Offset -> X: {avg_x:.3f}m, Y: {avg_y:.3f}m")

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

