import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

class RadarBEVNode(Node):
    def __init__(self):
        super().__init__('radar_bev_node')

        # Grid Parameters
        self.resolution = 0.05  # 10 cm per cell
        self.grid_size_x = 15.0  # meters forward
        self.grid_size_y = 15.0  # meters left/right
        self.width = int(self.grid_size_x / self.resolution)
        self.height = int(self.grid_size_y / self.resolution)
        
        # TF2 Setup: We need this to rotate the tilted points to the flat ground
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriber & Publisher
        self.sub_radar = self.create_subscription(
            PointCloud2, '/smart_radar/port_targets_0', self.radar_callback, 10)
        self.pub_grid = self.create_publisher(
            OccupancyGrid, '/planning/radar_bev_grid', 10)
            
        self.get_logger().info("Radar BEV Node Started. Waiting for TF...")

    def radar_callback(self, msg):
        try:
            # 1. Look up the spatial difference between the tilted radar and the flat base_link
            transform = self.tf_buffer.lookup_transform(
                'base_link', msg.header.frame_id, rclpy.time.Time())
                
            # 2. Mathematically rotate the 3D points so they are level with the ground
            level_msg = do_transform_cloud(msg, transform)
        except Exception as e:
            # If the launch file isn't running yet, skip this frame
            return

        # 3. Extract the level X, Y, Z coordinates
        pts_list = pc2.read_points_list(level_msg, field_names=("x", "y", "z"), skip_nans=True)
        if not pts_list:
            return

        points = np.array(pts_list, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] < 3:
            return
            
        # 4. Filter points within our 20x20m grid
        valid_idx = (points[:, 0] >= 0) & (points[:, 0] < self.grid_size_x) & \
                    (points[:, 1] >= -self.grid_size_y / 2) & (points[:, 1] < self.grid_size_y / 2) & \
                    (points[:, 2] > 0.10) & (points[:, 2] < 2.0)
        points = points[valid_idx]

        # 5. Initialize the Occupancy Grid array with -1 (Unknown Space/Transparent)
        grid_data = np.full((self.width, self.height), -1, dtype=np.int8)

        if len(points) > 0:
            x_indices = (points[:, 0] / self.resolution).astype(int)
            y_indices = ((points[:, 1] + (self.grid_size_y / 2)) / self.resolution).astype(int)
            
            # Map detected radar reflections as solid obstacles (100)
            grid_data[x_indices, y_indices] = 100

        # 6. Publish the grid natively in the flat base_link frame
        self.publish_grid(grid_data, level_msg.header.stamp, 'base_link')

    def publish_grid(self, grid_data, stamp, frame_id):
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = stamp
        grid_msg.header.frame_id = frame_id
        
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.height
        grid_msg.info.height = self.width
        grid_msg.info.origin.position.x = 0.0
        grid_msg.info.origin.position.y = -self.grid_size_y / 2
        grid_msg.info.origin.position.z = 0.0
        
        grid_msg.data = grid_data.flatten().tolist()
        self.pub_grid.publish(grid_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RadarBEVNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
