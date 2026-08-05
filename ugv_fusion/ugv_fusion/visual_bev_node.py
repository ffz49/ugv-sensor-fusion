import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import qos_profile_sensor_data
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import numpy as np
import cupy as cp  # <-- CuPy imported

class VisualBEVNode(Node):
    def __init__(self):
        super().__init__('visual_bev_node')

        # Grid Parameters
        self.resolution = 0.05  # 10 cm per cell
        self.grid_size_x = 15.0  # meters forward
        self.grid_size_y = 15.0  # meters left/right (total)
        self.width = int(self.grid_size_x / self.resolution)
        self.height = int(self.grid_size_y / self.resolution)

        # Traversability Thresholds
        self.obstacle_z_threshold = 0.2  # Any point 20cm above ground is an obstacle

        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers & Subscribers
        self.sub_cloud = self.create_subscription(
            PointCloud2, '/zed/zed_node/point_cloud/cloud_registered', self.cloud_callback, qos_profile_sensor_data)
        self.pub_grid = self.create_publisher(
            OccupancyGrid, '/planning/visual_bev_grid', 10)

    def cloud_callback(self, msg):
        try:
            # Look up the transform from the camera to the base_link footprint
            transform = self.tf_buffer.lookup_transform(
                'base_link', msg.header.frame_id, rclpy.time.Time())

            # Apply transformation to align the cloud flat to the ground
            transformed_cloud = do_transform_cloud(msg, transform)
        except Exception as e:
            self.get_logger().warn(f"TF Transform Error: {e}")
            return

        # Extract X, Y, Z coordinates cleanly into a 2D list
        pts_list = pc2.read_points_list(transformed_cloud, field_names=("x", "y", "z"), skip_nans=True)
        if not pts_list:
            return

        # Convert to NumPy first (fastest list parsing), then push directly to GPU via CuPy
        cpu_points = np.array(pts_list, dtype=np.float32)
        if cpu_points.ndim != 2 or cpu_points.shape[1] < 3:
            return
            
        points = cp.asarray(cpu_points) # <-- Data is now on the GPU

        # Filter out points that are too high (sky) or outside the grid bounds using GPU parallelization
        valid_idx = (points[:, 0] >= 0) & (points[:, 0] < self.grid_size_x) & \
                    (points[:, 1] >= -self.grid_size_y / 2) & (points[:, 1] < self.grid_size_y / 2) & \
                    (points[:, 2] < 2.0)
        points = points[valid_idx]

        if len(points) == 0:
            return

        # Map X, Y coordinates to grid cell indices
        x_indices = (points[:, 0] / self.resolution).astype(cp.int32)
        y_indices = ((points[:, 1] + (self.grid_size_y / 2)) / self.resolution).astype(cp.int32)

        # Initialize the Occupancy Grid array on the GPU with -1 (Unknown)
        grid_data = cp.full((self.width, self.height), -1, dtype=cp.int8)

        # Fast 2D height mapping
        free_mask = points[:, 2] < self.obstacle_z_threshold
        obs_mask = points[:, 2] >= self.obstacle_z_threshold

        # Apply Free Space (0) first, then overwrite with Obstacles (100)
        grid_data[x_indices[free_mask], y_indices[free_mask]] = 0
        grid_data[x_indices[obs_mask], y_indices[obs_mask]] = 100

        # Pull the final calculated grid back to the CPU for ROS publishing
        cpu_grid_data = cp.asnumpy(grid_data)

        self.publish_grid(cpu_grid_data, msg.header.stamp)

    def publish_grid(self, grid_data, stamp):
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = stamp
        grid_msg.header.frame_id = 'base_link'

        # Meta Data
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.height
        grid_msg.info.height = self.width
        grid_msg.info.origin.position.x = 0.0
        grid_msg.info.origin.position.y = -self.grid_size_y / 2
        grid_msg.info.origin.position.z = 0.0

        # Flatten array and convert to list for ROS 2 message definition
        grid_msg.data = grid_data.flatten().tolist()

        self.pub_grid.publish(grid_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisualBEVNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
