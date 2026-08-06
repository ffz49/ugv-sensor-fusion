from nav_msgs.msg import Odometry
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

class RadarBEVNode(Node):
    def __init__(self):
        super().__init__('radar_bev_node')

        # Odometry Subscriber for UGV velocity compensation
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.ugv_velocity = 0.0

        # Grid Parameters
        self.resolution = 0.05  # 10 cm per cell
        self.grid_size_x = 50.0  # meters forward
        self.grid_size_y = 30.0  # meters left/right
        self.offset_x = -5.0     # Start grid 5 meters behind the robot
        self.width = int((self.grid_size_x - self.offset_x) / self.resolution)
        self.height = int(self.grid_size_y / self.resolution)
        
        # TF2 Setup: We need this to rotate the tilted points to the flat ground
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Persistent memory for sparse radar hits
        self.ttl_grid = np.zeros((self.width, self.height), dtype=np.int8)

        # Subscriber & Publisher
        self.sub_radar = self.create_subscription(
            PointCloud2, '/smart_radar/port_targets_0', self.radar_callback, qos_profile_sensor_data)
        self.pub_grid = self.create_publisher(
            OccupancyGrid, '/planning/radar_bev_grid', 10)
            
        self.get_logger().info("Radar BEV Node Started. Waiting for TF...")

    def odom_callback(self, msg):
        self.ugv_velocity = msg.twist.twist.linear.x

    def radar_callback(self, msg):
        try:
            # 1. Look up the spatial difference between the tilted radar and the flat base_link
            transform = self.tf_buffer.lookup_transform(
                'base_link', msg.header.frame_id, rclpy.time.Time())
                
            # 2. Mathematically rotate the 3D points so they are level with the ground
            level_msg = do_transform_cloud(msg, transform)
        except Exception as e:
            self.get_logger().warn(f"Radar TF Error: {e}", throttle_duration_sec=2.0)
            return

        # 3. Extract X, Y, Z, RCS, and Radial Speed (Doppler)
        pts_list = pc2.read_points_list(level_msg, field_names=("x", "y", "z", "rcs", "radial_speed"), skip_nans=True)
        if not pts_list:
            return

        points = np.array(pts_list, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] < 5:
            return
            
        # 4. Filter bounds and weak organic returns (RCS > 5.0 dBsm)
        valid_idx = (points[:, 0] >= self.offset_x) & (points[:, 0] < self.grid_size_x) & \
                    (points[:, 1] >= -self.grid_size_y / 2) & (points[:, 1] < self.grid_size_y / 2) & \
                    (points[:, 2] > 0.10) & (points[:, 2] < 2.0) & \
                    (points[:, 3] > 5.0)
                    
        points = points[valid_idx]

        # 5. Decay the radar memory grid by 1 every frame
        self.ttl_grid = np.maximum(self.ttl_grid - 1, 0)

        # 6. Map detected radar reflections with Velocity-Scaled Splatting & Persistence
        if len(points) > 0:
            for pt in points:
                x, y, z, rcs, radial_speed = pt
                
                # Compensate for UGV forward motion to get true object velocity
                theta = math.atan2(y, x)
                v_abs = radial_speed - (self.ugv_velocity * math.cos(theta))
                
                # Dynamic Splatting: Faster moving objects get a larger footprint
                base_splat = 2
                speed_splat = int(abs(v_abs) * 0.5) 
                current_splat = min(base_splat + speed_splat, 6) # Cap max radius
                
                grid_x = int((x - self.offset_x) / self.resolution)
                grid_y = int((y + (self.grid_size_y / 2)) / self.resolution)
                
                # Apply footprint to the persistence grid
                for dx in range(-current_splat, current_splat + 1):
                    for dy in range(-current_splat, current_splat + 1):
                        nx = np.clip(grid_x + dx, 0, self.width - 1)
                        ny = np.clip(grid_y + dy, 0, self.height - 1)
                        self.ttl_grid[nx, ny] = 5  # Persist for 5 frames

        # 1. Declare empty space explicitly as FREE (0) instead of Unknown (-1)
        grid_data = np.zeros((self.width, self.height), dtype=np.int8)
        
        # (Your radar/visual obstacle logic stays the same)
        grid_data[self.ttl_grid > 0] = 100

        # 2. TRANSPOSE the grid (.T) before publishing!
        # This fixes the 90-degree rotation so ROS reads X and Y correctly.
        aligned_grid = grid_data.T

        # 3. Publish the aligned grid
        self.publish_grid(aligned_grid, level_msg.header.stamp, 'base_link')

    def publish_grid(self, grid_data, stamp, frame_id):
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = stamp
        grid_msg.header.frame_id = frame_id
        
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        grid_msg.info.origin.position.x = self.offset_x
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
