from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker  # <-- Added for RViz Text
import cupy as cp  # <-- GPU Acceleration Library
import numpy as np
import message_filters

class DempsterShaferFusionNode(Node):
    def __init__(self):
        super().__init__('dempster_shafer_fusion_node')

        # Publisher for live thesis graphs
        self.weight_pub = self.create_publisher(Float32MultiArray, '/fusion/ds_weights', 10)

        self.visual_grid = None
        self.radar_grid = None
        self.alpha = 1.0  
        self.beta = 1.0   

        self.create_subscription(Float32, '/environment/visual_clarity', self.alpha_callback, 10)
        self.create_subscription(Float32, '/environment/radar_beta', self.beta_callback, 10)

        self.pub_fused_grid = self.create_publisher(OccupancyGrid, '/planning/fused_bev_grid', 10)
        
        # New Publisher for RViz Text Marker
        self.pub_status_text = self.create_publisher(Marker, '/planning/fusion_status_text', 10)
        # Publisher for thesis latency metrics (milliseconds)
        self.pub_latency = self.create_publisher(Float32, '/fusion/latency_ms', 10)

        self.pub_fused_cloud = self.create_publisher(PointCloud2, '/planning/fused_memory_cloud', 10)

        # Synchronized Subscriptions (Replacing static timer)
        self.visual_sub = message_filters.Subscriber(self, OccupancyGrid, '/planning/visual_bev_grid')
        self.radar_sub = message_filters.Subscriber(self, OccupancyGrid, '/planning/radar_bev_grid')
        self.ts = message_filters.ApproximateTimeSynchronizer([self.visual_sub, self.radar_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)

        self.get_logger().info("GPU-Accelerated Dempster-Shafer Fusion Node Started.")

    def alpha_callback(self, msg):
        self.alpha = msg.data

    def beta_callback(self, msg):
        self.beta = msg.data

    def sync_callback(self, visual_msg, radar_msg):
        self.visual_grid = visual_msg
        self.radar_grid = radar_msg
        self.fuse_grids()

    def fuse_grids(self):
        if self.visual_grid is None or self.radar_grid is None:
            return

        # 1. Push ROS tuple data directly into GPU Memory
        cam_array = cp.array(self.visual_grid.data, dtype=cp.float32)
        rad_array = cp.array(self.radar_grid.data, dtype=cp.float32)

        # 2. Assign Initial Belief Masses (Processed on CUDA cores)
        
        # Create a dynamic 1D distance mask to match the flattened cam_array
        res = self.visual_grid.info.resolution
        w = self.visual_grid.info.width
        h = self.visual_grid.info.height
        
        x_coords = cp.linspace(0, h * res, h)
        y_coords = cp.linspace(-w * res / 2, w * res / 2, w)
        X, Y = cp.meshgrid(x_coords, y_coords, indexing='ij')
        
        # Flatten the 2D distance grid to match the 1D cam_array
        dist_grid = cp.sqrt(X**2 + Y**2).flatten()
        
        # Camera confidence decays linearly from 1.0 at 15m down to 0.0 at 30m
        cam_distance_weight = cp.clip(1.0 - (dist_grid - 15.0) / 15.0, 0.0, 1.0)
        effective_alpha = self.alpha * cam_distance_weight

        # Assign masses using distance-weighted camera confidence
        m_c_occ = cp.where(cam_array == 100, 0.9 * effective_alpha, 0.0)
        m_c_free = cp.where(cam_array == 0, 0.9 * effective_alpha, 0.0)
        m_c_unk = 1.0 - m_c_occ - m_c_free

        m_r_occ = cp.where(rad_array == 100, 0.9 * self.beta, 0.0)
        m_r_free = cp.where(rad_array == 0, 0.9 * self.beta, 0.0)
        m_r_unk = 1.0 - m_r_occ - m_r_free

        # 3. Calculate the Conflict Factor (K)
        K = (m_c_occ * m_r_free) + (m_c_free * m_r_occ)
        K = cp.clip(K, 0.0, 0.99) 

        # 4. Dempster's Rule of Combination 
        m_fused_occ = (m_c_occ * m_r_occ + m_c_occ * m_r_unk + m_c_unk * m_r_occ) / (1.0 - K)
        m_fused_free = (m_c_free * m_r_free + m_c_free * m_r_unk + m_c_unk * m_r_free) / (1.0 - K)

        # 5. Decision Thresholding
        fused_array = cp.full_like(cam_array, -1, dtype=cp.int8) 
        fused_array[m_fused_occ > 0.5] = 100
        fused_array[(m_fused_free > 0.5) & (m_fused_occ <= 0.5)] = 0

        # 6. Pull the result back to CPU Memory to publish
        fused_array_cpu = cp.asnumpy(fused_array)

        fused_msg = OccupancyGrid()
        fused_msg.header.stamp = self.get_clock().now().to_msg()
        fused_msg.header.frame_id = 'base_link'
        fused_msg.info = self.visual_grid.info
        
        # Scale 0.0-1.0 to 0-100, clip safely, and cast to an 8-bit integer for Nav2
        nav2_ready_array = np.clip(fused_array_cpu * 100, 0, 100).astype(np.int8)
        fused_msg.data = nav2_ready_array.flatten().tolist()
        
        self.pub_fused_grid.publish(fused_msg)
        
        weight_msg = Float32MultiArray()
        # Publish the current Alpha (Camera) and Beta (Radar) certainty weights
        weight_msg.data = [float(self.alpha), float(self.beta)]
        self.weight_pub.publish(weight_msg)

        # 7. Publish RViz Status Text
        self.publish_status_marker()

        # 8. Calculate and Publish End-to-End Processing Latency
        cam_stamp_sec = self.visual_grid.header.stamp.sec + (self.visual_grid.header.stamp.nanosec * 1e-9)
        current_sec = self.get_clock().now().nanoseconds * 1e-9
        latency_ms = (current_sec - cam_stamp_sec) * 1000.0

        latency_msg = Float32()
        latency_msg.data = float(latency_ms)
        self.pub_latency.publish(latency_msg)

        # --- NEW: GENERATE POINT CLOUD FOR NAV2 GLOBAL MEMORY ---
        # Find the row (X) and column (Y) indices where the fused grid is occupied (100)
        occupied_indices = np.where(nav2_ready_array == 100)
        
        if len(occupied_indices[0]) > 0:
            cloud_points = []
            resolution = self.visual_grid.info.resolution
            grid_size_y = self.visual_grid.info.height * resolution
            
            # Convert grid indices back to real-world base_link coordinates (meters)
            for x_idx, y_idx in zip(occupied_indices[0], occupied_indices[1]):
                x_meters = x_idx * resolution
                y_meters = (y_idx * resolution) - (grid_size_y / 2.0)
                cloud_points.append([float(x_meters), float(y_meters), 0.0])
                
            # Publish as a PointCloud2
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'base_link'
            
            pc_msg = pc2.create_cloud_xyz32(header, cloud_points)
            self.pub_fused_cloud.publish(pc_msg)

    def publish_status_marker(self):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "fusion_status"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Float the text 2 meters above the robot
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.0 
        
        # Text Scale (Height)
        marker.scale.z = 0.4
        
        # Text Color (White)
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        
        # The actual string displayed in RViz
        marker.text = f"Visual Alpha: {self.alpha:.2f}\nRadar Beta: {self.beta:.2f}"
        
        self.pub_status_text.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = DempsterShaferFusionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
