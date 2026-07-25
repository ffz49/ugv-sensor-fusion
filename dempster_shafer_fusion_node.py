import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
import cupy as cp  # <-- GPU Acceleration Library

class DempsterShaferFusionNode(Node):
    def __init__(self):
        super().__init__('dempster_shafer_fusion_node')

        self.visual_grid = None
        self.radar_grid = None
        self.alpha = 1.0  
        self.beta = 1.0   

        self.create_subscription(Float32, '/environment/visual_alpha', self.alpha_callback, 10)
        self.create_subscription(Float32, '/environment/radar_beta', self.beta_callback, 10)
        self.create_subscription(OccupancyGrid, '/planning/visual_bev_grid', self.visual_callback, 10)
        self.create_subscription(OccupancyGrid, '/planning/radar_bev_grid', self.radar_callback, 10)

        self.pub_fused_grid = self.create_publisher(OccupancyGrid, '/planning/fused_bev_grid', 10)
        
        self.timer = self.create_timer(0.1, self.fuse_grids)
        
        self.get_logger().info("GPU-Accelerated Dempster-Shafer Fusion Node Started.")

    def alpha_callback(self, msg):
        self.alpha = msg.data

    def beta_callback(self, msg):
        self.beta = msg.data

    def visual_callback(self, msg):
        self.visual_grid = msg

    def radar_callback(self, msg):
        self.radar_grid = msg

    def fuse_grids(self):
        if self.visual_grid is None or self.radar_grid is None:
            return

        # 1. Push ROS tuple data directly into GPU Memory
        cam_array = cp.array(self.visual_grid.data, dtype=cp.float32)
        rad_array = cp.array(self.radar_grid.data, dtype=cp.float32)

        # 2. Assign Initial Belief Masses (Processed on CUDA cores)
        m_c_occ = cp.where(cam_array == 100, 0.9 * self.alpha, 0.0)
        m_c_free = cp.where(cam_array == 0, 0.9 * self.alpha, 0.0)
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
        fused_msg.data = fused_array_cpu.tolist()
        
        self.pub_fused_grid.publish(fused_msg)

def main():
    rclpy.init()
    node = DempsterShaferFusionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
