import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

# THE CRITICAL FIX: Properly importing from zed_msgs, not zed_interfaces
from zed_msgs.msg import ObjectsStamped 

import numpy as np
from scipy.optimize import linear_sum_assignment
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

class SpatioTemporalAssociationNode(Node):
    def __init__(self):
        super().__init__('spatio_temporal_association_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.sub_camera = self.create_subscription(ObjectsStamped, '/zed/zed_node/obj_det/objects', self.camera_callback, 10)
        self.sub_radar = self.create_subscription(MarkerArray, '/planning/radar_bounding_boxes', self.radar_callback, 10)
        
        # This is the publisher that disappeared from RViz because the node crashed
        self.pub_diagnostic = self.create_publisher(MarkerArray, '/planning/diagnostic_fusion_markers', 10)
        
        self.latest_camera_boxes = []
        self.latest_radar_boxes = []
        self.distance_threshold = 2.0 
        
        self.get_logger().info("Spatio-Temporal Fusion Node Started.")

    def transform_point(self, x, y, z, from_frame):
        try:
            trans = self.tf_buffer.lookup_transform('base_link', from_frame, rclpy.time.Time())
            pt = PointStamped()
            pt.header.frame_id = from_frame
            pt.point.x, pt.point.y, pt.point.z = float(x), float(y), float(z)
            transformed_pt = tf2_geometry_msgs.do_transform_point(pt, trans)
            return [transformed_pt.point.x, transformed_pt.point.y, transformed_pt.point.z]
        except Exception as e:
            self.get_logger().error(f"TF Transform Error in Fusion: {e}", throttle_duration_sec=2.0)
            return None

    def radar_callback(self, msg):
        self.latest_radar_boxes = []
        for marker in msg.markers:
            if marker.type == Marker.CUBE:
                pt = self.transform_point(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z, marker.header.frame_id)
                if pt:
                    self.latest_radar_boxes.append([pt[0], pt[1], pt[2], marker.text, marker.id])
        self.fuse_objects()

    def camera_callback(self, msg):
        self.latest_camera_boxes = []
        for obj in msg.objects:
            pt = self.transform_point(obj.position[0], obj.position[1], obj.position[2], msg.header.frame_id)
            if pt:
                self.latest_camera_boxes.append([pt[0], pt[1], pt[2], obj.label, obj.label_id])

    def create_marker_pair(self, m_id, x, y, z, color, scale, ns, text, z_offset):
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = m_id
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = color
        
        t = Marker()
        t.header = m.header
        t.ns = ns + "_labels"
        t.id = m_id + 1000
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose.position.x, t.pose.position.y = float(x), float(y)
        t.pose.position.z = float(z) + (scale / 2.0) + z_offset 
        t.scale.z = 0.35 
        t.color.r, t.color.g, t.color.b, t.color.a = 1.0, 1.0, 1.0, 1.0
        t.text = text
        
        return [m, t]

    def fuse_objects(self):
        diagnostic_markers = MarkerArray()
        m_id = 0

        # Draw Unfused Camera (Blue) - Text stacked at 0.4m
        for cam in self.latest_camera_boxes:
            markers = self.create_marker_pair(m_id, cam[0], cam[1], cam[2], (0.0, 0.0, 1.0, 0.5), 1.0, "camera_raw", f"AI: {cam[3]}", 0.4)
            diagnostic_markers.markers.extend(markers)
            m_id += 1

        # Draw Unfused Radar (Red) - Text stacked at 0.8m
        for rad in self.latest_radar_boxes:
            markers = self.create_marker_pair(m_id, rad[0], rad[1], rad[2], (1.0, 0.0, 0.0, 0.5), 1.0, "radar_raw", f"{rad[3]}", 0.8)
            diagnostic_markers.markers.extend(markers)
            m_id += 1

        # Execute Fusion
        if len(self.latest_camera_boxes) > 0 and len(self.latest_radar_boxes) > 0:
            
            cam_coords = np.array([box[:3] for box in self.latest_camera_boxes], dtype=float)
            rad_coords = np.array([box[:3] for box in self.latest_radar_boxes], dtype=float)
            
            cost_matrix = np.linalg.norm(cam_coords[:, np.newaxis, :] - rad_coords[np.newaxis, :, :], axis=2)
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
            
            for cam_idx, rad_idx in zip(row_ind, col_ind):
                if cost_matrix[cam_idx, rad_idx] <= self.distance_threshold:
                    cam = self.latest_camera_boxes[cam_idx]
                    rad = self.latest_radar_boxes[rad_idx]
                    
                    fused_x = (cam[0] + rad[0]) / 2.0
                    fused_y = (cam[1] + rad[1]) / 2.0
                    fused_z = (cam[2] + rad[2]) / 2.0
                    
                    fused_text = f"FUSED: {cam[3]}\n{rad[3]}"
                    
                    # Draw Fused Object (Green) - Text stacked at 1.2m
                    markers = self.create_marker_pair(m_id, fused_x, fused_y, fused_z, (0.0, 1.0, 0.0, 0.8), 1.5, "fused_objects", fused_text, 1.2)
                    diagnostic_markers.markers.extend(markers)
                    m_id += 1

        self.pub_diagnostic.publish(diagnostic_markers)

def main(args=None):
    rclpy.init(args=args)
    node = SpatioTemporalAssociationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
