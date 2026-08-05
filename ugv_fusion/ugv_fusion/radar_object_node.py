import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from sklearn.cluster import DBSCAN

class RadarObjectNode(Node):
    def __init__(self):
        super().__init__('radar_object_node')
        self.sub_radar = self.create_subscription(
            PointCloud2, '/smart_radar/port_targets_0', self.radar_callback, qos_profile_sensor_data)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/planning/radar_bounding_boxes', 10)
        self.ugv_velocity = 0.0

    def odom_callback(self, msg):
        self.ugv_velocity = msg.twist.twist.linear.x

    def parse_pointcloud(self, msg):
        try:
            pts_generator = pc2.read_points(msg, field_names=("x", "y", "z", "rcs", "radial_speed"), skip_nans=True)
            points_list = [tuple(p) for p in pts_generator]
            if not points_list:
                return np.empty((0, 5), dtype=np.float32)
            return np.array(points_list, dtype=np.float32)
        except Exception as e:
            self.get_logger().debug(f"Failed to parse radar pointcloud fields: {e}")
            return np.empty((0, 5), dtype=np.float32)

    def radar_callback(self, msg):
        raw_points = self.parse_pointcloud(msg)
        if len(raw_points) == 0:
            return

        theta = np.arctan2(raw_points[:, 1], raw_points[:, 0])
        v_abs = raw_points[:, 4] - (self.ugv_velocity * np.cos(theta))
        raw_points[:, 4] = v_abs  

        weights = np.array([1.0, 1.0, 1.5, 0.1, 0.5]) 
        scaled_points = raw_points * weights

        clustering = DBSCAN(eps=1.5, min_samples=4).fit(scaled_points)
        labels = clustering.labels_

        clusters_data = []
        for label in set(labels):
            if label == -1:
                continue 
                
            cluster_points = raw_points[labels == label]
            min_x, min_y, min_z = np.min(cluster_points[:, :3], axis=0)
            max_x, max_y, max_z = np.max(cluster_points[:, :3], axis=0)
            avg_rcs = np.mean(cluster_points[:, 3])
            avg_v_abs = np.mean(cluster_points[:, 4])
            
            motion_state = "DYNAMIC" if abs(avg_v_abs) > 0.3 else "STATIC"
            clusters_data.append((min_x, min_y, min_z, max_x, max_y, max_z, avg_rcs, avg_v_abs, motion_state))

        if clusters_data:
            self.publish_bounding_boxes(clusters_data)

    def publish_bounding_boxes(self, clusters_data):
        marker_array = MarkerArray()
        for i, cluster in enumerate(clusters_data):
            min_x, min_y, min_z, max_x, max_y, max_z, avg_rcs, avg_v_abs, motion_state = cluster
            
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "radar_clusters"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float((min_x + max_x) / 2.0)
            marker.pose.position.y = float((min_y + max_y) / 2.0)
            marker.pose.position.z = float((min_z + max_z) / 2.0)
            marker.scale.x = max(float(max_x - min_x), 0.2)
            marker.scale.y = max(float(max_y - min_y), 0.2)
            marker.scale.z = max(float(max_z - min_z), 0.2)
            marker.color.a = 0.5 
            
            if motion_state == "DYNAMIC":
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
                
            # Embed data so the Fusion Node can read it directly
            radar_text = f"RCS: {avg_rcs:.1f} | V: {avg_v_abs:.1f}m/s"
            marker.text = radar_text
            marker_array.markers.append(marker)
            
            # Create the floating text label
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "radar_labels"
            text_marker.id = i + 1000 
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = marker.pose.position.x
            text_marker.pose.position.y = marker.pose.position.y
            text_marker.pose.position.z = marker.pose.position.z + (marker.scale.z / 2.0) + 0.8
            text_marker.scale.z = 0.35 
            text_marker.color.r, text_marker.color.g, text_marker.color.b, text_marker.color.a = 1.0, 1.0, 1.0, 1.0
            text_marker.text = radar_text
            marker_array.markers.append(text_marker)
            
        self.pub_markers.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = RadarObjectNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
