import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        
        # Path to CSV in "data" folder relative to this script
        package_share_dir = get_package_share_directory('cartesian_motion_controller_test')  
        csv_path = os.path.join(package_share_dir, 'data', 'grasp_carry_target.csv')
        
        # Load data: no header, 3 columns per row (X, Y, Z)
        data = np.genfromtxt(csv_path, delimiter=',', skip_header=0)
        self.positions = data.astype(float)

        # Publisher
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        
        # One-time timer to publish markers
        self.timer = self.create_timer(1.0, self.publish_markers_once)
        
    def publish_markers_once(self):
        marker_array = MarkerArray()
        duration = Duration()  # 0 = infinite lifetime
        duration.sec = 0
        duration.nanosec = 0
        
        # # Shoulder marker at origin, itś not necessary 
        # shoulder_marker = Marker()
        # shoulder_marker.header.frame_id = "sdr_reference"
        # shoulder_marker.header.stamp = self.get_clock().now().to_msg()
        # shoulder_marker.ns = "markers"
        # shoulder_marker.id = 0
        # shoulder_marker.type = Marker.SPHERE
        # shoulder_marker.action = Marker.ADD
        # shoulder_marker.pose.position.x = 0.0
        # shoulder_marker.pose.position.y = 0.0
        # shoulder_marker.pose.position.z = 0.0
        # shoulder_marker.pose.orientation.w = 1.0
        # shoulder_marker.scale.x = shoulder_marker.scale.y = shoulder_marker.scale.z = 0.065
        # shoulder_marker.color.r = 1.0
        # shoulder_marker.color.a = 1.0
        # shoulder_marker.lifetime = duration
        # marker_array.markers.append(shoulder_marker)
        
        # Initial position e-e marker (violet)
        eesdr_marker = Marker()
        eesdr_marker.header.frame_id = "silvestrobase_link"
        eesdr_marker.header.stamp = self.get_clock().now().to_msg()
        eesdr_marker.ns = "markers"
        eesdr_marker.id = 101
        eesdr_marker.type = Marker.SPHERE
        eesdr_marker.action = Marker.ADD
        eesdr_marker.pose.position.x = -0.06
        eesdr_marker.pose.position.y = 0.0
        eesdr_marker.pose.position.z = -0.35
        eesdr_marker.pose.orientation.w = 1.0
        eesdr_marker.scale.x = eesdr_marker.scale.y = eesdr_marker.scale.z = 0.05
        eesdr_marker.color.r = 0.5
        eesdr_marker.color.g = 0.0
        eesdr_marker.color.b = 1.0
        eesdr_marker.color.a = 1.0
        eesdr_marker.lifetime = duration
        eesdr_marker.text = "initial position e-e"  
        marker_array.markers.append(eesdr_marker)

        ee_marker = Marker()
        ee_marker.header.frame_id = "sdr_reference"
        ee_marker.header.stamp = self.get_clock().now().to_msg()
        ee_marker.ns = "markers"
        ee_marker.id = 100
        ee_marker.type = Marker.SPHERE
        ee_marker.action = Marker.ADD
        ee_marker.pose.position.x = 0.35
        ee_marker.pose.position.y = -0.06
        ee_marker.pose.position.z = 0.0
        ee_marker.pose.orientation.w = 1.0
        ee_marker.scale.x = ee_marker.scale.y = ee_marker.scale.z = 0.01
        ee_marker.color.r = 0.0
        ee_marker.color.g = 1.0
        ee_marker.color.b = 0.0
        ee_marker.color.a = 1.0
        ee_marker.lifetime = duration
        ee_marker.text = "initial position e-e"  
        marker_array.markers.append(ee_marker)

        # # Target markers
        # for i, pos in enumerate(self.positions):
        #     marker = Marker()
        #     marker.header.frame_id = "sdr_reference"
        #     marker.header.stamp = self.get_clock().now().to_msg()
        #     marker.ns = "markers"
        #     marker.id = i + 1  # start from 1
        #     marker.type = Marker.SPHERE
        #     marker.action = Marker.ADD
        #     marker.pose.position.x = float(pos[0])
        #     marker.pose.position.y = float(pos[1])
        #     marker.pose.position.z = float(pos[2])
        #     marker.pose.orientation.w = 1.0
        #     marker.scale.x = marker.scale.y = marker.scale.z = 0.05
        #     marker.color.g = 1.0
        #     marker.color.a = 1.0
        #     marker.lifetime = duration
        #     marker_array.markers.append(marker)

        #     #Text marker for each target
        #     text_marker = Marker()
        #     text_marker.header.frame_id = "sdr_reference"
        #     text_marker.header.stamp = self.get_clock().now().to_msg()
        #     text_marker.ns = "markers"
        #     text_marker.id = i + 101 
        #     text_marker.type = Marker.TEXT_VIEW_FACING
        #     text_marker.action = Marker.ADD
        #     text_marker.pose.position.x = float(pos[0]) + 0.01 
        #     text_marker.pose.position.y = float(pos[1]) + 0.01
        #     text_marker.pose.position.z = float(pos[2]) + 0.01
        #     text_marker.pose.orientation.w = 1.0
        #     text_marker.scale.z = 0.05  # Text size
        #     text_marker.color.r = 1.0
        #     text_marker.color.g = 1.0
        #     text_marker.color.b = 1.0
        #     text_marker.color.a = 1.0
        #     text_marker.text = f" {i+1}"
        #     text_marker.lifetime = duration
        #     marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)
        self.get_logger().info('Published all markers')

        # Cancel timer so this runs only once
        if hasattr(self, 'timer'):
            self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    try:
        rclpy.spin_once(node)  # just once, to publish the markers
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()