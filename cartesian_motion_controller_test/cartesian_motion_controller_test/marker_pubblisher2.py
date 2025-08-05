# try to plot the markers for two different sdr 
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
        
        # # sdr references  
        # # Initial position e-e marker (violet)
        # ee_marker = Marker()
        # ee_marker.header.frame_id = "sdr_reference"
        # ee_marker.header.stamp = self.get_clock().now().to_msg()
        # ee_marker.ns = "markers"
        # ee_marker.id = 100
        # ee_marker.type = Marker.SPHERE
        # ee_marker.action = Marker.ADD
        # ee_marker.pose.position.x = 0.35
        # ee_marker.pose.position.z = -0.06
        # ee_marker.pose.position.y = 0.0
        # ee_marker.pose.orientation.w = 1.0
        # ee_marker.scale.x = ee_marker.scale.y = ee_marker.scale.z = 0.05
        # ee_marker.color.r = 0.5
        # ee_marker.color.g = 0.0
        # ee_marker.color.b = 1.0
        # ee_marker.color.a = 1.0
        # ee_marker.lifetime = duration
        # ee_marker.text = "initial position e-e"  
        # marker_array.markers.append(ee_marker)

        # # Target markers
        # for i, pos in enumerate(self.positions):
        #     marker = Marker()
        #     marker.header.frame_id = "sdr_reference"
        #     marker.header.stamp.sec = 0
        #     marker.header.stamp.nanosec = 0
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

            # sdr silvestrobase_link
        # Initial position e-e marker (violet)
        ee2_marker = Marker()
        ee2_marker.header.frame_id = "silvestrobase_link"
        ee2_marker.header.stamp = self.get_clock().now().to_msg()
        ee2_marker.ns = "markers"
        ee2_marker.id = 100
        ee2_marker.type = Marker.SPHERE
        ee2_marker.action = Marker.ADD
        ee2_marker.pose.position.x = 0.35
        ee2_marker.pose.position.z = -0.06
        ee2_marker.pose.position.y = 0.0
        ee2_marker.pose.orientation.w = 1.0
        ee2_marker.scale.x = ee2_marker.scale.y = ee2_marker.scale.z = 0.05
        ee2_marker.color.r = 1.0
        ee2_marker.color.g = 0.0
        ee2_marker.color.b = 1.0
        ee2_marker.color.a = 1.0
        ee2_marker.lifetime = duration
        ee2_marker.text = "initial position e-e"  
        marker_array.markers.append(ee2_marker)

        # Target markers 
        for i, pos in enumerate(self.positions):
            marker2 = Marker()
            marker2.header.frame_id = "silvestrobase_link"
            marker2.header.stamp.sec = 0
            marker2.header.stamp.nanosec = 0
            marker2.ns = "markers"
            marker2.id = i + 1  # start from 1
            marker2.type = Marker.SPHERE
            marker2.action = Marker.ADD
            marker2.pose.position.x = float(pos[0])
            marker2.pose.position.y = float(pos[1])
            marker2.pose.position.z = float(pos[2])
            marker2.pose.orientation.w = 1.0
            marker2.scale.x = marker2.scale.y = marker2.scale.z = 0.05
            marker2.color.g = 0.0
            marker2.color.b = 1.0
            marker2.color.r = 0.0   
            marker2.color.a = 1.0
            marker2.lifetime = duration
            marker_array.markers.append(marker2)
        
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