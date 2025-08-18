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
        
        # Path to CSV in "data" folder of the package
        package_share_dir = get_package_share_directory('cartesian_motion_controller_test')
        csv_path = os.path.join(package_share_dir, 'data', 'grasp_carry_target.csv')
        
        # Load data (points in A = sdr_reference)
        data = np.genfromtxt(csv_path, delimiter=',', skip_header=0)
        self.positions_A = data.astype(float).reshape(-1, 3)

        # Rotation A->B (sdr_reference -> silvestrobase_link), rotation only
        self.R_BA = np.array([[0.0, 0.0, -1.0],
                              [-1.0, 0.0,  0.0],
                              [0.0, 1.0,  0.0]], dtype=float)

        # Publisher
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        
        # One-time timer to publish markers
        self.timer = self.create_timer(1.0, self.publish_markers_once)

    def rot_A_to_B(self, pA_xyz):
        """Rotate a 3D point from A (sdr_reference) to B (silvestrobase_link)."""
        return (self.R_BA @ np.asarray(pA_xyz, dtype=float)).tolist()

    def publish_markers_once(self):
        marker_array = MarkerArray()
        duration = Duration()  # 0 = infinite lifetime
        duration.sec = 0
        duration.nanosec = 0

        now = self.get_clock().now().to_msg()

        #  Initial position of end-effector (given in A) 
        # Original EE position was written as x=0.35, y=0.0, z=-0.06 but in A.
        pA_ee = [0.35, -0.06, 0.0]  
        xB, yB, zB = self.rot_A_to_B(pA_ee)

        ee_marker = Marker()
        ee_marker.header.frame_id = "silvestrobase_link"   # plot in B
        ee_marker.header.stamp = now
        ee_marker.ns = "markers"
        ee_marker.id = 100
        ee_marker.type = Marker.SPHERE
        ee_marker.action = Marker.ADD
        ee_marker.pose.position.x = float(xB)
        ee_marker.pose.position.y = float(yB)
        ee_marker.pose.position.z = float(zB)
        ee_marker.pose.orientation.w = 1.0
        ee_marker.scale.x = ee_marker.scale.y = ee_marker.scale.z = 0.05
        ee_marker.color.r = 1.0
        ee_marker.color.g = 0.0
        ee_marker.color.b = 0.0
        ee_marker.color.a = 1.0
        ee_marker.lifetime = duration
        ee_marker.text = "initial position e-e"
        marker_array.markers.append(ee_marker)

        #  Target markers (CSV points are in A → rotate to B) 
        for i, posA in enumerate(self.positions_A):
            xB, yB, zB = self.rot_A_to_B(posA)

            marker = Marker()
            marker.header.frame_id = "silvestrobase_link"   # plot in B
            marker.header.stamp = now
            marker.ns = "markers"
            marker.id = i + 1  # start from 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(xB)
            marker.pose.position.y = float(yB)
            marker.pose.position.z = float(zB)
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.05
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.5
            marker.lifetime = duration
            marker_array.markers.append(marker)

        # Target in sdr_reference
        for i, posA in enumerate(self.positions_A):
            markerA = Marker()
            markerA.header.frame_id = "sdr_reference"
            markerA.header.stamp = now
            markerA.ns = "markers"
            markerA.id = 1000 + i  # start from 1001
            markerA.type = Marker.SPHERE
            markerA.action = Marker.ADD
            markerA.pose.position.x = float(posA[0])
            markerA.pose.position.y = float(posA[1])
            markerA.pose.position.z = float(posA[2])
            markerA.pose.orientation.w = 1.0
            markerA.scale.x = markerA.scale.y = markerA.scale.z = 0.01  
            markerA.color.r = 0.0
            markerA.color.g = 1.0
            markerA.color.b = 0.0
            markerA.color.a = 1.0
            markerA.lifetime = duration
            marker_array.markers.append(markerA)

        self.marker_pub.publish(marker_array)
        self.get_logger().info('Published all markers (rotated A→B, frame=silvestrobase_link)')

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
