#publisher of single markers for understanding the trajectory
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')

        # Define target positions in the sdr_reference frame
        self.targets = [
            (0.35, -0.06, 0.0),
            (0.52, -0.06, -0.25),
            (0.52, 0.14, 0.0),
        ]

        self.frame_id = "sdr_reference"

        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.timer = self.create_timer(1.0, self.publish_markers_once)

    def publish_markers_once(self):
        marker_array = MarkerArray()
        duration = Duration(); duration.sec = 0; duration.nanosec = 0  # infinito

        for i, (x, y, z) in enumerate(self.targets, start=1):
            # Marker 
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "markers"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = float(z)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.05
            m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0
            m.lifetime = duration
            marker_array.markers.append(m)

            # Text marker
            t = Marker()
            t.header.frame_id = self.frame_id
            t.header.stamp = self.get_clock().now().to_msg()
            t.ns = "markers"
            t.id = 100 + i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(x) + 0.01
            t.pose.position.y = float(y) + 0.01
            t.pose.position.z = float(z) + 0.05
            t.pose.orientation.w = 1.0
            t.scale.z = 0.05
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = str(i)   # Incrementale
            t.lifetime = duration
            marker_array.markers.append(t)

        self.marker_pub.publish(marker_array)

        if hasattr(self, 'timer'):
            self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
