import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import os
from ament_index_python.packages import get_package_share_directory
import csv

class ActualVelPublisher(Node):
    def __init__(self):
        super().__init__('actualvel_publisher')

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/cartesian_motion_controller_silvestro/cartesian_input_sdr_reference',
            10
        )

        try:
            package_share_dir = get_package_share_directory('cartesian_motion_controller_test')
            csv_path = os.path.join(package_share_dir, 'data', 'actualvel_no_presentation_with_trials.csv')

            self.data = []
            with open(csv_path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    if row['phase_label'] == 'Release 54':
                        self.data.append([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                        break
                    else:
                        self.data.append([
                            float(row['pos1']),
                            float(row['pos2']),
                            float(row['pos3']),
                            float(row['pos4']),
                            float(row['pos5']),
                            float(row['pos6'])
                        ])
            self.get_logger().info(f'Loaded {len(self.data)} rows')

        except Exception as e:
            self.get_logger().error(f'Could not read CSV: {e}')
            self.data = []

        self.index = 0
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.publish_next)

    def publish_next(self):
        if self.index < len(self.data):
            row = self.data[self.index]
            self.index += 1
        else:
            self.get_logger().info('Stop.')
            self.timer.cancel()
            return

        msg = Float64MultiArray()
        msg.data = row
        self.publisher_.publish(msg)

        idx = self.index - 1 if self.index <= len(self.data) else len(self.data) - 1
        self.get_logger().info(f'[{idx}] Published: {[f"{x:.3f}" for x in row]}')


def main(args=None):
    rclpy.init(args=args)
    node = ActualVelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
