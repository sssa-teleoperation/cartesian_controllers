import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import os
from ament_index_python.packages import get_package_share_directory
import csv
import math

class TrialPublisher(Node):
    def __init__(self):
        super().__init__('trial_publisher')

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/cartesian_motion_controller_silvestro/cartesian_input_sdr_reference_interp',
            10
        )

        try:
            package_share_dir = get_package_share_directory('cartesian_motion_controller_test')
            csv_path = os.path.join(package_share_dir, 'data', 'velocity_no_presentation_with_trials.csv')

            self.data = []

            with open(csv_path, 'r', newline='') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    # ferma alla prima occorrenza di "Grasp 2" inserendo una riga di zeri
                    if row.get('phase_label', '') == 'Carry 2':
                        self.data.append([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                        break

                    # costruisci la riga di valori
                    try:
                        vals = [
                            float(row['Var1']),
                            float(row['Var2']),
                            float(row['Var3']),
                            float(row['Var4']),
                            float(row['Var5']),
                            float(row['Var6'])
                        ]
                    except (KeyError, ValueError) as e:
                        # se colonne mancanti o conversione fallita, salta
                        self.get_logger().warn(f"Skipping row due to parse error: {e}")
                        continue

                    # salta se ci sono NaN
                    if any(math.isnan(v) for v in vals):
                        self.get_logger().warn("Skipping row with NaN values")
                        continue

                    self.data.append(vals)

            self.get_logger().info(f'Loaded {len(self.data)} rows')

        except Exception as e:
            self.get_logger().error(f'Could not read CSV: {e}')
            self.data = []

        self.index = 0
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.publish_next)

        if not self.data:
            self.get_logger().warn('No data loaded. Node will stop.')
            self.timer.cancel()

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
    node = TrialPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
