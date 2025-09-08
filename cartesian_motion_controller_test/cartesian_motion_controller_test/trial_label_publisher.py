import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import os
from ament_index_python.packages import get_package_share_directory
import csv
import math

#  CONFIGURAZIONE FISSA (hardcoded) 
START_LABEL       = "Grasp 6"
END_LABEL         = "Grasp 7"
INCLUDE_START     = True   # includi la riga con START_LABEL
INCLUDE_END       = True   # includi la riga con END_LABEL
CASE_INSENSITIVE  = True   # confronta etichette in modo case-insensitive
INSERT_ZERO_BEFORE = False # inserisci frame di zeri prima del segmento
INSERT_ZERO_AFTER  = True  # inserisci frame di zeri dopo il segmento
ZERO_DURATION_S    = 0.5   # durata zeri (s)
DT                 = 0.02  # periodo pubblicazione (s)

class TrialPublisher(Node):
    def __init__(self):
        super().__init__('trial_publisher')

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            'input_raw',
            10
        )

        zeros_count = max(1, int(round(ZERO_DURATION_S / DT)))
        zero_frame = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        def norm(s: str) -> str:
            s = (s or '').strip()
            return s.lower() if CASE_INSENSITIVE else s

        try:
            package_share_dir = get_package_share_directory('cartesian_motion_controller_test')
            csv_path = os.path.join(package_share_dir, 'data', 'actualvel_no_presentation_with_trials.csv')

            self.data = []
            capturing = False
            start_found = False
            end_found = False

            start_norm = norm(START_LABEL)
            end_norm   = norm(END_LABEL)

            with open(csv_path, 'r', newline='') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    label_raw = row.get('phase_label', '')
                    label = norm(label_raw)

                    is_start = (start_norm != '' and label == start_norm)
                    is_end   = (end_norm   != '' and label == end_norm)

                    include_row = False

                    if not capturing:
                        if is_start:
                            start_found = True
                            capturing = True
                            if INSERT_ZERO_BEFORE:
                                self.data.extend([zero_frame[:] for _ in range(zeros_count)])
                            include_row = INCLUDE_START  # include la riga di start se richiesto
                    else:
                        include_row = True
                        if is_end and not INCLUDE_END:
                            include_row = False

                    if include_row:
                        try:
                            vals = [
                                float(row['pos1']),
                                float(row['pos2']),
                                float(row['pos3']),
                                float(row['pos4']),
                                float(row['pos5']),
                                float(row['pos6'])
                            ]
                        except (KeyError, ValueError) as e:
                            self.get_logger().warn(f"Skipping row due to parse error: {e}")
                            vals = None

                        if vals is not None:
                            if any(math.isnan(v) for v in vals):
                                self.get_logger().warn("Skipping row with NaN values")
                            else:
                                self.data.append(vals)

                    if capturing and is_end:
                        end_found = True
                        if INSERT_ZERO_AFTER:
                            self.data.extend([zero_frame[:] for _ in range(zeros_count)])
                        break

            if START_LABEL:
                self.get_logger().info(f"Start label: '{START_LABEL}' -> {'FOUND' if start_found else 'NOT FOUND'}")
            if END_LABEL:
                self.get_logger().info(f"End label: '{END_LABEL}' -> {'FOUND' if end_found else 'NOT FOUND'}")

            self.get_logger().info(f'Loaded {len(self.data)} rows between "{START_LABEL}" and "{END_LABEL}" (inclusive: start={INCLUDE_START}, end={INCLUDE_END}).')

        except Exception as e:
            self.get_logger().error(f'Could not read CSV: {e}')
            self.data = []

        self.index = 0
        self.dt = DT
        self.timer = self.create_timer(self.dt, self.publish_next)

        if not self.data:
            self.get_logger().warn('No data loaded for the selected labels. Node will stop.')
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
