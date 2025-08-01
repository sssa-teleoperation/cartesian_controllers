import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import time

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')

        self.publisher_ = self.create_publisher(Float64MultiArray, '/cartesian_motion_controller_silvestro/CartesianMotionControllerInput', 10)

        timer_period = 0.01  # try different values to see the effect
        # 0.01 seconds = 100 Hz, which is a good frequency for smooth, 50Hz is not good
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.start_time = time.time()

        self.freq = 0.2  # Hz
        self.max_displacement = 0.1  
        self.amp = self.max_displacement * np.pi * self.freq

        # Angular amplitude (just a small fixed value)
        self.ang_amp = 0.05

    def timer_callback(self):
        elapsed_time = time.time() - self.start_time

        msg = Float64MultiArray()

        vx = self.amp * np.sin(2 * np.pi * self.freq * elapsed_time)
        # vy = self.amp * np.sin(2 * np.pi * self.freq * elapsed_time + np.pi/2)
        # vz = self.amp * np.sin(2 * np.pi * self.freq * elapsed_time + np.pi)

        # wx = self.ang_amp * np.sin(2 * np.pi * self.freq * elapsed_time)
        # wy = self.ang_amp * np.sin(2 * np.pi * self.freq * elapsed_time + np.pi/2)
        # wz = self.ang_amp * np.sin(2 * np.pi * self.freq * elapsed_time + np.pi)
        vy=vz=wx=wy=wz=0.0  
        msg.data = [vx, vy, vz, wx, wy, wz]

        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = TestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
