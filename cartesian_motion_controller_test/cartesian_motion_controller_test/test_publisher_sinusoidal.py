import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
import signal

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/cartesian_motion_controller_silvestro/CartesianMotionControllerInput',
            10
        )

        self.timer_period = 0.01  # 100 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.start_time = time.time()

        self.freq = 0.2  # Hz
        self.max_displacement = 0.1
        self.amp = self.max_displacement * np.pi * self.freq
        self.ang_amp = 0.05

        self.is_shutting_down = False
        self.zero_sent = False  

    def publish_zero_once(self):
        if not self.zero_sent:
            msg = Float64MultiArray()
            msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.publisher_.publish(msg)
            self.zero_sent = True
            self.get_logger().info('Send 0 to velocity.')

    def timer_callback(self):
        if self.is_shutting_down:
            return  

        elapsed_time = time.time() - self.start_time
        msg = Float64MultiArray()

        vx = self.amp * np.sin(2 * np.pi * self.freq * elapsed_time)
        vy=vz=wx=wy=wz=0.0 
       
        msg.data = [vx, vy, vz, wx, wy, wz]
        self.publisher_.publish(msg)

    def begin_shutdown(self):
        if not self.is_shutting_down:
            self.is_shutting_down = True
            self.timer.cancel()          
            self.publish_zero_once()     

def signal_handler(signum, frame):
    if 'node' in globals():
        node.begin_shutdown()
       
        rclpy.shutdown()

def main():
    global node
    rclpy.init()
    node = TestPublisher()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        rclpy.spin(node)
    finally:
        if not node.zero_sent:
            node.publish_zero_once()
        node.destroy_node()
      
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
