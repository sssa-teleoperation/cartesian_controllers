import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
import signal

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
        
        # Add a flag to check if we're shutting down
        self.is_shutting_down = False

        # Add counter for stop messages
        self.stop_counter = 0
        self.max_stop_messages = 4  # Number of zero velocity messages to send

    def stop_motion(self):
        # Publish zero velocities
        if self.stop_counter < self.max_stop_messages:
            msg = Float64MultiArray()
            msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.publisher_.publish(msg)
            self.get_logger().info(f'Stopping motion - sending zero velocities ({self.stop_counter + 1}/{self.max_stop_messages})')
            self.stop_counter += 1

    def timer_callback(self):
        if self.is_shutting_down:
            self.stop_motion()
            if self.stop_counter >= self.max_stop_messages:
                self.destroy_node()
                rclpy.shutdown()
            return

        elapsed_time = time.time() - self.start_time

        msg = Float64MultiArray()

        vx = self.amp * np.sin(2 * np.pi * self.freq * elapsed_time)
        vy = self.amp * np.sin(2 * np.pi * self.freq * elapsed_time + np.pi/2)
        vz = self.amp * np.sin(2 * np.pi * self.freq * elapsed_time + np.pi)

        wx = self.ang_amp * np.sin(2 * np.pi * self.freq * elapsed_time)
        wy = self.ang_amp * np.sin(2 * np.pi * self.freq * elapsed_time + np.pi/2)
        wz = self.ang_amp * np.sin(2 * np.pi * self.freq * elapsed_time + np.pi)
        msg.data = [vx, vy, vz, wx, wy, wz]

        self.publisher_.publish(msg)

def signal_handler(signum, frame):
    # Mark the node as shutting down
    if 'node' in globals():
        node.is_shutting_down = True
        node.stop_motion()
    
def main():
    global node  # Make node global so signal handler can access it
    rclpy.init()
    node = TestPublisher()
    
    # Set up signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        rclpy.spin(node)
    finally:
        # Ensure we stop motion before shutting down
        node.stop_motion()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
