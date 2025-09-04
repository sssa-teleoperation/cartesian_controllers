import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import time



class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/cartesian_motion_controller_silvestro/CartesianMotionControllerInput', 10)

        timer_period = 0.02  #s, 50hz same of decoder, it's the pub frequency
        self.timer = self.create_timer(timer_period, self.timer_callback) #timer for pub

        self.start_time = time.time() #start of the node
        self.freq = 0.2  # Hz, it's frq for sin



    def timer_callback(self):

        elapsed_time = time.time() - self.start_time 

        msg = Float64MultiArray()
  
        if elapsed_time < 5.0: #gradino
            vz= 0.0
        elif elapsed_time < 7.0:
            vz= 0.1    
        else:
            vz = 0.0

        vy = vx = wx = wy = wz = 0.0  
   
        msg.data = [vx, vy, vz, wx, wy, wz]
        self.publisher_.publish(msg)


def main():
    rclpy.init()
    node = TestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Stop the controller by publishing zeros
    stop_msg = Float64MultiArray()
    stop_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    node.publisher_.publish(stop_msg)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
     main()