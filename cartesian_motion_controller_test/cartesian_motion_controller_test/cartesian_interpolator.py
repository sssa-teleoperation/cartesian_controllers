import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

RATE_HZ = 500.0          
K_FRAMES = 6            # interpolazione in K frames, parametro modificabile

class CartesianInterpolator(Node):
    def __init__(self):
        super().__init__('cartesian_interpolator')
        self.sub = self.create_subscription(
            Float64MultiArray,
            '/cartesian_motion_controller_silvestro/cartesian_input_base',
            self.on_input, 10)
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/cartesian_motion_controller_silvestro/CartesianMotionControllerInput',
            10)
        #internal state
        self.current = None   # ultimo pubblicato (len=6)
        self.target  = None   # ultimo ricevuto (len=6)
        self.frames_remaining = 0 # frames rimanenti per arrivare a target
        
        self.timer = self.create_timer(1.0/RATE_HZ, self.on_timer) #timer a 50Hz
        self.get_logger().info(f'Interpolator @ {RATE_HZ:.0f}Hz, K={K_FRAMES}')

    #callback for new input
    def on_input(self, msg):
        if len(msg.data) < 6:
            return
        data = list(msg.data[:6]) #prendo solo i primi 6 valori e li metto in una lista
        if self.current is None:
            self.current = data.copy() #la posizione corrente è la prima ricevuta
        self.target = data.copy()
        self.frames_remaining = max(1, K_FRAMES)

    #timer callback (decide what to publish)
    def on_timer(self):
        if self.current is None:
            return
        if self.target is None or self.frames_remaining <= 1:
            # vai diretto a target (o tieni current se manca), cioe se hai finito i frame va diretto a target
            if self.target is not None:
                self.current = self.target.copy()
        else:
            alpha = 1.0 / float(self.frames_remaining) #frazione da percorrere
            out = [self.current[i] + alpha*(self.target[i]-self.current[i]) for i in range(6)] #calcolo il punto intermedio
            self.current = out
            self.frames_remaining -= 1 #decremento i frame rimanenti

        msg = Float64MultiArray()
        msg.data = self.current
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    n = CartesianInterpolator()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
