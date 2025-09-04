import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

RATE_HZ  = 500.0        # frequenza di pubblicazione dell'interpolato 
K_FRAMES = 10           # numero di step per arrivare al prossimo target
EPS_LIN  = 1e-4         # soglia: variazione “reale” su vx,vy,vz
EPS_ANG  = 1e-4         # soglia: variazione “reale” su wx,wy,wz

class CartesianInterpolator(Node):

    def __init__(self):
        super().__init__('cartesian_interpolator')

        self.sub = self.create_subscription(
            Float64MultiArray,
            '/cartesian_motion_controller_silvestro/cartesian_input_base',
            self.on_input,
            10
        )
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/cartesian_motion_controller_silvestro/CartesianMotionControllerInput',
            10
        )

        # Stato interno
        self.current = None       # ultimo valore pubblicato (len=6)
        self.target  = None       # target attuale verso cui stiamo andando (len=6)
        self.last_target = None   # ultimo target ricevuto (per confrontare i cambi)
        self.frames_remaining = 0 # step rimanenti nel segmento corrente

        self.timer = self.create_timer(1.0 / RATE_HZ, self.on_timer)
        self.get_logger().info(f'Interpolator @ {RATE_HZ:.0f} Hz, K={K_FRAMES}, EPS=({EPS_LIN}, {EPS_ANG})')

    #  utility 
    @staticmethod
    def _diff_exceeds_eps(a, b):
        #Ritorna True se almeno una componente differisce oltre la soglia (lin/ang).
        # a e b sono liste lunghe 6
        for i in range(6):
            eps = EPS_LIN if i < 3 else EPS_ANG
            if abs(a[i] - b[i]) > eps:
                return True
        return False

    #  callback input 
    def on_input(self, msg: Float64MultiArray):
        if len(msg.data) < 6:
            return
        data = list(msg.data[:6])

        # primo messaggio: inizializza tutto
        if self.current is None:
            self.current = data.copy()
            self.target  = data.copy()
            self.last_target = data.copy()
            self.frames_remaining = 0
            return

        # “ aggiorna target SOLO se cambia davvero oltre EPS
        if self.last_target is None or self._diff_exceeds_eps(data, self.last_target):
            # parti dal current verso il nuovo target
            self.last_target = data.copy()
            self.target = data.copy()
            self.frames_remaining = max(1, K_FRAMES)
            # self.get_logger().info(f'New keypoint → reset K: {["%.5f"%v for v in data]}')
        else:
            pass

    #  timer publish 
    def on_timer(self):
        if self.current is None:
            return

        if self.target is None or self.frames_remaining <= 1:
            # Arrivata al target (o nessun target): mantieni/aggiorna e pubblica
            if self.target is not None:
                self.current = self.target.copy()
        else:
            # Interpolazione lineare verso il target in K step rimanenti
            alpha = 1.0 / float(self.frames_remaining)
            out = [ self.current[i] + alpha * (self.target[i] - self.current[i]) for i in range(6) ]
            self.current = out
            self.frames_remaining -= 1

        # Pubblica sempre l'ultimo current
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
