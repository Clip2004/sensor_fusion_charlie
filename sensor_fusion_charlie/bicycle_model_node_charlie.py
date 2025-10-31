import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray


class BicycleModelNodeCharlie(Node):
    def __init__(self):
        super().__init__('bicycle_model_node_charlie')

        # --- Parámetros del modelo ---
        self.lwb = 0.25       # distancia entre ejes (m)
        self.dt = 0.05       # paso de integración (s)

        # --- Variables de estado ---
        self.x = 0.0
        self.y = 0.0
        self.psi = 0.0

        # --- Entradas ---
        self.u1 = 0.0   # velocidad lineal [m/s]
        self.u2 = 0.0   # ángulo de dirección [rad]

        # --- Suscriptor a entradas ---
        self.sub = self.create_subscription(
            Float32MultiArray,
            'bicycle_inputs',
            self.inputs_callback,
            10)

        # --- Publicador de estados ---
        self.pub = self.create_publisher(Pose2D, 'bicycle_state', 10)

        # --- Timer principal ---
        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info('Nodo Bicycle Model iniciado.')

    def inputs_callback(self, msg):
        # msg.data = [u1, u2]
        self.u1 = msg.data[0]
        self.u2 = msg.data[1]

    def update(self):
        # --- Modelo cinemático ---
        x_dot = self.u1 * math.cos(self.psi)
        y_dot = self.u1 * math.sin(self.psi)
        psi_dot = (self.u1 / self.lwb) * math.tan(self.u2)

        # --- Integración de Euler ---
        self.x += x_dot * self.dt
        self.y += y_dot * self.dt
        self.psi += psi_dot * self.dt

        # --- Publicar estado ---
        state = Pose2D()
        state.x = self.x
        state.y = self.y
        state.theta = self.psi
        self.pub.publish(state)

        # --- Log opcional ---
        self.get_logger().info(f"x={self.x:.2f}, y={self.y:.2f}, psi={self.psi:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = BicycleModelNodeCharlie()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
