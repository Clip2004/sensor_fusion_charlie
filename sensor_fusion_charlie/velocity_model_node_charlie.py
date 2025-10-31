import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray
import numpy as np
from scipy import signal
import math


class VelocityModelNodeCharlie(Node):
    def __init__(self):
        super().__init__('velocity_model_node_charlie')

        # === Parámetros ===
        self.dt = 0.05  # tiempo de muestreo (s)
        self.max_angle_deg = 45.0
        self.min_angle_deg = -45.0

        # === Modelo de velocidad (función de transferencia discretizada) ===
        # Ejemplo: G(s) = 1 / (0.5s + 1)
        num = [0.026482]
        den = [0.20236, 1.0]
        system = signal.TransferFunction(num, den)
        self.sysd = system.to_discrete(self.dt, method='zoh')

        # Guardamos coeficientes discretos
        self.b = self.sysd.num[0]
        self.a = self.sysd.den[0]

        # Estados del filtro (inicializados en cero)
        self.u_hist = np.zeros(len(self.b))
        self.y_hist = np.zeros(len(self.a))

        # === Suscriptor a comandos Twist ===
        self.sub = self.create_subscription(
            TwistStamped,
            '/cmd_vel',
            self.cmd_callback,
            10)

        # === Publicador de salida (Float32MultiArray) ===
        self.pub = self.create_publisher(Float32MultiArray, '/bicycle_inputs', 10)

        self.timer = self.create_timer(self.dt, self.update)

        # Variables de entrada actual
        self.input_velocity = 0.0
        self.input_steer_cmd = 0.0

        self.get_logger().info('Nodo Velocity Model iniciado.')

    # --- Callback de entrada ---
    def cmd_callback(self, msg):
        self.input_velocity = msg.twist.linear.x   # entrada al modelo de velocidad
        self.input_steer_cmd = msg.twist.angular.x # comando de dirección

    # --- Actualización del modelo ---
    def update(self):
        # Filtrado de velocidad (modelo dinámico)
        u = self.input_velocity

        # Desplazamos los históricos
        self.u_hist = np.roll(self.u_hist, 1)
        self.y_hist = np.roll(self.y_hist, 1)
        self.u_hist[0] = u

        # Ecuación de diferencia (filtro IIR)
        y = (np.dot(self.b, self.u_hist) - np.dot(self.a[1:], self.y_hist[1:])) / self.a[0]
        self.y_hist[0] = y

        # Remapeo del ángulo de dirección (-1 a 1) → (-45°, 45°)
        steer_norm = max(min(self.input_steer_cmd, 0.5), -0.5)
        steer_angle = math.radians(
            (steer_norm * (self.max_angle_deg - self.min_angle_deg) / 2.0)
        )

        # Publicar mensaje con [velocidad, ángulo]
        msg = Float32MultiArray()
        msg.data = [float(y), float(steer_angle)]
        self.pub.publish(msg)

        # Log de depuración
        self.get_logger().info(f"u1={y:.2f} m/s, u2={math.degrees(steer_angle):.2f}°")


def main(args=None):
    rclpy.init(args=args)
    node = VelocityModelNodeCharlie()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
