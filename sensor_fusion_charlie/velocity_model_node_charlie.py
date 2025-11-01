import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray
import numpy as np
import math


class VelocityModelNodeCharlie(Node):
    def __init__(self):
        super().__init__('velocity_model_node_charlie')

        # === Parámetros ===
        self.dt = 0.05  # tiempo de muestreo (s)
        self.K = 0.026482
        self.tau = 0.20236
        self.max_angle_deg = 45.0
        self.min_angle_deg = -45.0

        # === Discretización (Euler hacia atrás) ===
        self.a1 = self.tau / (self.tau + self.dt)
        self.b0 = (self.dt * self.K) / (self.tau + self.dt)

        # Estado previo
        self.y_prev = 0.0

        # === Suscriptor y publicador ===
        self.sub = self.create_subscription(
            TwistStamped,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.pub = self.create_publisher(Float32MultiArray, '/bicycle_inputs', 10)

        # Timer de actualización
        self.timer = self.create_timer(self.dt, self.update)

        # Variables de entrada actuales
        self.input_velocity_cmd = 0.0
        self.input_steer_cmd = 0.0

        self.get_logger().info('Nodo Velocity Model (con escalamiento) iniciado.')

    # --- Callback de entrada ---
    def cmd_callback(self, msg):
        self.input_velocity_cmd = msg.twist.linear.x   # [-0.5, 0.5]
        self.input_steer_cmd = msg.twist.angular.z     # [-0.5, 0.5]
        
    def aplicar_deadzone(self, u_pwm, deadzone=60, max_pwm=100):
        if abs(u_pwm) < deadzone:
            return 0.0
        else:
            # Escala linealmente desde el umbral hasta el máximo
            return np.sign(u_pwm) * (abs(u_pwm) - deadzone) / (max_pwm - deadzone)

    # --- Actualización del modelo ---
    def update(self):
        # === 1. Escalamiento de entrada de velocidad ===
        # De [-0.5, 0.5] → [-100, 100]
        u_pwm = np.clip(self.input_velocity_cmd, -0.5, 0.5) * 200.0

        # === 2. Aplicar zona muerta ===
        u_eff = self.aplicar_deadzone(u_pwm, deadzone=60, max_pwm=100)

        # Escalamos u_eff al rango [0,1] para el modelo
        u_model = u_eff  # ya está normalizado a [-1,1]

        # === 3. Ecuación en diferencias (modelo dinámico) ===
        y = self.a1 * self.y_prev + self.b0 * (u_model * 100.0)  # reescalar según tu ganancia
        self.y_prev = y

        # === 4. Remapeo del ángulo de dirección ===
        steer_norm = np.clip(self.input_steer_cmd, -0.5, 0.5)
        steer_angle_deg = steer_norm * (self.max_angle_deg / 0.5)
        steer_angle_rad = math.radians(steer_angle_deg)

        # === 5. Publicar salida ===
        msg = Float32MultiArray()
        msg.data = [float(y), float(steer_angle_rad)]
        self.pub.publish(msg)

        # === Log ===
        self.get_logger().info(
            f"PWM={u_pwm:.1f}, Efectivo={u_eff:.2f}, Salida={y:.2f} m/s, Dirección={steer_angle_deg:.1f}°"
        )



def main(args=None):
    rclpy.init(args=args)
    node = VelocityModelNodeCharlie()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
