import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray, Float64
import numpy as np
import math


class VelocityModelNodeCharlie(Node):
    def __init__(self):
        super().__init__('velocity_model_node_charlie')

        # === Parámetros ===
        self.dt = 1/20.0  # tiempo de muestreo (s)
        self.K = 0.026482
        self.tau = 0.20236
        self.max_angle_deg = 45.0
        self.min_angle_deg = -45.0
        self.L = 0.26  # Distancia entre ejes del robot (m) - ajustar según tu robot

        # === Límite de PWM ===
        self.enable_pwm_limit = True  # Habilitar/deshabilitar límite de PWM
        self.max_pwm_percent = 0.65  # Límite de PWM: ±65%

        # === Discretización (Euler hacia atrás) ===
        self.a1 = self.tau / (self.tau + self.dt)
        self.b0 = (self.dt * self.K) / (self.tau + self.dt)

        # Estado previo
        self.y_prev = 0.0
        
        # Orientación
        self.yaw_model = 0.0  # Yaw estimado por modelo cinemático
        self.yaw_sensors = 0.0  # Yaw de giroscopio+magnetómetro
        self.yaw_fused_final = 0.0  # Fusión final
        
        # Parámetro de fusión
        self.alpha_fusion = 0.8  # Peso de sensores (0.8) vs modelo (0.2)

        # === Suscriptores ===
        self.cmd_sub = self.create_subscription(
            TwistStamped,
            '/cmd_vel',
            self.cmd_callback,
            10
        )
        
        # self.yaw_sub = self.create_subscription(
        #     Float64,
        #     '/yaw/fused',
        #     self.yaw_callback,
        #     10
        # )

        # === Publicadores ===
        self.bicycle_pub = self.create_publisher(Float32MultiArray, '/bicycle_inputs', 10)

        # Timer de actualización
        self.timer = self.create_timer(self.dt, self.update)

        # Variables de entrada actuales
        self.input_velocity_cmd = 0.0
        self.input_steer_cmd = 0.0
        self.current_velocity = 0.0
        self.current_steer_angle = 0.0

        self.get_logger().info('Nodo Velocity Model con fusión de orientación iniciado.')
        self.get_logger().info(f'Alpha fusión: {self.alpha_fusion} (sensores vs modelo)')
        if self.enable_pwm_limit:
            self.get_logger().info(f'⚠️  Límite de PWM habilitado: ±{self.max_pwm_percent*100:.0f}%')
        else:
            self.get_logger().info('ℹ️  Límite de PWM deshabilitado')

    def yaw_callback(self, msg):
        """Recibe la orientación fusionada de giroscopio+magnetómetro"""
        self.yaw_sensors = msg.data

    def cmd_callback(self, msg):
        self.input_velocity_cmd = msg.twist.linear.x   # [-0.5, 0.5]
        self.input_steer_cmd = msg.twist.angular.z     # [-0.5, 0.5]
        
    def aplicar_deadzone(self, u_pwm, deadzone=60, max_pwm=100):
        if abs(u_pwm) < deadzone:
            return 0.0
        else:
            return np.sign(u_pwm) * (abs(u_pwm) - deadzone) / (max_pwm - deadzone)

    def normalize_angle(self, angle):
        """Normaliza el ángulo al rango [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def update(self):
        # === 1. Modelo de velocidad lineal ===
        u_pwm = np.clip(self.input_velocity_cmd, -0.5, 0.5) * 200.0
        
        # Aplicar límite de PWM si está habilitado
        if self.enable_pwm_limit:
            max_pwm_value = self.max_pwm_percent * 200.0  # 70% de 200 = 140
            u_pwm = np.clip(u_pwm, -max_pwm_value, max_pwm_value)
        
        u_eff = self.aplicar_deadzone(u_pwm, deadzone=60, max_pwm=100)
        u_model = u_eff
        
        # Ecuación en diferencias
        y = self.a1 * self.y_prev + self.b0 * (u_model * 100.0)
        self.y_prev = y
        self.current_velocity = y

        # === 2. Cálculo del ángulo de dirección ===
        steer_norm = np.clip(self.input_steer_cmd, -0.5, 0.5)
        steer_angle_deg = steer_norm * (self.max_angle_deg / 0.5)
        steer_angle_rad = math.radians(steer_angle_deg)
        self.current_steer_angle = steer_angle_rad

        # === 3. Modelo cinemático de bicicleta para yaw ===
        # Velocidad angular = (v / L) * tan(δ)
        if abs(self.current_velocity) > 0.01:  # Evitar división por cero
            omega_model = (self.current_velocity / self.L) * math.tan(self.current_steer_angle)
            self.yaw_model += omega_model * self.dt
            self.yaw_model = self.normalize_angle(self.yaw_model)
        
        # === 4. Fusión de orientación ===
        # Fusión complementaria
        # self.yaw_fused_final = self.alpha_fusion * self.yaw_sensors + (1 - self.alpha_fusion) * self.yaw_model

        # === 5. Publicar salidas ===
        # Publicar entradas del modelo de bicicleta
        bicycle_msg = Float32MultiArray()
        bicycle_msg.data = [float(y), float(self.current_steer_angle)]
        self.bicycle_pub.publish(bicycle_msg)
        
        # # Publicar yaw del modelo
        # yaw_model_msg = Float64()
        # yaw_model_msg.data = self.yaw_model
        # self.yaw_model_pub.publish(yaw_model_msg)
        
        # # Publicar yaw fusionado final
        # yaw_fused_msg = Float64()
        # yaw_fused_msg.data = self.yaw_fused_final
        # self.yaw_fused_final_pub.publish(yaw_fused_msg)

        # === Log ===
        # self.get_logger().info(
        #     f"V={y:.2f} m/s, δ={steer_angle_deg:.1f}° | "
        #     f"Yaw Model: {math.degrees(self.yaw_model):.1f}° | "
        #     f"Yaw Sensors: {math.degrees(self.yaw_sensors):.1f}° | "
        #     f"Yaw Fused: {math.degrees(self.yaw_fused_final):.1f}°"
        # )


def main(args=None):
    rclpy.init(args=args)
    node = VelocityModelNodeCharlie()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
