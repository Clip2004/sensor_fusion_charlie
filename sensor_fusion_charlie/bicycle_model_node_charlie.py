import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray, Float64
from tf_transformations import quaternion_from_euler


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
        self.yaw_fused = 0.0

        # --- Suscriptor a entradas ---
        self.sub = self.create_subscription(
            Float32MultiArray,
            'bicycle_inputs',
            self.inputs_callback,
            10)
        self.yaw_sub = self.create_subscription(
            Float64,
            '/yaw/fused',
            self.yaw_callback,
            10)
        # --- Publicador de estados ---
        self.pub = self.create_publisher(Pose, '/robot1/pose', 10)

        # --- Timer principal ---
        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info('Nodo Bicycle Model iniciado.')

    def inputs_callback(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warn('Mensaje de entradas incompleto.')
            return
        # msg.data = [u1, u2]
        self.u1 = msg.data[0]
        self.u2 = msg.data[1]
    def yaw_callback(self,msg):
        self.yaw_fused = float(msg.data)
    def update(self):
        # --- Modelo cinemático ---
        x_dot = self.u1 * math.cos(self.yaw_fused)
        y_dot = self.u1 * math.sin(self.yaw_fused)
        # psi_dot = (self.u1 / self.lwb) * math.tan(self.u2)

        # --- Integración de Euler ---
        self.x += x_dot * self.dt
        self.y += y_dot * self.dt
        # self.psi += psi_dot * self.dt
        
        # Convertir a cuaternión para ROS
        qx, qy, qz, qw = quaternion_from_euler(0, 0, self.yaw_fused)

        # Crear mensaje Pose
        pose_msg = Pose()
        pose_msg.position.x = float(self.x)
        pose_msg.position.y = float(self.y)
        pose_msg.position.z = 0.0
        pose_msg.orientation.x = float(qx)
        pose_msg.orientation.y = float(qy)
        pose_msg.orientation.z = float(qz)
        pose_msg.orientation.w = float(qw)
        self.pub.publish(pose_msg)

        # --- Log opcional ---
        # self.get_logger().info(
        #     f"x={self.x:.2f} m, y={self.y:.2f} m, θ={math.degrees(self.psi):.1f}°, v={self.u1:.2f} m/s, δ={math.degrees(self.u2):.1f}°"
        # )
def main(args=None):
    rclpy.init(args=args)
    node = BicycleModelNodeCharlie()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
