#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time


class VelocityControl(Node):
    def __init__(self):
        super().__init__('vel_control_node_charlie')
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel_ctrl', 10)
        self.get_logger().info("✅ Velocity Control Node started")

        # Ejecuta el bucle principal
        self.run_loop()

    def scale_value(self, value):
        """Escala entrada [-100,100] a [-0.5,0.5]."""
        return max(min((value / 100.0) * 0.5, 0.5), -0.5)

    def publish_velocity(self, linear, angular):
        """Publica un mensaje TwistStamped."""
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'
        twist_msg.twist.linear.x = linear
        twist_msg.twist.angular.z = angular
        self.publisher_.publish(twist_msg)

    def run_loop(self):
        """Bucle principal: recibe lineal, angular y tiempo, envía y detiene."""
        while rclpy.ok():
            try:
                user_input = input("→ Ingrese lineal angular tiempo (por ej. 50 20 3): ").split()
                if len(user_input) != 3:
                    print("⚠️ Por favor ingrese tres valores: lineal angular tiempo")
                    continue

                linear_in, angular_in, duration = map(float, user_input)
                if not (-100 <= linear_in <= 100 and -100 <= angular_in <= 100 and duration > 0):
                    print("⚠️ Valores fuera de rango. Velocidades en [-100,100], tiempo > 0")
                    continue

                # Escalado a [-0.5, 0.5]
                linear_scaled = self.scale_value(linear_in)
                angular_scaled = self.scale_value(angular_in)

                # Publicar velocidades
                self.get_logger().info(f"🚀 Enviando | Lineal: {linear_scaled:.2f}, Angular: {angular_scaled:.2f} | Tiempo: {duration:.2f}s")
                start_time = time.time()

                # Publica durante el tiempo indicado
                while time.time() - start_time < duration and rclpy.ok():
                    self.publish_velocity(linear_scaled, angular_scaled)
                    time.sleep(0.1)  # frecuencia de actualización 10 Hz

                # Detiene el movimiento
                self.publish_velocity(0.0, 0.0)
                self.get_logger().info("🛑 Movimiento detenido. Listo para el siguiente comando.\n")

            except ValueError:
                print("⚠️ Entrada inválida, use números (ej. 50 20 3)")
            except EOFError:
                break


def main(args=None):
    rclpy.init(args=args)
    node = VelocityControl()
    try:
        node.run_loop()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
