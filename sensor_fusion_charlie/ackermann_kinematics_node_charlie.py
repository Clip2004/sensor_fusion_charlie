#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
import random

from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Point


# ============================================
# MAPA REAL DE U → ÁNGULO (DEGREES)
# ============================================
u_data_pos = np.array([0, 20, 30, 50, 100], dtype=float)
ang_data_pos = np.array([0, 5, 8.5, 19.5, 29.9], dtype=float)

u_data_neg = np.array([-100, -50, -30, -20, 0], dtype=float)
ang_data_neg = np.array([-31.5, -20, -9.5, -4.5, 0], dtype=float)


def direccion(u):
    u = np.clip(u*200, -100, 100)
    if u >= 0:
        ang = np.interp(u, u_data_pos, ang_data_pos)
    else:
        ang = np.interp(u, u_data_neg, ang_data_neg)
    return math.radians(ang)


# ============================================
# NODO ACKERMANN REALISTA — SOLO MODELO
# ============================================
class AckermannOdom(Node):

    def __init__(self):
        super().__init__("ackermann_kinematics_node_charlie")

        self.dt = 0.1
        self.L = 0.245

        # Estado del vehículo
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Dirección suavizada (realista)
        self.delta = 0.0       

        # Entradas
        self.v = 0.0
        self.u = 0.0

        # Suscriptores
        self.create_subscription(Float32MultiArray, '/modelo/lineal', self.cb_linear, 10)
        self.create_subscription(TwistStamped, "/cmd_vel", self.cb_u, 10)

        # Publicadores
        self.pub_pos = self.create_publisher(Point, "/posicion", 10)
        self.pub_yaw = self.create_publisher(Float32, "/modelo/yaw", 10)

        # Timer
        self.create_timer(self.dt, self.update_position)

        self.last_time = self.get_clock().now()
        self.get_logger().info("Nodo Ackermann-Odom REALISTA listo.")


    # -----------------------------------
    # CALLBACKS
    # -----------------------------------
    def cb_linear(self, msg: Float32MultiArray):
        self.v = msg.data[2]

    def cb_u(self, msg: TwistStamped):
        self.u = msg.twist.angular.z


    # -----------------------------------
    # MODELO ACKERMANN REALISTA
    # -----------------------------------
    def update_position(self):

        # dt real
        current_time = self.get_clock().now()
        self.dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # ========================================
        # 1. Dirección con transición suave
        # ========================================
        delta_target = direccion(self.u)
        alpha = 0.25  # suavizado realista
        self.delta = (1 - alpha) * self.delta + alpha * delta_target

        # Ruido realista cuando está casi recto
        if abs(self.delta) < math.radians(0.1):
            self.delta += random.uniform(-0.0005, 0.0005)

        # ========================================
        # 2. Cinemática Ackermann pura
        # ========================================
        omega = self.v * math.tan(self.delta) / self.L

        # Yaw con integración realista
        self.yaw += omega * self.dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # ========================================
        # 3. Movimiento en 2D
        # ========================================
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt

        # ========================================
        # Publicar posición
        # ========================================
        p = Point()
        p.x = self.x
        p.y = self.y
        p.z = 0.0
        self.pub_pos.publish(p)

        # Publicar yaw acumulado
        self.pub_yaw.publish(Float32(data=self.yaw))



def main(args=None):
    rclpy.init(args=args)
    node = AckermannOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
