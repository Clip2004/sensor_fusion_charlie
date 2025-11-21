#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np


# ============================================================
#                IMPLEMENTACIÓN FILTRO MADGWICK
# ============================================================
class MadgwickFilter:
    def __init__(self, beta=0.1, freq=20.0):
        self.beta = beta
        self.dt = 1.0 / freq

        # Estado del filtro (cuaternión)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

    def update(self, gx, gy, gz, ax, ay, az):
        """
        gx, gy, gz → giroscopio rad/s
        ax, ay, az → acelerómetro m/s^2
        """

        q1, q2, q3, q4 = self.q

        # Normalizar aceleración
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm < 1e-6:
            return self.q

        ax /= norm
        ay /= norm
        az /= norm

        # --- Gradiente de descenso ---
        f1 = 2*(q2*q4 - q1*q3) - ax
        f2 = 2*(q1*q2 + q3*q4) - ay
        f3 = 2*(0.5 - q2*q2 - q3*q3) - az

        J_11or24 = 2*q3
        J_12or23 = 2*q4
        J_13or22 = 2*q1
        J_14or21 = 2*q2
        
        # Jacobiano
        grad = np.array([
            J_13or22*f2 - J_11or24*f1,
            J_12or23*f1 + J_14or21*f2 - 4*q2*f3,
            J_12or23*f2 - J_13or22*f1 - 4*q3*f3,
            J_14or21*f1 + J_11or24*f2
        ])

        grad_norm = np.linalg.norm(grad)
        if grad_norm > 0:
            grad /= grad_norm

        # --- Derivada del cuaternión ---
        q_dot = 0.5 * self._quat_mult(self.q, np.array([0, gx, gy, gz])) - self.beta * grad

        # --- Integración ---
        self.q += q_dot * self.dt
        self.q /= np.linalg.norm(self.q)

        return self.q

    def _quat_mult(self, q, r):
        """Multiplicación de cuaterniones"""
        w0, x0, y0, z0 = q
        w1, x1, y1, z1 = r
        return np.array([
            -x0*x1 - y0*y1 - z0*z1 + w0*w1,
             x0*w1 + y0*z1 - z0*y1 + w0*x1,
            -x0*z1 + y0*w1 + z0*x1 + w0*y1,
             x0*y1 - y0*x1 + z0*w1 + w0*z1
        ])

    def get_euler(self):
        """Devuelve yaw estimado"""
        x, y, z, w = self.q[1], self.q[2], self.q[3], self.q[0]
        _, _, yaw = euler_from_quaternion([x, y, z, w])
        return yaw



# ============================================================
#           NODO PRINCIPAL CON FUSIÓN DE SENSORES
# ============================================================
class BicycleModelNodeCharlie(Node):
    def __init__(self):
        super().__init__('bicycle_model_node_charlie')

        # --- Parámetros del modelo ---
        self.lwb = 0.25
        self.dt = 0.05

        # --- Estados ---
        self.x = 0.0
        self.y = 0.0
        self.psi_model = 0.0          # orientación según modelo
        self.psi_fused = 0.0          # orientación final fusionada

        # Entradas
        self.u1 = 0.0
        self.u2 = 0.0

        # Filtro Madgwick (50 Hz)
        self.madgwick = MadgwickFilter(beta=0.1, freq=1/self.dt)
        # Matriz de rotación: R = Rz(90°) * Ry(180°)
        self.R_imu_to_body = np.array([
            [ 0., -1.,  0.],
            [-1.,  0.,  0.],
            [ 0.,  0., -1.]
        ])

        # --- Subscriptores ---
        self.sub_inputs = self.create_subscription(
            Float32MultiArray, 'bicycle_inputs', self.inputs_callback, 10)

        self.sub_imu = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 50)

        # --- Publicador ---
        self.pub = self.create_publisher(Pose, '/robot1/pose', 10)

        # Timer de actualización
        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info("Bicycle Model con Madgwick iniciado.")

    # ============================================================
    #                      CALLBACKS
    # ============================================================
    def inputs_callback(self, msg):
        if len(msg.data) < 2:
            return
        self.u1 = msg.data[0]
        self.u2 = msg.data[1]

    def imu_callback(self, msg: Imu):
        # Lectura cruda desde IMU (sensor frame)
        gyro_sensor = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ], dtype=float)

        accel_sensor = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ], dtype=float)

        # Transformar al body frame: v_body = R * v_sensor
        gyro_body = self.R_imu_to_body.dot(gyro_sensor)
        accel_body = self.R_imu_to_body.dot(accel_sensor)

        # Ahora alimentamos el Madgwick con las señales en body frame
        gx, gy, gz = float(gyro_body[0]), float(gyro_body[1]), float(gyro_body[2])
        ax, ay, az = float(accel_body[0]), float(accel_body[1]), float(accel_body[2])

        # Asegúrate que giroscopio esté en rad/s (la mayoría de IMUs lo están),
        # y la aceleración en m/s^2. Si tu IMU da en deg/s o g, convierte aquí.

        # Actualizar Madgwick con las mediciones transformadas
        self.madgwick.update(gx, gy, gz, ax, ay, az)


    # ============================================================
    #                     LOOP PRINCIPAL
    # ============================================================
    def update(self):
        # --- Modelo cinemático Ackermann ---
        x_dot = self.u1 * math.cos(self.psi_model)
        y_dot = self.u1 * math.sin(self.psi_model)
        psi_dot = (self.u1 / self.lwb) * math.tan(self.u2)

        self.x += x_dot * self.dt
        self.y += y_dot * self.dt
        self.psi_model += psi_dot * self.dt

        # --- Obtención del yaw de IMU vía Madgwick ---
        psi_imu = self.madgwick.get_euler()

        # --- FUSIÓN ORIENTACIONES ---
        alpha = 0.5  # coeficiente de fusión (IMU domina lento, modelo rápido)
        self.psi_fused = (1 - alpha) * self.psi_model + alpha * psi_imu

        # --- Publicar orientación fusionada ---
        q = quaternion_from_euler(0, 0, self.psi_fused)

        pose_msg = Pose()
        pose_msg.position.x = float(self.x)
        pose_msg.position.y = float(self.y)
        pose_msg.position.z = 0.0
        pose_msg.orientation.x = float(q[0])
        pose_msg.orientation.y = float(q[1])
        pose_msg.orientation.z = float(q[2])
        pose_msg.orientation.w = float(q[3])

        self.pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BicycleModelNodeCharlie()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
