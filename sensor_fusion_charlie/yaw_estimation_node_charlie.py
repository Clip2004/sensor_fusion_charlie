#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math
import numpy as np

class YawGyroNode(Node):
    def __init__(self):
        super().__init__('yaw_gyro_filtered_node')
        self.sub = self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10)
        self.pub = self.create_publisher(Float64, '/yaw/fused', 10)
        
        self.last_time = None
        self.yaw = 0.0
        self.beta = 0.95  # filtro pasa bajos
        self.filtered_gyro_z = 0.0
        self.gyro_bias_z = 0.0
        self.bias_samples = []
        self.bias_ready = False
        
        self.get_logger().info('🚀 Nodo de yaw por giroscopio iniciado')

    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        gyro_z = msg.angular_velocity.z

        # === Calibración inicial de bias ===
        if not self.bias_ready:
            self.bias_samples.append(gyro_z)
            if len(self.bias_samples) >= 200:  # ~2 segundos de datos a 100Hz
                self.gyro_bias_z = np.mean(self.bias_samples)
                self.bias_ready = True
                self.get_logger().info(f'🧭 Bias estimado: {self.gyro_bias_z:.6f} rad/s')
            return

        # === Filtro pasa bajos sobre gyro_z ===
        self.filtered_gyro_z = (
            self.beta * self.filtered_gyro_z + (1 - self.beta) * gyro_z
        )

        # === Compensar bias y calcular yaw ===
        angular_vel_z = self.filtered_gyro_z - self.gyro_bias_z
        self.yaw += angular_vel_z * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))  # limitar entre -pi y pi

        # === Publicar yaw filtrado ===
        msg_out = Float64()
        msg_out.data = -self.yaw
        self.pub.publish(msg_out)

        # Log cada 50 mensajes
        if not hasattr(self, 'count'):
            self.count = 0
        self.count += 1
        if self.count % 10 == 0:
            self.get_logger().info(f'📊 Yaw (gyro filtrado): {math.degrees(self.yaw):.2f}°')

def main(args=None):
    rclpy.init(args=args)
    node = YawGyroNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('⏹️ Nodo detenido por el usuario')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
