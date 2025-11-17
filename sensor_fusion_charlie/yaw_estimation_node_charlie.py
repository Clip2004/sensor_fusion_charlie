#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math
import numpy as np

# ========== Funciones auxiliares ==========
def quat_to_yaw(w, x, y, z) -> float:
    """Convierte quaternion a yaw (ángulo alrededor del eje Z)"""
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    return math.atan2(siny_cosp, cosy_cosp)

def wrap_pi(a):
    """Normaliza ángulo entre -pi y pi"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi

# ========== Filtro Madgwick ==========
class MadgwickAHRS:
    """
    Filtro Madgwick para fusión de IMU (giroscopio + acelerómetro)
    Estima orientación usando un filtro de gradiente descendente
    """
    def __init__(self, beta=0.1):
        # Quaternion de orientación (inicialmente sin rotación)
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        self.beta = beta  # Ganancia del filtro (mayor = más peso a acelerómetro)

    def update_imu(self, gx, gy, gz, ax, ay, az, dt):
        """
        Actualiza el quaternion con datos de giroscopio y acelerómetro
        gx, gy, gz: velocidades angulares (rad/s)
        ax, ay, az: aceleraciones (m/s²)
        dt: delta de tiempo (s)
        """
        # Normalizar acelerómetro
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm < 1e-6:
            # Si acelerómetro inválido, solo integrar giroscopio
            self._integrate_gyro(gx, gy, gz, dt)
            return
        ax /= norm
        ay /= norm
        az /= norm

        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3

        # Gradiente descendente correctivo (versión IMU)
        _2q0 = 2.0*q0; _2q1 = 2.0*q1; _2q2 = 2.0*q2; _2q3 = 2.0*q3
        _4q0 = 4.0*q0; _4q1 = 4.0*q1; _4q2 = 4.0*q2
        _8q1 = 8.0*q1; _8q2 = 8.0*q2
        q0q0 = q0*q0; q1q1 = q1*q1; q2q2 = q2*q2; q3q3 = q3*q3

        # Función objetivo y Jacobiano
        s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay
        s1 = _4q1*q3q3 - _2q3*ax + 4.0*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az
        s2 = 4.0*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az
        s3 = 4.0*q1q1*q3 - _2q1*ax + 4.0*q2q2*q3 - _2q2*ay

        # Normalizar gradiente
        norm_s = math.sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3)
        if norm_s > 1e-9:
            s0 /= norm_s; s1 /= norm_s; s2 /= norm_s; s3 /= norm_s
        else:
            s0 = s1 = s2 = s3 = 0.0

        # Tasa de cambio del quaternion por giroscopio
        qDot0 = 0.5*(-q1*gx - q2*gy - q3*gz) - self.beta*s0
        qDot1 = 0.5*( q0*gx + q2*gz - q3*gy) - self.beta*s1
        qDot2 = 0.5*( q0*gy - q1*gz + q3*gx) - self.beta*s2
        qDot3 = 0.5*( q0*gz + q1*gy - q2*gx) - self.beta*s3

        # Integrar para obtener quaternion
        q0 += qDot0*dt; q1 += qDot1*dt; q2 += qDot2*dt; q3 += qDot3*dt

        # Normalizar quaternion
        norm_q = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        if norm_q < 1e-9:
            return
        self.q0 = q0 / norm_q
        self.q1 = q1 / norm_q
        self.q2 = q2 / norm_q
        self.q3 = q3 / norm_q

    def _integrate_gyro(self, gx, gy, gz, dt):
        """Integra solo giroscopio (cuando acelerómetro no es válido)"""
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        qDot0 = 0.5*(-q1*gx - q2*gy - q3*gz)
        qDot1 = 0.5*( q0*gx + q2*gz - q3*gy)
        qDot2 = 0.5*( q0*gy - q1*gz + q3*gx)
        qDot3 = 0.5*( q0*gz + q1*gy - q2*gx)
        q0 += qDot0*dt; q1 += qDot1*dt; q2 += qDot2*dt; q3 += qDot3*dt
        norm_q = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        if norm_q < 1e-9:
            return
        self.q0 = q0 / norm_q
        self.q1 = q1 / norm_q
        self.q2 = q2 / norm_q
        self.q3 = q3 / norm_q

    def yaw(self):
        """Retorna el ángulo yaw actual desde el quaternion"""
        return quat_to_yaw(self.q0, self.q1, self.q2, self.q3)

# ========== Nodo ROS2 ==========

# ========== Nodo ROS2 ==========
class YawGyroNode(Node):
    def __init__(self):
        super().__init__('yaw_estimation_node_charlie')
        
        # Suscripción a IMU
        self.sub = self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10)
        
        # Publicadores para diferentes estimaciones
        self.pub_gyro = self.create_publisher(Float64, '/yaw/gyro', 10)
        self.pub_madgwick = self.create_publisher(Float64, '/yaw/madgwick', 10)
        self.pub_fused = self.create_publisher(Float64, '/yaw/fused', 10)
        
        # Estado para estimación por giroscopio puro
        self.last_time = None
        self.yaw_gyro = 0.0
        self.beta_lpf = 0.95  # filtro pasa bajos
        self.filtered_gyro_z = 0.0
        self.gyro_bias_z = 0.0
        self.bias_samples = []
        self.bias_ready = False
        
        # Filtro Madgwick
        self.madgwick = MadgwickAHRS(beta=0.1)  # beta: 0.1 es un buen balance
        
        # Parámetro de fusión entre gyro y Madgwick
        self.alpha_fusion = 0.7  # Peso de gyro en fusión (0.7 = 70% gyro, 30% Madgwick)
        self.yaw_fused = 0.0
        
        self.get_logger().info('🚀 Nodo de estimación de yaw (Gyro + Madgwick) iniciado')
        self.get_logger().info(f'   Beta Madgwick: {self.madgwick.beta}')
        self.get_logger().info(f'   Alpha fusión: {self.alpha_fusion}')

    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).nanoseconds * 1e-9
        if dt <= 0:  # Protección contra dt inválido
            return
        self.last_time = current_time

        # Extraer datos del IMU
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # === Calibración inicial de bias del giroscopio ===
        if not self.bias_ready:
            self.bias_samples.append(gz)
            if len(self.bias_samples) >= 200:  # ~2 segundos de datos a 100Hz
                self.gyro_bias_z = np.mean(self.bias_samples)
                self.bias_ready = True
                self.get_logger().info(f'🧭 Bias estimado: {self.gyro_bias_z:.6f} rad/s')
            return

        # === 1. ESTIMACIÓN POR GIROSCOPIO PURO (con filtro pasa bajos) ===
        # Filtro pasa bajos sobre gz
        self.filtered_gyro_z = (
            self.beta_lpf * self.filtered_gyro_z + (1 - self.beta_lpf) * gz
        )
        
        # Compensar bias y calcular yaw
        angular_vel_z = self.filtered_gyro_z - self.gyro_bias_z
        self.yaw_gyro += angular_vel_z * dt
        self.yaw_gyro = wrap_pi(self.yaw_gyro)  # Limitar entre -pi y pi

        # === 2. ESTIMACIÓN CON FILTRO MADGWICK ===
        # Compensar bias en todos los ejes del giroscopio antes de Madgwick
        gx_corr = gx
        gy_corr = gy
        gz_corr = gz - self.gyro_bias_z
        
        # Actualizar filtro Madgwick
        self.madgwick.update_imu(gx_corr, gy_corr, gz_corr, ax, ay, az, dt)
        yaw_madgwick = self.madgwick.yaw()

        # === 3. FUSIÓN DE AMBAS ESTIMACIONES ===
        # Filtro complementario entre gyro puro y Madgwick
        # alpha_fusion alto = más confianza en gyro puro
        # alpha_fusion bajo = más confianza en Madgwick
        self.yaw_fused = self.alpha_fusion * self.yaw_gyro + \
                        (1 - self.alpha_fusion) * yaw_madgwick
        self.yaw_fused = wrap_pi(self.yaw_fused)

        # === PUBLICAR RESULTADOS ===
        # Yaw del giroscopio puro
        msg_gyro = Float64()
        msg_gyro.data = -self.yaw_gyro  # Negativo para seguir convención
        self.pub_gyro.publish(msg_gyro)
        
        # Yaw de Madgwick
        msg_madgwick = Float64()
        msg_madgwick.data = -yaw_madgwick
        self.pub_madgwick.publish(msg_madgwick)
        
        # Yaw fusionado
        msg_fused = Float64()
        msg_fused.data = -self.yaw_fused
        self.pub_fused.publish(msg_fused)

        # Log cada 50 mensajes
        if not hasattr(self, 'count'):
            self.count = 0
        self.count += 1
        if self.count % 50 == 0:
            self.get_logger().info(
                f'📊 Yaw → Gyro: {math.degrees(self.yaw_gyro):7.2f}° | '
                f'Madgwick: {math.degrees(yaw_madgwick):7.2f}° | '
                f'Fused: {math.degrees(self.yaw_fused):7.2f}°'
            )

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
