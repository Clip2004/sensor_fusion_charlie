#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math

class YawEstimationNode(Node):
    def __init__(self):
        super().__init__('yaw_estimation_node_charlie')
        
        # Suscripción al topic IMU
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10
        )
        
        # Publicadores para las estimaciones de yaw
        self.yaw_gyro_pub = self.create_publisher(Float64, '/yaw/gyro', 10)
        self.yaw_accel_pub = self.create_publisher(Float64, '/yaw/accel', 10)
        self.yaw_fused_pub = self.create_publisher(Float64, '/yaw/fused', 10)
        
        # Estado interno
        self.yaw_gyro = 0.0  # Yaw estimado por giroscopio (integración)
        self.yaw_accel = 0.0  # Yaw estimado por acelerómetro
        self.last_time = None
        
        # Parámetros de fusión complementaria
        self.alpha = 0.98  # Peso del giroscopio en filtro complementario
        self.yaw_fused = 0.0  # Yaw fusionado
        
        self.get_logger().info('🚀 Nodo de estimación de yaw iniciado')
        self.get_logger().info('📡 Suscrito a /imu/data_raw')
        self.get_logger().info(f'🔧 Alpha (peso gyro): {self.alpha}')

    def imu_callback(self, msg: Imu):
        """
        Procesa mensajes IMU y estima yaw mediante:
        1. Integración de velocidad angular en z (giroscopio)
        2. Cálculo de ángulo desde aceleraciones x,y (acelerómetro)
        3. Fusión complementaria de ambas estimaciones
        """
        current_time = self.get_clock().now()
        
        # Primera iteración: inicializar tiempo
        if self.last_time is None:
            self.last_time = current_time
            return
        
        # Calcular dt
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time
        
        # --- 1. ESTIMACIÓN POR GIROSCOPIO (Integración) ---
        angular_vel_z = msg.angular_velocity.z
        self.yaw_gyro += angular_vel_z * dt
        
        # --- 2. ESTIMACIÓN POR ACELERÓMETRO ---
        # El acelerómetro mide la componente gravitacional
        # Podemos estimar el ángulo de inclinación en el plano XY
        # atan2(ay, ax) nos da una estimación del yaw si el robot está en un plano horizontal
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        
        # Estimación de yaw basada en aceleración (asumiendo movimiento en plano horizontal)
        # Nota: esto es una aproximación; en la práctica, el acelerómetro da mejor info 
        # sobre pitch/roll que sobre yaw. Para yaw puro se necesita magnetómetro.
        # Aquí calculamos el ángulo de la aceleración horizontal como proxy
        if abs(accel_x) > 0.01 or abs(accel_y) > 0.01:
            self.yaw_accel = math.atan2(accel_y, accel_x)
        
        # --- 3. FUSIÓN COMPLEMENTARIA ---
        # Combina la precisión a corto plazo del giroscopio con la estabilidad 
        # a largo plazo del acelerómetro
        self.yaw_fused = self.alpha * (self.yaw_fused + angular_vel_z * dt) + \
                         (1 - self.alpha) * self.yaw_accel
        
        # --- PUBLICAR RESULTADOS ---
        # Yaw del giroscopio
        yaw_gyro_msg = Float64()
        yaw_gyro_msg.data = self.yaw_gyro
        self.yaw_gyro_pub.publish(yaw_gyro_msg)
        
        # Yaw del acelerómetro
        yaw_accel_msg = Float64()
        yaw_accel_msg.data = self.yaw_accel
        self.yaw_accel_pub.publish(yaw_accel_msg)
        
        # Yaw fusionado
        yaw_fused_msg = Float64()
        yaw_fused_msg.data = self.yaw_fused
        self.yaw_fused_pub.publish(yaw_fused_msg)
        
        # Log periódico (cada 50 mensajes para no saturar)
        if not hasattr(self, 'msg_count'):
            self.msg_count = 0
        self.msg_count += 1
        
        if self.msg_count % 50 == 0:
            self.get_logger().info(
                f'📊 Yaw - Gyro: {math.degrees(self.yaw_gyro):.2f}° | '
                f'Accel: {math.degrees(self.yaw_accel):.2f}° | '
                f'Fused: {math.degrees(self.yaw_fused):.2f}°'
            )

def main(args=None):
    rclpy.init(args=args)
    node = YawEstimationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('⏹️  Nodo detenido por el usuario')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
