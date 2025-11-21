import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import TwistStamped
import numpy as np


class ModeloVelocidad:
    def __init__(self):
        # 🔹 Modelos experimentales ajustados (en m/s)
        # Resultados reales del ajuste (+0.1 y -0.1)
        self.modelos = {
            "positivo": {"K": 0.026482, "tau": 0.20236},
            "negativo": {"K": 0.026482*0.8, "tau": 0.20236}
        }

    def obtener_modelo(self, entrada):
        """Devuelve K y tau según el signo de la entrada."""
        if entrada >= 0:
            return self.modelos["positivo"]["K"], self.modelos["positivo"]["tau"]
        else:
            return self.modelos["negativo"]["K"], self.modelos["negativo"]["tau"]

class ModeloLinealNode(Node):
    def __init__(self):
        super().__init__('modelo_lineal_node')
        self.modelo = ModeloVelocidad()

        # Variables internas
        self.v_in = 0.0          # entrada normalizada (-1..1)
        self.v_est = 0.0         # velocidad estimada (m/s)
        self.v_est_prev = 0.0
        self.dt = 0.1         
        self.last_time = None

        # Comparación con velocidad real (opcional)
        self.v_real = 0.0
        self.enable_comparison = True

        # 🔹 Suscriptores y publicadores
        self.create_subscription(TwistStamped, '/cmd_vel', self.lineal_callback, 10)

        self.pub_modelo = self.create_publisher(Float32MultiArray, '/modelo/lineal', 10)

        # 🔹 Timer de actualización
        self.create_timer(self.dt, self.actualizar_modelo)
        self.get_logger().info("🚀 Nodo modelo_lineal_node iniciado con modelos ajustados en m/s.")

    def lineal_callback(self, msg: TwistStamped):
        """
        Recibe la entrada de velocidad (por ejemplo, PWM o cmd_vel normalizado)
        y la escala a -0.1 .. 0.1 (según modelos experimentales).
        """
        self.v_in = float(np.clip(msg.twist.linear.x*200, -100, 100))
        # 🔹 Zona muerta: si la entrada está por debajo del 50% => no se mueve
        if 60 < self.v_in < 60:
            self.v_in = 0.0

        # if 0.1 > self.v_in and 0.0 < self.v_in:
        #     self.v_in = 0
        # elif -0.1 < self.v_in and 0.0 > self.v_in:
        #     self.v_in = 0
        # else: 
        #     pass

    def actualizar_modelo(self):
        """Actualiza el modelo lineal y publica resultados."""
        current_time = self.get_clock().now()
        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt <= 0 or dt > 0.5:
            dt = self.dt

        # Si la entrada es casi nula, reiniciar
        if self.v_in == 0.0:
            self.v_est = 0.0
            self.v_est_prev = 0.0
            msg = Float32MultiArray()
            msg.data = [0.0, 0.0, 0.0, 0.0]
            self.pub_modelo.publish(msg)
            return

        # Obtener modelo (según signo de la entrada)
        K, tau = self.modelo.obtener_modelo(self.v_in)

        # 🔹 Ecuación diferencial discreta: v_dot = (K*u - v_est) / tau
        v_dot = (K * self.v_in - self.v_est) / tau

        # Integrar para estimar velocidad
        self.v_est += v_dot * dt

        # Limitar a rango de trabajo típico del robot
        self.v_est = float(np.clip(self.v_est, -1.0, 1.0))  # rango m/s

        # Calcular aceleración (ax = Δv / Δt)
        ax = (self.v_est - self.v_est_prev) / dt if dt > 0 else 0.0
        self.v_est_prev = self.v_est
        ay = 0.0  # movimiento lineal en X

        # Publicar [ax, ay, vx, vy]
        msg = Float32MultiArray()
        msg.data = [ax, ay, self.v_est, 0.0]
        self.pub_modelo.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = ModeloLinealNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
