#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu, MagneticField, LaserScan
from geometry_msgs.msg import TwistStamped
from tf2_ros import TransformBroadcaster
import numpy as np

def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q

def quat_to_yaw(w, x, y, z) -> float:
    # ROS (x forward, y left, z up), yaw around +z
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    return math.atan2(siny_cosp, cosy_cosp)

def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def quaternion_from_euler(ai, aj, ak):
    """
    Convert intrinsic Euler angles (roll=ai, pitch=aj, yaw=ak)
    into a quaternion [x, y, z, w].

    Args:
        ai (float): roll angle in radians
        aj (float): pitch angle in radians
        ak (float): yaw angle in radians

    Returns:
        np.ndarray: quaternion [x, y, z, w] representing the same orientation
    """
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = Quaternion()
    q.x = cj*sc - sj*cs
    q.y = cj*ss + sj*cc
    q.z = cj*cs - sj*sc
    q.w = cj*cc + sj*ss

    return q
# === NEW: Minimal Madgwick (gyro+accel; optional mag) ===
class MadgwickAHRS:
    def __init__(self):       
        # Quaternion of sensor frame relative to Earth frame
        self.q0 = 1.0; self.q1 = 0.0; self.q2 = 0.0; self.q3 = 0.0
        self.beta = 0.05  # default gain, will be set by node
        self.has_mag = False

    def set_has_mag(self, has_mag: bool):
        self.has_mag = has_mag

    def update_imu(self, gx, gy, gz, ax, ay, az, dt):
        # Units: g in m/s^2 (we’ll normalize), gyro in rad/s, dt in s
        # Normalize accel
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm < 1e-6:
            # accel invalid; integrate gyro only
            self._integrate_gyro(gx, gy, gz, dt)
            return
        ax /= norm; ay /= norm; az /= norm

        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3

        # Gradient descent corrective step (IMU version)
        _2q0 = 2.0*q0; _2q1 = 2.0*q1; _2q2 = 2.0*q2; _2q3 = 2.0*q3
        _4q0 = 4.0*q0; _4q1 = 4.0*q1; _4q2 = 4.0*q2
        _8q1 = 8.0*q1; _8q2 = 8.0*q2
        q0q0 = q0*q0; q1q1 = q1*q1; q2q2 = q2*q2; q3q3 = q3*q3

        s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay
        s1 = _4q1*q3q3 - _2q3*ax + 4.0*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az
        s2 = 4.0*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az
        s3 = 4.0*q1q1*q3 - _2q1*ax + 4.0*q2q2*q3 - _2q2*ay

        norm_s = math.sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3)
        if norm_s > 1e-9:
            s0 /= norm_s; s1 /= norm_s; s2 /= norm_s; s3 /= norm_s
        else:
            s0 = s1 = s2 = s3 = 0.0

        # Gyro rate of change of quaternion
        qDot0 = 0.5*(-q1*gx - q2*gy - q3*gz) - self.beta*s0
        qDot1 = 0.5*( q0*gx + q2*gz - q3*gy) - self.beta*s1
        qDot2 = 0.5*( q0*gy - q1*gz + q3*gx) - self.beta*s2
        qDot3 = 0.5*( q0*gz + q1*gy - q2*gx) - self.beta*s3

        q0 += qDot0*dt; q1 += qDot1*dt; q2 += qDot2*dt; q3 += qDot3*dt

        # Normalize
        norm_q = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        if norm_q < 1e-9:  # fallback
            return
        self.q0 = q0 / norm_q; self.q1 = q1 / norm_q; self.q2 = q2 / norm_q; self.q3 = q3 / norm_q

    def _integrate_gyro(self, gx, gy, gz, dt):
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        qDot0 = 0.5*(-q1*gx - q2*gy - q3*gz)
        qDot1 = 0.5*( q0*gx + q2*gz - q3*gy)
        qDot2 = 0.5*( q0*gy - q1*gz + q3*gx)
        qDot3 = 0.5*( q0*gz + q1*gy - q2*gx)
        q0 += qDot0*dt; q1 += qDot1*dt; q2 += qDot2*dt; q3 += qDot3*dt
        norm_q = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        if norm_q < 1e-9:
            return
        self.q0 = q0 / norm_q; self.q1 = q1 / norm_q; self.q2 = q2 / norm_q; self.q3 = q3 / norm_q

    def yaw(self):
        return quat_to_yaw(self.q0, self.q1, self.q2, self.q3)

class AckermannKinematics(Node):
    def __init__(self):
        super().__init__('ackermann_kinematics')

        # === Declare ALL parameters first ===
        # Geometry (meters)
        self.declare_parameter('wheelbase', 0.26)
        self.declare_parameter('front_track', 0.235)
        self.declare_parameter('wheel_radius', 0.098/2)

        # Frames
        self.declare_parameter('odom_frame', 'odom2')
        self.declare_parameter('base_frame', 'base_link2')

        # Topics
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('joint_states_topic', 'joint_states')
        self.declare_parameter('scan_topic', '/scan')           # ⬅️ NUEVO
        self.declare_parameter('scan_republish_topic', '/scan2') # ⬅️ NUEVO
        # Timing / limits
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('max_steer', 0.5)
        self.declare_parameter('steer_slew', 10.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('max_velocity_limit', 0.65)  # ⬅️ NUEVO: Límite del 80%

        # Joint names
        self.declare_parameter('fl_steer_joint', 'front_left_steer_joint')
        self.declare_parameter('fr_steer_joint', 'front_right_steer_joint')
        self.declare_parameter('fl_wheel_joint', 'front_left_wheel_joint')
        self.declare_parameter('fr_wheel_joint', 'front_right_wheel_joint')
        self.declare_parameter('rl_wheel_joint', 'rear_left_wheel_joint')
        self.declare_parameter('rr_wheel_joint', 'rear_right_wheel_joint')

        # Sign multipliers
        self.declare_parameter('fl_spin_mul', 1.0)
        self.declare_parameter('fr_spin_mul', 1.0)
        self.declare_parameter('rl_spin_mul', 1.0)
        self.declare_parameter('rr_spin_mul', 1.0)
        self.declare_parameter('right_steer_mul', 1.0)

        # IMU & fusion params
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('mag_topic', '/imu/mag')
        self.declare_parameter('use_imu', True)
        self.declare_parameter('madgwick_beta', 0.1)
        self.declare_parameter('yaw_fuse_gain', 0.9)
        self.declare_parameter('auto_yaw_align', True)
        self.declare_parameter('max_yaw_innov', 0.35)
        self.declare_parameter('accel_min_g', 6.0)
        self.declare_parameter('accel_max_g', 13.0)
        
        # Load params
        self.L = float(self.get_parameter('wheelbase').value)
        self.T = float(self.get_parameter('front_track').value)
        self.rw = float(self.get_parameter('wheel_radius').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.rate_hz = float(self.get_parameter('rate').value)
        self.max_steer = float(self.get_parameter('max_steer').value)
        self.steer_slew = float(self.get_parameter('steer_slew').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.max_velocity_limit = float(self.get_parameter('max_velocity_limit').value)  # ⬅️ NUEVO
        
        self.j_fl_steer = self.get_parameter('fl_steer_joint').value
        self.j_fr_steer = self.get_parameter('fr_steer_joint').value
        self.j_fl_wheel = self.get_parameter('fl_wheel_joint').value
        self.j_fr_wheel = self.get_parameter('fr_wheel_joint').value
        self.j_rl_wheel = self.get_parameter('rl_wheel_joint').value
        self.j_rr_wheel = self.get_parameter('rr_wheel_joint').value

        self.mul_fl = float(self.get_parameter('fl_spin_mul').value)
        self.mul_fr = float(self.get_parameter('fr_spin_mul').value)
        self.mul_rl = float(self.get_parameter('rl_spin_mul').value)
        self.mul_rr = float(self.get_parameter('rr_spin_mul').value)
        self.mul_right_steer = float(self.get_parameter('right_steer_mul').value)
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v_cmd = 0.0
        self.delta_cmd = 0.0
        self.delta_actual = 0.0
        self.theta_fl = 0.0; self.theta_fr = 0.0; self.theta_rl = 0.0; self.theta_rr = 0.0

        # Joystick velocity model - Discretización Euler hacia atrás (MENOS INERCIA)
        K = 0.026482*1.3
        tau = 0.20236
        Ts = 1.0 / self.rate_hz  # Período de muestreo
        
        # Verificar que Ts sea razonable
        if Ts > 0.1:  # Si rate < 10 Hz
            self.get_logger().warn(f'Rate muy bajo ({self.rate_hz} Hz). Recomendado ≥ 20 Hz')
        
        # === Discretización con Euler hacia atrás (Backward Euler) ===
        # y[n] = a1 * y[n-1] + b0 * u[n]
        # donde:
        #   a1 = tau / (tau + Ts)
        #   b0 = (Ts * K) / (tau + Ts)
        self.a1 = tau / (tau + Ts)
        self.b0 = (Ts * K) / (tau + Ts)
        
        # Estado previo (solo necesitamos 1 valor previo)
        self.y_prev = 0.0

        # Historial para filtro de primer orden
        self.u_speed_hist = [0.0, 0.0]  # entrada [n], [n-1]
        self.v_hist = [0.0]              # salida [n-1]

        # === IMU filter & fusion state ===
        self.use_imu = bool(self.get_parameter('use_imu').value)
        self.yaw_fuse_gain = float(self.get_parameter('yaw_fuse_gain').value)
        self.auto_yaw_align = bool(self.get_parameter('auto_yaw_align').value)
        self.max_yaw_innov = float(self.get_parameter('max_yaw_innov').value)
        self.accel_min_g = float(self.get_parameter('accel_min_g').value)
        self.accel_max_g = float(self.get_parameter('accel_max_g').value)
        
        beta = float(self.get_parameter('madgwick_beta').value)
        self.madgwick = MadgwickAHRS()
        self.madgwick.beta = beta
        
        self.imu_last_time = None
        self.yaw_offset = 0.0
        self.yaw_aligned = False
        self.latest_accel = None
        self.has_mag = False

        # Publishers and Subscribers
        self.odom_pub = self.create_publisher(Odometry, self.get_parameter('odom_topic').value, 10)
        self.joint_pub = self.create_publisher(JointState, self.get_parameter('joint_states_topic').value, 10)
        self.tf_pub = TransformBroadcaster(self)
        self.cmd_vel_sub = self.create_subscription(TwistStamped, '/cmd_vel', self.cmd_cb, 10)

        # ⬅️ NUEVO: Suscripción y publicación del scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.scan_cb,
            10
        )
        self.scan_pub = self.create_publisher(
            LaserScan,
            self.get_parameter('scan_republish_topic').value,
            10
        )
        self.latest_scan = None  # ⬅️ Buffer para el último scan

        if self.use_imu:
            self.imu_sub = self.create_subscription(Imu, self.get_parameter('imu_topic').value, self.imu_cb, 50)
            mag_topic = self.get_parameter('mag_topic').value
            if isinstance(mag_topic, str) and len(mag_topic) > 0:
                self.mag_sub = self.create_subscription(MagneticField, mag_topic, self.mag_cb, 20)
                self.has_mag = True
                self.madgwick.set_has_mag(True)
            else:
                self.has_mag = False
                self.madgwick.set_has_mag(False)

        # Timer
        self.dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info('Ackermann kinematics: odom + TF + joint_states + IMU yaw fusion')
    # ⬅️ NUEVO: Callback para guardar el scan
    def scan_cb(self, msg: LaserScan):
        """Guarda el último scan recibido"""
        self.latest_scan = msg

    def mag_cb(self, msg: MagneticField):
        # For brevity: we’re not doing the full 9-DoF Madgwick update here.
        # If you want mag fusion, either:
        #  - extend Madgwick with the 9-axis update, or
        #  - rely on your IMU driver’s orientation output (preferred).
        # Placeholder: ignored (kept simple).
        pass
    def aplicar_deadzone(self, u_input, deadzone=50, max_input=100):
        """
        Aplica zona muerta y remapea el rango útil
        Args:
            u_input: Entrada en escala PWM (0-100 o -100 a 100)
            deadzone: Umbral de zona muerta (60 = 60% de PWM)
            max_input: Valor máximo esperado (100)
        Returns:
            Salida remapeada al rango [0, 1.0] o 0 si está en zona muerta
        """
        if abs(u_input) < deadzone:
            return 0.0
        else:
            return np.sign(u_input) * (abs(u_input) - deadzone) / (max_input - deadzone)
        
    def cmd_cb(self, msg: TwistStamped):
        # === Procesamiento de entrada ===
        cmd_input = msg.twist.linear.x  # Rango típico: [-0.5, 0.5]
        
        # Escalar a PWM: [-0.5, 0.5] -> [-100, 100]
        u_pwm = np.clip(cmd_input, -0.5, 0.5) * 200.0
        
        # Aplicar límite de PWM máximo (70% por defecto)
        if self.max_velocity_limit < 1.0:
            max_pwm_value = self.max_velocity_limit * 100.0  # 70% de 100 = 70
            u_pwm = np.clip(u_pwm, -max_pwm_value, max_pwm_value)
        
        # Aplicar deadzone (60%) con remapeo
        u_eff = self.aplicar_deadzone(u_pwm, deadzone=60, max_input=100)
        
        # Escalar a entrada del modelo
        u_model = u_eff * 100.0  # Convertir [0,1] -> [0,100]
        
        # === Aplicar modelo discreto (Euler hacia atrás) ===
        # y[n] = a1 * y[n-1] + b0 * u[n]
        y = self.a1 * self.y_prev + self.b0 * u_model
        self.y_prev = y
        
        v = float(y)
        d = float(msg.twist.angular.z)
        d = max(-self.max_steer, min(self.max_steer, d))
        self.v_cmd = v
        self.delta_cmd = d
    def imu_cb(self, msg: Imu):
        # Gyro (rad/s)
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        # Accel (m/s^2)
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        self.latest_accel = (ax, ay, az)

        # dt from IMU timestamps
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.imu_last_time is None:
            self.imu_last_time = t
            return
        dt = t - self.imu_last_time
        if dt <= 0.0 or dt > 0.2:
            # Skip if time jumps or too slow
            self.imu_last_time = t
            return

        # Validate accel magnitude (reduce effect if crazy)
        acc_norm = math.sqrt(ax*ax + ay*ay + az*az)
        if self.accel_min_g <= acc_norm <= self.accel_max_g:
            self.madgwick.update_imu(gx, gy, gz, ax, ay, az, dt)
        else:
            # integrate gyro only
            self.madgwick._integrate_gyro(gx, gy, gz, dt)

        self.imu_last_time = t
    def update(self):
        # Slew-limit steering
        if self.steer_slew > 0.0:
            max_step = self.steer_slew * self.dt
            err = self.delta_cmd - self.delta_actual
            self.delta_actual += math.copysign(min(abs(err), max_step), err)
        else:
            self.delta_actual = self.delta_cmd

        v = self.v_cmd
        d = self.delta_actual

        # Bicycle model yaw rate
        if abs(d) < 1e-9:
            wz_model = 0.0
        else:
            wz_model = (v / self.L) * math.tan(d)

        # --- Position integration uses fused yaw ---
        self.x   += v * math.cos(self.yaw) * self.dt
        self.y   += v * math.sin(self.yaw) * self.dt

        # --- Yaw fusion ---
        # 1) Predict using model yaw rate
        yaw_pred = wrap_pi(self.yaw + wz_model * self.dt)

        if self.use_imu and self.imu_last_time is not None:
            yaw_imu_raw = self.madgwick.yaw()

            # One-time align so odom doesn't jump
            if self.auto_yaw_align and not self.yaw_aligned:
                self.yaw_offset = wrap_pi(yaw_pred - yaw_imu_raw)
                self.yaw_aligned = True

            yaw_imu = wrap_pi(yaw_imu_raw + self.yaw_offset)

            # Innovation gating
            innov = wrap_pi(yaw_imu - yaw_pred)
            if abs(innov) > self.max_yaw_innov:
                innov = math.copysign(self.max_yaw_innov, innov)

            # Complementary blend
            gamma = self.yaw_fuse_gain
            self.yaw = wrap_pi(yaw_pred + gamma * innov)
        else:
            # No IMU → pure model
            self.yaw = yaw_pred
        self.get_logger().info('Yaw: {:.3f} deg'.format(math.degrees(-self.yaw)))
        # Usar timestamp actual del sistema (no del timer)
        now = self.get_clock().now().to_msg()
        
        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quat(-self.yaw)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = wz_model
        # Minimal covariances (tune as needed)
        odom.pose.covariance[0] = 1e-4
        odom.pose.covariance[7] = 1e-4
        odom.pose.covariance[35] = 1e-3
        odom.twist.covariance[0] = 1e-3
        odom.twist.covariance[35] = 1e-3
        self.odom_pub.publish(odom)

        # TF odom -> base_link
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = yaw_to_quat(-self.yaw)
            self.tf_pub.sendTransform(t)

            # --- base_link2 -> base_footprint (frame para SLAM) ---
            t_footprint = TransformStamped()
            t_footprint.header.stamp = now
            t_footprint.header.frame_id = 'base_link2'
            t_footprint.child_frame_id = 'base_footprint'
            t_footprint.transform.translation.x = 0.0
            t_footprint.transform.translation.y = 0.0
            t_footprint.transform.translation.z = 0.0
            t_footprint.transform.rotation.x = 0.0
            t_footprint.transform.rotation.y = 0.0
            t_footprint.transform.rotation.z = 0.0
            t_footprint.transform.rotation.w = 1.0
            self.tf_pub.sendTransform(t_footprint)

            # TF base_link2 -> laser_frame (IMPORTANTE: usar mismo timestamp)
            t2 = TransformStamped()
            t2.header.stamp = now  # Mismo timestamp que los demás TFs
            t2.header.frame_id = "base_link2"
            t2.child_frame_id = "laser_frame"
            t2.transform.translation.x = 0.13
            t2.transform.translation.y = 0.0
            t2.transform.translation.z = 0.13
            t2.transform.rotation = yaw_to_quat(np.pi)
            self.tf_pub.sendTransform(t2)

            t3 = TransformStamped()
            t3.header.stamp = now
            t3.header.frame_id = "base_link2"
            t3.child_frame_id = "imu_link"
            t3.transform.translation.x = -0.02
            t3.transform.translation.y = 0.0
            t3.transform.translation.z = 0.02
            t3.transform.rotation = quaternion_from_euler(0.0, np.pi,np.pi/2.0 )
            self.tf_pub.sendTransform(t3)

        # === ⬅️ NUEVO: Republicar scan con timestamp sincronizado ===
        if self.latest_scan is not None:
            scan_copy = LaserScan()
            # Copiar todos los datos del scan original
            scan_copy.header.stamp = now  # ⬅️ MISMO timestamp que TFs
            scan_copy.header.frame_id = "laser_frame"
            scan_copy.angle_min = self.latest_scan.angle_min
            scan_copy.angle_max = self.latest_scan.angle_max
            scan_copy.angle_increment = self.latest_scan.angle_increment
            scan_copy.time_increment = self.latest_scan.time_increment
            scan_copy.scan_time = self.latest_scan.scan_time
            scan_copy.range_min = self.latest_scan.range_min
            scan_copy.range_max = self.latest_scan.range_max
            scan_copy.ranges = self.latest_scan.ranges
            scan_copy.intensities = self.latest_scan.intensities
            self.scan_pub.publish(scan_copy)

        # --- Steering angles (true Ackermann) ---
        if abs(d) < 1e-9:
            dl = 0.0
            dr = 0.0
        else:
            R = self.L / math.tan(d)
            min_R = self.T/2.0 + 1e-6
            if abs(R) < min_R:
                R = math.copysign(min_R, R)
            dl = math.atan(self.L / (R - self.T/2.0))
            dr = math.atan(self.L / (R + self.T/2.0))
        dr *= self.mul_right_steer

        # Wheel linear speeds for spin viz
        if abs(d) < 1e-9:
            v_fl = v_fr = v_rl = v_rr = v
        else:
            R = self.L / math.tan(d)
            R_rl = abs(R - self.T/2.0)
            R_rr = abs(R + self.T/2.0)
            R_fl = math.hypot(self.L, R - self.T/2.0)
            R_fr = math.hypot(self.L, R + self.T/2.0)
            wz = abs((v / self.L) * math.tan(d))
            sgn = math.copysign(1.0, v)
            v_rl = wz * R_rl * sgn
            v_rr = wz * R_rr * sgn
            v_fl = wz * R_fl * sgn
            v_fr = wz * R_fr * sgn

        # Integrate wheel spin positions [rad]
        self.theta_fl += (v_fl / self.rw) * self.dt * self.mul_fl
        self.theta_fr += (v_fr / self.rw) * self.dt * self.mul_fr
        self.theta_rl += (v_rl / self.rw) * self.dt * self.mul_rl
        self.theta_rr += (v_rr / self.rw) * self.dt * self.mul_rr

        # Publish JointState (usar mismo timestamp)
        js = JointState()
        js.header.stamp = now
        js.name = [
            self.j_fl_steer, self.j_fr_steer,
            self.j_fl_wheel, self.j_fr_wheel,
            self.j_rl_wheel, self.j_rr_wheel
        ]
        js.position = [dl, dr, self.theta_fl, self.theta_fr, self.theta_rl, self.theta_rr]
        self.joint_pub.publish(js)

def main():
    rclpy.init()
    node = AckermannKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
