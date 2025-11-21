#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import copy
import math


class OdomTFPublisher(Node):
    def __init__(self):
        super().__init__('tf_broadcaster_node_charlie')

        # === Subscripciones ===
        self.create_subscription(Point, "/posicion", self.position_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Float32, '/modelo/yaw', self.yaw_callback, 10)

        # === Publicadores ===
        self.scan_pub = self.create_publisher(LaserScan, '/scan2', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom/imu_kf', 10)

        # === Broadcasters ===
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # === Variables ===
        self.x = 0.0
        self.y = 0.0
        self.x_last = 0.0
        self.y_last = 0.0
        self.yaw_used = 0.0
        self.scan = None

        # Publicación periódica de odom + TF
        self.last_time = self.get_clock().now()
        self.pub_dt = 0.01
        self.create_timer(self.pub_dt, self.publish_tf_and_odom)

        # Publicar TF estáticos
        self.publish_static_tfs()

        self.get_logger().info("✅ Nodo OdomTFPublisher inicializado.")


    # ==========================================
    # STATIC TRANSFORMS
    # ==========================================
    def publish_static_tfs(self):

        # ---- base_link -> laser_link (rotado 180° en Z) ----
        laser_tf = TransformStamped()
        laser_tf.header.stamp = rclpy.time.Time().to_msg()  # STATIC TF → TIME = 0
        laser_tf.header.frame_id = "base_link"
        laser_tf.child_frame_id = "laser_link"
        laser_tf.transform.translation.x = 0.0
        laser_tf.transform.translation.y = 0.0
        laser_tf.transform.translation.z = 0.2

        # Rotación 180° en Z
        laser_tf.transform.rotation.x = 0.0
        laser_tf.transform.rotation.y = 0.0
        laser_tf.transform.rotation.z = 1.0
        laser_tf.transform.rotation.w = 0.0

        # ---- base_link -> imu_frame ----
        imu_tf = TransformStamped()
        imu_tf.header.stamp = laser_tf.header.stamp
        imu_tf.header.frame_id = "base_link"
        imu_tf.child_frame_id = "imu_frame"
        imu_tf.transform.translation.x = 0.0
        imu_tf.transform.translation.y = 0.0
        imu_tf.transform.translation.z = 0.05
        imu_tf.transform.rotation.x = 0.0
        imu_tf.transform.rotation.y = 0.0
        imu_tf.transform.rotation.z = 0.0
        imu_tf.transform.rotation.w = 1.0

        # Publicar estáticos
        self.static_tf_broadcaster.sendTransform([laser_tf, imu_tf])



    # ==========================================
    # CALLBACKS
    # ==========================================
    def position_callback(self, msg: Point):
        self.x_last = self.x
        self.y_last = self.y
        self.x = msg.x
        self.y = msg.y

    def lidar_callback(self, msg: LaserScan):
        self.scan = msg

    def yaw_callback(self, msg: Float32):
        self.yaw_used = msg.data


    # ==========================================
    # PUBLICACIÓN DINÁMICA: odom -> base_link
    # ==========================================
    def publish_tf_and_odom(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0 or dt > 0.5:
            dt = self.pub_dt
        self.last_time = current_time

        now = current_time.to_msg()

        # Velocidades desde derivada aproximada
        vx = (self.x - self.x_last) / dt
        vy = (self.y - self.y_last) / dt

        # ------------------------------------
        # TF: odom -> base_link
        # ------------------------------------
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Cuaternión desde yaw
        qz = math.sin(self.yaw_used / 2.0)
        qw = math.cos(self.yaw_used / 2.0)

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

        # ------------------------------------
        # ODOMETRY MESSAGE
        # ------------------------------------
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy

        self.odom_pub.publish(odom)

        # ------------------------------------
        # REPUBLISH SCAN EN laser_link
        # ------------------------------------
        if self.scan is not None:
            scan2 = copy.deepcopy(self.scan)
            scan2.header.stamp = now
            scan2.header.frame_id = "laser_link"
            self.scan_pub.publish(scan2)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
