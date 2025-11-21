#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from geometry_msgs.msg import Quaternion
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
class PoseToTFNode(Node):
    def __init__(self):
        super().__init__('pose_broadcaster_node_charlie')

        # Suscripciones
        self.pose_sub = self.create_subscription(Pose, '/robot1/pose', self.pose_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publicadores
        self.tf_broadcaster = TransformBroadcaster(self)
        self.scan_pub = self.create_publisher(LaserScan, '/scan2', 10)

        # Variables internas
        self.latest_pose = None
        self.latest_scan = None

        # Timer (20 Hz)
        self.timer_period = 1.0 / 50.0
        self.timer = self.create_timer(self.timer_period, self.broadcast_tf)

        self.get_logger().info("✅ Nodo PoseToTF inicializado correctamente.")

    def pose_callback(self, msg: Pose):
        self.latest_pose = msg

    def scan_callback(self, msg: LaserScan):
        """Guarda el último scan y actualiza su frame"""
        msg.header.frame_id = "laser_frame"
        self.latest_scan = msg

    def broadcast_tf(self):
        if self.latest_pose is None or self.latest_scan is None:
            return

        # === Tiempo actual ===
        now = self.get_clock().now().to_msg()
        msg = self.latest_pose

        # --- odom2 -> base_link2 ---
        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = 'odom2'
        t1.child_frame_id = 'base_link2'
        t1.transform.translation.x = msg.position.x
        t1.transform.translation.y = msg.position.y
        t1.transform.translation.z = msg.position.z
        t1.transform.rotation = msg.orientation
        self.tf_broadcaster.sendTransform(t1)

        # --- base_link2 -> base_footprint (frame para SLAM) ---
        t_footprint = TransformStamped()
        t_footprint.header.stamp = now
        t_footprint.header.frame_id = 'base_link2'
        t_footprint.child_frame_id = 'base_footprint'
        t_footprint.transform.translation.x = 0.0
        t_footprint.transform.translation.y = 0.0
        t_footprint.transform.translation.z = -msg.position.z  # Proyectar al suelo (z=0 en odom2)
        t_footprint.transform.rotation.x = 0.0
        t_footprint.transform.rotation.y = 0.0
        t_footprint.transform.rotation.z = 0.0
        t_footprint.transform.rotation.w = 1.0  # Sin rotación
        self.tf_broadcaster.sendTransform(t_footprint)
        
        # --- base_link2 -> laser_frame ---
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'base_link2'
        t2.child_frame_id = 'laser_frame'
        t2.transform.translation.x = 0.2
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.15
        q = quaternion_from_euler(0.0, 0.0, np.pi)
        t2.transform.rotation = q
        self.tf_broadcaster.sendTransform(t2)
        t3 = TransformStamped()
        t3.header.stamp = now
        t3.header.frame_id = "base_link2"
        t3.child_frame_id = "imu_link"
        t3.transform.translation.x = -0.02
        t3.transform.translation.y = 0.0
        t3.transform.translation.z = 0.02
        t3.transform.rotation = quaternion_from_euler(0.0,0.0,0.0)
        self.tf_broadcaster.sendTransform(t3)
        # --- Publicar scan sincronizado ---
        scan_copy = LaserScan()
        scan_copy = self.latest_scan
        scan_copy.header.stamp = now        # ⬅️ MISMO tiempo que TF
        scan_copy.header.frame_id = "laser_frame"
        self.scan_pub.publish(scan_copy)


def main(args=None):
    rclpy.init(args=args)
    node = PoseToTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
