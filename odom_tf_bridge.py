#!/usr/bin/env python3
# GLIMは "~/odom"(実質 /glim_ros/odom) にオドメトリを配信するが、
# Nav2のcontroller_server/velocity_smootherが期待する "/odom" とは
# トピック名が一致しない上、角速度(twist.angular)も一切設定されない。
# そのため、RPP/DWBの加速度制限ロジックが「現在速度=常にゼロ」の前提でしか
# 計算できず、回転指令が1周期分の加速度上限(例: 0.08rad/s)から進めなくなる。
#
# このノードは "odom -> imu" のTFを購読し、位置差分から並進・角速度を
# 有限差分で計算して、Nav2が期待する /odom (nav_msgs/Odometry) に配信する。
#
# 使い方: python3 ~/odom_tf_bridge.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros
import math

ODOM_FRAME = "odom"
# 実測(2026-09-06): センサー(imu)は左右車輪の回転中心より前方7cm。
# base_linkは、その回転中心に静的TF(imu->base_link, x=-0.07)で定義している
# (start_nav.sh参照)。imuのままだと、その場回転のたびに位置が7cm円を描き、
# xy_goal_toleranceの境界を出入りしてDWBが振動する原因になっていた。
BASE_FRAME = "base_link"
PUBLISH_RATE_HZ = 30.0


def quat_to_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class OdomTfBridge(Node):
    def __init__(self):
        super().__init__('odom_tf_bridge')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.prev_t = None
        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None
        self.last_vx_body = 0.0
        self.last_vy_body = 0.0
        self.last_vyaw = 0.0

        self.create_timer(1.0 / PUBLISH_RATE_HZ, self.tick)
        self.get_logger().info(f"Publishing /odom from {ODOM_FRAME}->{BASE_FRAME} TF at {PUBLISH_RATE_HZ}Hz")

    def tick(self):
        try:
            tf = self.tf_buffer.lookup_transform(ODOM_FRAME, BASE_FRAME, rclpy.time.Time())
        except Exception:
            return

        t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = quat_to_yaw(tf.transform.rotation)

        # GLIMのTF更新頻度がこのノードの取得頻度(30Hz)より低い場合、
        # 同じタイムスタンプのTFを重複取得することがある。その周期は
        # 速度をゼロ扱いにせず、直近の計算値をそのまま使い回す
        # (ゼロを混ぜるとNav2側の速度平均が下振れし、加速できなくなる)。
        vx_body, vy_body, vyaw = self.last_vx_body, self.last_vy_body, self.last_vyaw
        if self.prev_t is not None:
            dt = t - self.prev_t
            if dt > 1e-4:
                dx = x - self.prev_x
                dy = y - self.prev_y
                # 世界(odom)座標系の速度をロボット(imu)座標系へ回転
                cos_y = math.cos(self.prev_yaw)
                sin_y = math.sin(self.prev_yaw)
                vx_body = (cos_y * dx + sin_y * dy) / dt
                vy_body = (-sin_y * dx + cos_y * dy) / dt
                vyaw = wrap_angle(yaw - self.prev_yaw) / dt
                self.last_vx_body, self.last_vy_body, self.last_vyaw = vx_body, vy_body, vyaw

        self.prev_t, self.prev_x, self.prev_y, self.prev_yaw = t, x, y, yaw

        odom = Odometry()
        odom.header.stamp = tf.header.stamp
        odom.header.frame_id = ODOM_FRAME
        odom.child_frame_id = BASE_FRAME
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = tf.transform.rotation
        odom.twist.twist.linear.x = vx_body
        odom.twist.twist.linear.y = vy_body
        odom.twist.twist.angular.z = vyaw
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = OdomTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
