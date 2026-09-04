#!/usr/bin/env python3
# ナビゲーション中の /cmd_vel_nav, /cmd_vel, /odom, /livox/imu, /goal_pose, map->imu の
# 姿勢(yaw) を記録する診断ツール。
# 使い方: python3 ~/log_nav_topics.py
# (Ctrl+Cで終了、cmd_vel_log.csv / pose_log.csv / goal_log.csv を~/に保存)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf2_ros
import time
import math
import csv
import os
import yaml

OUT_DIR = os.path.expanduser("~")
NAV2_PARAMS_PATH = os.path.expanduser("~/ros2_ws/src/my_robot_nav/config/nav2_params.yaml")

def quat_to_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def read_yaw_goal_tolerance():
    try:
        with open(NAV2_PARAMS_PATH) as f:
            params = yaml.safe_load(f)
        return params['controller_server']['ros__parameters']['general_goal_checker']['yaw_goal_tolerance']
    except Exception:
        return None

class NavLogger(Node):
    def __init__(self):
        super().__init__('nav_debug_logger')
        self.yaw_goal_tolerance = read_yaw_goal_tolerance()
        self.get_logger().info(f"yaw_goal_tolerance from nav2_params.yaml: {self.yaw_goal_tolerance}")

        self.cmd_f = open(os.path.join(OUT_DIR, "cmd_vel_log.csv"), "w", newline="")
        self.cmd_w = csv.writer(self.cmd_f)
        self.cmd_w.writerow(["t", "topic", "linear_x", "angular_z"])

        self.pose_f = open(os.path.join(OUT_DIR, "pose_log.csv"), "w", newline="")
        self.pose_w = csv.writer(self.pose_f)
        self.pose_w.writerow(["t", "x", "y", "yaw_rad"])

        self.goal_f = open(os.path.join(OUT_DIR, "goal_log.csv"), "w", newline="")
        self.goal_w = csv.writer(self.goal_f)
        self.goal_w.writerow(["t", "x", "y", "yaw_rad", "yaw_goal_tolerance_rad"])

        self.create_subscription(Twist, '/cmd_vel_nav', lambda m: self.cmd_cb(m, 'cmd_vel_nav'), 10)
        self.create_subscription(Twist, '/cmd_vel', lambda m: self.cmd_cb(m, 'cmd_vel'), 10)
        # odom_tf_bridge.py が計算する実測速度。指令値(cmd_vel)と同じCSV/列構成で
        # 記録することで、速度追従(指令通りに実際に動けているか)を直接比較できる。
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        # GLIMの推定処理を一切通していない、センサそのものの角速度。
        # odom_measured(GLIM推定)と重ねて見ることで、GLIMの推定が実際の動きから
        # 乖離していないか(=SLAMの追跡破綻かどうか)を目視に頼らず確認できる。
        self.create_subscription(Imu, '/livox/imu', self.imu_cb, 50)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(0.05, self.pose_cb)  # 20Hz

        self.get_logger().info("Logging /cmd_vel_nav, /cmd_vel, /goal_pose, and map->imu TF...")

    def cmd_cb(self, msg, topic):
        self.cmd_w.writerow([time.time(), topic, msg.linear.x, msg.angular.z])
        self.cmd_f.flush()

    def odom_cb(self, msg):
        self.cmd_w.writerow([time.time(), 'odom_measured', msg.twist.twist.linear.x, msg.twist.twist.angular.z])
        self.cmd_f.flush()

    def imu_cb(self, msg):
        self.cmd_w.writerow([time.time(), 'imu_raw', 0.0, msg.angular_velocity.z])
        self.cmd_f.flush()

    def goal_cb(self, msg):
        yaw = quat_to_yaw(msg.pose.orientation)
        self.goal_w.writerow([time.time(), msg.pose.position.x, msg.pose.position.y, yaw, self.yaw_goal_tolerance])
        self.goal_f.flush()
        self.get_logger().info(f"Goal received: x={msg.pose.position.x:.3f} y={msg.pose.position.y:.3f} yaw={yaw:.3f} rad")

    def pose_cb(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'imu', rclpy.time.Time())
            yaw = quat_to_yaw(tf.transform.rotation)
            self.pose_w.writerow([time.time(), tf.transform.translation.x, tf.transform.translation.y, yaw])
            self.pose_f.flush()
        except Exception:
            pass

def main():
    rclpy.init()
    node = NavLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_f.close()
        node.pose_f.close()
        node.goal_f.close()

if __name__ == '__main__':
    main()
