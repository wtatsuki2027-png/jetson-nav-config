#!/usr/bin/env python3
# ナビゲーション中の座標・姿勢と速度を記録する診断ツール（log_nav_topics.pyの後継）。
# 使い方: python3 ~/nav_logs/nav_logger.py
# (Ctrl+Cで終了。実行開始時刻ごとのフォルダ ~/nav_logs/data/<YYYYMMDD_HHMMSS>/ に
#  pose.csv と velocity.csv を保存する)
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
from datetime import datetime

NAV_LOGS_DIR = os.path.expanduser("~/nav_logs")
NAV2_PARAMS_PATH = os.path.expanduser("~/ros2_ws/src/my_robot_nav/config/nav2_params.yaml")


def quat_to_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
    # a - b を [-pi, pi] に正規化した最短角度差
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


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

        run_dir = os.path.join(NAV_LOGS_DIR, "data", datetime.now().strftime("%Y%m%d_%H%M%S"))
        os.makedirs(run_dir, exist_ok=True)
        self.get_logger().info(f"Logging to {run_dir}")

        # ロボットの座標・姿勢用。source列でrobot_pose(20Hzで継続記録)と
        # goal_pose(RVizで指定される度に1行だけ)を区別する。
        self.pose_f = open(os.path.join(run_dir, "pose.csv"), "w", newline="")
        self.pose_w = csv.writer(self.pose_f)
        self.pose_w.writerow(["t", "source", "x", "y", "yaw_rad", "yaw_goal_tolerance_rad"])

        # 並進速度・角速度用。topic列で4種類(指令値2段階+実測+生IMU)を区別する。
        # dist_to_goal/angle_to_goalは、記録時点で最後に受信したロボット姿勢と
        # 目標姿勢から計算した残り距離・残り角度(どちらか未受信ならば空欄)。
        self.vel_f = open(os.path.join(run_dir, "velocity.csv"), "w", newline="")
        self.vel_w = csv.writer(self.vel_f)
        self.vel_w.writerow(["t", "topic", "linear_x", "angular_z", "dist_to_goal_m", "angle_to_goal_rad"])

        self.last_robot_pose = None  # (x, y, yaw)
        self.last_goal_pose = None   # (x, y, yaw)

        self.create_subscription(Twist, '/cmd_vel_nav', lambda m: self.cmd_cb(m, 'cmd_vel_nav'), 10)
        self.create_subscription(Twist, '/cmd_vel', lambda m: self.cmd_cb(m, 'cmd_vel'), 10)
        # odom_tf_bridge.py が計算する実測速度。指令値と同じCSVに記録することで、
        # 速度追従(指令通りに実際に動けているか)を直接比較できる。
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        # GLIMの推定処理を一切通していない、センサそのものの角速度。
        # odom_measured(GLIM推定)と重ねて見ることで、GLIMの推定が実際の動きから
        # 乖離していないか(=SLAMの追跡破綻かどうか)を目視に頼らず確認できる。
        self.create_subscription(Imu, '/livox/imu', self.imu_cb, 50)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(0.05, self.pose_cb)  # 20Hz

        self.get_logger().info(
            "Logging cmd_vel_nav/cmd_vel/odom_measured/imu_raw -> velocity.csv, "
            "robot_pose/goal_pose -> pose.csv"
        )

    def _goal_errors(self):
        if self.last_robot_pose is None or self.last_goal_pose is None:
            return "", ""
        rx, ry, ryaw = self.last_robot_pose
        gx, gy, gyaw = self.last_goal_pose
        dist = math.hypot(gx - rx, gy - ry)
        ang = angle_diff(gyaw, ryaw)
        return dist, ang

    def cmd_cb(self, msg, topic):
        dist, ang = self._goal_errors()
        self.vel_w.writerow([time.time(), topic, msg.linear.x, msg.angular.z, dist, ang])
        self.vel_f.flush()

    def odom_cb(self, msg):
        dist, ang = self._goal_errors()
        self.vel_w.writerow([time.time(), 'odom_measured', msg.twist.twist.linear.x, msg.twist.twist.angular.z, dist, ang])
        self.vel_f.flush()

    def imu_cb(self, msg):
        dist, ang = self._goal_errors()
        self.vel_w.writerow([time.time(), 'imu_raw', 0.0, msg.angular_velocity.z, dist, ang])
        self.vel_f.flush()

    def goal_cb(self, msg):
        yaw = quat_to_yaw(msg.pose.orientation)
        self.last_goal_pose = (msg.pose.position.x, msg.pose.position.y, yaw)
        self.pose_w.writerow([time.time(), 'goal_pose', msg.pose.position.x, msg.pose.position.y, yaw, self.yaw_goal_tolerance])
        self.pose_f.flush()
        self.get_logger().info(f"Goal received: x={msg.pose.position.x:.3f} y={msg.pose.position.y:.3f} yaw={yaw:.3f} rad")

    def pose_cb(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            yaw = quat_to_yaw(tf.transform.rotation)
            x, y = tf.transform.translation.x, tf.transform.translation.y
            self.last_robot_pose = (x, y, yaw)
            self.pose_w.writerow([time.time(), 'robot_pose', x, y, yaw, ""])
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
        node.pose_f.close()
        node.vel_f.close()


if __name__ == '__main__':
    main()
