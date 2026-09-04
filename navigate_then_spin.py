#!/usr/bin/env python3
# 「位置に到達するまでは並進のみ、到達後は回転のみ」を構造的に保証するオーケストレータ。
#
# 仕組み:
#   1. /goal_pose_2phase を購読 (bt_navigatorが自前で購読している /goal_pose とは
#      別トピック。同じにすると二重にナビゲーションが走ってしまうため分離している)
#   2. NavigateToPoseアクションを送る。このとき general_goal_checker.yaw_goal_tolerance が
#      ほぼpiに緩められている前提なので、姿勢に関わらず「位置に着いた時点」で成功する
#      (=このフェーズでは並進のみが働く。DWBのRotateToGoal/GoalAlignがどう転んでも、
#       ゴールチェッカーが緩いので位置到達=即成功として扱われる)
#   3. NavigateToPose成功後、現在のmap->imuのyawと目標yawの差分を計算し、
#      Spinビヘイビア(並進を一切出せない、その場回転専用アクション)にその差分だけを渡す。
#      Spinが完了した時点で、位置合わせと姿勢合わせが完全に分離された形で完了する。
#
# 使い方:
#   1. RVizの「2D Goal Pose」ツールの Tool Properties > Topic を
#      /goal_pose_2phase に変更しておく(デフォルトの /goal_pose のままだと
#      bt_navigatorが直接反応してしまい、このスクリプトと二重に動いてしまう)
#   2. python3 ~/navigate_then_spin.py を起動
#   3. RVizで2D Goal Poseを送ると、自動でこの2段階シーケンスが実行される
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
import tf2_ros
import math


def quat_to_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class NavigateThenSpin(Node):
    def __init__(self):
        super().__init__('navigate_then_spin')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.busy = False
        self.create_subscription(PoseStamped, '/goal_pose_2phase', self.goal_cb, 10)
        self.get_logger().info('Ready. Waiting for /goal_pose_2phase...')

    def goal_cb(self, msg):
        if self.busy:
            self.get_logger().warn('Already navigating, ignoring new goal.')
            return
        self.busy = True
        self.target_yaw = quat_to_yaw(msg.pose.orientation)
        self.get_logger().info(
            f'--- Phase 1: NavigateToPose (position only, target_yaw={self.target_yaw:.3f} rad kept for reference) ---')

        goal = NavigateToPose.Goal()
        goal.pose = msg
        self.nav_client.wait_for_server()
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.nav_goal_response_cb)

    def nav_goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('NavigateToPose goal rejected.')
            self.busy = False
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.nav_result_cb)

    def nav_result_cb(self, future):
        result = future.result()
        self.get_logger().info(f'Phase 1 (position) finished, status={result.status}')
        self.start_spin_phase()

    def start_spin_phase(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'imu', rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f'Could not get current pose for spin phase: {e}')
            self.busy = False
            return

        current_yaw = quat_to_yaw(tf.transform.rotation)
        delta = wrap_angle(self.target_yaw - current_yaw)
        self.get_logger().info(
            f'--- Phase 2: Spin (rotation only) current_yaw={current_yaw:.3f} '
            f'target_yaw={self.target_yaw:.3f} delta={delta:.3f} rad ---')

        goal = Spin.Goal()
        goal.target_yaw = delta
        goal.time_allowance.sec = 30
        self.spin_client.wait_for_server()
        send_future = self.spin_client.send_goal_async(goal)
        send_future.add_done_callback(self.spin_goal_response_cb)

    def spin_goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Spin goal rejected.')
            self.busy = False
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.spin_result_cb)

    def spin_result_cb(self, future):
        result = future.result()
        self.get_logger().info(f'Phase 2 (rotation) finished, status={result.status}. Sequence complete.')
        self.busy = False


def main():
    rclpy.init()
    node = NavigateThenSpin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
