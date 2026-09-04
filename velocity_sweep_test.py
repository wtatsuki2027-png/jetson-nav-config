#!/usr/bin/env python3
# 角速度の指令値と実測値(/odom, odom_tf_bridge.py経由)のズレ(不感帯・静止摩擦)を
# 調べるための掃引テスト。Nav2を経由せず直接 /cmd_vel に指令を送る。
#
# 使い方:
#   1. ~/start_nav.sh <map.yaml> を起動しておく(GLIM + odom_tf_bridge.py + safe_bridge.py が動く)
#   2. python3 ~/log_nav_topics.py を別ターミナルで起動しておく(記録用)
#   3. python3 ~/velocity_sweep_test.py を実行
#   4. 終わったら ~/cmd_vel_log.csv の "cmd_vel" 行と "odom_measured" 行を突き合わせて分析する
#
# ロボットは実際に回転するので、周囲に十分なスペースがあることを確認してから実行すること。
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

# 掃引する角速度の値 (rad/s)。符号は交互にして、同じ方向に回り続けないようにする。
STEPS = [0.05, -0.1, 0.15, -0.2, 0.3, -0.4, 0.5, -0.6, 0.8, -1.0]
HOLD_SEC = 5.0      # 各速度を維持する時間
PAUSE_SEC = 2.0      # 各ステップの間の静止時間


class VelocitySweepTest(Node):
    def __init__(self):
        super().__init__('velocity_sweep_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def send(self, omega, duration):
        msg = Twist()
        msg.angular.z = omega
        end = time.time() + duration
        while time.time() < end and rclpy.ok():
            self.pub.publish(msg)
            time.sleep(0.1)

    def stop(self):
        self.pub.publish(Twist())


def main():
    rclpy.init()
    node = VelocitySweepTest()
    try:
        node.stop()
        time.sleep(1.0)
        for omega in STEPS:
            node.get_logger().info(f"=== commanding omega={omega:.3f} rad/s for {HOLD_SEC}s ===")
            node.send(omega, HOLD_SEC)
            node.get_logger().info("--- pause (stop) ---")
            node.stop()
            time.sleep(PAUSE_SEC)
        node.get_logger().info("Sweep complete.")
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
