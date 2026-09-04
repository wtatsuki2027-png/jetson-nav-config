import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import struct
import time

# 192.168.0.40 (robot09) の設定
TARGET_IP = "192.168.0.40"
TARGET_PORT = 10001

# /cmd_vel がこの秒数以上届かなかった場合、明示的にゼロ速度を送る
# (Nav2側が完全にコマンド配信を止めるバグが起きても、ロボットが最後の
#  指令を出し続けないようにするための安全装置)
WATCHDOG_TIMEOUT_SEC = 1.0
WATCHDOG_CHECK_PERIOD_SEC = 0.2

class SafeBridge(Node):
    def __init__(self):
        super().__init__('safe_bridge_node')
        # Nav2の /cmd_vel_nav は controller_server の未平滑化(生)出力。
        # velocity_smoother が /cmd_vel_nav を受けて加減速制限・不感帯処理をした後、
        # 最終的な速度指令として /cmd_vel に出力する。ロボットへ送るのは必ずこちら。
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.sock = None
        self.last_recv_time = time.time()
        self.stopped_sent = False
        self.connect_to_robot()
        self.create_timer(WATCHDOG_CHECK_PERIOD_SEC, self.watchdog_cb)

    def connect_to_robot(self):
        """ロボットへの接続を試みるメソッド（何度でも呼べるように分離）"""
        if self.sock is not None:
            self.sock.close()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(2.0)  # 2秒でタイムアウトさせる（無限ブロック防止）
        try:
            self.sock.connect((TARGET_IP, TARGET_PORT))
            self.sock.settimeout(None) # 接続成功後はブロッキングモードに戻す
            self.get_logger().info(f'Connected to robot09 ({TARGET_IP}:{TARGET_PORT})')
        except Exception as e:
            self.get_logger().warn(f'Connection failed: {e}. Will retry later.')
            self.sock = None

    def cmd_vel_callback(self, msg):
        self.last_recv_time = time.time()
        self.stopped_sent = False
        self.send_twist(msg)

    def watchdog_cb(self):
        # /cmd_vel が一定時間届かない場合、Nav2側が指令配信を完全に停止した
        # (既知のリカバリーループ不具合など)可能性がある。最後の指令を出し
        # 続けないよう、一度だけ明示的にゼロ速度を送る。
        if self.stopped_sent:
            return
        if time.time() - self.last_recv_time < WATCHDOG_TIMEOUT_SEC:
            return
        self.get_logger().warn(
            f'No /cmd_vel received for {WATCHDOG_TIMEOUT_SEC}s. Sending zero velocity as a safety stop.')
        self.send_twist(Twist())
        self.stopped_sent = True

    def send_twist(self, msg):
        # 接続されていない場合は再接続を試みる
        if self.sock is None:
            self.connect_to_robot()
            if self.sock is None:
                return  # 接続失敗時は送信を諦めてスキップ

        try:
            # --- messages.h の PacketHeader に完全準拠させる ---
            type_byte = struct.pack('<B', 1)      # MSG_TWIST = 1
            size_bytes = struct.pack('>I', 48)    # size=48, Big-Endian
            header = type_byte + size_bytes       # 合計 5バイト

            # --- Twist 構造体 (Vector3 x 2) ---
            payload = struct.pack('<dddddd',
                msg.linear.x, msg.linear.y, msg.linear.z,
                msg.angular.x, msg.angular.y, msg.angular.z)

            # 合計 53バイトを送信
            self.sock.sendall(header + payload)

            # ログスパムを防ぐため、10回に1回程度ログを出すなどの工夫もアリです
            # self.get_logger().info(f'Sent to robot09: v={msg.linear.x:.2f}, w={msg.angular.z:.2f}')

        except Exception as e:
            self.get_logger().error(f'Send failed: {e}. Connection lost.')
            # 送信エラーが起きたらソケットを破棄し、次回コールバック時に再接続させる
            self.sock.close()
            self.sock = None

def main():
    rclpy.init()
    node = SafeBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            if node.sock:
                node.sock.close()
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
