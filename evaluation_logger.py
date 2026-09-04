#!/usr/bin/env python3
# DWBの /evaluation (dwb_msgs/msg/LocalPlanEvaluation) を常時記録する診断ツール。
# 全候補軌道をそのまま出力すると膨大になるため、毎周期「最良(best_index)」の
# 候補だけを抜き出し、その速度指令と各クリティックのスコア内訳を記録する。
# 使い方: python3 ~/evaluation_logger.py (Ctrl+Cで終了、~/evaluation_log.csvに保存)
import rclpy
from rclpy.node import Node
from dwb_msgs.msg import LocalPlanEvaluation
import time
import csv
import os

OUT_PATH = os.path.expanduser("~/evaluation_log.csv")


class EvaluationLogger(Node):
    def __init__(self):
        super().__init__('evaluation_logger')
        self.f = open(OUT_PATH, "w", newline="")
        self.w = csv.writer(self.f)
        self.header_written = False
        self.create_subscription(LocalPlanEvaluation, '/evaluation', self.cb, 10)
        self.get_logger().info(f"Logging best-candidate /evaluation breakdown to {OUT_PATH}")

    def cb(self, msg):
        if not msg.twists:
            return
        best = msg.twists[msg.best_index]
        row = {
            "t": time.time(),
            "best_index": msg.best_index,
            "worst_index": msg.worst_index,
            "n_twists": len(msg.twists),
            "cmd_vx": best.traj.velocity.x,
            "cmd_vy": best.traj.velocity.y,
            "cmd_vtheta": best.traj.velocity.theta,
            "total": best.total,
        }
        for cs in best.scores:
            row[f"{cs.name}.raw"] = cs.raw_score
            row[f"{cs.name}.scale"] = cs.scale

        if not self.header_written:
            self.w.writerow(list(row.keys()))
            self.header_written = True
        self.w.writerow(list(row.values()))
        self.f.flush()


def main():
    rclpy.init()
    node = EvaluationLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
