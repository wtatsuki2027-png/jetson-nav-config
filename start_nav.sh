#!/bin/bash
# ナビゲーション一式を1コマンドで起動するスクリプト。
# Ctrl+C を押すと、このスクリプトが起動した全プロセスをまとめて終了する。
#
# 使い方:
#   ./start_nav.sh [地図のyamlパス]
#   例: ./start_nav.sh ~/map_practice/7_14/my_map.yaml
#   引数を省略すると bringup_launch.py のデフォルト地図が使われる。
#
# 注意: robot09側の `sudo sh canup.sh` / `sudo ./main` は
#       sudoのパスワード入力が必要なため、これとは別のターミナルで
#       今まで通り手動実行してください。

set -u

MAP_YAML="${1:-}"

echo "=== 古い残骸プロセスを掃除 ==="
pkill -f safe_bridge.py 2>/dev/null
pkill -f odom_tf_bridge.py 2>/dev/null
pkill -f static_transform_publisher 2>/dev/null
pkill -f glim_rosnode 2>/dev/null
pkill -f rviz_MID360_launch 2>/dev/null
pkill -f bringup_launch.py 2>/dev/null
sleep 1

# このスクリプトが起動した全バックグラウンドジョブを、
# Ctrl+C (SIGINT) やスクリプト終了時にまとめて終了させる。
cleanup() {
    echo
    echo "=== 終了処理: 起動した全プロセスを停止します ==="
    kill $(jobs -p) 2>/dev/null
    wait 2>/dev/null
    echo "=== 停止完了 ==="
}
trap cleanup SIGINT SIGTERM EXIT

echo "=== Livoxドライバ + RViz 起動 ==="
(cd ~/seminar/scripts && ros2 launch livox_ros_driver2 rviz_MID360_launch.py) &
sleep 3

echo "=== GLIM 起動 ==="
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~/seminar/scripts/config_glim) &
sleep 2

echo "=== TCP中継 (safe_bridge.py) 起動 ==="
python3 ~/safe_bridge.py &

echo "=== /odom 中継 (odom_tf_bridge.py) 起動 ==="
python3 ~/odom_tf_bridge.py &

echo "=== 静的TF 起動 ==="
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
  --frame-id lidar --child-frame-id livox_frame &
sleep 2

echo "=== Nav2 起動 ==="
if [ -n "$MAP_YAML" ]; then
    ros2 launch ~/bringup_launch.py map:=$(realpath "$MAP_YAML") &
else
    ros2 launch ~/bringup_launch.py &
fi

echo
echo "=== 全プロセス起動完了。停止するには Ctrl+C を押してください ==="

# いずれかのジョブが終了するか、Ctrl+Cが押されるまで待機
wait
