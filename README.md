# Jetson側 ナビゲーション関連ファイル

GLIM（LiDAR-IMU SLAM）とNav2を組み合わせた自律走行ロボットの、Jetson Orin Nano側の
独自コード一式。詳しい経緯・既知の不具合・アーキテクチャの解説は`CLAUDE.md`を参照。

## ファイル一覧と役割

| ファイル | 役割 |
|---|---|
| `start_nav.sh` | Jetson側の全プロセス（Livox+RViz, GLIM, TCP中継, `/odom`生成, Nav2本体）を1コマンドで起動するスクリプト。Ctrl+Cで全プロセスをまとめて終了する |
| `bringup_launch.py` | Nav2本体（AMCL除外）・`map_server`・lifecycle managerを起動するlaunchファイル。地図パスと`nav2_params.yaml`を指定して呼び出す |
| `ros2_ws/src/my_robot_nav/config/nav2_params.yaml` | Nav2の全パラメータ本体。`controller_server`（速度計算）・`costmap`・`planner_server`など全ノードの設定 |
| `ros2_ws/src/my_robot_nav/launch/bringup_launch.py` | `bringup_launch.py`と同名の別ファイル。用途は要確認（本来使っているのはトップレベルの方） |
| `safe_bridge.py` | `/cmd_vel`を購読し、独自バイナリ形式に変換してTCPでrobot09（モータ制御機）へ送信する |
| `odom_tf_bridge.py` | 【自作】GLIMが配信しない「速度（特に角速度）」を、TFの差分から計算して`/odom`として配信する。Nav2の速度フィードバック欠落バグを解消するために作成 |
| `nav_logs/nav_logger.py` | ナビゲーション中の座標・姿勢（`pose.csv`）と速度（`velocity.csv`）をCSVに記録する診断ツール。実行ごとに`nav_logs/data/<日時>/`へ保存する。`log_nav_topics.py`の後継 |
| `evaluation_logger.py` | DWBの`/evaluation`（各候補軌道のクリティック別スコア内訳）を記録する診断ツール |
| `navigate_then_spin.py` | 並進フェーズと回転フェーズを構造的に分離するオーケストレータ（位置合わせ後にSpinビヘイビアで姿勢合わせ） |
| `velocity_sweep_test.py` | 角速度を段階的に変えて`/cmd_vel`に直接送り、指令値と実測値のズレ（不感帯・モータ特性）を調べる掃引テスト |
| `nav2_param_learn.yaml` | `nav2_params.yaml`の全項目に解説コメントを付けた学習用コピー |
| `my_nav_config_rviz/5_30.rviz` | ナビゲーション観察用に保存したRViz表示設定 |
| `CLAUDE.md` | プロジェクト全体の背景・アーキテクチャ・既知の不具合の詳細記録 |
| `.gitignore` | このリポジトリで追跡するファイルを絞り込む設定（vendored SLAMリポジトリ・大きなバイナリデータ等は除外） |

## 含めていないもの

`glim`, `glim_ext`, `glim_ros2`, `livox_ros_driver2`は、それぞれ独自のupstream GitHubリポジトリ
（例: [koide3/glim](https://github.com/koide3/glim)）なのでこのリポジトリには含めていない。
地図データ（`map_practice/`）・GLIM設定（`config_glim/`）等も、頻繁に変更しないため対象外。
