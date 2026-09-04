# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

This is not a single application — it's a Jetson/Ubuntu robot workstation home directory for a LiDAR-IMU
SLAM (mapping) and autonomous navigation project. It combines a vendored GLIM SLAM stack, a ROS2 Nav2
navigation setup, a small custom TCP bridge, and seminar/teaching material. Work here typically involves
ROS2 (Humble), C++/CMake, Python launch/glue scripts, and point-cloud/map data.

`source /opt/ros/humble/setup.bash` is sourced automatically from `~/.bashrc`. ROS2 distro is **Humble**.

## Directory map

| Path | What it is | Owned by user? |
|---|---|---|
| `ros2_ws/` | Main colcon workspace (see below) | mixed |
| `ros_bridge_tcp_simple/` | Custom C++ TCP bridge library for cmd_vel/odom, used off-ROS-network (e.g. talking to a robot's onboard controller over plain TCP) | yes |
| `seminar/` | GLIM/ICP/PGO teaching material: Python tutorials, venv, and scripts (`start.sh`, `record.sh`) for running Livox+GLIM as composable nodes | yes |
| `map_practice/` | Saved GLIM mapping session dumps (`graph.txt/bin`, `values.bin`, trajectories, `.ply`/`.pgm` map exports), one subdir per mapping run/date | data |
| `safe_bridge.py`, `bringup_launch.py`, `ply2map.py` | Standalone top-level scripts (see "Custom scripts" below) | yes |
| `my_nav_config_rviz/` | Saved RViz configs for navigation | yes |
| `*.ply`, `*_backup_*.tar.gz` | Point-cloud map exports and full-environment backups (large, not source) | data |

### `ros2_ws/src/` packages

- **`glim/`**, **`glim_ext/`**, **`glim_ros2/`** — vendored third-party repos from `koide3/*` (GLIM 3D
  LiDAR-IMU mapping framework, its extension modules, and its ROS2 wrapper). Each is its own git clone with
  an `origin` remote pointing at GitHub. Treat these as upstream code: prefer not to modify in place, and if
  a fix is needed, check whether it belongs upstream instead.
  - `glim_ext/modules/` holds pluggable odometry/mapping extensions (e.g. `velocity_suppressor`,
    `imu_validator`, `gravity_estimator`, `orb_slam_frontend`, `flat_earther`, `gnss_global`,
    loop detectors). Each is configured via a matching `config/config_*.json`.
  - `glim/config/*.json` and `glim_ext/config/*.json` are the runtime-tunable SLAM parameters (odometry,
    preprocessing, global mapping, sensors, viewer). These are what you edit to change mapping behavior —
    not the C++ source — for most tuning tasks.
- **`livox_ros_driver2/`** — vendored third-party driver (`Livox-SDK/livox_ros_driver2`) for the Livox
  MID360 LiDAR. Its device IP is configured in `livox_ros_driver2/config/MID360_config.json`.
- **`my_robot_nav/`** — user-authored Nav2 bringup: `launch/bringup_launch.py` and
  `config/nav2_params.yaml` (AMCL, costmaps, planners; note `base_frame_id: "imu"`, not `base_link` —
  this workspace's odometry frame comes from GLIM/IMU, not wheel odometry). **Not currently a real colcon
  package** (no `package.xml`/`setup.py`), so it isn't built or found via `ros2 launch my_robot_nav ...`;
  it's invoked directly, e.g. `ros2 launch ~/bringup_launch.py` (the top-level copy) or by path.
- **`udp_bridge.py`** — a loose script (not part of any package) that forwards `/cmd_vel` over UDP to a
  fixed IP as packed `(float v, float omega)`. Contains mojibake Japanese comments (originally Shift-JIS,
  now mis-decoded) — treat as legacy/reference, prefer `safe_bridge.py`/`ros_bridge_tcp_simple` for new work.

### Custom scripts (`~`)

- **`safe_bridge.py`** — standalone rclpy node (run with plain `python3`, not a package) that subscribes to
  `/cmd_vel` and forwards it over TCP to the stick PC at `TARGET_IP = "192.168.0.40"` (the comment says
  `robot06`, but that machine died and was physically replaced by `robot09` — same IP/port, so the code is
  still correct, just the comment is stale), using a 5-byte
  `PacketHeader{type:u8, size:u32 big-endian}` followed by a `Twist` payload of 6 little-endian doubles —
  this **must stay wire-compatible** with `ros_bridge_tcp_simple/include/messages.h`'s `PacketHeader` and
  `Twist` structs. Auto-reconnects on send failure. **Must stay on `/cmd_vel`, not `/cmd_vel_nav`** — see the
  `cmd_vel` vs `cmd_vel_nav` note below; this was a real bug found and fixed in this repo's history.
- **`bringup_launch.py`** — launches Nav2 (via `nav2_bringup`'s `navigation_launch.py`, AMCL excluded),
  `map_server`, and a lifecycle manager, loading `my_robot_nav/config/nav2_params.yaml` and a map YAML from
  `map_practice/<session>/`.
- **`ply2map.py`** — CLI: slices a `.ply` point cloud by Z-height band and rasterizes it into a 2D
  occupancy grid (`<name>.pgm` + `<name>.yaml`, ROS map_server format). Usage:
  `python3 ply2map.py input.ply -o my_map --res 0.05 --zmin 0.2 --zmax 1.5`.

### `ros_bridge_tcp_simple/`

Small standalone CMake C++ library (no ROS dependency) providing `RosTcpServer`/`RosTcpClient` classes that
exchange ROS-message-shaped structs (`Twist`, `Odometry`, `Imu`) over raw TCP using the packet format
defined in `include/messages.h` (1-byte type + 4-byte big-endian size header, little-endian struct payload,
all `__attribute__((packed))`). This is the wire protocol `safe_bridge.py` speaks. `include/config.h` has
shared constants (default port 8080, buffer/message size limits, timeouts).

### Stick PC (`robot09`) — physical robot control, not on this filesystem

The robot's differential-drive motors are driven by a **separate machine** (`robot09`, Ubuntu 16.04, IP
`192.168.0.40`, can't run ROS2) over CAN, not by anything in this home directory. It's reachable over the
LAN but its code lives at `robot@192.168.0.40:~/Development/T+/watanabe/tatsuki/` (a sibling `T+/` tree
under that account also holds other people's/robots' experiments, e.g. `watanabe/WheeledRobot_watanabe/`).
There's no git repo there.

- **To inspect/edit it from here**: SSH key-based auth is set up (key: `~/.ssh/id_ed25519_robot09`,
  registered in `robot09:~/.ssh/authorized_keys`). The whole `~/Development/T+/` tree is mounted via sshfs
  at `~/stick_pc_mount` (`sshfs -o IdentityFile=~/.ssh/id_ed25519_robot09,reconnect
  robot@192.168.0.40:/home/robot/Development/T+ ~/stick_pc_mount`) — mount the parent `T+/` dir, not just
  `tatsuki/`, because `tatsuki/common.h` includes shared headers via `../_heders/...`. Once mounted, treat
  it as a normal local path (Read/Edit/Bash all work through it). Remount with
  `fusermount3 -u ~/stick_pc_mount` first if it goes stale.
- **`tatsuki/main.cpp`** runs a `RosTcpServer` on port 10001 (`ros_bridge_tcp_simple`-family code, a fork
  living alongside the Jetson-side copy — the two have drifted, don't assume they're identical) and, in a
  tight loop paced by `SMP_uTIME` (3ms), receives the latest `Twist`, calls `control()`, then reads/writes
  the CAN bus (`receive_CAN0()`/`send_CAN0()`).
- **`tatsuki/control.h`** converts `cmd_v`/`cmd_omega` (from the received Twist) to per-wheel motor "power"
  (0–300, sent raw over CAN) via simple differential-drive kinematics and a single empirical linear gain
  (`K_pwm`). **This is open-loop**: encoder values (`Md_CAN0[0].enc1/enc2`, `velEnc1/velEnc2`) are received
  over CAN every cycle but the active `control()` doesn't use them for feedback — there's no PID/velocity
  closed loop running on the robot. (An earlier encoder-feedback version is present but commented out.)
  Tread width is `L = 0.430` m (physically measured; matches `WheeledRobot_watanabe/parameters.h`'s
  `TREAD`). Wheel radius (`WHEEL_RADIUS 0.258` in that sibling dir) is **not used at all** in `tatsuki`'s
  conversion — `K_pwm` (velocity-in-m/s → power) implicitly absorbs it, so don't assume the two directories'
  gain constants are interchangeable (different unit conventions, independently tuned).
- After editing anything under `tatsuki/`, **rebuild on robot09** (`cd
  ~/Development/T+/watanabe/tatsuki && make` over ssh — there's no cross-compile setup here) before the
  next run; `make` alone picks up `control.h` changes since it's a listed dependency of `main.o`.
- Running it requires `sudo` (raw CAN socket): `sudo sh canup.sh` (brings up the CAN interface) then
  `sudo ./main`.

## Build & run

### ROS2 workspace (`ros2_ws/`)

```bash
cd ~/ros2_ws
colcon build --symlink-install                       # build everything
colcon build --symlink-install --packages-select livox_ros_driver2   # build one package
source install/setup.bash                             # after building, before ros2 run/launch
```
`livox_ros_driver2` has occasionally needed `--cmake-args -DHUMBLE_ROS=humble` when the distro isn't
auto-detected.

### `ros_bridge_tcp_simple`

```bash
cd ~/ros_bridge_tcp_simple
mkdir -p build && cd build
cmake .. && make
```
Produces `libros_tcp_bridge.a` plus `example_server`/`example_client` demo binaries.

### Mapping with GLIM (via seminar scripts)

```bash
cd ~/seminar/scripts
./start.sh manager   # terminal 1: start the ROS2 component container
./start.sh livox     # terminal 2: load the Livox driver component (uses livox_ros_driver2's MID360_config.json)
./start.sh glim      # terminal 2: load GLIM (uses ./config_glim)
```
Mapped data is dumped to `/tmp/dump`; recorded sessions get moved into `map_practice/<name>/`.
`./record.sh [-o save_path]` records a rosbag to `~/rosbags` (or `save_path`).

Alternative direct invocation seen in history:
```bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py &
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~/seminar/scripts/config_glim)
```

### Navigation stack

```bash
python3 ~/safe_bridge.py &
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
  --frame-id lidar --child-frame-id livox_frame &
ros2 launch ~/bringup_launch.py
```

### Converting a mapping session to a Nav2 map

```bash
python3 ~/ply2map.py map_practice/<session>/<cloud>.ply -o map_practice/<session>/my_map
```
Then point `bringup_launch.py`'s `map` argument (or its default) at the resulting `.yaml`.

### Full end-to-end run (mapping a hallway, then navigating it)

This is the actual sequence used in practice, across three machines/terminals:

```bash
# [robot09] motor control — bring up CAN, then run the TCP-to-CAN bridge (needs sudo, raw CAN socket)
cd ~/Development/T+/watanabe/tatsuki
sudo sh canup.sh
sudo ./main

# [Jetson, terminal 1] Livox driver + GLIM SLAM (mapping while manually driving the robot via ROS2 keyboard teleop)
cd ~/seminar/scripts && ros2 launch livox_ros_driver2 rviz_MID360_launch.py & \
  sleep 3; ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$(realpath ~/seminar/scripts/config_glim)

# [Jetson, terminal 2] TCP bridge to robot09 + static TF + Nav2 bringup
python3 ~/safe_bridge.py & \
  ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
  --frame-id lidar --child-frame-id livox_frame & \
  sleep 2; ros2 launch ~/bringup_launch.py
```

Once a GLIM map has been built and saved (see "Mapping with GLIM" above), convert it with `ply2map.py`,
point `bringup_launch.py`'s map arg at the result, then drive navigation by setting a **2D Goal Pose in
RViz** (no code involved — this is what triggers `bt_navigator`/`controller_server`).

## Known issues / in-progress debugging (see also project memory)

- **`tatsuki/control.h` direction polarity was found reversed for BOTH translation and rotation**
  (physical wiring, not a logic bug) and fixed by negating the *final* combined `calc_da1`/`calc_da2`
  (not `cmd_v` alone — an earlier attempt to negate only `cmd_v` broke rotation while fixing translation).
  If the motor board is ever replaced, re-verify with `teleop_twist_keyboard` (`i`/`,`/`j`/`l`) before
  trusting navigation again — a new board may have different polarity.
- **`robot09` has no RTC battery**: its clock resets to an arbitrary stale value on every reboot and only
  ticks forward from there. File timestamps under `.../data/*.csv` are therefore unreliable for finding
  "the most recent run" — compare `date +%s` (current) against `stat -c "%Y" <file>` (epoch), or just note
  the exact `Data file: data/XXXXXXXX-data.csv` line printed at program start.
- **`tatsuki/parameters.h`'s `END_TIME`** silently ends the whole program (clean shutdown, TCP server included)
  after this many seconds — was `300.0` (5 min, causing "robot stops responding after a while" mid-session),
  raised to `28800.0` (8h).
- **A persistent, hard-to-reproduce motor driver board fault** (uncommanded drift with zero PWM command,
  magnitude varies run-to-run) was diagnosed on `robot09`'s CAN motor driver via `WheeledRobot_takaki`
  sweep tests. Board has **not been replaced** as of the last session. An encoder-feedback compensation was
  built and validated in `tatsuki/control.h` but was reverted (kept out of the "clean" build) pending the
  hardware swap — check git-less history / ask before assuming it's still absent.
- **`~/start_nav.sh <map.yaml>`** is the current one-shot launcher for the Jetson-side stack (Livox+RViz,
  GLIM, `safe_bridge.py`, static TF, Nav2) — uses `trap ... SIGINT` so Ctrl+C on it cleanly kills every
  child process it started, preventing the "duplicate `safe_bridge.py`/GLIM/RViz processes pile up" bug that
  was found and cost real debugging time (each re-run of the old multi-`&`-in-one-terminal pattern left
  orphaned background jobs). `robot09`'s `sudo sh canup.sh && sudo ./main` still has to be run manually in
  its own terminal (needs an interactive sudo password).
- **`~/log_nav_topics.py`**: standalone rclpy node for live-diagnosing navigation issues — subscribes to
  `/cmd_vel_nav` + `/cmd_vel` and polls the `map`→`imu` TF at 20Hz, writing `~/cmd_vel_log.csv` /
  `~/pose_log.csv` (flushed every sample, safe to read while it's still running). Run it, reproduce the
  issue, Ctrl+C it, then inspect the CSVs. This is how the DWB oscillation was diagnosed (see below).
- **A previously-used map was built with the Livox mounted physically backwards relative to the current
  (correct) mounting.** Since AMCL is excluded, the static map's coordinate frame and a fresh GLIM session's
  live frame are independent — a ~180° sensor remount between mapping and nav sessions desyncs them with no
  error raised, and *looks* exactly like a control oscillation bug (robot fights to face the wrong way).
  Current known-good map: `map_practice/7_14/my_map.yaml` (built with the corrected mounting).
- **`RViz2` shows the GLIM `standard_viewer` at a hardcoded `2560x1440`** (`seminar/scripts/config_glim/config_viewer.json`,
  `viewer_width`/`viewer_height`) — reduced to `1280x800` so it doesn't open maximized every launch.
  `~/start_nav.sh` also points `livox_ros_driver2`'s `rviz_MID360_launch.py` at
  `~/my_nav_config_rviz/5_30.rviz` instead of the driver's own default display config (one-line edit in that
  vendored launch file — a deliberate local customization, not meant to go upstream).
- **DWB final-heading oscillation (long-running investigation, not fully closed)**: after fixing the map
  desync and wheel polarity above, the robot still overshoots/oscillates specifically when the goal requires
  *rotation after some translation* (pure in-place rotation goals converge fine). Ruled out: GLIM yaw noise
  while stationary (≈0.5° std, too small), `PathAlign`/`GoalAlign` critic instability (set to `0.0`, no
  effect), duplicate-process TCP flakiness (fixed separately, oscillation persisted). Confirmed contributing
  factors: `FollowPath.max_vel_theta` too high relative to `vtheta_samples` gives DWB coarse angular-velocity
  quantization near zero error (lowering `max_vel_theta` and/or raising `vtheta_samples` reduced but didn't
  eliminate it); `FollowPath.acc_lim_x` at `1.0` empirically correlated with oscillation, `0.5` empirically
  resolved it once. Leading open hypothesis: `RotateToGoal.scale` (currently tuned way down to `~7` while
  chasing the above) is now *weaker* than `PathDist.scale`/`GoalDist.scale` (32/24 each, never reduced), so
  once a real (non-trivial-length) path existed before arrival, those two critics may out-vote `RotateToGoal`
  during the final rotate-in-place phase — next step was raising `RotateToGoal.scale` back up
  (~32-40) while keeping the sampling-resolution fixes, not yet confirmed. Check current
  `my_robot_nav/config/nav2_params.yaml` values directly rather than trusting any specific numbers here, since
  this was actively being tuned.

## Architecture notes

- **Data flow**: Livox MID360 (`livox_ros_driver2`) → GLIM (`glim`/`glim_ros2`, extended by `glim_ext`
  modules) produces LiDAR-IMU odometry/mapping, publishing pose (`imu` frame) and building a global map →
  Nav2 (`my_robot_nav` config, launched via `bringup_launch.py`) consumes that pose plus a pre-built map
  (from `ply2map.py`) for localization-free navigation (AMCL is explicitly excluded from the bringup) →
  Nav2 emits `/cmd_vel` (or app publishes to `/cmd_vel_nav`) → either `udp_bridge.py` or
  `safe_bridge.py`/`ros_bridge_tcp_simple` forwards velocity commands off the ROS2 graph to the physical
  robot controller over a plain TCP/UDP socket (the robot side is a separate, non-ROS system reachable at a
  hardcoded IP).
- **Two independent robot-command transports exist** (`udp_bridge.py` over UDP, and
  `safe_bridge.py` + `ros_bridge_tcp_simple` over TCP with a defined packet protocol). Check which one is
  actually wired up (target IP/port, `/cmd_vel` vs `/cmd_vel_nav` topic) before assuming either is current.
- **GLIM config is data-driven**: behavior changes for mapping/odometry usually mean editing the JSON files
  under `glim/config/` or `glim_ext/config/` (or a copy like `seminar/scripts/config_glim`), not the C++.
- **Vendored vs. custom code**: `glim*` and `livox_ros_driver2` are separate upstream git clones inside
  `ros2_ws/src/`; everything else in this repo (`my_robot_nav`, `ros_bridge_tcp_simple`, `seminar`, the
  top-level `*.py` scripts) is local/custom and has no upstream to sync against.
- Filenames and comments throughout are a mix of Japanese and English; some legacy files (`udp_bridge.py`)
  have mis-decoded (mojibake) comments from an encoding mismatch — don't try to "fix" the garbled text
  without checking the original source encoding (likely Shift-JIS).
- **`/cmd_vel` vs `/cmd_vel_nav` — do not confuse these.** `nav2_bringup`'s stock `navigation_launch.py`
  (included by `bringup_launch.py`) remaps `controller_server`'s output to `/cmd_vel_nav` (raw, unsmoothed
  DWB output) and has `velocity_smoother` consume `/cmd_vel_nav` and publish the final, smoothed command
  back out as `/cmd_vel` (accel/decel limits and `deadband_velocity` only apply on this final hop). Anything
  that actually drives the robot (`safe_bridge.py`, `udp_bridge.py`, `teleop_twist_keyboard` during manual
  driving) must subscribe to **`/cmd_vel`**, never `/cmd_vel_nav` — otherwise `velocity_smoother` is silently
  bypassed and any tuning done there has no effect. This was found as a live bug (`safe_bridge.py` was on
  `/cmd_vel_nav`) and fixed.
- **AMCL is excluded from `bringup_launch.py`**, so there is no relocalization step: the live pose comes
  straight from GLIM/IMU with no correction against the saved map. Navigation only works if it's run in the
  *same continuous GLIM session* that built the map (or the robot is physically placed at the exact pose GLIM
  started from) — restarting GLIM between mapping and navigating resets its origin and the robot's pose will
  no longer line up with the static map, with no error raised.
- **`bringup_launch.py`'s `map` argument defaults to `~/map_practice/5_19/my_map.yaml`** (an old session).
  Always pass `map:=<path/to/new/session>/my_map.yaml` explicitly after generating a new map with
  `ply2map.py` — otherwise Nav2 silently navigates against a stale map.
- **No closed-loop velocity control anywhere in the pipeline**: Nav2's `odom`/pose comes from GLIM/IMU
  (`base_frame_id: imu`), not wheel encoders, and `robot09`'s `control()` (see "Stick PC" above) doesn't use
  its own encoder feedback either. So the only place velocity/heading errors get corrected is Nav2's DWB
  local planner — there's no lower-level PID anywhere to fall back on. Keep this in mind when diagnosing
  motion issues (e.g. oscillation, overshoot): the fix almost always belongs in `nav2_params.yaml`'s
  `controller_server`/`FollowPath` block, not in the CAN control code, *unless* the open-loop
  velocity→PWM gain (`K_pwm` in `tatsuki/control.h`) or motor deadband is implicated.
