# Machine-Vision — 带深度相机的视觉导航机器人仿真

基于 **ROS 2 Jazzy + Gazebo Harmonic + YOLO + 大模型（VLM/LLM）** 的机器人视觉仿真项目：
机器人在 Gazebo 世界里通过机载相机观察环境，YOLO 做实时目标检测，避障节点用深度相机测距并给出
左/中/右三区最近距离，支持键盘控制与大模型兜底决策；本仓库另含**机器人/相机位置感知**。

## 功能特性

- **带相机机器人模型**：`robot_with_camera.sdf` —— RGB 相机 + 深度相机（与 RGB 同位置、同视场，像素对齐）。
- **键盘控制**：`keyboard_joint_controller.py` 控制相机旋转与底盘平移；**空闲时完全不发布指令**，
  并用 `/avoider_override` 与避障节点协作（按键期间自动接管、松开交还）。
- **实时目标检测**：`yolo.sh` 启动 YOLO，订阅 `/camera`，发布 `/yolo/detections`。
- **规则优先避障 + 大模型兜底**：`obstacle_avoider` 用深度图算左/中/右三区最近距离，发布
  `/nearest_obstacle`、`/obstacle_zones`；模糊场景调用 LLM（本地 LM Studio 优先、云端兜底，失败回落纯规则）。
- **深度彩色图**：避障包内的 `depth_probe` 把深度图转成伪彩色 `/depth_camera/colorized`。
- **VLM 相机旋转**：`vlm_navigator` 让大模型看图后决定相机转动方向。
- **机器人/相机位置感知**（`robot_pose_monitor.py`，本仓库新增）：由 `/joint_states` 真值关节角解算
  `x / y / yaw`，发布 `/robot_pose`、`/camera_pose`、`/odom`、`/robot_path` 与 TF
  `odom → base_link → camera_link → camera_optical_frame`。
- **目标点导航**（`navigate_to_pose.py`，本仓库新增）：给一个坐标（命令行 / 终端输入 / `/goal_pose` /
  `/navigate_to`），机器人用已有自定位平滑走过去并转向指定朝向，到点误差约 2 cm；
  带定位断流、被抢占、超时、越界四重保护。

## 环境要求

| 组件 | 版本 |
|------|------|
| Ubuntu | 24.04（本项目在 WSL2 + Ubuntu 24.04 上验证） |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic（命令 `gz`） |
| Python | 3.12 |

## 目录结构

```
Machine-Vision/
├── all.sh                            # 一键：清理残留 + 自动编译避障包 + 依次起 仿真/YOLO/RViz
├── start_simulation.sh               # 仿真 + 键盘 + 全部话题桥
├── yolo.sh                           # 启动 YOLO 检测
├── run_obstacle_avoider.sh           # 启动避障节点（含 LLM failover 配置 + 桥保活）
├── run_navigate.sh                   # 一键启动导航（可带坐标参数）
├── view_camera.sh                    # RViz2（相机图 + 轨迹）+ 深度彩色图发布 + 静态 TF
├── nodes/                            # 本仓库写的 ROS 节点
│   ├── keyboard_joint_controller.py  # 键盘控制（空闲静默，与避障协作）
│   ├── robot_pose_monitor.py         # 位置感知节点：位姿解算 + TF
│   └── navigate_to_pose.py           # 导航节点：输入坐标 → 平滑移动到坐标处
├── config/
│   ├── robot_pose.rviz               # view_camera.sh 用的 RViz 配置（Fixed Frame=odom，带轨迹）
│   └── requirements.txt              # 系统 Python 侧依赖
├── worlds/
│   └── robot_with_camera.sdf         # Gazebo 世界 + 机器人模型（RGB 相机 + 深度相机）
├── mod/                              # 仿真用第三方模型（车辆 / 行人 / 地面等）
├── obstacle_ws/                      # colcon 工作区：避障节点 + depth_probe
├── vlm_navigator/                    # colcon 包：VLM 相机旋转
└── yolo_ros2_ws/                     # YOLO colcon 工作区（第三方）
```

## 快速开始

> **放哪里**：脚本都按「自己所在目录」定位仓库根（顶部一行 ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"），
> 所以克隆到任何位置都能直接跑，不要求 `~/gz_ros`、也不需要软链接：
> ```bash
> git clone https://github.com/gnmn2233-tech/Machine-Vision
> ```
> 只有手写 `cd yolo_ros2_ws && colcon build` 这类命令，才需要你自己先 cd 到仓库根目录。

### 1. 安装依赖

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-interfaces ros-jazzy-rviz2 ros-jazzy-tf2-ros
pip3 install -r config/requirements.txt      # 报 externally-managed-environment 时加 --break-system-packages
```

### 2. 编译两个工作区

```bash
# ① yolo_ros2_ws
cd yolo_ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
```

```bash
# ② obstacle_ws（依赖 yolo_msgs，先 source 上一个工作区）
cd obstacle_ws && source /opt/ros/jazzy/setup.bash \
  && source ../yolo_ros2_ws/install/setup.bash && colcon build --symlink-install
```

> YOLO 权重不在仓库里（`.gitignore` 排除了 `*.pt`）：首次运行会自动下载 `yolov8m.pt`，
> 也可手动放到 `yolo_ros2_ws/`。

### 3. 运行

```bash
bash all.sh        # 一键：仿真 + YOLO + RViz（会先自动编译避障包）
```

或分终端（脚本都自定位仓库根，任意目录执行都行）：

```bash
bash start_simulation.sh        # 终端1：仿真 + 键盘 + 全部话题桥
bash yolo.sh                    # 终端2：YOLO 检测
bash run_obstacle_avoider.sh    # 终端3：避障（可选）
bash view_camera.sh             # 终端4：RViz + 深度彩色图（可选）
```

### 4. 位置感知

```bash
python3 nodes/robot_pose_monitor.py   # 需要 start_simulation.sh 已在运行
# 轨迹/位姿不用另开窗口：all.sh 第 3 路的 RViz2 已带 Path(/robot_path) 与 /odom 箭头
```

> `all.sh` 已经把它接成第 4 路，并且第 3 路那个 RViz2 窗口本身就用本文的 `robot_pose.rviz`
> 启动（Fixed Frame = `odom`）——所以**轨迹不用另开窗口**，跑 `all.sh` 就在那个 RViz2 里看到
> `Path(/robot_path)` 的线和 `Odometry(/odom)` 的箭头；要看相机图仍照旧 Add → Image。

## 位置感知（机器人 / 相机位姿）

数据来自 Gazebo 真值关节角：SDF 里的 `JointStatePublisher` 同时发布
`drive_x_joint` / `drive_y_joint` / `base_yaw_joint`。

```
x   = drive_x_joint 位置            世界 x (m)
y   = drive_y_joint 位置            世界 y (m)
yaw = base_yaw_joint 位置           左转为正 (rad)；节点内做了 ±π 去卷，连续转多圈不跳变
base 世界位姿 = (x, y, 0.075, yaw)             base 连杆比模型原点高 0.075 m
相机 世界位姿 = base + R(yaw)·(0.1, 0, 0.205)   相机高度恒为 0.28 m
```

> 相机高度是 **0.28 m**（模型原点上方）。SDF 里 `camera_joint` 的 `z=0.175` 被 `cylinder` 连杆自身的
> `<pose>0 0 0</pose>` 覆盖，不要按 0.075+0.175+0.28 去算。

| 话题 | 类型 | 说明 |
|------|------|------|
| `/robot_pose` | `geometry_msgs/PoseStamped` | 机器人位姿（frame: odom） |
| `/camera_pose` | `geometry_msgs/PoseStamped` | 相机光心位姿（frame: odom） |
| `/odom` | `nav_msgs/Odometry` | 里程计（yaw 已修正，frame: odom → base_link） |
| `/robot_path` | `nav_msgs/Path` | 运动轨迹（RViz 显示） |
| `/robot_label` | `visualization_msgs/MarkerArray` | 机器人头顶的坐标文字（RViz MarkerArray 显示 `x/y/yaw`，frame: odom） |
| TF | — | `odom → base_link → camera_link → camera_optical_frame` |

### 在终端查看当前位置（使用者最常用）

**前提**：两个终端在跑 —— `bash start_simulation.sh`（提供 `/joint_states`）
和 `python3 nodes/robot_pose_monitor.py`（位置感知节点）。

**① 最省事：位置感知节点自己每 1 秒打印一行**

```
[INFO] [robot_pose_monitor]: x=+1.50 y=-0.80 yaw=+34.3 deg | 相机(+1.58, -0.74, 0.28)
```

**② 另开一个终端订阅一次**（Windows 用户先执行 `wsl` 进入 Ubuntu-24.04）

```bash
source /opt/ros/jazzy/setup.bash      # ~/.bashrc 已自动 source 过则可省略
ros2 topic echo /robot_pose --once
```

真实输出：

```yaml
header:
  stamp:
    sec: 1791451462
    nanosec: 646682793
  frame_id: odom
pose:
  position:
    x: 1.4999999999995859
    y: -0.7999999999950517
    z: 0.075
  orientation:
    x: 0.0
    y: 0.0
    z: 0.29187398255587677
    w: 0.9564567832928844
---
```

**③ 只要坐标数字**

```bash
ros2 topic echo /robot_pose --once --field pose.position
# x: 1.4999999999999962
# y: -0.7999999999999968
# z: 0.075
```

**④ 连续刷新**（去掉 `--once`，Ctrl+C 退出）

```bash
ros2 topic echo /robot_pose
```

**⑤ 相机光心的位置**（比机器人中心高，且随云台转动绕着机器人画小圆）

```bash
ros2 topic echo /camera_pose --once
# position: x=1.5827  y=-0.7438  z=0.28
```

**怎么把四元数读成朝向角**：`yaw = atan2(2(w·z + x·y), 1 − 2(y² + z²))`。
上面那组 `z=0.2919, w=0.9565` 就是 **34.3°**。不想算就直接看 ① 里节点打印的 `yaw=+34.3 deg`，
或读 `/odom`（同样的四元数，`child_frame_id` 标了 `base_link`）。

**排错**

| 现象 | 原因 / 处理 |
|------|------|
| `--once` 一直卡住不输出 | 没人在发布 → 确认 `start_simulation.sh` 和 `nodes/robot_pose_monitor.py` 都在跑 |
| `ros2: command not found` | 忘了 `source /opt/ros/jazzy/setup.bash`，或不在 WSL 里 |
| 数值一直是 0 | `/joint_states` 没数据 → `ros2 topic echo /joint_states --once`，应含 `drive_x_joint` / `drive_y_joint` / `base_yaw_joint` |

```bash
ros2 topic hz /robot_pose       # 发布频率（约 20 Hz）
ros2 node list                  # 应能看到 /robot_pose_monitor
ros2 run tf2_tools view_frames  # 生成 TF 树 PDF
```

## 目标点导航（输入坐标，机器人自己走过去）

`navigate_to_pose.py` 把「给一个坐标」变成「机器人平滑移动到这个坐标」。
它复用机器人已有的自定位（`/joint_states` 真值 → `x / y / yaw`），不重复造轮子。

**为什么不是 `/cmd_vel` 那一套**：本底盘不是差速轮，而是两个 prismatic 关节
（`drive_x_joint` / `drive_y_joint`，各 ±9 m）加一个转向关节（`base_yaw_joint`），由 Gazebo 的
`JointPositionController` 做**位置伺服**——没有 `/cmd_vel` 可发，所以导航发布的是「下一个位置指令」。

**运动方式和键盘控制一致：只会「原地转向」和「沿车头前进」，不横移。**

1. 目标方向与车头夹角超过 `heading_gate`（默认 8.6°）→ 先**原地转向**对准，期间不前进；
2. 对准后沿车头方向直行，每拍前进 `v·dt`（`v = min(v_max, √(2·a·dist))`，离终点越慢），
   边走边用 `turn_gain` 做小角度修正，车头始终朝运动方向；
3. 进 2 cm 内停车；若目标带了朝向，再原地转到该朝向。

指令用内部累积量而非「实测 + 增量」递推——位置伺服有滞后，指令贴着实测就没有误差可追，
转向会被拖成每 tick 零点几度（实测约 2°/s，累积后约 50°/s）。

### 开跑

```bash
bash start_simulation.sh     # 终端1：仿真 + 话题桥（bash all.sh 也可以）
bash run_navigate.sh         # 终端2：不带参数 = 交互模式
```

终端2 会先打印当前位置和用法，然后停在 `坐标 >` 提示符等你输入：

```
================================================================
  目标点导航 — 输入坐标, 机器人自己走过去
  当前位置: x=+0.00 m  y=+0.00 m  yaw=+0.0 deg
================================================================
输入格式 (每行一条, 回车执行):
  x y          走到 (x, y), 朝向不变          例: 2.0 -1.5
  x y 角度     走到 (x, y) 并转到该角度(度)    例: 1.0 1.0 90
  stop         取消当前目标, 停在原地
  s            看当前位置与状态
  h            再看一遍这份帮助
  q            退出

坐标单位米, 原点是仿真启动位置; 底盘行程 ±9 m。
================================================================

坐标 > 1.0 1.0
  已接收 (+1.00, +1.00)
[INFO] → 前进中 剩余 1.41 m, 速度 0.50 m/s | 当前 (+0.00, +0.00) -> 目标 (+1.00, +1.00)
[INFO] ✓ 已到达 (+1.00, +1.00) | 实际 x=+0.99 y=+0.99 yaw=+0.0 deg | 误差 19 mm
坐标 > 
```

到点后提示符会重新出现，可以接着下一条目标。

也可以在命令行一次给全：

```bash
bash run_navigate.sh 2.0 -1.5              # 走到 (2, -1.5)，朝向不变
bash run_navigate.sh 2.0 -1.5 --yaw 90     # 走到 (2, -1.5) 并转到 90 度
bash run_navigate.sh 1 0 2 1 0 0           # 多航点：(1,0) → (2,1) → (0,0)
bash run_navigate.sh -p v_max:=0.8 3 -3    # 改最大速度 (m/s)
```

### 也可以用话题 / RViz

| 入口 | 用法 |
|------|------|
| `/goal_pose`（`geometry_msgs/PoseStamped`） | RViz 里用 **2D Goal Pose** 点一下即走 |
| `/navigate_to`（`geometry_msgs/Pose2D`） | `ros2 topic pub --once /navigate_to geometry_msgs/msg/Pose2D "{x: 1.0, y: 1.5, theta: 1.5708}"` |
| `/nav_cancel`（`std_msgs/Bool`） | `ros2 topic pub --once /nav_cancel std_msgs/msg/Bool "{data: true}"` |
| 终端输入 | 交互模式下输入 `stop` 回车，或 `q` 退出 |

> `/goal_pose` 里的 `orientation` 默认被忽略（默认四元数 `w=1` 视为「没指定朝向」）。
> 想让 RViz 点出来的朝向也生效，启动时加 `-p use_orientation:=true`。

### 怎么看进度 / 完成

```bash
ros2 topic echo /navigation_status      # IDLE / MOVING / ARRIVED + 实时坐标
ros2 topic echo /nav_goal_reached --once
```

节点每秒打印一次进度，到点打印误差：

```
[INFO] [navigate_to_pose]: → 前进中 剩余 0.92 m, 速度 0.50 m/s | 当前 (+0.82, -0.41) -> 目标 (+0.00, +0.00)
[INFO] [navigate_to_pose]: ✓ 已到达 (+0.00, +0.00) | 实际 x=+0.02 y=-0.01 yaw=+60.0 deg | 误差 19 mm
```

### 可调参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `v_max` | 0.5 | 平移最大速度 (m/s) |
| `a_max` | 1.0 | 加减速 (m/s²)，决定进站减速距离 |
| `w_max` | 1.0 | 转向最大角速度 (rad/s) |
| `arrive_eps` | 0.02 | 到点判定的位置阈值 (m) |
| `yaw_eps` | 0.02 | 到点判定的朝向阈值 (rad，≈1.1°) |
| `control_hz` | 50 | 控制与发布频率 |
| `goal_timeout` | 180 | 单个目标点的超时 (s) |
| `heading_gate` | 0.15 | 朝向偏差超过该值 (rad，≈8.6°) 先原地转向再走 |
| `turn_gain` | 2.0 | 前进时朝向修正的 P 增益 |
| `use_orientation` | false | 让 `/goal_pose` 里的四元数当作目标朝向 |

改法：`bash run_navigate.sh -p v_max:=0.8 3 -3`（`-p` 会转成 ROS 参数传给节点）。

### 安全保护（实测过的四种）

| 保护 | 行为 |
|------|------|
| 目标越界 | 超出 ±9 m 行程直接拒绝并报错，不发指令 |
| 定位断流 | `/joint_states` 断 1 s → **暂停推进**（机器人自然停住），断 6 s → 放弃目标；恢复后自动接着走 |
| 控制权被抢 | 半秒内反而离目标更远 25 cm → 判定被键盘/避障/手动接管，立即中止导航并交还 |
| 超时 | 单个目标 180 s 未到点 → 停车 |

导航运动期间节点会发 `/avoider_override=1`（与键盘控制器的约定一致，让避障节点让路）；
停止运动或空闲时静默，不覆盖键盘的接管信号。

`keyboard_joint_controller.py` 订阅了 `/navigation_status`：状态为 `MOVING` 时键盘控制器**完全静默**
（不碰关节话题、也不发 override），日志打 `导航接管中, 键盘静默`；导航一结束自动打
`导航已结束, 键盘恢复`，方向键立刻又能用。所以 `all.sh` 全栈开着也不会和导航抢关节话题。

### 排错

| 现象 | 原因 / 处理 |
|------|------|
| `✗ 没检测到 Gazebo 仿真进程` | 先在另一个终端 `bash start_simulation.sh` |
| 一直「还没收到 /joint_states」 | 话题桥没起：`ros2 topic echo /joint_states --once` 应有三个关节；也可 `ros2 node list --no-daemon` 确认 `/ros_gz_bridge` 在 |
| 走了一半自己停 | 看日志：`定位中断` 会等恢复；`控制权被抢占` 说明有别的节点在发 `/drive_*` |
| 机器人抖动 / 抢关节 | 确认只有一个导航或键盘节点在跑：`pgrep -af 'navigate_to_pose|keyboard_joint' ` |
| 到点后轻微残留误差 | 正常，到点阈值 20 mm（视野内看不出）；要更严就调小 `arrive_eps`，但仿真物理抖动可能反而到不了 |

## 话题速查

| 话题 | 类型 | 说明 |
|------|------|------|
| `/camera` | `sensor_msgs/Image` | RGB 图像 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | RGB 内参 |
| `/depth_camera` | `sensor_msgs/Image` | 深度图（32FC1，单位米，inf = 未命中） |
| `/depth_camera/camera_info` | `sensor_msgs/CameraInfo` | 深度内参 |
| `/depth_camera/colorized` | `sensor_msgs/Image` | 深度伪彩色（view_camera.sh 里的 depth_probe 发布） |
| `/joint_states` | `sensor_msgs/JointState` | 三个关节真值 |
| `/camera_joint_controller/commands` | `std_msgs/Float64` | 相机旋转关节目标角 |
| `/drive_x/commands`、`/drive_y/commands` | `std_msgs/Float64` | 底盘平移位置指令 |
| `/nearest_obstacle` | `std_msgs/Float64` | 最近障碍距离（米，inf = 空旷） |
| `/obstacle_zones` | `std_msgs/Float32MultiArray` | 左/中/右三区距离 |
| `/avoider_override` | `std_msgs/Float64` | 键盘手动接管信号（1 = 手动中） |
| `/yolo/detections` | `yolo_msgs/DetectionArray` | YOLO 检测结果 |
| `/robot_pose` 等 | — | 见上一节「位置感知」 |
| `/navigation_status` | `std_msgs/String` | 导航状态（IDLE / MOVING / ARRIVED + 实时坐标） |
| `/nav_goal_reached` | `std_msgs/Bool` | 到点信号（true = 到达） |
| `/navigate_to` | `geometry_msgs/Pose2D` | 输入导航目标（x, y, theta） |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 输入导航目标（RViz 2D Goal Pose 走这条） |
| `/nav_cancel` | `std_msgs/Bool` | 取消当前导航 |

## 在 Windows 上克隆（重要）

Windows 的 Git 默认 `core.autocrlf=true`，会把整个仓库检出成 CRLF：**所有文件显示为已修改**，
而且 `.sh` 在 bash 下直接报 `\r` 语法错误；此时若 `git add -A` 提交会毁掉整个仓库。

本仓库已包含 `.gitattributes`（`* text=auto eol=lf`）来规范行尾。为保险起见，克隆后仍建议执行一次：

```bash
git config --global core.autocrlf false
```

## 第三方组件与许可

- **yolo_ros**（`yolo_ros2_ws/src/yolo_ros`）：来自 [mgonzs13/yolo_ros](https://github.com/mgonzs13/yolo_ros)，GPL-3.0。
- **Gazebo 模型**（`mod/`）：来自 Gazebo 官方/社区模型库，版权归原模型作者。
- **yolov8*.pt**：Ultralytics 预训练权重（AGPL-3.0），未随仓库分发，按上文自行下载。
