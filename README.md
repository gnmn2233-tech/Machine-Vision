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
├── robot_with_camera.sdf          # Gazebo 世界 + 机器人模型（RGB 相机 + 深度相机）
├── start_simulation.sh            # 仿真 + 键盘 + 全部话题桥
├── all.sh                         # 一键：清理残留 + 自动编译避障包 + 依次起 仿真/YOLO/RViz
├── keyboard_joint_controller.py   # 键盘控制（空闲静默，与避障协作）
├── yolo.sh                        # 启动 YOLO 检测
├── run_obstacle_avoider.sh        # 启动避障节点（含 LLM failover 配置 + 桥保活）
├── view_camera.sh                 # RViz2（相机图 + 轨迹）+ 深度彩色图发布 + 静态 TF
├── mod/                           # 仿真用第三方模型（车辆 / 行人 / 地面等）
├── obstacle_ws/                   # colcon 工作区：避障节点 + depth_probe
├── vlm_navigator/                 # colcon 包：VLM 相机旋转
├── yolo_ros2_ws/                  # YOLO colcon 工作区（第三方）
│
│   ── 机器人/相机位置感知（本次新增） ──
├── robot_pose_monitor.py          # 位置感知节点：位姿解算 + TF
├── view_pose.sh                   # RViz2 查看位姿与轨迹
├── robot_pose.rviz                #   两个可视化脚本共用的 RViz 配置
└── requirements.txt               # 系统 Python 侧依赖
```

## 快速开始

> **放哪里**：各脚本按 `~/gz_ros` 这个目录布局引用路径（例如 `$HOME/gz_ros/mod`、
> `$HOME/gz_ros/yolo_ros2_ws/install`）。建议直接克隆到这个位置：
> ```bash
> git clone https://github.com/gnmn2233-tech/Machine-Vision ~/gz_ros
> ```
> 若克隆到别处，可建个软链接：`ln -s <你的目录> ~/gz_ros`。

### 1. 安装依赖

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-interfaces ros-jazzy-rviz2 ros-jazzy-tf2-ros
pip3 install -r requirements.txt      # 报 externally-managed-environment 时加 --break-system-packages
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

或分终端（都不要求当前目录，但 `all.sh` 用相对路径，必须在仓库根目录执行）：

```bash
bash start_simulation.sh        # 终端1：仿真 + 键盘 + 全部话题桥
bash yolo.sh                    # 终端2：YOLO 检测
bash run_obstacle_avoider.sh    # 终端3：避障（可选）
bash view_camera.sh             # 终端4：RViz + 深度彩色图（可选）
```

### 4. 位置感知

```bash
python3 robot_pose_monitor.py   # 需要 start_simulation.sh 已在运行
bash view_pose.sh               # 另开终端：单独的位姿窗口（可选）
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
和 `python3 robot_pose_monitor.py`（位置感知节点）。

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
| `--once` 一直卡住不输出 | 没人在发布 → 确认 `start_simulation.sh` 和 `robot_pose_monitor.py` 都在跑 |
| `ros2: command not found` | 忘了 `source /opt/ros/jazzy/setup.bash`，或不在 WSL 里 |
| 数值一直是 0 | `/joint_states` 没数据 → `ros2 topic echo /joint_states --once`，应含 `drive_x_joint` / `drive_y_joint` / `base_yaw_joint` |

```bash
ros2 topic hz /robot_pose       # 发布频率（约 20 Hz）
ros2 node list                  # 应能看到 /robot_pose_monitor
ros2 run tf2_tools view_frames  # 生成 TF 树 PDF
```

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
