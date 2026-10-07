# Machine-Vision — 带深度相机的视觉导航机器人仿真

基于 **ROS 2 Jazzy + Gazebo Harmonic + YOLO + 大模型（VLM/LLM）** 的机器人视觉仿真项目。
机器人在 Gazebo 世界里通过机载相机观察环境：YOLO 做目标检测，避障节点用深度相机测距，
并支持键盘控制、大模型兜底决策、以及**机器人/相机位置感知**与**A→B 导航**。

## 功能特性

- **带相机机器人模型**：`robot_with_camera.sdf`，RGB 相机 + **深度相机**（与 RGB 同位置同视场、像素对齐）。
- **键盘控制**：`keyboard_joint_controller.py` 控制相机旋转与底盘平移；**空闲时完全静默**，
  由 `/avoider_override` 与避障节点协作，按键期间自动接管、松开交还。
- **实时目标检测**：YOLO（`yolo.sh`）订阅 `/camera`，发布 `/yolo/detections`。
- **规则优先避障 + 大模型兜底**：`obstacle_avoider` 用深度图算左/中/右三区最近距离，
  发布 `/nearest_obstacle`、`/obstacle_zones`；模糊场景用 LLM（本地 LM Studio 优先、云端兜底）。
- **深度可视化**：`depth_probe`（避障包内）发布 `/depth_camera/colorized` 深度彩色图。
- **VLM 相机旋转**：`vlm_navigator` 让大模型看图后决定相机转动方向。
- **机器人/相机位置感知**（新增）：`robot_pose_monitor.py` 用 Gazebo 真值关节角解算位姿，
  发布 `/robot_pose`、`/camera_pose`、`/odom`、`/robot_path` 与 TF。
- **A→B 导航**（新增）：`goal_navigator.py` go-to-goal：先对准 → 直行 → 到点停；
  前方 0.5 m 内有障碍就停车让开；在 RViz 用「2D Goal Pose」点一下目标，并在 Gazebo 里生成目标球。
- **深度可视化（增强版）**（新增）：`depth_view.py` 输出 `/depth_view` —— 深度伪彩色 + 左/中/右分区读数
  + YOLO 检测框与每个目标的距离，可定时存 PNG。
- **深度相机自检**（新增）：`check_depth.py` 一条命令查 6 项。

## 环境要求

本项目在 **WSL2 + Ubuntu 24.04 LTS** 上开发、验证。

| 组件 | 版本 |
|------|------|
| Ubuntu | 24.04 LTS |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic（命令 `gz`） |
| Python | 3.12（系统自带，与 ROS Jazzy 匹配） |

## 目录结构

```
Machine-Vision/
├── robot_with_camera.sdf          # Gazebo 世界 + 机器人模型（RGB 相机 + 深度相机）
├── start_simulation.sh            # 仿真 + 键盘 + 全部话题桥（含深度、关节状态、实体服务）
├── all.sh                         # 一键：清理残留 + 自动编译避障包 + 依次起仿真/YOLO/RViz
├── keyboard_joint_controller.py   # 键盘控制（空闲静默，与避障协作）
├── yolo.sh                        # 启动 YOLO 检测
├── run_obstacle_avoider.sh        # 启动避障节点（含 LLM failover 配置 + 桥保活）
├── view_camera.sh                 # RViz2 + 深度彩色图发布 + 静态 TF
├── mod/                           # 仿真用第三方模型（车辆/行人/地面等）
├── obstacle_ws/                   # colcon 工作区：避障节点 + depth_probe
├── vlm_navigator/                 # colcon 包：VLM 相机旋转
├── yolo_ros2_ws/                  # YOLO colcon 工作区（第三方）
│
│   ── 以下为「位置感知 / 导航 / 深度可视化」新增 ──
├── robot_pose_monitor.py          # 位置感知节点：/robot_pose /camera_pose /odom /robot_path + TF
├── view_pose.sh                   # RViz2 只定位姿与轨迹
├── robot_pose.rviz                #   上面的 RViz 配置
├── goal_navigator.py              # A→B 导航节点（go-to-goal + 深度急停 + Gazebo 目标球）
├── run_navigator.sh               # 启动导航节点
├── view_nav.sh                    # RViz2 导航可视化（位姿/轨迹/深度图/目标点/2D Goal Pose 工具）
├── robot_nav.rviz                 #   上面的 RViz 配置
├── depth_view.py                  # 深度可视化：伪彩色 + 检测框/距离 + 左中右分区 → /depth_view
├── view_depth.sh                  # 一键启动上面的节点 + rqt_image_view
├── check_depth.py                 # 深度相机自检（一条命令查 6 项）
└── requirements.txt               # 系统 Python 侧依赖（requests/aiohttp/pynput/opencv 等）
```

## 快速开始

### 1. 安装依赖并编译

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-rviz2 ros-jazzy-rqt-image-view
sudo apt install ros-jazzy-ros-gz-interfaces ros-jazzy-tf2-ros
pip3 install -r requirements.txt      # 如报 externally-managed 可加 --break-system-packages

cd ~/gz_ros/yolo_ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
cd ~/gz_ros/obstacle_ws   && source /opt/ros/jazzy/setup.bash && \
    source ~/gz_ros/yolo_ros2_ws/install/setup.bash && colcon build --symlink-install
```

### 2. 运行

```bash
# 终端1：仿真 + 键盘 + 全部桥接
bash ~/gz_ros/start_simulation.sh
# 终端2：YOLO 检测
bash ~/gz_ros/yolo.sh
# 终端3：避障节点（可选）
bash ~/gz_ros/run_obstacle_avoider.sh
# 或用 all.sh 一键起前面三路（会先自动编译避障包）
bash ~/gz_ros/all.sh
```

### 3. 新增功能

```bash
# 位置感知（导航的前提，必须先起）
python3 ~/gz_ros/robot_pose_monitor.py
bash ~/gz_ros/view_pose.sh            # 另开终端看位姿与轨迹

# A→B 导航
bash ~/gz_ros/run_navigator.sh
bash ~/gz_ros/view_nav.sh             # RViz 里点 "2D Goal Pose" → 地面点一下

# 深度可视化
bash ~/gz_ros/view_depth.sh           # rqt 里话题选 /depth_view

# 深度相机自检
python3 ~/gz_ros/check_depth.py
```

## 位置感知（机器人 / 相机位姿）

数据全部来自 Gazebo 真值关节角（`/joint_states`，需要 SDF 里 `JointStatePublisher` 同时发布
`drive_x_joint` / `drive_y_joint` / `base_yaw_joint`）。

```
x   = drive_x_joint 位置          （世界 x）
y   = drive_y_joint 位置          （世界 y）
yaw = base_yaw_joint 位置         （左转为正；节点内做了 ±π 去卷，连续转多圈不跳变）
base   世界位姿 = (x, y, 0.075, yaw)
相机   世界位姿 = base + R(yaw)·(0.1, 0, 0.205)，高度恒为 0.28 m
```

| 话题 | 类型 | 说明 |
|------|------|------|
| `/robot_pose` | `geometry_msgs/PoseStamped` | 机器人位姿（frame: odom） |
| `/camera_pose` | `geometry_msgs/PoseStamped` | 相机光心位姿（frame: odom） |
| `/odom` | `nav_msgs/Odometry` | 里程计（yaw 已修正） |
| `/robot_path` | `nav_msgs/Path` | 运动轨迹（RViz 显示） |
| TF | — | `odom → base_link → camera_link → camera_optical_frame` |

> 相机安装高度是 **0.28 m**（模型原点上方）。注意 SDF 里 `camera_joint` 的 `z=0.175` 被
> `cylinder` 连杆自身的 `<pose>0 0 0</pose>` 覆盖，不要照抄 0.53。

## A→B 导航（`goal_navigator.py`）

| 话题 | 方向 | 说明 |
|------|------|------|
| `/robot_pose` | 订阅 | 当前位姿 |
| `/goal_pose` | 订阅 | 目标点（RViz「2D Goal Pose」默认话题） |
| `/depth_camera` | 订阅 | 深度图（避障；可用 `-p depth_topic:=` 改） |
| `/avoider_override` | 订阅 | 键盘手动接管信号：为 1 时导航暂停让路 |
| `/camera_joint_controller/commands` `/drive_x/commands` `/drive_y/commands` | 发布 | 复用键盘同一套指令接口 |
| `/nav_status` | 发布 | `IDLE/ALIGN/DRIVE/BLOCKED/ARRIVED/MANUAL` |
| `/goal_marker` | 发布 | RViz 里的目标点标记 |

控制律（50 Hz）：

| 条件 | 状态 | 行为 |
|------|------|------|
| 距目标 < `goal_tol`(0.20 m) | `ARRIVED` | 停 |
| 前方扇区深度 < `stop_dist`(0.50 m) | `BLOCKED` | 停车，朝更空旷一侧转（带 1.2 s 强制让开滞回） |
| 朝向误差 > `yaw_tol`(0.15 rad) | `ALIGN` | 原地对准 |
| 其余 | `DRIVE` | 沿当前朝向前进，接近目标线性减速 |

**Gazebo 目标球**：点目标时通过 `/world/default/create` 服务在目标位置生成一个半径 0.12 m 的绿球，
到达后变蓝，换目标时旧的删除、新的生成。
球心高度默认 **0.70 m**（`-p marker_z:=` 可调）——必须让球避开深度相机的避障行带，
否则机器人会被自己放的目标球当成障碍卡死（贴地球在 0.45 m 处正好落进行带）。

```bash
# 命令行发目标
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: odom}, pose: {position: {x: 3.0, y: 2.0}, orientation: {w: 1.0}}}'
ros2 topic echo /nav_status        # 观察状态机
```

## 深度可视化（`depth_view.py`）

`bash ~/gz_ros/view_depth.sh` 或 `python3 ~/gz_ros/depth_view.py`，输出 `/depth_view`：

- 深度图**伪彩色**（近处暖色、远处冷色、无穷远黑）；
- **左/中/右分区**（绿色竖线，35%/65%，与导航避障同一套分区）+ 顶部各区行带内最近距离；
- **YOLO 检测框** + 每个目标的距离（框内深度中位数或最小值）；
- 跑 `yolov8*-seg` 分割模型时，把 `Detection.mask` 多边形填成实心剪影。

| 参数 | 默认 | 说明 |
|------|------|------|
| `depth_topic` / `rgb_topic` | `/depth_camera` / `/camera` | 输入话题 |
| `background` | `depth` | `depth` 或 `rgb` |
| `colormap` | `turbo` | `turbo/jet/hot/bone/rainbow/viridis` |
| `min_range` / `max_range` | 0.30 / 10.0 | 伪彩色映射范围（m） |
| `scale` | 2 | 放大倍数 |
| `band` | `0.40,0.85` | 取各区最近距离的行带（改 `0.40,0.60` 可只看物体不含地面） |
| `box_stat` | `median` | `median` / `min` |
| `show_mask` / `mask_alpha` | true / 0.85 | 分割剪影填充 |
| `save_dir` / `save_interval` | 空 / 5.0 | 定时存 PNG（"拍照"） |

## 控制权共存（重要）

三个东西都想控制同一组关节话题：**键盘**、**避障节点**、**导航节点**。

- 键盘控制器**空闲时完全不发布**，并持续发布 `/avoider_override`（1=手动按键中，0=空闲），
  所以它天然能和避障/导航共存；按方向键即手动接管。
- 导航节点订阅 `/avoider_override`，检测到 1 时进入 `MANUAL` 状态、停止发布，松开后自动继续；
  无目标（`IDLE`）时也完全不发指令。
- ⚠️ **避障节点和导航节点不要同时开**：两者都发布 `/camera_joint_controller/commands` 且互不感知，
  会互相顶掉（表现为机器人卡住不动）。要演示避障就单独跑，别开 `run_navigator.sh`。

## 第三方组件与许可

- **yolo_ros**（`yolo_ros2_ws/src/yolo_ros`）：来自 [mgonzs13/yolo_ros](https://github.com/mgonzs13/yolo_ros)，GPL-3.0。
- **Gazebo 模型**（`mod/`）：来自 Gazebo 官方/社区模型库，版权归原模型作者。
- **yolov8m.pt**：Ultralytics 预训练权重（AGPL-3.0），未随包分发，需自行下载并放在 `yolo_ros2_ws/`。
