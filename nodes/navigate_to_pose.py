#!/usr/bin/env python3
"""目标点导航节点: 输入坐标 -> 机器人移动到坐标处。

与本仓库底盘结构严格对应 (robot_with_camera.sdf):
  world --drive_x_joint(prismatic ±10m)--> chassis_x
        --drive_y_joint(prismatic ±10m)--> chassis_y
        --base_yaw_joint(revolute)     --> base

底盘是【位置伺服】不是差速轮: 没有 /cmd_vel, 只有
  /drive_x/commands  std_msgs/Float64   (JointPositionController, p=30 d=10)
  /drive_y/commands  std_msgs/Float64   (同上)
  /camera_joint_controller/commands     (控制 base_yaw_joint, p=15 d=20)
所以本节点的控制量 = "下一个位置指令"。

运动律与 keyboard_joint_controller.py 完全一致 —— 底盘只会「原地转向」和「沿车头前进」:
  1) 目标方向与车头夹角 > heading_gate (默认 34 deg) -> 先原地转向对准, 期间不前进;
     夹角之内边走边转 (弧线), 速度按夹角平方衰减 —— 不再为 10 度偏差整个停一下;
  2) 对准后沿车头方向直行, 每 tick 前进 v*dt (v = min(v_max, sqrt(2*a*dist))),
     边走边用 turn_gain 做小角度修正, 车头始终朝运动方向;
  3) 进入 arrive_eps 停车; 若目标带了朝向, 再原地转到该朝向。
指令用内部累积量 (cmd_x/cmd_y/cmd_yaw), 不按"实测+增量"递推: 位置伺服有滞后,
指令贴着实测就没有误差可追, 转向会被拖成每 tick 零点几度。

定位来源 (机器人已有自定位, 本节点不重复实现):
  /joint_states  三个关节真值 -> x, y, yaw   (与 robot_pose_monitor.py 同源)

输入目标坐标的四种方式 (任选其一):
  1) 命令行      python3 navigate_to_pose.py 2.0 -1.5
                 python3 navigate_to_pose.py 2.0 -1.5 --yaw 90
                 python3 navigate_to_pose.py 1 1 2 2 3 0   多航点依次走
  2) 交互输入    不带坐标启动, 之后每行输入 "x y" 或 "x y 角度(度)", 回车出发
  3) ROS 话题    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
                   "{header: {frame_id: odom}, pose: {position: {x: 2.0, y: -1.5}}}"
                 RViz 里用 "2D Goal Pose" 点一下也走这条路
  4) ROS 话题    /navigate_to  geometry_msgs/Pose2D  (x y 单位 m, theta 单位 rad)

中途取消: 输入 stop / Ctrl+C, 或发 /nav_cancel (std_msgs/Bool 或 Empty)。

发布:
  /navigation_status  std_msgs/String  IDLE/MOVING/ARRIVED/ABORTED + 进度
  /nav_goal_reached   std_msgs/Bool    true = 到点
  /avoider_override   std_msgs/Float64 1 = 导航接管 (让避障节点让路)
"""
import math
import os
import sys
import threading
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Pose2D, PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, Float64, String

# ---- 自动避障 ----
# 深度图 -> 9 个扇区的「还能走多远」-> 车头该指哪儿 / 现在能不能挪。
# 规划器是同目录的 avoid_planner.py (纯几何, 不依赖 ROS, 可离线跑测试)。
# 依赖缺失时本节点照常导航, 只是不避障 —— 不因为一个可选功能把导航拖死。
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from cv_bridge import CvBridge
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from avoid_planner import (AvoidPlanner, Config as AvoidConfig, DODGE, HOLD,
                               NAV, Plan, compute_sectors)
    from grid_map import GridMap
    AVOID_OK = True
    AVOID_ERR = ''
except Exception as _avoid_exc:      # 缺 cv_bridge / numpy / 同目录文件
    AVOID_OK = False
    AVOID_ERR = str(_avoid_exc)

try:
    from llm_hint import HintConfig, LlmHint, Scene, fmt_clear
    LLM_OK = True
    LLM_ERR = ''
except Exception as _llm_exc:        # 缺同目录 llm_hint / 没装 requests
    LLM_OK = False
    LLM_ERR = str(_llm_exc)

try:
    import cv2                       # 只用来把相机帧压成 JPEG 喂 VLM
    import numpy as np
except Exception:
    cv2 = None
    np = None

# ---- YOLO 检测框 (可选) ----
# 把 /yolo/detections 画到喂给 VLM 的那张图上: 模型自己认不出「前景里有几个
# 什么东西」, 但它看得懂图上的方框和字。YOLO 给框, 几何给方位刻度,
# 各司其职。依赖缺失 / YOLO 没起 都只是少几个框, 不影响导航。
try:
    from yolo_msgs.msg import DetectionArray as YoloDetectionArray
except Exception:
    YoloDetectionArray = None

AVOID_DEPTH_TTL = 0.8   # 深度图超过这么久没更新 -> 不信旧图, 按原逻辑走

# ---- 与 SDF 一致 ----
JOINT_X = 'drive_x_joint'
JOINT_Y = 'drive_y_joint'
JOINT_YAW = 'base_yaw_joint'
DRIVE_LIMIT = 10.0         # prismatic 行程 ±10 m (可达区 20x20, 地面 22x22 留 1 m 边)
TOPIC_X = '/drive_x/commands'
TOPIC_Y = '/drive_y/commands'
TOPIC_YAW = '/camera_joint_controller/commands'

# ---- 控制默认值 ----
DEFAULT_V_MAX = 0.5        # 平移最大速度 m/s
DEFAULT_A_MAX = 1.0        # 加减速 m/s^2
DEFAULT_W_MAX = 1.0        # 转向最大角速度 rad/s
DEFAULT_ARRIVE = 0.02      # 到点判定: 位置误差 < 2 cm
DEFAULT_YAW_EPS = 0.02     # 到点判定: 朝向误差 < 1.15 deg
DEFAULT_HZ = 50.0
DEFAULT_TIMEOUT = 180.0    # 单个目标点超时 (s)
HEADING_GATE = 0.15        # 朝向偏差超过该值 (rad) 先原地转向, 不前进
                           # (2026-11-04 试过 0.60 边走边转: mixed (1,8) 从 61 s 变 79 s,
                           #  绕行时带着 20-35 度偏角前进, 车头追不上指令, 反而更碎)
NEAR_GATE = 0.15           # 距目标 < NEAR_GATE_DIST 时收紧到这个门限, 保终点精度
NEAR_GATE_DIST = 0.80
NEAR_FIELD = 0.30          # 进入该距离 (m) 后不再让避障介入: 近场 dx/dy 抖动比障碍还大
NEAR_HEADING = 0.10        # 进入该距离 (m) 后才锁目标方向 (m): 0.30 m 就锁会让车头
                           # 黏在旧方向上、以固定速度冲过目标, 在终点绕圈 (踩过这个坑)
MAX_LEAD = 0.55            # 指令最多超前实测多少 (m)。这个值直接决定最高速度:
                           # 稳态速度 ≈ MAX_LEAD / 伺服滞后时间, 0.25 m 配 0.85 s 滞后
                           # 只有 0.29 m/s, 实测 8 m 走了 28 s 就是这个原因 (v_max 0.5 用不满)
MAX_YAW_LEAD = 0.35        # 角度指令最多超前实测多少 (rad)
TURN_GAIN = 2.0            # 前进过程中朝向修正的 P 增益
STALE_PAUSE = 1.0          # 定位断流超过该时长 -> 暂停推进 (等定位恢复)
STALE_ABORT = 6.0          # 定位断流超过该时长 -> 放弃当前目标
# 抢占检测: 本节点是"位置伺服"式控制 (指令永远跟着实测位置走), 所以没法用
# "指令-实测"偏差判断控制权是否被抢。改用更直接的物理判据: 实际位置在远离目标。
# 正常前进时 dist 只会减小; 若 STEAL_WINDOW 时间内 dist 反而增加了 STEAL_GROW,
# 说明有别的东西 (键盘控制器/避障节点/手动拖拽) 在驱动这台机器人。
STEAL_GROW = 0.25          # m
STEAL_WINDOW = 0.6         # s


def wrap_pi(a):
    """把角度折到 (-pi, pi]。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


class AngleUnwrapper:
    """周期角 -> 连续角 (转多圈不跳变)。与 robot_pose_monitor.py 保持一致。"""

    def __init__(self):
        self.last_raw = None
        self.turns = 0.0
        self.value = 0.0

    def update(self, raw):
        if self.last_raw is None:
            self.value = raw
        else:
            d = raw - self.last_raw
            if d > math.pi:
                self.turns -= 2.0 * math.pi
            elif d < -math.pi:
                self.turns += 2.0 * math.pi
            self.value = self.turns + raw
        self.last_raw = raw
        return self.value

    def nearest(self, target):
        """把绝对目标角换成离当前连续角最近的等价角。"""
        return self.value + wrap_pi(target - self.value)


class Navigator(Node):
    IDLE, MOVING, ARRIVED, ABORTED = 'IDLE', 'MOVING', 'ARRIVED', 'ABORTED'

    def __init__(self):
        super().__init__('navigate_to_pose')

        self.declare_parameter('v_max', DEFAULT_V_MAX)
        self.declare_parameter('a_max', DEFAULT_A_MAX)
        self.declare_parameter('w_max', DEFAULT_W_MAX)
        self.declare_parameter('arrive_eps', DEFAULT_ARRIVE)
        self.declare_parameter('yaw_eps', DEFAULT_YAW_EPS)
        self.declare_parameter('control_hz', DEFAULT_HZ)
        self.declare_parameter('goal_timeout', DEFAULT_TIMEOUT)
        self.declare_parameter('heading_gate', HEADING_GATE)
        self.declare_parameter('turn_gain', TURN_GAIN)
        self.declare_parameter('max_lead', MAX_LEAD)
        self.declare_parameter('face_motion', False)   # 已废弃: 现在总是朝运动方向
        self.declare_parameter('use_orientation', False)
        self.declare_parameter('avoid', True)              # 运动途中自动避障
        self.declare_parameter('avoid_depth_topic', '/depth_camera')
        self.declare_parameter('avoid_scan_topic', '/scan')   # 激光雷达: 给了就优先用它
        self.declare_parameter('llm', True)                 # 避障拿不定主意时问 LLM/VLM
        # 局部栅格 + A* (grid_map.py): 给 VFH 一层「该往哪走」。
        # 默认关 —— 2026-11-04 实测: 开着反而在 mixed 场景必卡 (见 README 记录),
        # 三个常规场景 (person/lowbox/sym) 无差别。想试就 -p grid_on:=true。
        self.declare_parameter('grid_on', False)
        # 响应时间统计 (ТЗ 4.2.2). 0 = 关闭 (默认, 零开销);
        # N > 0 = 采满 N 个连续周期打印一次分位数, 随即自动停止 —— 只测一次, 写报告用。
        self.declare_parameter('latency_report_every', 0)
        # 卡死早停: 位姿连续这么久没有推进 (>0.25 m 以内打转) 就判这次导航失败,
        # 打印 ✗ 并以退出码 2 结束 —— 卡住就是没绕过去, 没必要耗到外层超时。
        # 0 = 关闭 (想让它一直试就一直试)。测试脚本默认 45 s。
        self.declare_parameter('stall_abort', 45.0)
        self.declare_parameter('llm_view_topic', '/camera')  # VLM 用的彩色图
        # YOLO 框叠加: 把检测框画在喂 VLM 的图上。关掉就回到纯相机图。
        # 框的坐标是 /camera 原图像素, 和 llm_view_topic 不同源会错位 —— 两边默认都是 /camera。
        self.declare_parameter('llm_draw_dets', True)
        self.declare_parameter('llm_det_topic', '/yolo/detections')
        self.declare_parameter('llm_det_timeout', 1.0)   # 框超时就不画, 避免贴着旧框说话

        self.v_max = float(self.get_parameter('v_max').value)
        self.a_max = float(self.get_parameter('a_max').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.arrive_eps = float(self.get_parameter('arrive_eps').value)
        self.yaw_eps = float(self.get_parameter('yaw_eps').value)
        self.hz = float(self.get_parameter('control_hz').value)
        self.goal_timeout = float(self.get_parameter('goal_timeout').value)
        self.heading_gate = float(self.get_parameter('heading_gate').value)
        self.turn_gain = float(self.get_parameter('turn_gain').value)
        self.max_lead = float(self.get_parameter('max_lead').value)
        self.face_motion = bool(self.get_parameter('face_motion').value)
        # /goal_pose 里的四元数是否当作目标朝向 (RViz 的 2D Goal Pose 会给朝向)
        self.use_orientation = bool(self.get_parameter('use_orientation').value)
        self.dt = 1.0 / self.hz

        # ---- 实测位姿 (来自 /joint_states 真值) ----
        self.x = 0.0
        self.y = 0.0
        self.yaw = AngleUnwrapper()
        self.got_joints = False
        self._last_joint_t = 0.0
        self._last_cmd_x = 0.0
        self._last_cmd_y = 0.0
        self._last_cmd_yaw = 0.0
        # 内部累积指令 (位置伺服要靠指令领先实测才会动, 见 on_timer)
        self.cmd_x = None
        self.cmd_y = None
        self.cmd_yaw = None
        self.want_heading = None   # 锁定的目标方向 (近场不再重算)

        # ---- 目标与状态 ----
        self.queue = []            # 待走目标点 [(x, y, yaw_or_None), ...]
        self.goal = None           # 当前目标 (x, y, yaw_or_None)
        self.state = self.IDLE
        self.goal_start = 0.0
        self.steal_since = None
        self._stale_warned = False
        self._override_on = False
        self._status_log_t = 0.0
        self._lock = threading.Lock()

        # ---- 接口 ----
        self.pub_x = self.create_publisher(Float64, TOPIC_X, 10)
        self.pub_y = self.create_publisher(Float64, TOPIC_Y, 10)
        self.pub_yaw = self.create_publisher(Float64, TOPIC_YAW, 10)
        self.pub_override = self.create_publisher(Float64, '/avoider_override', 10)
        self.pub_latency = self.create_publisher(Float64, '/nav_latency', 10)
        self.pub_status = self.create_publisher(String, '/navigation_status', 10)
        self.pub_reached = self.create_publisher(Bool, '/nav_goal_reached', 10)

        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal_pose, 10)
        self.create_subscription(Pose2D, '/navigate_to', self.on_goal_pose2d, 10)
        self.create_subscription(Bool, '/nav_cancel', lambda m: self.cancel('外部取消'), 10)

        # ---- 自动避障 ----
        self.avoid_wanted = bool(self.get_parameter('avoid').value)
        self.avoid_depth_topic = str(self.get_parameter('avoid_depth_topic').value)
        self.avoid_on = AVOID_OK and self.avoid_wanted
        acfg = AvoidConfig()
        # LLM_WEIGHT=0 等价于「几何永不判平局」-> 模型意见进不了决策, 用来跑 A/B 基线。
        # 默认 1: 只有两侧绕行偏角差 <= llm_tie_deg 时, 模型才有权改搜索顺序。
        if os.environ.get('LLM_WEIGHT', '1').strip().lower() in ('0', 'false', 'no', 'off'):
            acfg.llm_tie_deg = -1.0
        self.avoider = AvoidPlanner(acfg) if self.avoid_on else None
        # ---- 局部栅格 + A* (grid_map.py) ----
        # 只负责「大致往哪走」: 贴脸/窄缝/净空这些安全判据全在 AvoidPlanner 里,
        # 地图算出方向, VFH 决定这一步到底安不安全 —— 两层各管一件事。
        self.grid_on = bool(self.get_parameter('grid_on').value)
        self.gmap = GridMap() if (self.avoid_on and self.grid_on) else None
        self._gpath = None         # 最近一次 A* 折线 (世界坐标)
        self._gplan_t = 0.0
        self._dpt_t = 0.0        # 上一次把深度弧描进栅格的时刻 (s)        # 上次重新规划的时刻
        self._gaim = None          # 本拍前视点 (世界坐标) / None
        self._glog_t = 0.0
        self.plan = None           # 本周期避障决策 (None = 不介入)
        self.sectors = None        # 9 个扇区的最近距离 (m), inf = 空旷
        self._depth_t = 0.0
        self._depth_warned = False
        self.avoid_scan_topic = str(self.get_parameter('avoid_scan_topic').value)
        self._scan = None          # 最近一帧激光雷达 (ranges, angle_min, angle_increment)
        self._scan_t = 0.0
        # ---- 响应时间度量: 传感器帧 -> 控制指令 (ТЗ 4.2.2) ----
        self._lat = deque(maxlen=5000)   # 端到端延迟样本 (秒)
        self._lat_n = 0
        self._lat_every = int(self.get_parameter('latency_report_every').value)
        self._lat_on = self._lat_every > 0    # 一次性开关: 采满即关
        # ---- 卡死早停 (stall_abort) ----
        self._stall_abort = float(self.get_parameter('stall_abort').value)
        self._stall_hist = deque(maxlen=8000)
        self._avoid_log_t = 0.0
        self._plan_sig = None       # 上一次打印过的避障状态签名, 用来去重 (见日志段)
        self._warn_danger = False   # 是否已经报过「贴脸」警告, 出去就复位
        self.bridge = CvBridge() if self.avoid_on else None
        if self.avoid_on:
            qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
            self.create_subscription(Image, self.avoid_depth_topic, self.on_depth, qos)
            # 激光雷达: 360 度一次看全, 避障的空档判断以它为准; 深度图退为补充
            # (相机只有 60 度视场, 想看清侧面必须转头, 而车头转速只有约 1 rad/s)
            self.create_subscription(LaserScan, self.avoid_scan_topic, self.on_scan, 10)
            self.get_logger().info(
                f'Автоматический объезд включён ({self.avoid_scan_topic} + '
                f'{self.avoid_depth_topic}): '
                f'безопасный просвет {self.avoider.cfg.safe_dist:.2f} м, '
                f'жёсткий стоп {self.avoider.cfg.hard_dist:.2f} м')
        elif AVOID_OK:
            self.get_logger().info('避障关闭 (параметр avoid=false), только навигация')
        else:
            self.get_logger().warn(f'避障 недоступно ({AVOID_ERR}), только навигация')

        # ---- LLM/VLM 顾问: 只在几何自己拿不定主意时才问一句「往哪边让」 ----
        # 分工是「模型定边, 几何定角」: 它回的 LEFT/RIGHT 只改搜索顺序,
        # 安全线(够不够宽、能不能开)永远由深度图算, 模型说错话也撞不了。
        self.llm_wanted = bool(self.get_parameter('llm').value)
        self.llm_view_topic = str(self.get_parameter('llm_view_topic').value)
        self.hint = None
        self._frame = None           # 最近一帧彩色图 (BGR), 真要问 VLM 时才标注+编码
        self._view_t = 0.0
        self._vision_warm = False    # 视觉端预热过没有
        self._pic_combo = False      # 上一张图是不是「前视+雷达」拼图 (prompt 要照着说)
        self._dets = []              # 最近一帧 YOLO 检测: (name, score, cx, cy, bw, bh), 原图像素坐标
        self._det_t = 0.0
        self._det_ttl = float(self.get_parameter('llm_det_timeout').value)
        self._draw_dets_ok = bool(self.get_parameter('llm_draw_dets').value)
        self._view_wh = None         # 最近一帧原图尺寸, 框是原图坐标, 缩放时要用
        self._llm_side = 0           # 上一条已打日志的意见, 免得重复刷屏
        self._llm_word = ''          # 上一条意见的词 (LEFT/RIGHT), 回喂给模型保持连续性
        self._llm_conf = 0.0         # 上一条意见的置信度, 只进日志
        # 强行绕行窗口: 确认卡死之后由本层直接接管航向一小段。几何的 _feasible
        # 只看得见雷达能看见的东西, 矮箱子(low_box.sdf, 高 0.10 m)整个在扫描面
        # 之下, 所以它永远判出一条笔直的「通畅」路线, 车就一边发前进指令一边
        # 顶着箱子不动。这个窗口是视觉唯一真正不可替代的用处。
        self._detour_side = 0        # 往哪边绕 (+1 左 / -1 右), 0 = 没在绕
        self._detour_heading = None  # 锁定的绝对绕行角 (None = 没在绕)
        self._detour_until = 0.0
        self._detour_last = 0        # 上次往哪边绕, 用来「失败就翻边」
        self._detour_fail = 0        # 同一趟行程里连续卡住几次
        # 深度贴脸快速脱困: 见 on_timer 里同一行注释。_fg_since 是深度报警
        # 开始计时的时刻, _fg_quick 表示本次脱困由它触发(不再等模型 6 s)。
        self._fg_since = None
        self._fg_x0 = 0.0
        self._fg_y0 = 0.0
        self._fg_yaw0 = 0.0
        self._fg_quick = False
        self._last_stall_t = 0.0
        self._llm_ask_log_t = 0.0
        self._pos_hist = []          # (时间, x, y): 判断「说在走却挪不动」
        if self.avoid_on and self.llm_wanted and LLM_OK:
            hc = HintConfig.from_env()
            if hc.on:
                def _hint_log(m, lvl='warn'):
                    # 失败走 WARN, 成功走 INFO —— 否则「模型答了但弃权」和
                    # 「模型没答」在日志里长得一模一样, 排查全靠猜。
                    if lvl == 'warn':
                        self.get_logger().warn(m)
                    else:
                        self.get_logger().info(m)

                self.hint = LlmHint(hc, log=_hint_log)
                vision = bool(hc.vision and hc.vlm_model)
                self.get_logger().info(
                    f'LLM-советник включён: {hc.provider}, {hc.local_model}'
                    + (f', зрение {hc.vlm_model}' if vision else ', без картинки')
                    + f', спрашиваю не чаще {hc.min_interval:.1f} с')
                self.get_logger().info(
                    f'LLM настройки: вход {"12 секторов" if hc.ring else "L/C/R"}, '
                    f'выход {"JSON-схема" if hc.json_schema else "свободный текст"}, '
                    f'решение {"перевес при равенстве" if acfg.llm_tie_deg > 0 else "только лог"}')
                self.hint.warmup()          # 先把文本端拉热, 免得起手就要意见时等冷启动
                self._vision_warm = False
                if vision and cv2 is not None:
                    qos_v = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT,
                                       history=HistoryPolicy.KEEP_LAST)
                    self.create_subscription(Image, self.llm_view_topic, self.on_view, qos_v)
                    # YOLO 框: 和相机图同时喂给 VLM。没装 yolo_msgs 就静默跳过
                    det_topic = str(self.get_parameter('llm_det_topic').value)
                    if YoloDetectionArray is not None and self._draw_dets_ok:
                        self.create_subscription(YoloDetectionArray, det_topic,
                                                 self.on_detections, 10)
                        self.get_logger().info(
                            f'VLM 图上叠加 YOLO 框: 订阅 {det_topic}')
                    elif not self._draw_dets_ok:
                        self.get_logger().info('VLM 图不叠加检测框 (llm_draw_dets=false)')
                    else:
                        self.get_logger().info('yolo_msgs нет — VLM 图不带检测框')
                elif vision:
                    self.get_logger().warn('VLM задан, но cv2 нет — работаю без картинки')
            else:
                self.get_logger().info('LLM-советник выключен (LLM_ON=0)')
        elif self.avoid_on and self.llm_wanted:
            self.get_logger().warn(f'LLM-советник недоступен ({LLM_ERR}), только геометрия')

        self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            f'Навигация запущена: v_max={self.v_max} м/с, допуск={self.arrive_eps} м, '
            f'частота={self.hz:.0f} Гц')
        self.get_logger().info(
            'Куда ехать: аргументы "x y" / ввод в терминале "x y [угол]" / /goal_pose или /navigate_to')

    # ================= 输入 =================
    def on_depth(self, msg):
        """深度图 -> 扇区距离。相机与车头同轴, 所以「转头」本身就是在测侧方。"""
        if self.avoider is None:
            return
        try:
            arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self.sectors = compute_sectors(arr, n_sectors=self.avoider.cfg.n_sectors)
            self._depth_t = time.time()
        except Exception as exc:
            if not self._depth_warned:
                self._depth_warned = True
                self.get_logger().warn(f'深度图解析失败: {exc}')
                self.sectors = None

    def on_scan(self, msg):
        """激光雷达 -> 一帧全向距离。避障靠它, 不再依赖深度图的 60 度视场。"""
        if self.avoider is None:
            return
        self._scan = (msg.ranges, msg.angle_min, msg.angle_increment)
        self._scan_t = time.time()
        if self.gmap is not None:
            # 雷达点直接进栅格: 450 束/帧 10 Hz, 是唯一能把「形状」记下来的地方
            # (VFH 的 36 个扇区只记「这个方位现在多空」, 记不住 0.30 m 的死缝)。
            self.gmap.apply_scan(msg.ranges, msg.angle_min, msg.angle_increment,
                                 self.x, self.y, self.yaw.value, self._scan_t)

    def _grid_aim(self, now, gx, gy, have_scan):
        """局部栅格 + A*: 回本拍的前视点 (世界坐标); 没把握就 None。

        A* 每 0.4 s 重算一次而不是每周期: 栅格本身 10 Hz 在长, 0.4 s 里路径偏差
        到不了影响安全的程度, 省下的是每个周期 8000 次 A* 展开。
        求不出前视点 (栅格空 / 算不出路 / 路径太短) 就返回 None,
        调用方原样退回纯 VFH 行为 —— 这一层只会「多给一个方向」, 不会新增死法。
        """
        gm = self.gmap
        if gm is None:
            return None
        # 深度兜底报「正前方 X 米有东西」-> 在那一点补一个粘滞点。
        # 0.10 m 高的矮箱整个在雷达扫描面 (z=0.235 m) 之下, 雷达对它一无所见,
        # 深度相机是它唯一的证人。补一次就够: sticky 内部对同一格不重复续命,
        # 否则车一直贴着同一个箱子时那格永不销毁, 绕过去之后还在堵。
        # 深度补点: 沿「深度看到它的那条弧」逐扇区投, 而不是投在车头正前方。
        # 按车头投的横向误差 = d * sin(偏角): 箱子在视野边缘 25 度、1.4 m 处就偏
        # 0.59 m, 而膨胀半径只有 0.25 m —— A* 眼里那仍是空地, 于是照样直冲。
        # 投点按 0.5 s 一次, 每帧都投会在 8 s 的 sticky 存活期里拉出一条随车移动的
        # 点轨迹, 把走过的路全糊死, 车自己就退不回来了。
        if self.avoider is not None and (now - self._dpt_t) >= 0.50:
            _arc = [(r, d) for r, d in getattr(self.avoider, 'depth_arc', ())
                    if 0.10 < d < 1.90]
            _dmin = min((d for _, d in _arc), default=None)
            for _rel, _d in _arc:
                if _dmin is None or _d > _dmin + 0.60:
                    continue          # 只描矮障碍那一块; 远处的真墙雷达已经管了
                self._dpt_t = now
                _th = self.yaw.value + _rel
                gm.sticky(self.x + _d * math.cos(_th),
                          self.y + _d * math.sin(_th), now)
        if now - self._gplan_t >= 0.4:
            self._gplan_t = now
            gm.rebuild(now)
            self._gpath = gm.plan(self.x, self.y, gx, gy)
            if now - self._glog_t >= 2.0:
                self._glog_t = now
                self.get_logger().info(
                    '⛶ карта: точек %d, раздуто %d, путь %s'
                    % (gm.n_occ, len(gm.blocked),
                       ('%.1f м' % gm.path_len) if self._gpath else 'нет'))
        if not self._gpath:
            return None
        aim = gm.lookahead(self._gpath, self.x, self.y)
        self._gaim = aim
        return aim

    def _maybe_ask_llm(self, now, plan, force=''):
        """三个触发点, 都从「几何自己在犹豫」出发, 不是定时轮询。

        flap  2 秒内绕行换边 >= 2 次 —— 左右横跳, 几何选不出偏好;
        tie   左右净空都 >= 0.70 m 且相差 < 0.30 m —— 两边都能过, 纯平局;
        stall 说在前进, 但 8 秒内车身没挪动 0.30 m 且已贴近障碍 —— 真卡住了。
        三种触发都带图: 带图 + JSON 约束的热态延迟只要 0.85-0.91 s,
        而 tie/flap 恰恰是「两边数值都够宽、几何说不出走哪边」的真问题。
        """
        if self.hint is None or plan is None:
            return
        # 原来这里还有一条 or plan.state == NAV 就 return —— 意思是「几何一切
        # 正常就别打扰模型」。可矮障碍恰恰死在这个前提上: 雷达看不见箱子,
        # 几何认定前方通畅 -> state=NAV -> 函数直接返回 -> 连 stall 都不判,
        # 车身顶在箱子上 15 s 也问不出一句话。改成只把 NAV 当作「flap/tie
        # 没意义」的开关, stall 判据照常跑。
        navigating = (plan.state == NAV)
        self._pos_hist.append((now, self.x, self.y))
        # 窗口 13 s: 二级 stall 判据要 12 s 的观察期, 窗口若只有 9.5 s
        # 就永远攒不出那么长的历史, 判据形同虚设。
        while len(self._pos_hist) > 2 and now - self._pos_hist[0][0] > 13.0:
            self._pos_hist.pop(0)

        left = self.avoider.clearance(60.0, now)
        right = self.avoider.clearance(-60.0, now)
        ahead = self.avoider.clearance(0.0, now, half_span=0.0)

        trig = '' if navigating else ('flap' if self.avoider.flap(now) else '')
        if force == 'stall':
            # 外部已经判定卡死(深度贴脸 + 1.5 s 没挪窝), 不再走 12 s 观察期。
            # 但记账必须在这里做 —— 下面的 stall 分支靠 not trig 短路, 进不来。
            if not trig:
                trig = 'stall'
                self._fg_quick = True
                if now - self._last_stall_t > 20.0:
                    self._detour_fail = 0
                self._detour_fail += 1
                self._last_stall_t = now
                self._detour_until = now + 16.0
                self._detour_heading = None
        # inf = 这一侧完全空旷, 恰恰是"两边都能过"的典型平局。原来要求
        # math.isfinite(left) and math.isfinite(right), 于是对称墙、空旷场
        # 两侧都是 inf 时 tie 永不成立 —— LLM 加权从来没被触发过。把 inf
        # 折成 scan_max 再比, None(没有测量) 仍然排除。
        l2 = 9.0 if (left is not None and not math.isfinite(left)) else left
        r2 = 9.0 if (right is not None and not math.isfinite(right)) else right
        # 两侧都是 inf(折成 9.0) 的意思是「四周开阔, 往哪边走都行」—— 这不是
        # 需要模型裁断的平局, 而是根本没得选。走廊场景实测: 起跑第 2 秒 L/C/R
        # 全是 inf, 被判成 tie 问了 9B, 模型对着一片空地的图只能瞎回 LEFT(5.0 s),
        # 白烧一次 GPU 还把车头往左拉了 50 度。真平局得两侧都有实打实的边界。
        open_field = (l2 is not None and r2 is not None
                      and l2 >= 8.9 and r2 >= 8.9)
        # 阈值从 (>=0.70, 差<0.30) 放宽到 (>=0.55, 差<0.45): 0.70 是按「两个方向都
        # 够宽才算平局」定的, 可车宽 0.40 m, 0.55 m 的净空照样过得去 —— 原来的门槛
        # 让绝大多数真实平局根本不进触发器 (一趟 16 次询问里 tie 只占个位数)。
        if (not trig and not navigating and not open_field
                and l2 is not None and r2 is not None
                and min(l2, r2) >= 0.55 and abs(l2 - r2) < 0.45):
            trig = 'tie'
        if not trig and plan.advance and self._pos_hist:
            # 真卡住才算: 8 秒挪不动 0.30 m。原先写「5 秒 0.60 m」太松 ——
            # 正常绕行时原地转向两秒也算 stall, 一趟 68 秒的行程问了 6 次,
            # 每次都白拉一次 9B, 日志被淹没, 也拖慢 GPU。
            t0, x0, y0 = self._pos_hist[0]
            near = any(v is not None and math.isfinite(v) and v < 1.20
                       for v in (left, ahead, right))
            moved = math.hypot(self.x - x0, self.y - y0)
            # 两级判据。一级(原有): 贴近障碍却挪不动 —— 经典卡死。
            # 二级(新增): 雷达四周一片空旷(near=False), 车身却也挪不动 —— 这是
            # 矮障碍的指纹。low_box.sdf 的箱子高 0.10 m, 而雷达装在 z≈0.235,
            # 射线从箱子顶上飞过去, 几何据此认定「前方通畅」, 一直发前进指令,
            # 车身却顶在箱子上纹丝不动(lb1: 车头前缘 2.35 正顶箱子前缘 2.3,
            # 卡了 15 s 一句 спрашиваю 都没有)。几何与物理自相矛盾的那一刻,
            # 恰恰是最该抬眼看图的一刻 —— 雷达看不见的东西, 相机看得见。
            # 二级门槛更严(12 s / 0.15 m), 免得正常慢速巡航被误判。
            # 已经在绕行中就别再判一次卡死: 绕行头两秒位移本来就小, 判据必然
            # 再次成立, 于是每来一次就把锁定角清掉、重新翻一次边 —— lb8 实测
            # 车在原地 +55/-55 每秒交替抽筋, 一步没挪。绕行窗口过期后
            # _detour_heading 会被清成 None, 那时才重新判。
            if self._detour_heading is None and (
                    (now - t0 >= 8.0 and near and moved < 0.30) or
                    (now - t0 >= 12.0 and not near and moved < 0.15)):
                trig = 'stall'
                # 同时打开强制绕行窗口。几何下一拍仍会判 NAV 直冲(它看不见那
                # 个东西), 光问一句不问出个动作, 车会一直顶着等下一句回答。
                # 窗口 6 s: 0.7 s 原地转 40 度 + 约 5 s 直行 ≈ 2.5 m, 横向
                # 挪开约 1.6 m。矮箱子只有 0.8 m 宽, 够绕过它; 再长就白跑远路
                # (雷达始终判「通畅」, 窗口不会自己收, 只能靠计时)。
                # 窗口 16 s: 从「确认卡死」到「模型答完」还要 3-10 s(冷态更久),
                # 窗口得把这段等待也算进去, 否则模型回来时窗口早就关了。
                # 同一趟行程里又卡住了 => 上次那个方向没绕出去, 下一轮翻边。
                # 20 秒没再卡就说明上一次是真的绕过去了, 计数归零。
                if now - self._last_stall_t > 20.0:
                    self._detour_fail = 0
                self._detour_fail += 1
                self._last_stall_t = now
                self._detour_until = now + 16.0
                self._detour_heading = None

        # 以前只有 stall 才带图, 理由是「其余两种是数值问题, 给图只会拖慢」。
        # 实测推翻了这个顾虑: 带图 + JSON 约束的热态延迟 0.85-0.91 s, 而 stall
        # 恰恰是最不需要看图的一种 (车已经卡死, 图里也看不出路); tie/flap 才是
        # 「两边都空, 到底走哪边」这个真问题, 那正是图能回答的。
        picture = self._picture(plan, now, left, ahead, right) if trig else None
        scene = Scene(trigger=trig,
                      goal_bearing=math.degrees(wrap_pi(plan.goal_dir - self.yaw.value)),
                      left=left, ahead=ahead, right=right,
                      ring=self.avoider.ring(now) if self.hint.cfg.ring else (),
                      prev_answer=self._llm_word, image=picture,
                      combo=self._pic_combo)
        why = self.hint.consider(now, scene)
        if why and now - self._llm_ask_log_t > 1.0:
            self._llm_ask_log_t = now
            self.get_logger().info(
                f'llm спрашиваю ({why}): цель {scene.goal_bearing:+.0f} deg, '
                f'L={fmt_clear(left)} C={fmt_clear(ahead)} R={fmt_clear(right)}'
                + (f', колец {len(scene.ring)}' if scene.ring else '')
                + (', с картинкой' if picture else ''))

    def _encode(self, img):
        ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        return buf.tobytes() if ok else None

    def _annotate(self, frame, goal_bearing=0.0, left=None, ahead=None, right=None):
        """把几何自己算出来的东西画到图上, 再交给 VLM。

        依据两条实测结论:
        1) Prompt-in-Image (arXiv 2508.01678): 文字直接画进图里, Qwen2.5-VL 的
           POPE 80.2% -> 84.3%, 幻觉率下降 (LLaVA 系反而崩, 我们用的正是 Qwen 系);
        2) Grid-Augmented Vision (arXiv 2411.18270): 图上叠刻度能显著改善空间定位。
        模型「目测距离」一贯不准 —— 实测 4 组截然不同的图一律回 LEFT —— 但它照着
        刻度说方向是准的。所以这里只画**方向**, 距离用文字写死在底部, 不让它猜。
        相机 hfov = 60 deg, 所以横向 1 像素 = 60/w 度。
        """
        img = frame.copy()
        h, w = img.shape[:2]
        cx = w // 2
        dpx = 60.0 / float(w)
        for d in (-30, -20, -10, 10, 20, 30):
            x = int(round(cx + d / dpx))
            cv2.line(img, (x, 0), (x, h), (90, 90, 90), 1)
            cv2.putText(img, '%+d' % d, (max(0, x - 13), 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.line(img, (cx, 0), (cx, h), (0, 255, 255), 1)          # 车头正前方
        cv2.line(img, (0, h // 2), (w, h // 2), (0, 255, 255), 1)
        cv2.putText(img, 'LEFT', (6, h // 2 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, 'RIGHT', (w - 74, h // 2 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, 'AHEAD 0', (cx - 34, h - 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
        if abs(goal_bearing) <= 30.0:
            gx = int(round(cx + goal_bearing / dpx))
            cv2.arrowedLine(img, (cx, h - 22), (gx, h - 22), (0, 220, 0), 2, tipLength=0.3)
            cv2.putText(img, 'GOAL %+.0f' % goal_bearing, (4, h - 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 220, 0), 1, cv2.LINE_AA)
        # 距离不给它看, 直接写死: 目测距离是它最不可靠的一项
        cv2.rectangle(img, (0, 0), (w, 0), (0, 0, 0), -1)
        bar = 'L %s   C %s   R %s   m' % (fmt_clear(left), fmt_clear(ahead),
                                          fmt_clear(right))
        cv2.rectangle(img, (0, h - 16), (w, h), (0, 0, 0), -1)
        cv2.putText(img, bar, (6, h - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (255, 255, 255), 1, cv2.LINE_AA)
        self._draw_dets(img)          # YOLO 框画在最上层, 不被底部黑条盖住
        return img

    def _radar_panel(self, now):
        """把 360 度雷达画成俯视扇区图, 专门回答「左右哪边更空」。

        为什么非要有它: 前视相机只有 60 度, 而「左右哪边开阔」这个信息**不在
        画面里**。实测 9B 对 4 组截然不同的前视图一律回 LEFT —— 不是模型笨,
        是让它回答一个输入里没有的量的题。补上这张图之后, 4 个场景判对 3 个
        (左空 / 右空 / 两边都空全对; 只有「两边都堵」它仍选 LEFT, 而那种情况
        几何层本来就 HOLD, 答错也无害)。

        画法: 车在圆心, 图上方 = 车头, 图左 = 车的左侧 (与 clearance 的正角度
        一致), 每 10 度一个扇区, 扇形半径 = 该方向净空, 颜色按净空分档。
        """
        if self.avoider is None or cv2 is None or np is None:
            return None
        W, H = 336, 252
        img = np.full((H, W, 3), 24, np.uint8)
        cx, cy, rp, rmax = W // 2, H // 2, 106, 4.0
        for i in range(36):
            deg = -180 + i * 10
            r = self.avoider.clearance(float(deg), now, half_span=5.0)
            if r is None:
                rr, col = 0.25, (58, 58, 58)      # 没测过: 灰, 别假装空旷
            else:
                rr = rmax if not math.isfinite(r) else max(0.25, min(r, rmax))
                col = ((40, 40, 230) if rr < 0.80 else
                       (40, 150, 250) if rr < 1.40 else
                       (40, 215, 240) if rr < 2.50 else (110, 215, 90))
            a0 = math.radians(deg - 5.0)
            a1 = math.radians(deg + 5.0)
            px = rr / rmax * rp
            # 正角度 = 左侧, 所以左侧画在屏幕左边 (x 减小), 车头朝上 (y 减小)
            x0, y0 = cx - px * math.sin(a0), cy - px * math.cos(a0)
            x1, y1 = cx - px * math.sin(a1), cy - px * math.cos(a1)
            poly = np.array([[cx, cy], [x0, y0], [x1, y1]], np.int32)
            cv2.fillPoly(img, [poly], col)
            # 相邻扇区共用一条边, 光栅化时这条边两边的像素会各漏一半, 画出
            # 一圈 1 px 的暗缝。prompt 里说了「灰 = 没测过」, 模型会把缝读成
            # 「正前方没数据」。补一条同色线盖掉。
            cv2.line(img, (cx, cy), (int(x0), int(y0)), col, 1)
        for m in (1, 2, 3, 4):
            rad = int(m / rmax * rp)
            cv2.circle(img, (cx, cy), rad, (72, 72, 72), 1)
            if m < 4:            # 4 m 就是外沿, 标上去会压住车头线
                cv2.putText(img, '%dm' % m, (cx + 3, cy - rad - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.32, (150, 150, 150), 1)
        cv2.line(img, (cx, 4), (cx, 22), (255, 255, 255), 2)   # 车头方向
        cv2.putText(img, 'TOP DOWN  UP=AHEAD', (6, 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, (235, 235, 235), 1)
        cv2.putText(img, 'LEFT', (6, H - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (235, 235, 235), 1, cv2.LINE_AA)
        cv2.putText(img, 'RIGHT', (W - 62, H - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (235, 235, 235), 1, cv2.LINE_AA)
        cv2.putText(img, 'm', (cx + 2, cy + rp + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, (150, 150, 150), 1)
        return img

    def _picture(self, plan, now, left, ahead, right):
        """现在要不要带图。带就现场标注一帧 —— 只在真要问的时候做, 频率很低。

        combo 模式: 左边前视标注图 (336x252), 右边雷达俯视扇区图 (336x252),
        中间一条分隔线, 整张 672x252 —— 实测 199 token / 2.12 s, 比单喂一张
        640x480 原图 (300 token / 3.05 s) 还便宜。
        """
        self._pic_combo = False
        if not self.hint.cfg.vision or self._frame is None:
            return None
        if now - self._view_t > 1.5:      # 图太旧就别用, 车已经挪过去了
            return None
        try:
            gb = math.degrees(wrap_pi(plan.goal_dir - self.yaw.value)) if plan else 0.0
            front = self._annotate(self._frame, gb, left, ahead, right)
            if (self.hint.cfg.vision_mode != 'combo'
                    or cv2 is None or np is None):
                return self._encode(front)
            radar = self._radar_panel(now)
            if radar is None:             # 雷达没数据: 退回单张前视图, 别给半张图
                return self._encode(front)
            f2 = cv2.resize(front, (radar.shape[1], radar.shape[0]))
            cv2.putText(f2, 'FRONT VIEW', (6, 14), cv2.FONT_HERSHEY_SIMPLEX,
                        0.36, (235, 235, 235), 1)
            seam = np.full((radar.shape[0], 4, 3), 96, np.uint8)
            self._pic_combo = True
            return self._encode(np.hstack([f2, seam, radar]))
        except Exception:
            return None

    def on_detections(self, msg):
        """缓存最新一帧 YOLO 检测 (原图像素坐标)。"""
        try:
            dets = []
            for d in msg.detections:
                b = d.bbox
                dets.append((str(d.class_name), float(d.score),
                             float(b.center.position.x), float(b.center.position.y),
                             float(b.size.x), float(b.size.y)))
            self._dets = dets
            self._det_t = time.time()
        except Exception:
            pass                      # 检测只是锦上添花

    def _draw_dets(self, img):
        """把 YOLO 框画到已缩放的 VLM 图上 (框是原图坐标, 按宽度比例缩)。

        为什么要画在图上而不是写进 prompt: 同一张图里的位置关系是模型最擅长
        读的东西, 而「框 3 在左下角」这种文字描述它大概率对不上号。类别名 + 置信度
        直接印在框上, 它能一眼看出「那个框里是 person」。
        """
        if not self._dets or not self._draw_dets_ok:
            return
        if time.time() - self._det_t > self._det_ttl:
            return                    # 旧框比无框更坏: 会把模型指到错的方向
        wh = self._view_wh
        if not wh or not wh[0]:
            return
        s = img.shape[1] / float(wh[0])
        hh, ww = img.shape[:2]
        for name, score, cx, cy, bw, bh in self._dets:
            x1 = int(round((cx - bw / 2.0) * s))
            y1 = int(round((cy - bh / 2.0) * s))
            x2 = int(round((cx + bw / 2.0) * s))
            y2 = int(round((cy + bh / 2.0) * s))
            x1 = max(0, min(ww - 1, x1))
            x2 = max(0, min(ww - 1, x2))
            y1 = max(0, min(hh - 1, y1))
            y2 = max(0, min(hh - 1, y2))
            if x2 - x1 < 4 or y2 - y1 < 4:
                continue
            col = (60, 60, 255) if name == 'person' else (60, 255, 60)
            cv2.rectangle(img, (x1, y1), (x2, y2), col, 1)
            tag = '%s %.2f' % (name, score)
            ty = y1 - 4 if y1 > 13 else y2 + 12
            cv2.putText(img, tag, (x1, ty), cv2.FONT_HERSHEY_SIMPLEX,
                        0.36, col, 1, cv2.LINE_AA)

    def on_view(self, msg):
        """彩色相机 -> 内存里最新一帧。编码推迟到真要问 VLM 的时候:
        标注要有目标方位和净空, 那是主循环才知道的东西, 这里拿不到。"""
        try:
            arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = arr.shape[:2]
            self._view_wh = (w, h)   # 原图尺寸: YOLO 框是原图坐标
            if w > 448:              # 实测 320/448/640 的延迟几乎一样 (0.85-0.91 s),
                s = 448.0 / w        # 瓶颈是输出 token 数不是图像; 448 够看清又省带宽
                arr = cv2.resize(arr, (int(w * s), int(h * s)))
            self._frame = arr
            self._view_t = time.time()
            if not self._vision_warm and self.hint is not None:
                # 拿第一张真图把视觉编码器拉热: 冷启 2.7-3.5 s, 热后 0.85-0.91 s
                self._vision_warm = True
                jpg = self._encode(arr)
                if jpg:
                    self.hint.warmup(jpg)
        except Exception:
            pass                     # 图像只是锦上添花, 坏了不影响导航

    def on_joint_state(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if JOINT_X in pos:
            self.x = float(pos[JOINT_X])
        if JOINT_Y in pos:
            self.y = float(pos[JOINT_Y])
        if JOINT_YAW in pos:
            self.yaw.update(float(pos[JOINT_YAW]))
        if not self.got_joints and (JOINT_X in pos and JOINT_Y in pos and JOINT_YAW in pos):
            self.got_joints = True
            self._last_cmd_x = self.x
            self._last_cmd_y = self.y
            self._last_cmd_yaw = self.yaw.value
            self.get_logger().info(
                f'Позиция получена (/joint_states): x={self.x:+.2f} y={self.y:+.2f} '
                f'yaw={math.degrees(self.yaw.value):+.1f} deg')
        self._last_joint_t = time.time()

    def _checked(self, x, y):
        """目标合法性检查: prismatic 行程 ±10 m。"""
        if abs(x) > DRIVE_LIMIT or abs(y) > DRIVE_LIMIT:
            self.get_logger().error(
                f'目标 ({x:+.2f}, {y:+.2f}) 超出底盘行程 +/-{DRIVE_LIMIT} m, 拒绝')
            return None
        return (float(x), float(y))

    def add_goal(self, x, y, yaw=None, append=False):
        pt = self._checked(x, y)
        if pt is None:
            return
        with self._lock:
            if not append:
                self.queue.clear()
            self.queue.append((pt[0], pt[1], yaw))
        if append and self.state == self.MOVING:
            self.get_logger().info(f'Добавил точку ({x:+.2f}, {y:+.2f}), в очереди {len(self.queue)}')
            return
        self._start_next()

    def add_goals(self, pts):
        """pts: [(x, y, yaw_or_None), ...] 依次导航。"""
        with self._lock:
            self.queue.clear()
            for x, y, yaw in pts:
                pt = self._checked(x, y)
                if pt is not None:
                    self.queue.append((pt[0], pt[1], yaw))
        self._start_next()

    def on_goal_pose(self, msg):
        p = msg.pose
        # 四元数 -> yaw
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # 默认四元数 (0,0,0,±1) = 没指定朝向; 想让它生效就 -p use_orientation:=true
        default_q = abs(q.x) + abs(q.y) + abs(q.z) < 1e-9 and abs(abs(q.w) - 1.0) < 1e-9
        has_yaw = self.use_orientation or not default_q
        self.get_logger().info(
            f'收到 /goal_pose: ({p.position.x:+.2f}, {p.position.y:+.2f})'
            + (f' yaw={math.degrees(yaw):+.1f} deg' if has_yaw else ''))
        self.add_goal(p.position.x, p.position.y, yaw if has_yaw else None)

    def on_goal_pose2d(self, msg):
        yaw = float(msg.theta) if abs(msg.theta) > 1e-9 else None
        self.get_logger().info(f'Получен /navigate_to: ({msg.x:+.2f}, {msg.y:+.2f})'
                               + (f' yaw={math.degrees(yaw):+.1f} deg' if yaw else ''))
        self.add_goal(msg.x, msg.y, yaw)

    # ================= 状态机 =================
    def _start_next(self):
        with self._lock:
            if not self.queue:
                if self.state == self.MOVING:
                    self.get_logger().info('Очередь целей пуста')
                self.goal = None
                self.state = self.IDLE
                return
            self.goal = self.queue.pop(0)
        if not self.got_joints:
            self.get_logger().warn('Нет /joint_states (позиции), жду данные...')
        self.cmd_x = self.cmd_y = None      # 新目标: 指令基准重新对齐实测
        self.want_heading = None
        if self.avoider is not None:
            self.avoider.reset()            # 新目标: 换个起点, 旧方位记忆全作废
        self.state = self.MOVING
        self.goal_start = time.time()
        self.steal_since = None
        self._stall_hist.clear()            # 新目标: 卡死计时重新开始
        gx, gy, gyaw = self.goal
        self.get_logger().info(
            f'Старт -> ({gx:+.2f}, {gy:+.2f})'
            + (f' yaw={math.degrees(gyaw):+.1f} deg' if gyaw is not None else '')
            + f', осталось целей: {len(self.queue)}')

    def cancel(self, why='отмена пользователем'):
        with self._lock:
            self.queue.clear()
            self.goal = None
        if self.state == self.MOVING:
            self.get_logger().warn(f'Отмена: {why} (стоим на x={self.x:+.2f} y={self.y:+.2f})')
        self.state = self.IDLE
        self.plan = None
        if self.avoider is not None:
            self.avoider.reset()
        if self._override_on:
            self._override_on = False
            self.pub_override.publish(Float64(data=0.0))
        self.pub_reached.publish(Bool(data=False))
        self._publish_status('отменено')
        if self._lat_on and self._lat_n:      # 行程提前结束: 有多少算多少, 只报一次
            self._lat_on = False
            self._log_latency('итог')

    # ================= 主循环 =================
    def on_timer(self):
        if not self.got_joints:
            return
        # 用**真实**周期, 不是 1/hz。on_timer 里挂着 LLM、栅格、日志, 实际频率常年
        # 低于 50 Hz, 而位移是按 dt 累积的 —— 频率掉多少, 速度就按比例掉多少。
        # 实测 0.30 m/s 上不去 (设定 0.5) 就是这个原因: 指令每次只挪 v*(1/50),
        # 每秒却只跑 30-35 次。改成真实 dt 后同一个 v 才是真的 m/s。
        _t_now = time.monotonic()
        dt = _t_now - getattr(self, '_tick_t', _t_now - self.dt)
        if dt <= 1e-4 or dt > 0.20:
            dt = self.dt
        self._tick_t = _t_now
        self._tick_n = getattr(self, '_tick_n', 0) + 1
        # 定位断流保护: 先暂停推进等恢复 (机器人自然停住), 久不恢复才放弃目标
        silent = time.time() - self._last_joint_t
        if silent > STALE_ABORT:
            if self.state == self.MOVING:
                self.get_logger().error(
                    f'Нет данных {silent:.1f}с (/joint_states молчит), бросаю цель')
                self.cancel('нет данных о позиции')
            return
        if silent > STALE_PAUSE:
            if self.state == self.MOVING:
                if not self._stale_warned:
                    self._stale_warned = True
                    self.get_logger().warn(f'Позиция пропала {silent:.1f}с, пауза...')
                # 指令回收到最后已知位姿, 让位置伺服就地刹住, 不照着旧指令继续冲
                if self._override_on:
                    self.pub_x.publish(Float64(data=self.x))
                    self.pub_y.publish(Float64(data=self.y))
                    self.pub_yaw.publish(Float64(data=self.yaw.value))
                self.cmd_x = self.cmd_y = None      # 恢复后重新对齐实测
            return
        if self._stale_warned:
            self._stale_warned = False
            self.get_logger().info('Позиция восстановлена, продолжаю')

        if self.state != self.MOVING or self.goal is None:
            # 空闲时不发 /avoider_override, 免得盖掉键盘控制器发出的接管信号
            if self._override_on:
                self._override_on = False
                self.pub_override.publish(Float64(data=0.0))
            return

        # ---- 超时保护 ----
        if time.time() - self.goal_start > self.goal_timeout:
            self.get_logger().error(f'Таймаут ({self.goal_timeout:.0f}с), стоп')
            self.cancel('таймаут')
            return

        gx, gy, gyaw = self.goal
        dx, dy = gx - self.x, gy - self.y
        dist = math.hypot(dx, dy)

        # ---- 抢占检测: 半秒内反而离目标更远 > 25 cm -> 有别人在驱动底盘 ----
        now = time.time()
        if self.steal_since is None:
            self.steal_since = (now, dist)
        else:
            ref_t, ref_dist = self.steal_since
            if now - ref_t >= STEAL_WINDOW:
                if dist > ref_dist + STEAL_GROW:
                    self.get_logger().warn(
                        f'Перехват управления (до цели было {ref_dist:.2f} м, стало {dist:.2f} м, '
                        f'ушли назад на {dist - ref_dist:.2f} м), навигация прервана')
                    self.cancel('перехват управления (клавиатура/объезд)')
                    return
                self.steal_since = (now, dist)

        # ---- 运动律: 只会「原地转向」和「沿车头前进」, 不横移 (与键盘控制同一思路) ----
        # 指令必须是内部累积量: 位置伺服有滞后, 若按「实测 + 增量」递推, 指令永远贴着实测,
        # 伺服就没有误差可追 -> 转一圈会被拖成每 tick 零点几度。累积后指令领先, 伺服才真正动。
        if self.cmd_x is None:
            self.cmd_x, self.cmd_y, self.cmd_yaw = self.x, self.y, self.yaw.value
        if abs(wrap_pi(self.cmd_yaw - self.yaw.value)) > 0.6 or \
           abs(self.cmd_x - self.x) > 1.0 or abs(self.cmd_y - self.y) > 1.0:
            # 被键盘/外部拖走, 或指令跑飞: 以实测为基准重新对齐
            self.cmd_x, self.cmd_y, self.cmd_yaw = self.x, self.y, self.yaw.value

        dist_cmd = math.hypot(gx - self.cmd_x, gy - self.cmd_y)
        arrived_xy = dist <= self.arrive_eps and dist_cmd <= self.arrive_eps

        # ---- 自动避障: 由本节点自己决定「要不要让」, 不是外部开关 ----
        # 只在运动途中、且离目标还有一段距离时介入: 0.30 m 内 dx/dy 的抖动比
        # 障碍本身还大, 近场一律交给原来的靠拢逻辑, 免得在终点门口乱摆头。
        plan = None
        if self.avoider is not None and dist > NEAR_FIELD:
            have_scan = self._scan is not None and now - self._scan_t <= AVOID_DEPTH_TTL
            have_depth = self.sectors is not None and now - self._depth_t <= AVOID_DEPTH_TTL
            if have_scan or have_depth:
                # 模型意见先塞进去, 参与本周期的选向 (它只改顺序, 不改任何安全线)
                side_now = self.hint.side(now) if self.hint is not None else 0
                if side_now:
                    conf_now = self.hint.conf(now)
                    self.avoider.note_llm(side_now, now, conf=conf_now)
                    if side_now != self._llm_side:
                        self._llm_side = side_now
                        self._llm_conf = conf_now
                        self._llm_word = 'LEFT' if side_now > 0 else 'RIGHT'
                        self.get_logger().info(
                            f'llm совет: {"левее" if side_now > 0 else "правее"}'
                            f' ({self.hint.last_reason}, {self.hint.last_latency:.1f} с, '
                            f'уверенность {conf_now:.2f})')
                elif self._llm_side:
                    self._llm_side = 0      # 意见过期了, 下次新的再打一遍日志
                # 地图先算「该往哪走」: 把 A* 折线上车前方 1.5 m 的点当本拍方向。
                # 没有前视点 (栅格空 / 算不出路) 时 aim=None, 行为与纯 VFH 完全一致。
                aim = self._grid_aim(now, gx, gy, have_scan)
                plan = self.avoider.update(self.x, self.y, self.yaw.value,
                                           (gx, gy), self.sectors, now,
                                           scan=self._scan if have_scan else None,
                                           aim=aim)
                # ---- 深度贴脸快速脱困 ----
                # 矮障碍(0.10 m 高)整个在雷达扫描面之下(雷达 z=0.235), 几何判
                # 「前方通畅」, 只有深度相机看得见。原来全靠二级 stall 判据
                # (12 s 挪不动 0.15 m)才反应 —— m01262 实测: 994 s 车就顶住了,
                # 1007 s 才判出来, 白等 12.1 s, 而这段恰好压在起点第一段。
                # 深度报 0.50 m 以内、车身 1.5 s 没挪窝, 就直接当卡死处理:
                # 走的是同一套 stall 通道(照样问模型、照样第一轮优先用它的方向),
                # 只是不再等那 12 s —— 询问次数一次都没少。
                _force = ''
                _fg = self.avoider.front_gap
                _fg_ok = (_fg is not None and math.isfinite(_fg)
                          and _fg < 0.55)
                if (plan.advance and plan.state != HOLD and dist > 0.80
                        and _fg_ok):
                    if self._fg_since is None:
                        self._fg_since = now
                        self._fg_x0, self._fg_y0 = self.x, self.y
                        self._fg_yaw0 = self.yaw.value
                    elif (math.hypot(self.x - self._fg_x0,
                                     self.y - self._fg_y0) > 0.10
                          or abs(wrap_pi(self.yaw.value
                                         - self._fg_yaw0)) > 0.35):
                        # 还在动: 起点跟着车走, 否则一次小位移就把计时器
                        # 永久废掉 —— 第一版就栽在这里, 起点钉死在贴脸那一刻,
                        # 车挪了 0.17 m 之后 hypot 永远 > 0.10, 一次都没触发,
                        # 又退回 12 s 的二级判据 (ab5 第二趟 62.9 s)。
                        self._fg_since = now
                        self._fg_x0, self._fg_y0 = self.x, self.y
                        self._fg_yaw0 = self.yaw.value
                    elif now - self._fg_since >= 1.5:
                        _force = 'stall'
                elif _fg is not None and _fg > 0.75:
                    # 只有「明确远离」才清零: 深度在 0.5-0.75 之间抖动时保持
                    # 计时, 不然贴脸读数一掉就又从头数 1.5 s。
                    self._fg_since = None
                self._maybe_ask_llm(now, plan, force=_force)
                # 强制绕行: 卡死且模型已经指出往哪边, 就在这一层直接接管航向。
                # 几何算不出这一步 —— 顶住车的那个矮箱子整个在雷达扫描面之下,
                # 它的 _feasible 对前方每一度都判「通畅」, 永远只给出笔直一条线,
                # 所以模型的方向必须在这里落地, 否则它答一百次车也纹丝不动。
                if now >= self._detour_until:
                    self._detour_heading = None     # 窗口结束, 交回几何
                    self._fg_quick = False
                elif aim is not None:
                    # 地图已经给出绕行方向了, 让「90 度垂直侧移」退居二线:
                    # 两套绕行同时作用会互相拆台 (地图往左、强制绕行往右),
                    # 车就在原地左右甩头。A* 真没路时 aim=None, 这一层照旧接管。
                    self._detour_heading = None
                elif (self._detour_heading is None
                        and plan.advance and plan.state != HOLD):
                    # 锁定一个**绝对**绕行角, 定了就不再改。三条教训都来自实测:
                    # ①挂在当前车头(yaw + 40)是正反馈, 车原地转两整圈
                    #   (lb4: 131 -> 178 -> -129 -> -55);
                    # ②每帧重算方向会和模型后续意见打架, 车在 +40/-40 之间摇摆,
                    #   钉死在原处(lb6);
                    # ③全靠模型定方向也不稳 —— 矮箱子在它视野里只露右下角一小块,
                    #   同一趟里它先答 LEFT 再答 RIGHT(lb7)。所以模型只在第一轮
                    #   有优先权, 之后交给雷达净空, 都不空就翻边重试。
                    s = 0
                    if self._detour_fail <= 1 and self._llm_side:
                        s = self._llm_side
                    elif (self._detour_fail <= 1 and not self._fg_quick
                          and now < self._last_stall_t + 6.0):
                        s = 0       # 第一轮先等模型 6 秒, 它答了就用它的
                        # 深度快速判据触发时不走这条 —— 车已经顶在障碍上,
                        # 再等 6 s 等于把省下的 12 s 又还回去一半。
                    if not s:
                        lc = self.avoider.clearance(60.0, now)
                        rc = self.avoider.clearance(-60.0, now)
                        lv = (9.0 if not math.isfinite(lc) else lc) \
                            if lc is not None else -1.0
                        rv = (9.0 if not math.isfinite(rc) else rc) \
                            if rc is not None else -1.0
                        if abs(lv - rv) > 0.25:
                            s = 1 if lv > rv else -1      # 雷达更空的那一侧
                        else:
                            s = -self._detour_last if self._detour_last else 1
                    if s:
                        self._detour_side = s
                        self._detour_last = s
                        self._detour_until = now + 7.0
                        # 90 度而不是 55: p18 场景(车距矮箱前缘只剩 0.35 m 才起步)
                        # 实测 55 度也救不了 —— 纵向分量 cos55=0.57, 走 0.6 m 就再次
                        # 顶住, 横向只挪了 0.49 m, 而箱宽 0.8 m + 车半宽 0.15 m 要求
                        # 至少侧移 0.55 m。日志里车头在 +121/-9 度之间每秒翻边、
                        # 位置只在 ±0.03 m 内抖, 全程 180 s 没出去。改为垂直侧移:
                        # 先原地转到与目标垂直, 再直行 —— 这个底盘只会沿车头直行,
                        # 垂直方向才是它唯一能「横着走」的方式。
                        self._detour_heading = wrap_pi(
                            plan.goal_dir + math.radians(90.0 * s))
                # 原来这里卡 plan.state == NAV, 意思是「只有几何判直冲时才接管」。
                # 可 depth 兜底(:576)已经会把矮箱判成一堵墙, 于是几何输出的是
                # DODGE +30 度而不是 NAV —— 接管条件永远不成立, 兜底逻辑形同虚设。
                # p18 日志: 全程 138 行都是「DODGE +29 deg ... [挡: depth 0.42 m]」,
                # 车头被这个 +30 度带着走, 而 30 度在 0.05 m 的贴脸间隙里毫无用处。
                if (self._detour_heading is not None
                        and plan.advance and plan.state != HOLD):
                    cl = self.avoider.clearance(90.0 * self._detour_side, now)
                    plan = Plan(state=DODGE, heading=self._detour_heading,
                                goal_dir=plan.goal_dir, side=self._detour_side,
                                advance=True,
                                reason='обход невидимого (%+d deg)'
                                       % (90 * self._detour_side),
                                clear=cl if cl is not None else float('inf'))
            else:
                # 诊断: 两者都过期时避障整段关闭(plan=None), 车会笔直撞上去却不报错。
                if now - getattr(self, '_sens_log_t', 0.0) > 2.0:
                    self._sens_log_t = now
                    self.get_logger().warn(
                        f'датчики протухли -> объезд выключен: '
                        f'scan={"есть" if self._scan is not None else "нет"} '
                        f'(age {now - self._scan_t:.1f} с), '
                        f'depth={"есть" if self.sectors is not None else "нет"} '
                        f'(age {now - self._depth_t:.1f} с)')
                self.avoider.reset()    # 雷达和深度图都断了就别照着旧数据判断
        self.plan = plan
        # 避障日志: 原来「每 1.0 s 无条件打一行」, 同一个障碍能连刷十几次 (实测 lb10
        # 绕行 10 s 打了 8 行一模一样的 [挡: depth 0.88 m]), 而且 1.29 m 就写「挡」也
        # 容易让人以为随时会撞 —— 1.40 m 是**提前打方向**的量, 不是会撞的判据。现在分三档:
        #   ⚠ близко : 正前方 < safe_dist(0.60 m), 真会撞, 进入那一刻报一次, 出去复位
        #   提示      : 换了方向 / 换了挡的东西 / 刚进避障, 报一次
        #   其余      : 静默 (同一障碍持续中、停下等它走, 都不再刷屏)
        if plan is not None:
            _fg = self.avoider.front_gap
            _danger = math.isfinite(_fg) and _fg < self.avoider.cfg.safe_dist
            if _danger and not self._warn_danger:
                self._warn_danger = True
                self.get_logger().warn(
                    f'⚠ близко: {_fg:.2f} м ({"стоим" if not plan.advance else "объезд"})'
                    + (f' [{self.avoider.block_why}]' if self.avoider.block_why else ''))
            elif not _danger:
                self._warn_danger = False
        if plan is not None and plan.state != NAV:
            # 签名里**不能**放 block_why 原文: 它带距离数字, 每帧都在变
            # (1.40 -> 1.12 -> 0.89 -> 0.69), 等于没去重。只按「状态 + 是否前进 +
            # 航向 20 度一档」认, 再留 8 s 一次的心跳, 长时间静默时也能看出还活着。
            _sig = (plan.state, plan.advance,
                    int(round(math.degrees(plan.heading) / 20.0)))
            if _sig != self._plan_sig and now - self._avoid_log_t >= 3.0:
                self._plan_sig = _sig
                self._avoid_log_t = now
                self.get_logger().info(
                    f'объезд {plan.state}: курс {math.degrees(plan.heading):+.0f} deg '
                    f'(цель {math.degrees(plan.goal_dir):+.0f}), просвет {plan.clear:.2f} м, '
                    f'{"едем" if plan.advance else "только поворот"} — {plan.reason}'
                    + (f' [挡: {self.avoider.block_why}]'
                       if self.avoider.block_why else '')
                    # 模型有没有真的改过序: 原来这条只写进 avoid_planner.pref_src,
                    # 而 pref_src 全项目没人读 —— 想知道模型参没参与只能 grep DBG,
                    # 而 DBG 又只在 from_scratch 里打印, 等于没有观测手段。
                    + (f' [модель: {self.avoider.tie_info}]'
                       if getattr(self.avoider, 'tie_info', '') else '')
                    + (' [выбор: модель]' if self.avoider.pref_src == 'llm' else ''))

        # 近场锁定方向: 距离很小时 dx/dy 抖动会让 atan2 乱跳, 车头会在终点前乱转
        if plan is not None:
            self.want_heading = plan.heading    # 避障方向 (含「先转过去看一眼」)
        elif dist > NEAR_HEADING or self.want_heading is None:
            # dist 恰好为 0 (目标就是当前位置: 复位/空目标) 时不能留 None,
            # 否则下面 wrap_pi(None - yaw) 直接 TypeError 把节点打死。
            self.want_heading = math.atan2(dy, dx) if dist > 1e-6 else self.yaw.value
        err = wrap_pi(self.want_heading - self.yaw.value)  # 目标方向 vs 车头
        arrived_yaw = True

        hold = plan is not None and plan.state == HOLD
        no_adv = plan is not None and not plan.advance

        # 门限分两档: 远离目标时允许大角度边走边转 (弧线), 进到终点跟前再收紧 ——
        # 近场划弧会把 2 cm 的到点精度做丢, 那里该老实停下来对齐。
        gate = self.heading_gate if dist_cmd >= NEAR_GATE_DIST \
            else min(self.heading_gate, NEAR_GATE)

        if hold:
            # 视野内处处都太窄 (被围死/走廊尽头): 指令收回到实测位姿, 位置伺服就地
            # 刹住, 原地等着, 不硬挤 —— 挤过去就是撞。
            phase = 'стоп: препятствие вплотную'
            self.cmd_x, self.cmd_y = self.x, self.y
            self.cmd_yaw = self.yaw.value
        elif not arrived_xy and (abs(err) > gate or no_adv):
            # 车头偏得太多, 或避障要求「先转头观察」: 原地转向, 期间不前进
            phase = f'поворот (отклонение {math.degrees(err):+.0f} deg)'
            w = min(self.w_max, max(math.sqrt(4.0 * abs(err)), 0.05))
            self.cmd_yaw += math.copysign(min(w * dt, abs(err)), err)
        elif not arrived_xy:
            # 已对准: 沿车头直行, 边走边小角度修正 (车头始终朝运动方向)
            phase = 'едем'
            v = min(self.v_max, max(math.sqrt(2.0 * self.a_max * dist_cmd), 0.03))
            if dist_cmd < 0.5:
                # 近场按比例降速: 0.30 m -> 0.18 m/s, 0.10 m -> 0.06 m/s。
                # 固定 0.18 m/s 会以恒定速度冲过目标点, 在终点周围转圈 (踩过这个坑)。
                v = min(v, max(0.06, 0.6 * dist_cmd))
            if plan is not None and plan.state == DODGE:
                # 绕行限速按净空自适应。原来是死 0.30, 而日志实测(mixed 0,8)显示:
                # 直行段伺服能稳稳跑 0.50 (指令-实测偏差只有 2 cm), 绕行段却被这一句
                # 钉在 0.30, 全程一半时间在爬 —— 8 m 走 32 s 里至少 8 s 是白等的。
                _cl = plan.clear if math.isfinite(plan.clear) else 9.0
                _vcap = 0.30 + 0.30 * max(0.0, min(1.0, (_cl - 0.60) / 0.90))
                v = min(v, _vcap)                   # 净空 0.6 m -> 0.30, 1.5 m 以上 -> 0.60
            # 偏差越大越慢, 但不停车 —— 这就是「边走边转」与原来「停-转-停」的差别。
            # 平方衰减 + 下限 0.45: 门前 34 度偏角时还剩 22 cm/s, 依旧能转过去。
            # 基准用固定的 1.0 rad 而不是 gate: gate 收小时 (近场 0.15) 否则会把
            # 10 度的正常修正也砍掉一半速度
            v *= max(0.45, 1.0 - (abs(err) / 1.0) ** 2)
            step = min(v * dt, dist_cmd)            # 指令不越过目标
            self.cmd_x += math.cos(self.cmd_yaw) * step
            self.cmd_y += math.sin(self.cmd_yaw) * step
            w = max(-self.w_max, min(self.w_max, self.turn_gain * err))
            self.cmd_yaw += w * dt
        else:
            phase = 'на месте'

        # 到点后若指定了朝向, 原地转到该朝向
        if arrived_xy and gyaw is not None:
            tgt = self.yaw.nearest(gyaw)
            yerr = wrap_pi(tgt - self.yaw.value)
            arrived_yaw = abs(yerr) <= self.yaw_eps
            if not arrived_yaw:
                w = min(self.w_max, max(math.sqrt(4.0 * abs(yerr)), 0.05))
                self.cmd_yaw += math.copysign(min(w * dt, abs(yerr)), yerr)
                phase = f'доворот (отклонение {math.degrees(yerr):+.0f} deg)'
            else:
                self.cmd_yaw = tgt

        # 指令不得超前实测太多: 伺服滞后/定位断流时, 超前量就是会冲过头的距离
        lead = math.hypot(self.cmd_x - self.x, self.cmd_y - self.y)
        if lead > self.max_lead:
            k = self.max_lead / lead
            self.cmd_x = self.x + (self.cmd_x - self.x) * k
            self.cmd_y = self.y + (self.cmd_y - self.y) * k
        lead_yaw = wrap_pi(self.cmd_yaw - self.yaw.value)
        if abs(lead_yaw) > MAX_YAW_LEAD:
            self.cmd_yaw = self.yaw.value + math.copysign(MAX_YAW_LEAD, lead_yaw)

        cmd_x, cmd_y, yaw_cmd = self.cmd_x, self.cmd_y, self.cmd_yaw

        cmd_x = max(-DRIVE_LIMIT, min(DRIVE_LIMIT, cmd_x))
        cmd_y = max(-DRIVE_LIMIT, min(DRIVE_LIMIT, cmd_y))

        self.pub_x.publish(Float64(data=cmd_x))
        self.pub_y.publish(Float64(data=cmd_y))
        self.pub_yaw.publish(Float64(data=yaw_cmd))
        self._record_latency(time.time())
        self._override_on = True
        self.pub_override.publish(Float64(data=1.0))     # 运动期间由导航接管
        self._last_cmd_x, self._last_cmd_y, self._last_cmd_yaw = cmd_x, cmd_y, yaw_cmd

        # ---- 卡死早停: 45 s 挪不动 0.25 m 就是没绕过去 ----
        # 只在「该走但走不动」时判定: 到点后的原地转向、起点等定位都不参与。
        if self._stall_abort > 0.0 and not arrived_xy and self.state == self.MOVING:
            t_st = time.time()
            self._stall_hist.append((t_st, self.x, self.y))
            while self._stall_hist and t_st - self._stall_hist[0][0] > self._stall_abort:
                self._stall_hist.popleft()
            if (len(self._stall_hist) > 50
                    and t_st - self._stall_hist[0][0] >= self._stall_abort * 0.95):
                _t0, sx, sy = self._stall_hist[0]
                if math.hypot(self.x - sx, self.y - sy) < 0.25:
                    self.get_logger().error(
                        f'✗ Застрял: {self._stall_abort:.0f}с без продвижения '
                        f'(x={self.x:+.2f} y={self.y:+.2f}), бросаю цель')
                    self.cmd_x, self.cmd_y = self.x, self.y
                    self.pub_x.publish(Float64(data=self.x))
                    self.pub_y.publish(Float64(data=self.y))
                    self.pub_override.publish(Float64(data=1.0))
                    self._publish_status('застрял')
                    self._stall_hist.clear()
                    raise SystemExit(2)

        # ---- 到点 ----
        if arrived_xy and arrived_yaw:
            self.pub_x.publish(Float64(data=self.x))
            self.pub_y.publish(Float64(data=self.y))
            self._last_cmd_x, self._last_cmd_y = self.x, self.y
            self.state = self.ARRIVED
            self.pub_reached.publish(Bool(data=True))
            self.get_logger().info(
                f'✓ Прибыл ({gx:+.2f}, {gy:+.2f})'
                f' | факт x={self.x:+.2f} y={self.y:+.2f} '
                f'yaw={math.degrees(self.yaw.value):+.1f} deg'
                f' | ошибка {dist * 1000:.0f} мм')
            self._publish_status('прибыл')
            if not self.queue and self._lat_on and self._lat_n:
                self._lat_on = False
                self._log_latency('итог')
            self.goal = None
            self._start_next()                       # 队列里还有就继续走
            return

        # ---- 1 Hz 进度日志 ----
        t = time.time()
        if t - getattr(self, '_tick_stat_t', 0.0) >= 5.0:
            self._tick_stat_t = t
            self._tick_n = getattr(self, '_tick_n', 0)
            self.get_logger().info(f'цикл управления {self._tick_n / 5.0:.1f} Гц')
            self._tick_n = 0
        if t - self._status_log_t >= 1.0:
            self._status_log_t = t
            speed = min(self.v_max, max(math.sqrt(2.0 * self.a_max * dist), 0.03))
            self.get_logger().info(
                f'→ {phase} осталось {dist:.2f} м, скорость {speed:.2f} м/с '
                f'| 当前 ({self.x:+.2f}, {self.y:+.2f}) yaw={math.degrees(self.yaw.value):+.0f} deg'
                f' -> цель ({gx:+.2f}, {gy:+.2f})')
            self._publish_status(phase)

    def _record_latency(self, t_cmd):
        """一次「传感器帧 -> 控制指令」的端到端延迟 (秒), 对应 ТЗ 4.2.2。

        帧取本周期真正参与决策的那一帧 —— 雷达/深度里最新的到达时刻。传感器
        还没上线时这条指标没有意义 (避障也没开), 直接跳过。
        """
        if not self._lat_on:
            return
        t_frame = max(self._scan_t, self._depth_t)
        if t_frame <= 0.0 or t_cmd <= t_frame:
            return
        lat = t_cmd - t_frame
        self._lat.append(lat)
        self._lat_n += 1
        self.pub_latency.publish(Float64(data=lat))
        if self._lat_n >= self._lat_every:
            self._lat_on = False                       # 采满即停, 只测一次
            self._log_latency(f'一次性测量, {self._lat_n} 个连续周期')

    def _log_latency(self, tag=''):
        """打印累计的响应时间分位数, 格式直接对应验收表。"""
        v = sorted(self._lat)
        if not v:
            return
        n = len(v)

        def q(p):
            return v[min(n - 1, int(round(p * (n - 1))))]

        over = sum(1 for x in v if x > 1.0)
        self.get_logger().info(
            f'⏱ 响应时间 {tag}: n={n} p50={q(0.5) * 1000:.0f}мс '
            f'p95={q(0.95) * 1000:.0f}мс max={v[-1] * 1000:.0f}мс '
            f'| >1с: {over} ({over * 100.0 / n:.1f}%)')

    def _publish_status(self, text):
        msg = String()
        msg.data = f'{self.state} | {text} | x={self.x:+.2f} y={self.y:+.2f} ' \
                   f'yaw={math.degrees(self.yaw.value):+.1f} deg'
        self.pub_status.publish(msg)


# ================= 终端交互输入 =================
HELP_TEXT = """Формат ввода (по строке, Enter - выполнить):
  x y          ехать в (x, y), курс не меняется        Пример: 2.0 -1.5
  x y угол     ехать в (x, y) и повернуть на угол(°)   Пример: 1.0 1.0 90
  stop         отменить цель, стоять на месте
  s            позиция и состояние
  h            показать эту справку
  q            выход

Метры, начало координат - место запуска симуляции; ход шасси ±10 м.""".strip()


def pose_line(node):
    if not node.got_joints:
        return 'жду /joint_states...'
    return (f'x={node.x:+.2f} m  y={node.y:+.2f} m  '
            f'yaw={math.degrees(node.yaw.value):+.1f} deg')


def stdin_loop(node):
    """无参数启动时, 让用户直接在终端敲坐标出发。"""
    time.sleep(0.8)      # 等第一帧定位, 横幅里好显示当前位置
    bar = '=' * 64
    print('\n' + bar)
    print('  Навигация по цели — введите координаты, робот поедет сам')
    print(f'  Текущая позиция: {pose_line(node)}')
    print(bar)
    print(HELP_TEXT)
    print(bar + '\n')
    try:
        while rclpy.ok():
            sys.stdout.write('Координаты > ')
            sys.stdout.flush()
            line = sys.stdin.readline()
            if not line:
                break
            line = line.strip()
            if not line:
                continue
            low = line.lower()
            if low in ('q', 'quit', 'exit'):
                break
            if low in ('h', 'help', '?'):
                print(HELP_TEXT)
                continue
            if low in ('s', 'status'):
                print(f'  Позиция: {pose_line(node)} | Состояние: {node.state} '
                      f'| Осталось целей: {len(node.queue)}')
                continue
            if low in ('stop', 'c', 'cancel'):
                node.cancel('stop из терминала')
                continue
            parts = line.split()
            try:
                x, y = float(parts[0]), float(parts[1])
                yaw = math.radians(float(parts[2])) if len(parts) > 2 else None
            except (ValueError, IndexError):
                print('  Неверный формат, нужно: x y [угол deg]   Пример: 2.0 -1.5  или  1.0 1.0 90')
                continue
            print('  Принято' + (f' ({x:+.2f}, {y:+.2f}) yaw={math.degrees(yaw):+.1f} deg'
                              if yaw is not None else f' ({x:+.2f}, {y:+.2f})'))
            node.add_goal(x, y, yaw)
    except (KeyboardInterrupt, EOFError):
        pass


def parse_cli():
    """从命令行取坐标。--ros-args 之后的交给 rclpy。"""
    argv = sys.argv[1:]
    # 去掉 ROS 参数段
    if '--ros-args' in argv:
        idx = argv.index('--ros-args')
        ros_part = argv[idx:]
        argv = argv[:idx]
    else:
        ros_part = []
    # 过滤 --key value / --flag
    nums, yaw, append, face = [], None, False, False
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in ('--yaw', '-y') and i + 1 < len(argv):
            yaw = math.radians(float(argv[i + 1]))
            i += 2
            continue
        if a in ('-p', '--param') and i + 1 < len(argv):
            # 透传 ROS 参数: -p v_max:=0.8  ->  --ros-args -p v_max:=0.8
            ros_part.extend(['--ros-args', '-p', argv[i + 1]])
            i += 2
            continue
        if a in ('--append',):
            append = True
            i += 1
            continue
        if a in ('--face-motion',):
            face = True
            i += 1
            continue
        if a.startswith('-') and not a[1:2].isdigit() and a not in ('-h',):
            # 未知开关, 跳过 (不吞参数值)
            i += 1
            continue
        try:
            nums.append(float(a))
        except ValueError:
            pass
        i += 1
    return nums, yaw, append, face, ros_part


def main(args=None):
    nums, yaw, append, face_motion, ros_part = parse_cli()
    rclpy.init(args=[sys.argv[0]] + ros_part)
    node = Navigator()
    if face_motion:
        node.face_motion = True

    # 命令行坐标: 成对解析 -> 航点列表; 最后多出的一个忽略
    pts = []
    for i in range(0, len(nums) - 1, 2):
        pts.append((nums[i], nums[i + 1], None))
    if pts:
        if len(pts) == 1:
            pts[0] = (pts[0][0], pts[0][1], yaw)
            node.add_goal(*pts[0])
        else:
            if yaw is not None:
                pts[-1] = (pts[-1][0], pts[-1][1], yaw)
            node.add_goals(pts)
    else:
        t = threading.Thread(target=stdin_loop, args=(node,), daemon=True)
        t.start()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.cancel('выход')
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
