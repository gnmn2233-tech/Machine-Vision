#!/usr/bin/env python3
"""障碍物避障 / 相机转向节点。

决策原则:
  1. 相机看到障碍在左 -> 向右转(看开); 障碍在右 -> 向左转; 无碍 -> 停。
  2. 确定性规则优先; 模型只在"两侧都有障碍 / 仅中间有障碍"这类模糊场景做补充决策。
     模型失败或未配置时自动用规则兜底, 节点永远可用。
  3. 角度符号: 正值 -> 相机向左, 负值 -> 相机向右 (与 base_yaw_joint +z 轴一致)。

深度测距 (深度相机 /depth_camera, 32FC1 米, 与 /camera 同位置同视场像素对齐):
  - 每个像素先按"地面期望深度"剔除地面(相机水平固定高度, 每行地面深度是定值),
    剩下的近处像素才算障碍, 再按 左/中/右 三区取最近距离。
  - 深度能测出"多远", 所以深度规则优先于 YOLO: 一侧贴脸就不往那侧转、
    三区都远则视为空旷。深度不可用时自动回落 YOLO 规则, 行为与不带深度时一致。
  - 对外发布: /nearest_obstacle (Float64 米, inf=前方空旷) 与 /obstacle_zones
    (Float32MultiArray [左, 中, 右] 米), 供键盘/大模型/后续导航模块取用。
  - YOLO 检测框用深度图中位数测距, 距离一并喂给模型。

模型后端 (环境变量配置, 只依赖 requests):
  本地优先 + 云端兜底 (故障转移):
    LLM_PROVIDER=failover
    主: LLM_LOCAL_URL=http://127.0.0.1:1234/v1  LLM_LOCAL_MODEL=qwen2.5-7b-instruct-1m
         (LM Studio OpenAI 兼容; 本地异常/超时/无输出时自动切主)
    备: LLM_CLOUD_URL=https://api.openai.com/v1  LLM_CLOUD_KEY=sk-...  LLM_CLOUD_MODEL=gpt-4o-mini
         (云端也失败 -> 回落纯规则, 节点永远可用)
  本地 Ollama:
    LLM_PROVIDER=ollama  OLLAMA_URL=http://localhost:11434  OLLAMA_MODEL=llava:7b
  云端 OpenAI 兼容 API (OpenAI / DeepSeek / Qwen / Moonshot / Zhipu 等):
    LLM_PROVIDER=openai  OPENAI_API_KEY=sk-...  OPENAI_BASE_URL=https://api.openai.com/v1  OPENAI_MODEL=gpt-4o-mini
  纯规则 (关闭模型):
    LLM_PROVIDER=off
  默认 auto:
    有 OPENAI_API_KEY -> openai; 否则 -> ollama
"""
import base64
import math
import os
import re
import threading
import time

import cv2
import numpy as np
import requests
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float32MultiArray, Float64
from yolo_msgs.msg import DetectionArray

ROTATE_STEP = 0.03        # 每周期(0.04s)的目标角增量, 控制转速
ASK_INTERVAL = 3.0        # 模糊场景请求模型的最小间隔(秒)
DIR_HOLD = 1.0            # 转向方向最小保持时间(秒): 滞回防来回抖
RESYNC_TIMEOUT = 3.0      # 接管释放后等待 /joint_states 重同步的超时(秒): 超时强制恢复避障
LEFT_PERCENT = 0.4
RIGHT_PERCENT = 0.6
LLM_TIMEOUT = 30
MAX_IMAGE_W = 512

# ---- 深度测距 (depth camera) ----
DEPTH_TOPIC = '/depth_camera'
DEPTH_TIMEOUT = 1.0       # 秒: 深度图超时未更新 -> 视为无深度, 回落 YOLO 规则
# 相机光心离地高度(米); 相机位置/机器人装配变了要同步改, 否则地面会被算成贴脸障碍。
CAM_HEIGHT = 0.28
CAM_HFOV = 1.047          # 相机水平视场 (与 SDF 中 horizontal_fov 一致)
GROUND_RATIO = 0.75       # 像素深度 < 地面期望深度 * 该比例 -> 判为障碍(否则是地面)
ZONE_NEAR = 1.5           # 米: 该距离内视为近处障碍, 需要让开
ZONE_CLEAR = 3.0          # 米: 三区都比它远 -> 视为前方空旷
STOP_DIST = 0.5           # 米: 贴脸距离, 绝不转向该侧
SIDE_MARGIN = 0.3         # 米: 两侧距离差小于它视为"相当"
DEPTH_PERCENTILE = 10.0   # 每区取该百分位作为最近障碍距离(抗离群像素)


class SmoothYaw:
    """无极丝滑转向器 (与 keyboard_joint_controller.py / vlm_camera_rotator.py 保持一致)。

    - 角度永不回卷(不 %2π): 目标角无限累积, revolute 关节可连续转任意圈,
      跨 ±π 边界零突跳 —— 这是"无极转向"的关键。
    - 发布角以 限速+限加速度 的梯形速度规划逼近目标: 恒速转动、远端减速
      缓停, 指令无阶跃 —— 这是"丝滑转向"的关键。
    """

    def __init__(self, max_rate=0.75, accel=10.0, dt=0.04):
        self.target = 0.0
        self.pos = 0.0
        self.vel = 0.0
        self.max_rate = max_rate
        self.accel = accel
        self.dt = dt

    def step_delta(self, delta):
        self.target += delta

    def set_target(self, t):
        self.target = t

    def tick(self):
        err = self.target - self.pos
        if abs(err) < 1e-9:
            self.vel = 0.0
            return self.pos
        direction = 1.0 if err > 0.0 else -1.0
        dist = abs(err)
        stop_dist = self.vel * self.vel / (2.0 * self.accel)
        if dist <= stop_dist:
            v_des = math.sqrt(max(2.0 * self.accel * dist, 0.0))
        else:
            v_des = self.max_rate
        if direction * self.vel >= 0.0:
            self.vel += direction * self.accel * self.dt
        else:
            self.vel -= direction * 2.0 * self.accel * self.dt
        if abs(self.vel) > v_des:
            self.vel = direction * v_des
        self.pos += self.vel * self.dt
        return self.pos


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_width = 640
        self.image_height = 480
        self._objects = []
        self._object_dists = []         # 与 _objects 一一对应的障碍距离(米, inf=没测到)

        # 深度测距状态: 左/中/右三区最近障碍距离(米, inf=该区无近处障碍)
        self.latest_depth = None
        self.depth_zones = (math.inf, math.inf, math.inf)
        self.depth_stamp = 0.0
        self.depth_action = None        # 深度规则动作
        self._depth_warned = False
        self._yolo_action = None        # YOLO 规则动作(深度不可用时的兜底)

        # 无极丝滑转向: 角度无限累积不 wrap, 发布角限速逼近目标
        self.smooth = SmoothYaw(max_rate=ROTATE_STEP / 0.04, accel=10.0, dt=0.04)
        self.rotate_dir = 0.0           # +1 左, -1 右, 0 停
        self._last_dir = 0.0            # 上一有效方向 (滞回用)
        self._dir_lock_until = 0.0      # 当前方向锁定截止时刻
        self.rule_action = None         # 规则动作
        self.llm_action = None          # 模型动作
        self.llm_action_time = 0.0

        self._llm_lock = threading.Lock()
        self._llm_busy = False
        self._last_ask = 0.0
        self._last_fail_log = 0.0
        self._joint_synced = False      # 是否已从 /joint_states 同步真实关节角
        self._boot_time = time.time()
        self._sync_warned = False
        self._overridden = False        # 键盘接管中 (避障停发)
        self._resync_pending = False    # 接管释放后等待重同步
        self._resync_t0 = 0.0

        qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.det_sub = self.create_subscription(DetectionArray, '/yolo/detections', self.on_detections, qos)
        self.img_sub = self.create_subscription(Image, '/camera', self.on_image, 10)
        self.depth_sub = self.create_subscription(Image, DEPTH_TOPIC, self.on_depth, qos)
        self.js_sub = self.create_subscription(JointState, '/joint_states', self.on_joint_states, qos)
        self.ov_sub = self.create_subscription(Float64, '/avoider_override', self.on_override, 10)
        self.cmd_pub = self.create_publisher(Float64, '/camera_joint_controller/commands', 10)
        self.dist_pub = self.create_publisher(Float64, '/nearest_obstacle', 10)
        self.zones_pub = self.create_publisher(Float32MultiArray, '/obstacle_zones', 10)
        self.timer = self.create_timer(0.04, self.tick)

        self.provider = self._detect_provider()
        self.get_logger().info(f'避障节点启动: 模型后端={self.provider}, 模型={self._model_name()}')

    # ---------------- 配置 ----------------
    def _model_name(self):
        if self.provider == 'openai':
            return os.environ.get('OPENAI_MODEL', 'gpt-4o-mini')
        if self.provider == 'failover':
            local = os.environ.get('LLM_LOCAL_MODEL', 'qwen2.5-7b-instruct-1m')
            cloud = os.environ.get('LLM_CLOUD_MODEL', 'gpt-4o-mini')
            return f'{local}(本地) / {cloud}(云端兜底)'
        return os.environ.get('OLLAMA_MODEL', 'llava:7b')

    def _detect_provider(self):
        p = os.environ.get('LLM_PROVIDER', 'auto').strip().lower()
        if p in ('off', 'none', 'rule', 'rules'):
            return 'off'
        if p == 'failover':
            return 'failover'
        if p == 'openai':
            return 'openai'
        if p == 'ollama':
            return 'ollama'
        if os.environ.get('OPENAI_API_KEY'):
            return 'openai'
        return 'ollama'

    # ---------------- 传感器回调 ----------------
    def on_joint_states(self, msg):
        """同步真实关节角: 启动时零漂移起步; 键盘接管释放后重新对齐。

        只在 启动首次 / 接管释放(resync) 时对齐 —— 运行时不得覆盖
        避障自己累加的 target (否则永远转不动)。
        """
        if self._joint_synced and not self._resync_pending:
            return
        try:
            idx = list(msg.name).index('base_yaw_joint')
        except ValueError:
            return
        angle = msg.position[idx]
        self.smooth.pos = angle              # 已发布角 = 真实角
        self.smooth.target = angle           # 目标角 = 真实角, 不做初始化回跳
        was_resync = self._resync_pending
        self._joint_synced = True
        self._resync_pending = False
        if was_resync:
            self.get_logger().info(f'接管释放: 已重新同步关节角 {angle:.3f} rad, 避障恢复')
        else:
            self.get_logger().info(f'已同步真实关节角 base_yaw_joint={angle:.3f} rad, 零漂移起步')

    def on_override(self, msg):
        """键盘接管信号: >0.5 键盘说话(避障停发); <=0.5 键盘交还(避障恢复)。"""
        ov = msg.data > 0.5
        if ov and not self._overridden:
            self._overridden = True
            self.get_logger().info('键盘接管: 避障暂停发布')
        elif not ov and self._overridden:
            self._overridden = False
            self._resync_pending = True
            self._resync_t0 = time.time()
            self.get_logger().info('键盘释放: 等待关节角重同步')

    def on_image(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_width = max(img.shape[1], 1)
            self.image_height = max(img.shape[0], 1)
            if img.shape[1] > MAX_IMAGE_W:
                h = int(img.shape[0] * MAX_IMAGE_W / img.shape[1])
                img = cv2.resize(img, (MAX_IMAGE_W, max(h, 1)))
            self.latest_image = img
        except Exception:
            pass

    # ---------------- 深度测距 ----------------
    def _depth_ok(self):
        """深度数据是否新鲜: 桥断了/没起深度相机时自动失效, 退回 YOLO 规则。"""
        return self.depth_stamp > 0.0 and (time.time() - self.depth_stamp) < DEPTH_TIMEOUT

    def on_depth(self, msg):
        """深度图 -> 左/中/右三区最近障碍距离, 并发布测距话题。

        先剔除地面: 相机水平且离地高度固定, 每一像素行对应一个确定的地面深度
        (越靠画面下方越近), 深度不小于"地面期望"的像素就是地面本身。
        这样画面下部不会把脚下的地面当成障碍, 矮障碍也仍能被测到。
        """
        try:
            arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception:
            try:
                arr = self.bridge.imgmsg_to_cv2(msg)
            except Exception:
                return
        arr = np.asarray(arr, dtype=np.float32)
        if arr.ndim != 2 or arr.size == 0:
            return
        self.latest_depth = arr
        h, w = arr.shape[:2]

        rows = np.arange(h, dtype=np.float32)
        ground = CAM_HEIGHT * ((w / 2.0) / math.tan(CAM_HFOV / 2.0)) / np.maximum(rows - h / 2.0, 1e-3)
        obstacle = np.isfinite(arr) & (arr > 0.0) & (rows[:, None] <= h / 2.0)
        below = rows > h / 2.0
        obstacle[below] |= (arr[below] < ground[below, None] * GROUND_RATIO)

        xl, xr = int(w * LEFT_PERCENT), int(w * RIGHT_PERCENT)
        zones = (
            self._zone_dist(arr[:, :xl], obstacle[:, :xl]),
            self._zone_dist(arr[:, xl:xr], obstacle[:, xl:xr]),
            self._zone_dist(arr[:, xr:], obstacle[:, xr:]),
        )
        self.depth_zones = zones
        self.depth_stamp = time.time()
        self.depth_action = self._depth_rule(zones)
        self.rule_action = self._fuse_rules()

        m = Float64()
        m.data = float(min(zones))
        self.dist_pub.publish(m)
        z = Float32MultiArray()
        z.data = [float(v) for v in zones]
        self.zones_pub.publish(z)

    @staticmethod
    def _zone_dist(depth, mask):
        """区域内障碍像素的最近距离(取低百分位, 抗离群点); 没有障碍像素则 inf。"""
        if mask.size == 0:
            return math.inf
        vals = depth[mask][::4]                      # 下采样, 距离统计不需要全分辨率
        vals = vals[np.isfinite(vals) & (vals > 0.0)]
        if vals.size == 0:
            return math.inf
        return float(np.percentile(vals, DEPTH_PERCENTILE))

    def _depth_rule(self, zones):
        """深度规则: 用距离(而不是检测框个数)决定往哪边看。"""
        left, center, right = zones
        if min(left, center, right) >= ZONE_CLEAR:
            return None                                   # 三区都远 -> 前方空旷
        if left < STOP_DIST and right >= STOP_DIST:
            return 'TURN_RIGHT'                           # 左侧贴脸 -> 只能往右
        if right < STOP_DIST and left >= STOP_DIST:
            return 'TURN_LEFT'
        if left < STOP_DIST and right < STOP_DIST:
            return None                                   # 两侧都贴脸 -> 保持, 乱转更糟
        if center < ZONE_NEAR:                            # 正前方有近障碍 -> 让向更空的一侧
            return 'TURN_LEFT' if left >= right else 'TURN_RIGHT'
        if left < right - SIDE_MARGIN:
            return 'TURN_RIGHT'                           # 左侧更近 -> 向右看
        if right < left - SIDE_MARGIN:
            return 'TURN_LEFT'
        if center < ZONE_CLEAR:
            return 'TURN_LEFT' if left >= right else 'TURN_RIGHT'
        return None

    def _fuse_rules(self):
        """深度优先, YOLO 兜底: 深度说空旷而 YOLO 看到障碍时宁可信其有。"""
        if not self._depth_ok() or self.depth_action is None:
            return self._yolo_action
        return self.depth_action

    def _box_distance(self, det):
        """用深度图给 YOLO 检测框测距(框内障碍像素中位数, 米); 测不到返回 inf。"""
        d = self.latest_depth
        if d is None or not self._depth_ok():
            return math.inf
        try:
            x = int(det.bbox.center.position.x)
            y = int(det.bbox.center.position.y)
            bw = int(det.bbox.size.x)
            bh = int(det.bbox.size.y)
        except Exception:
            return math.inf
        h, w = d.shape[:2]
        x0, x1 = max(x - bw // 2, 0), min(x + bw // 2, w)
        y0, y1 = max(y - bh // 2, 0), min(y + bh // 2, h)
        if x1 <= x0 or y1 <= y0:
            return math.inf
        roi = d[y0:y1, x0:x1]
        vals = roi[np.isfinite(roi) & (roi > 0.0)]
        if vals.size == 0:
            return math.inf
        return float(np.median(vals))

    def on_detections(self, msg):
        objects = []
        dists = []
        for det in msg.detections:
            if getattr(det, 'score', 0.0) < 0.5:
                continue
            try:
                cx = det.bbox.center.position.x / self.image_width
            except Exception:
                cx = -1.0
            name = getattr(det, 'class_name', '') or str(getattr(det, 'class_id', '?'))
            objects.append((name, cx))
            dists.append(self._box_distance(det))

        self._objects = objects
        self._object_dists = dists
        self._yolo_action = self._rule_decision(objects)
        self.rule_action = self._fuse_rules()
        ambiguous = self._is_ambiguous(objects)

        if not ambiguous:
            # 明确或无障碍时, 旧的模型结果作废
            self.llm_action = None
        elif self.provider != 'off':
            now = time.time()
            if now - self._last_ask >= ASK_INTERVAL:
                self._last_ask = now
                self._ask_llm_async(objects)

    # ---------------- 决策 ----------------
    def _side_count(self, objects, side):
        if side == 'left':
            return sum(1 for _, cx in objects if 0.0 <= cx < LEFT_PERCENT)
        return sum(1 for _, cx in objects if cx > RIGHT_PERCENT)

    def _is_ambiguous(self, objects):
        if not objects:
            return False
        left = self._side_count(objects, 'left')
        right = self._side_count(objects, 'right')
        return not (left > 0 and right == 0) and not (right > 0 and left == 0)

    def _rule_decision(self, objects):
        if not objects:
            return None
        left = self._side_count(objects, 'left')
        right = self._side_count(objects, 'right')
        if left > 0 and right == 0:
            return 'TURN_RIGHT'   # 障碍在左 -> 相机向右看
        if right > 0 and left == 0:
            return 'TURN_LEFT'    # 障碍在右 -> 相机向左看
        # 两侧都有 -> 朝障碍更稀疏/更靠边的一侧转 (不只看数量, 看分布)
        lx = [cx for _, cx in objects if 0.0 <= cx < LEFT_PERCENT]
        rx = [cx for _, cx in objects if cx > RIGHT_PERCENT]
        if left == right and lx and rx:
            # 数量相同: 比较"离中轴最近"的障碍 —— 哪侧先堵住正前, 转开另一侧
            l_crowd = max(lx)          # 左侧障碍中最靠中间的 (威胁最大)
            r_crowd = min(rx)          # 右侧障碍中最靠中间的 (威胁最大)
            return 'TURN_LEFT' if r_crowd <= 0.5 + (0.5 - l_crowd) else 'TURN_RIGHT'
        return 'TURN_LEFT' if left < right else 'TURN_RIGHT'

    def _validate(self, action):
        if not self._objects:
            return None
        left = self._side_count(self._objects, 'left')
        right = self._side_count(self._objects, 'right')
        if left > 0 and right == 0:
            return 'TURN_RIGHT'
        if right > 0 and left == 0:
            return 'TURN_LEFT'
        return action

    def _apply_action(self, action):
        """应用转向方向, 带方向滞回 (hysteresis):
        - 相机转向会改变障碍在视野里的分布 -> 规则结果跟着翻转 -> 指令反方向来回抖。
        - 一旦开始转某方向, 锁定 DIR_HOLD 秒内禁止反向切换;
          反向信号只作为"候选", 锁定到期后再评估。
        - 从静止(0)启动新方向不受锁限制 (避免死锁).
        """
        now = time.time()
        new_dir = {'TURN_LEFT': 1.0, 'TURN_RIGHT': -1.0}.get(action, 0.0)
        if new_dir == self._last_dir:
            self.rotate_dir = new_dir
            return
        # 方向要变化: 当前正在转且未过锁定期 -> 拒绝切换, 保持原方向
        if self._last_dir != 0.0 and new_dir != 0.0 and now < self._dir_lock_until:
            return
        self._last_dir = new_dir
        self.rotate_dir = new_dir
        if new_dir != 0.0:
            self._dir_lock_until = now + DIR_HOLD

    # ---------------- 主定时器 ----------------
    def tick(self):
        # 键盘接管中: 完全停发, 让位给键盘控制器
        if self._overridden:
            return
        # 接管释放待重同步: 等 joint_state 对齐(清 resync_pending)后再恢复, 否则位置跳变
        if self._resync_pending:
            if time.time() - self._resync_t0 > RESYNC_TIMEOUT:
                # /joint_states 迟迟不到 (桥断了) —— 不能永久停发, 强制恢复避障.
                # 因未知真实角度, 从避障自己冻结的角度起步, 可能有轻微跳变.
                self._resync_pending = False
                self.get_logger().warn(
                    f'接管释放 {RESYNC_TIMEOUT:.0f}s 仍未收到 /joint_states 重同步, '
                    '强制恢复避障 (可能轻微跳变; 请检查 joint_state 桥是否在跑)')
            else:
                return
        # 启动 5s 仍未同步关节角 -> 提示一次 (可能 bridge 没起或话题名不对)
        if not self._joint_synced and not self._sync_warned and time.time() - self._boot_time > 5.0:
            self._sync_warned = True
            self.get_logger().warn(
                '未收到 /joint_states 的 base_yaw_joint 角度, 从 0 起步; '
                '若镜头回跳请检查 start_simulation.sh 的 joint_state 桥')
        act = self.rule_action
        if self.llm_action:
            act = self._validate(self.llm_action)
        self._apply_action(act)

        if self.rotate_dir != 0.0:
            # 无极累加目标角 (不 wrap, 跨 ±π 无突跳), 由平滑器限速输出
            self.smooth.step_delta(self.rotate_dir * ROTATE_STEP)

        # idle 静默: 无障碍/不转向时不发布, 让位给键盘等其它控制器独占话题
        if self.rotate_dir == 0.0 and abs(self.smooth.target - self.smooth.pos) < 1e-6:
            return

        msg = Float64()
        msg.data = self.smooth.tick()
        self.cmd_pub.publish(msg)

    # ---------------- LLM (本地 Ollama / 云端 OpenAI 兼容) ----------------
    def _ask_llm_async(self, objects):
        with self._llm_lock:
            if self._llm_busy:
                return
            self._llm_busy = True
        image = self.latest_image
        threading.Thread(target=self._llm_worker, args=(objects, image), daemon=True).start()

    def _llm_worker(self, objects, image):
        action = None
        try:
            if self.provider == 'openai':
                action = self._ask_openai(objects, image)
            elif self.provider == 'failover':
                action = self._ask_failover(objects, image)
            else:
                action = self._ask_ollama(objects, image)
            if action:
                self.get_logger().info(f'模型决策: {action}')
        except Exception as e:
            now = time.time()
            if now - self._last_fail_log > 10:
                self._last_fail_log = now
                self.get_logger().warn(f'模型调用失败, 本次使用规则: {e}')
        finally:
            with self._llm_lock:
                self.llm_action = action
                self.llm_action_time = time.time()
                self._llm_busy = False

    @staticmethod
    def _fmt_dist(d):
        """距离转文本: 测不到写 none, 其余保留两位小数(单位米)。"""
        return 'none' if not math.isfinite(d) else f'{d:.2f}'

    def _prompt(self, objects):
        left = self._side_count(objects, 'left')
        right = self._side_count(objects, 'right')
        center = len(objects) - left - right
        # 多物体: 明细位置 + 深度测距一起喂给模型 —— 不光"有没有", 还要"多远"
        parts = []
        for i, (name, cx) in enumerate(objects[:12]):
            d = self._object_dists[i] if i < len(self._object_dists) else math.inf
            parts.append(f'{name}@x={cx:.2f},dist={self._fmt_dist(d)}m')
        detail = '; '.join(parts) or 'none'
        if self._depth_ok():
            zl, zc, zr = self.depth_zones
            depth_note = (
                f'Depth camera zones (nearest obstacle per zone, meters, none=clear): '
                f'left={self._fmt_dist(zl)}, center={self._fmt_dist(zc)}, right={self._fmt_dist(zr)}. '
                f'An obstacle closer than 0.5m means that side is blocked. '
            )
        else:
            depth_note = 'No depth data available. '
        return (
            'You control a robot camera. Decide which direction to turn to keep the safest view. '
            f'Obstacle counts: left={left}, right={right}, center={center}. '
            f'Detailed obstacles (name@x-position, 0=far left, 0.5=center, 1=far right): {detail}. '
            f'{depth_note}'
            'Left < 0.4, right > 0.6, center in between. '
            'Pick the side with fewer AND farther obstacles (avoid clusters); '
            'never choose a side that is blocked closer than 0.5m. '
            'Reply with EXACTLY one token: TURN_LEFT or TURN_RIGHT.'
        )

    @staticmethod
    def _parse(text):
        m = re.search(r'TURN_(LEFT|RIGHT)', (text or '').upper().replace('-', '_'))
        return m.group(0) if m else None

    @staticmethod
    def _vision_on(model):
        """模型是否带视觉: LLM_VISION=0/1 强制, 否则按模型名自动判断."""
        v = os.environ.get('LLM_VISION', '').strip().lower()
        if v in ('0', 'false', 'no'):
            return False
        if v in ('1', 'true', 'yes'):
            return True
        m = (model or '').lower()
        keys = ('llava', 'vl', 'vision', 'gpt-4o', 'gpt-4.1', 'gpt-5',
                'gemini', 'moondream', 'glm-4v', 'qwen-vl', 'o1', 'o3', 'o4')
        return any(k in m for k in keys)

    @staticmethod
    def _encode_image(img):
        if img is None:
            return None
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if not ok:
            return None
        return base64.b64encode(buf).decode('utf-8')

    def _ask_ollama(self, objects, image):
        base = os.environ.get('OLLAMA_URL', 'http://localhost:11434').rstrip('/')
        model = os.environ.get('OLLAMA_MODEL', 'llava:7b')
        # 只有视觉模型才附图片; 纯文本模型(llama3.2/deepseek 等)只看 YOLO 统计
        b64 = self._encode_image(image) if self._vision_on(model) else None
        payload = {
            'model': model,
            'prompt': self._prompt(objects),
            'images': [b64] if b64 else None,
            'stream': False,
            'options': {'temperature': 0, 'num_predict': 20},
        }
        timeout = float(os.environ.get('OLLAMA_TIMEOUT', '120'))
        r = requests.post(base + '/api/generate', json=payload, timeout=timeout)
        r.raise_for_status()
        return self._parse(r.json().get('response', ''))

    def _ask_failover(self, objects, image):
        """本地优先, 失败自动切云端 (故障转移)。

        本地 (LLM_LOCAL_URL, 默认 LM Studio) 异常/超时/无有效输出 -> 云端 (LLM_CLOUD_*);
        云端也未配置或失败 -> 抛异常, 上层回落纯规则, 节点永远可用。
        """
        try:
            action = self._ask_openai_compat(
                objects, image,
                base=os.environ.get('LLM_LOCAL_URL', 'http://127.0.0.1:1234/v1').rstrip('/'),
                key=os.environ.get('LLM_LOCAL_KEY', 'lm-studio'),
                model=os.environ.get('LLM_LOCAL_MODEL', 'qwen2.5-7b-instruct-1m'),
                timeout=float(os.environ.get('LLM_LOCAL_TIMEOUT', '30')))
            if action:
                return action
            raise RuntimeError('本地模型未输出有效指令')
        except Exception as e:
            now = time.time()
            if now - self._last_fail_log > 10:
                self._last_fail_log = now
                self.get_logger().warn(f'本地 LLM 不可用, 尝试云端兜底: {e}')
        key = os.environ.get('LLM_CLOUD_KEY', '')
        if not key:
            raise RuntimeError('未配置云端 key (LLM_CLOUD_KEY), 跳过云端')
        return self._ask_openai_compat(
            objects, image,
            base=os.environ.get('LLM_CLOUD_URL', 'https://api.openai.com/v1').rstrip('/'),
            key=key,
            model=os.environ.get('LLM_CLOUD_MODEL', 'gpt-4o-mini'),
            timeout=float(os.environ.get('LLM_CLOUD_TIMEOUT', '30')))

    def _ask_openai(self, objects, image):
        return self._ask_openai_compat(
            objects, image,
            base=os.environ.get('OPENAI_BASE_URL', 'https://api.openai.com/v1').rstrip('/'),
            key=os.environ.get('OPENAI_API_KEY', ''),
            model=os.environ.get('OPENAI_MODEL', 'gpt-4o-mini'),
            timeout=float(os.environ.get('OPENAI_TIMEOUT', '30')))

    def _ask_openai_compat(self, objects, image, base, key, model, timeout):
        content = [{'type': 'text', 'text': self._prompt(objects)}]
        b64 = self._encode_image(image)
        if b64 and self._vision_on(model):
            content.append({'type': 'image_url', 'image_url': {'url': f'data:image/jpeg;base64,{b64}'}})
        payload = {
            'model': model,
            'messages': [{'role': 'user', 'content': content}],
            'max_tokens': 20,
            'temperature': 0,
        }
        headers = {'Authorization': f'Bearer {key}', 'Content-Type': 'application/json'}
        timeout = float(os.environ.get('OPENAI_TIMEOUT', '30'))
        r = requests.post(base + '/chat/completions', json=payload, headers=headers, timeout=timeout)
        r.raise_for_status()
        return self._parse(r.json()['choices'][0]['message']['content'])

    # ---------------- 关闭 ----------------
    def destroy_node(self):
        self.get_logger().info('避障节点退出')
        super().destroy_node()


def main():
    rclpy.init()
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
