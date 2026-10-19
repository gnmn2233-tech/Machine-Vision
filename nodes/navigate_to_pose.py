#!/usr/bin/env python3
"""目标点导航节点: 输入坐标 -> 机器人移动到坐标处。

与本仓库底盘结构严格对应 (robot_with_camera.sdf):
  world --drive_x_joint(prismatic ±9m)--> chassis_x
        --drive_y_joint(prismatic ±9m)--> chassis_y
        --base_yaw_joint(revolute)     --> base

底盘是【位置伺服】不是差速轮: 没有 /cmd_vel, 只有
  /drive_x/commands  std_msgs/Float64   (JointPositionController, p=30 d=10)
  /drive_y/commands  std_msgs/Float64   (同上)
  /camera_joint_controller/commands     (控制 base_yaw_joint, p=15 d=20)
所以本节点的控制量 = "下一个位置指令"。

运动律与 keyboard_joint_controller.py 完全一致 —— 底盘只会「原地转向」和「沿车头前进」:
  1) 目标方向与车头夹角 > heading_gate (默认 8.6 deg) -> 先原地转向对准, 期间不前进;
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
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Pose2D, PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String

# ---- 与 SDF 一致 ----
JOINT_X = 'drive_x_joint'
JOINT_Y = 'drive_y_joint'
JOINT_YAW = 'base_yaw_joint'
DRIVE_LIMIT = 9.0          # prismatic 行程 ±9 m (世界地面 20x20)
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
NEAR_FIELD = 0.30          # 进入该距离 (m) 后锁定目标方向, 免得终点附近方向抖动
MAX_LEAD = 0.25            # 指令最多超前实测多少 (m): 这就是伺服滞后时可能冲过的距离
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
        self.declare_parameter('face_motion', False)   # 已废弃: 现在总是朝运动方向
        self.declare_parameter('use_orientation', False)

        self.v_max = float(self.get_parameter('v_max').value)
        self.a_max = float(self.get_parameter('a_max').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.arrive_eps = float(self.get_parameter('arrive_eps').value)
        self.yaw_eps = float(self.get_parameter('yaw_eps').value)
        self.hz = float(self.get_parameter('control_hz').value)
        self.goal_timeout = float(self.get_parameter('goal_timeout').value)
        self.heading_gate = float(self.get_parameter('heading_gate').value)
        self.turn_gain = float(self.get_parameter('turn_gain').value)
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
        self.pub_status = self.create_publisher(String, '/navigation_status', 10)
        self.pub_reached = self.create_publisher(Bool, '/nav_goal_reached', 10)

        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal_pose, 10)
        self.create_subscription(Pose2D, '/navigate_to', self.on_goal_pose2d, 10)
        self.create_subscription(Bool, '/nav_cancel', lambda m: self.cancel('外部取消'), 10)

        self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            f'Навигация запущена: v_max={self.v_max} м/с, допуск={self.arrive_eps} м, '
            f'частота={self.hz:.0f} Гц')
        self.get_logger().info(
            'Куда ехать: аргументы "x y" / ввод в терминале "x y [угол]" / /goal_pose или /navigate_to')

    # ================= 输入 =================
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
        """目标合法性检查: prismatic 行程 ±9 m。"""
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
        self.state = self.MOVING
        self.goal_start = time.time()
        self.steal_since = None
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
        if self._override_on:
            self._override_on = False
            self.pub_override.publish(Float64(data=0.0))
        self.pub_reached.publish(Bool(data=False))
        self._publish_status('отменено')

    # ================= 主循环 =================
    def on_timer(self):
        if not self.got_joints:
            return
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
        # 近场锁定方向: 距离很小时 dx/dy 抖动会让 atan2 乱跳, 车头会在终点前乱转
        if dist > NEAR_FIELD or self.want_heading is None:
            if dist > 1e-6:
                self.want_heading = math.atan2(dy, dx)
        err = wrap_pi(self.want_heading - self.yaw.value)  # 目标方向 vs 车头
        arrived_yaw = True

        if not arrived_xy and abs(err) > self.heading_gate:
            # 车头偏得太多: 先原地转向对准, 期间不前进
            phase = f'поворот (отклонение {math.degrees(err):+.0f} deg)'
            w = min(self.w_max, max(math.sqrt(4.0 * abs(err)), 0.05))
            self.cmd_yaw += math.copysign(min(w * self.dt, abs(err)), err)
        elif not arrived_xy:
            # 已对准: 沿车头直行, 边走边小角度修正 (车头始终朝运动方向)
            phase = 'едем'
            v = min(self.v_max, max(math.sqrt(2.0 * self.a_max * dist_cmd), 0.03))
            if dist_cmd < 0.5:
                v = min(v, 0.18)                    # 近场慢速靠拢, 避免冲过头
            step = min(v * self.dt, dist_cmd)       # 指令不越过目标
            self.cmd_x += math.cos(self.cmd_yaw) * step
            self.cmd_y += math.sin(self.cmd_yaw) * step
            w = max(-self.w_max, min(self.w_max, self.turn_gain * err))
            self.cmd_yaw += w * self.dt
        else:
            phase = 'на месте'

        # 到点后若指定了朝向, 原地转到该朝向
        if arrived_xy and gyaw is not None:
            tgt = self.yaw.nearest(gyaw)
            yerr = wrap_pi(tgt - self.yaw.value)
            arrived_yaw = abs(yerr) <= self.yaw_eps
            if not arrived_yaw:
                w = min(self.w_max, max(math.sqrt(4.0 * abs(yerr)), 0.05))
                self.cmd_yaw += math.copysign(min(w * self.dt, abs(yerr)), yerr)
                phase = f'доворот (отклонение {math.degrees(yerr):+.0f} deg)'
            else:
                self.cmd_yaw = tgt

        # 指令不得超前实测太多: 伺服滞后/定位断流时, 超前量就是会冲过头的距离
        lead = math.hypot(self.cmd_x - self.x, self.cmd_y - self.y)
        if lead > MAX_LEAD:
            k = MAX_LEAD / lead
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
        self._override_on = True
        self.pub_override.publish(Float64(data=1.0))     # 运动期间由导航接管
        self._last_cmd_x, self._last_cmd_y, self._last_cmd_yaw = cmd_x, cmd_y, yaw_cmd

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
            self.goal = None
            self._start_next()                       # 队列里还有就继续走
            return

        # ---- 1 Hz 进度日志 ----
        t = time.time()
        if t - self._status_log_t >= 1.0:
            self._status_log_t = t
            speed = min(self.v_max, max(math.sqrt(2.0 * self.a_max * dist), 0.03))
            self.get_logger().info(
                f'→ {phase} осталось {dist:.2f} м, скорость {speed:.2f} м/с '
                f'| 当前 ({self.x:+.2f}, {self.y:+.2f}) yaw={math.degrees(self.yaw.value):+.0f} deg'
                f' -> цель ({gx:+.2f}, {gy:+.2f})')
            self._publish_status(phase)

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

Метры, начало координат - место запуска симуляции; ход шасси ±9 м.""".strip()


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
