#!/usr/bin/env python3
"""A->B 导航节点 (go-to-goal)。

输入:
  /robot_pose                 geometry_msgs/PoseStamped  当前位姿 (由 robot_pose_monitor.py 发布, frame: odom)
  /goal_pose                  geometry_msgs/PoseStamped  目标点 (RViz 工具栏 "2D Goal Pose" 默认就是这个话题)
  /depth_camera/depth_image   sensor_msgs/Image          32FC1 深度图 (可选, 用于前方急停)

输出:
  /camera_joint_controller/commands    std_msgs/Float64  偏航目标角 (无回卷, 无限累加, 与键盘控制器同一套接口)
  /drive_x/commands /drive_y/commands  std_msgs/Float64  世界系平移位置指令 (prismatic, 限幅 ±drive_limit)
  /nav_status                          std_msgs/String   IDLE / ALIGN / DRIVE / BLOCKED / ARRIVED
  /goal_marker                         visualization_msgs/Marker  目标点可视化

控制律 (50Hz):
  1) dist < goal_tol                        -> 到点, 停, ARRIVED
  2) |朝向误差| > yaw_tol                   -> ALIGN: 原地对准 (停止平移)
  3) 否则                                    -> DRIVE: 沿当前朝向前进, 接近目标线性减速
  4) use_depth 且 前方扇区深度 < stop_dist   -> BLOCKED: 停车, 往更空旷的一侧转

注意: 无目标时 (IDLE) 本节点不发布任何关节指令, 不会和键盘控制器抢控制权;
      收到目标 /goal_pose 的那一刻才接管, 并把内部指令同步到当前实测位姿, 不会产生跳变。
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String
from visualization_msgs.msg import Marker

IDLE, ALIGN, DRIVE, BLOCKED, ARRIVED = 'IDLE', 'ALIGN', 'DRIVE', 'BLOCKED', 'ARRIVED'
MANUAL = 'MANUAL'          # 键盘手动接管中, 导航让出控制权


def wrap(a):
    """角度归一化到 [-pi, pi]"""
    return math.atan2(math.sin(a), math.cos(a))


class Slew:
    """限速 + 限加速度的梯形速度规划器 (与 keyboard_joint_controller.py 的 SmoothYaw 同款)。"""

    def __init__(self, max_rate, accel, dt):
        self.target = 0.0
        self.pos = 0.0
        self.vel = 0.0
        self.max_rate = max_rate
        self.accel = accel
        self.dt = dt

    def set_target(self, t):
        self.target = t

    def jump_to(self, v):
        """无平滑地直接置位 (只在同步实测状态时使用)"""
        self.target = v
        self.pos = v
        self.vel = 0.0

    def tick(self):
        err = self.target - self.pos
        if abs(err) < 1e-9:
            self.vel = 0.0
            return self.pos
        direction = 1.0 if err > 0.0 else -1.0
        dist = abs(err)
        stop_dist = self.vel * self.vel / (2.0 * self.accel)
        v_des = math.sqrt(max(2.0 * self.accel * dist, 0.0)) if dist <= stop_dist else self.max_rate
        if direction * self.vel >= 0.0:
            self.vel += direction * self.accel * self.dt
        else:
            self.vel -= direction * 2.0 * self.accel * self.dt
        if abs(self.vel) > v_des:
            self.vel = direction * v_des
        self.pos += self.vel * self.dt
        return self.pos


class GoalNavigator(Node):
    DT = 0.02

    def __init__(self):
        super().__init__('goal_navigator')

        # ---------------- 参数 ----------------
        self.max_speed = float(self.declare_parameter('max_speed', 0.25).value)
        self.yaw_tol = float(self.declare_parameter('yaw_tol', 0.15).value)
        self.goal_tol = float(self.declare_parameter('goal_tol', 0.20).value)
        self.stop_dist = float(self.declare_parameter('stop_dist', 0.50).value)
        self.slow_radius = float(self.declare_parameter('slow_radius', 0.60).value)
        self.use_depth = bool(self.declare_parameter('use_depth', True).value)
        self.block_commit = float(self.declare_parameter('block_commit', 1.2).value)
        # 深度图话题: 本仓库用 /depth_camera (32FC1, 米); 旧版 rgbd 相机则是 /depth_camera/depth_image
        self.depth_topic = str(self.declare_parameter('depth_topic', '/depth_camera').value)
        # 键盘手动接管信号 (键盘控制器空闲时发 0, 按键期间发 1)
        self.override_topic = str(self.declare_parameter('override_topic', '/avoider_override').value)
        self._manual_active = False
        self.drive_limit = float(self.declare_parameter('drive_limit', 9.0).value)
        self.auto_goal = bool(self.declare_parameter('auto_goal', False).value)
        self.goal_x = float(self.declare_parameter('goal_x', 0.0).value)
        self.goal_y = float(self.declare_parameter('goal_y', 0.0).value)
        # Gazebo 世界里的目标球 (点目标时生成, 到达后变蓝)
        self.spawn_marker = bool(self.declare_parameter('spawn_marker', True).value)
        self.marker_name = str(self.declare_parameter('marker_name', 'nav_goal_marker').value)
        self.marker_world = str(self.declare_parameter('marker_world', 'default').value)
        # 球心高度: 必须让球完全避开深度相机的避障行带 (详见 _marker_sdf 注释), 0.70 是实测安全值
        self.marker_z = float(self.declare_parameter('marker_z', 0.70).value)

        # ---------------- 状态 ----------------
        self.pose = None            # (x, y, yaw) 实测
        self.goal = None            # (gx, gy)
        self.depth = None           # np.ndarray (h, w) float32, 单位 m
        self.status = IDLE
        self.cmd_x = 0.0
        self.cmd_y = 0.0
        self.block_dir = 0.0
        self._block_eval_t = -1e9
        self._blocked_until = -1e9
        self._front_min = None
        self._last_log = 0.0
        self._pending_marker = None
        self._marker_alive = False
        self._marker_warned = False
        self._marker_gen = 0

        # ---------------- 限速器 ----------------
        self.yaw_cmd = Slew(max_rate=1.2, accel=6.0, dt=self.DT)
        # 平移限速器的速率上限略高于 max_speed, 让它可以追上目标而不成为瓶颈
        slew_rate = max(self.max_speed * 1.5, 0.4)
        self.x_cmd = Slew(max_rate=slew_rate, accel=0.8, dt=self.DT)
        self.y_cmd = Slew(max_rate=slew_rate, accel=0.8, dt=self.DT)

        # ---------------- 接口 ----------------
        self.create_subscription(PoseStamped, '/robot_pose', self.on_pose, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_subscription(Image, self.depth_topic, self.on_depth, 5)
        self.create_subscription(Float64, self.override_topic, self.on_override, 10)

        self.yaw_pub = self.create_publisher(Float64, '/camera_joint_controller/commands', 10)
        self.x_pub = self.create_publisher(Float64, '/drive_x/commands', 10)
        self.y_pub = self.create_publisher(Float64, '/drive_y/commands', 10)
        self.status_pub = self.create_publisher(String, '/nav_status', 10)
        self.marker_pub = self.create_publisher(Marker, '/goal_marker', 10)

        # Gazebo 实体服务 (在 world 里真的生成一个目标球)
        self.spawn_cli = self.create_client(
            SpawnEntity, f'/world/{self.marker_world}/create')
        self.del_cli = self.create_client(
            DeleteEntity, f'/world/{self.marker_world}/remove')

        self.create_timer(self.DT, self.tick)
        self.get_logger().info(
            f'A->B 导航节点启动: max_speed={self.max_speed}m/s yaw_tol={self.yaw_tol}rad '
            f'goal_tol={self.goal_tol}m stop_dist={self.stop_dist}m use_depth={self.use_depth}')
        self.get_logger().info('发目标: RViz 里用 "2D Goal Pose" 点一下, 或 ros2 topic pub --once /goal_pose ...')

        if self.auto_goal:
            self.set_goal(self.goal_x, self.goal_y)

    # ---------------- 回调 ----------------
    def on_pose(self, msg):
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        first = self.pose is None
        self.pose = (x, y, yaw)
        if first:
            # 首次拿到位姿: 内部指令直接对齐实测, 保证上电瞬间不产生跳变指令
            self.cmd_x, self.cmd_y = x, y
            self.x_cmd.jump_to(x)
            self.y_cmd.jump_to(y)
            self.yaw_cmd.jump_to(yaw)
            self.get_logger().info(f'已收到位姿: x={x:+.2f} y={y:+.2f} yaw={math.degrees(yaw):+.1f}deg')

    def on_goal(self, msg):
        fid = msg.header.frame_id
        if fid not in ('', 'odom', 'map', 'world'):
            self.get_logger().warn(f'目标点坐标系 {fid!r} 不是 odom, 仍按 odom 处理')
        self.set_goal(msg.pose.position.x, msg.pose.position.y)

    def on_depth(self, msg):
        try:
            if msg.encoding != '32FC1':
                return
            arr = np.frombuffer(msg.data, dtype=np.float32)
            row = msg.step // 4
            if row <= 0 or arr.size < row * msg.height:
                return
            self.depth = arr.reshape(msg.height, row)[:, :msg.width]
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'深度图解析失败: {e}', once=True)

    def on_override(self, msg):
        """键盘手动接管信号: 1 = 用户在按键手动开, 导航暂停让路; 0 = 交还, 导航继续。"""
        active = msg.data > 0.5
        if active != self._manual_active:
            self._manual_active = active
            self.get_logger().info(
                '检测到键盘手动接管, 导航暂停 (松开键盘后自动继续)' if active
                else '键盘已交还控制权, 导航继续')

    # ---------------- 目标 ----------------
    def set_goal(self, gx, gy):
        self.goal = (float(gx), float(gy))
        if self.pose is not None:
            x, y, yaw = self.pose
            self.cmd_x, self.cmd_y = x, y
            self.x_cmd.jump_to(x)
            self.y_cmd.jump_to(y)
            self.yaw_cmd.jump_to(yaw)
        self.status = IDLE
        self.block_dir = 0.0
        self._block_eval_t = -1e9
        self._blocked_until = -1e9
        self.get_logger().info(f'收到新目标: ({gx:+.2f}, {gy:+.2f})')
        self._publish_marker()
        self._update_marker(gx, gy, arrived=False)

    # ---------------- 深度 ----------------
    def _sector_depth(self, c0, c1):
        """取图像中间一条横带的深度低分位数 (米), 无效返回 None"""
        if self.depth is None:
            return None
        h, w = self.depth.shape
        band = self.depth[int(h * 0.40):int(h * 0.85), int(w * c0):int(w * c1)]
        valid = band[np.isfinite(band) & (band > 0.05)]
        if valid.size < 20:
            return None
        return float(np.percentile(valid, 5))

    def _front_blocked(self):
        if not self.use_depth:
            return False
        d = self._sector_depth(0.35, 0.65)
        self._front_min = d
        return d is not None and d < self.stop_dist

    def _blocked_now(self):
        """前方是否受阻。带滞回: 一旦判定受阻, 强制让开至少 block_commit 秒再重新判断,
        避免 BLOCKED/DRIVE 高频抖动、边转边蹭着往障碍上挪。"""
        now = self.get_clock().now().nanoseconds / 1e9
        front_blocked = self._front_blocked()   # 顺便刷新 _front_min 供日志使用
        if now < self._blocked_until:
            return True
        if front_blocked:
            self._blocked_until = now + self.block_commit
            return True
        return False

    # ---------------- 控制 ----------------
    def _hold(self, x, y):
        """停止平移: 把指令钉在当前实测位置 (限速器负责平滑刹车)"""
        self.cmd_x, self.cmd_y = x, y
        self.x_cmd.set_target(x)
        self.y_cmd.set_target(y)

    def _steer(self, yaw, desired):
        """把偏航指令朝目标方向转, 取离当前指令最近的等效角, 保证连续不跳变"""
        tgt_abs = yaw + wrap(desired - yaw)
        self.yaw_cmd.set_target(self.yaw_cmd.pos + wrap(tgt_abs - self.yaw_cmd.pos))

    def _drive_step(self, x, y, yaw, v):
        step = v * self.DT
        self.cmd_x += math.cos(yaw) * step
        self.cmd_y += math.sin(yaw) * step
        # 防漂移: 指令与实测偏差过大时向实测收敛 (限速器会把这个修正平滑掉)
        if abs(self.cmd_x - x) > 0.15:
            self.cmd_x += 0.1 * (x - self.cmd_x)
        if abs(self.cmd_y - y) > 0.15:
            self.cmd_y += 0.1 * (y - self.cmd_y)
        self.cmd_x = max(-self.drive_limit, min(self.drive_limit, self.cmd_x))
        self.cmd_y = max(-self.drive_limit, min(self.drive_limit, self.cmd_y))
        self.x_cmd.set_target(self.cmd_x)
        self.y_cmd.set_target(self.cmd_y)

    def _avoid(self, yaw):
        """前方受阻: 原地朝更空旷的一侧转; 每次让开动作结束后才重新评估左右"""
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._block_eval_t >= self.block_commit:
            self._block_eval_t = now
            left = self._sector_depth(0.05, 0.35)
            right = self._sector_depth(0.65, 0.95)
            l = left if left is not None else 0.0
            r = right if right is not None else 0.0
            self.block_dir = 1.0 if l >= r else -1.0
            self.get_logger().warn(
                f'前方受阻 (正前={-1 if self._front_min is None else self._front_min:.2f}m), '
                f'向{"左" if self.block_dir > 0 else "右"}侧让开 (左={l:.2f}m 右={r:.2f}m)')
        self.yaw_cmd.set_target(self.yaw_cmd.pos + self.block_dir * 0.8 * self.DT)

    def tick(self):
        status = IDLE
        if self._manual_active:
            # 键盘正在手动控制: 只跟随实测, 不发布任何指令(把话题让给键盘)
            status = MANUAL
            if self.pose is not None:
                self._hold(self.pose[0], self.pose[1])
        elif self.pose is not None and self.goal is not None:
            x, y, yaw = self.pose
            gx, gy = self.goal
            dist = math.hypot(gx - x, gy - y)
            desired = math.atan2(gy - y, gx - x)

            if dist < self.goal_tol:
                status = ARRIVED
                self._hold(x, y)
                self.goal = None
                self.get_logger().info(f'已到达目标 ({gx:+.2f}, {gy:+.2f}), 实际 ({x:+.2f}, {y:+.2f})')
                self._update_marker(gx, gy, arrived=True)
            elif self._blocked_now():
                status = BLOCKED
                self._hold(x, y)
                self._avoid(yaw)
            elif abs(wrap(desired - yaw)) > self.yaw_tol:
                status = ALIGN
                self._hold(x, y)
                self._steer(yaw, desired)
            else:
                status = DRIVE
                v = self.max_speed
                if dist < self.slow_radius:
                    v = max(self.max_speed * dist / self.slow_radius, 0.06)
                self._drive_step(x, y, yaw, v)
                self._steer(yaw, desired)

        if status != self.status:
            self.status = status
        self.publish()

    # ---------------- 发布 ----------------
    def publish(self):
        if self.status in (IDLE, MANUAL):
            # 空闲 / 键盘手动接管: 不发布关节指令(避免抢话题), 内部状态跟随实测
            if self.pose is not None:
                x, y, yaw = self.pose
                self.cmd_x, self.cmd_y = x, y
                self.x_cmd.jump_to(x)
                self.y_cmd.jump_to(y)
                self.yaw_cmd.jump_to(yaw)
        else:
            self.yaw_pub.publish(Float64(data=self.yaw_cmd.tick()))
            self.x_pub.publish(Float64(data=self.x_cmd.tick()))
            self.y_pub.publish(Float64(data=self.y_cmd.tick()))

        m = String()
        m.data = self.status
        self.status_pub.publish(m)

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_log > 1.0:
            self._last_log = now
            if self.pose is not None:
                x, y, yaw = self.pose
                extra = ''
                if self.goal is not None:
                    extra = (f' 目标=({self.goal[0]:+.2f},{self.goal[1]:+.2f}) '
                             f'距离={math.hypot(self.goal[0] - x, self.goal[1] - y):.2f}m')
                if self._front_min is not None:
                    extra += f' 正前方={self._front_min:.2f}m'
                self.get_logger().info(
                    f'[{self.status}] x={x:+.2f} y={y:+.2f} yaw={math.degrees(yaw):+.1f}deg{extra}')
            if self.goal is not None:
                self._publish_marker()

    # ---------------- Gazebo 世界里的目标球 ----------------
    def _marker_sdf(self, name, color):
        """Gazebo 里的目标标记: 只有一个球体(无碰撞), 球心高度 marker_z (默认 0.70m)。

        为什么球是"悬空"的 —— 深度相机装在 z=0.28m、水平前视, 避障只用图像 40%~85% 行带:
            行带下沿 z = 0.28 - 0.30*d      行带上沿 z = 0.28 + 0.09*d
        · 球心 0.70m / 半径 0.12m: 球底 0.58m, 在 d<3.5m 时恒在行带上沿之上;
          更远时深度 >3.5m, 也远大于 stop_dist(0.5m) -> 永远不会被当成障碍触发急停。
        · 如果把球放到地上 (球顶 0.20m), 机器人开到 0.45m 处时球正好落进行带,
          深度 0.45m < stop_dist 0.5m -> 机器人会被自己放的目标球卡死在离目标 0.7m 处 (实测踩过)。
        """
        r, g, b = color
        mat = f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>'
        return (
            "<?xml version='1.0'?>"
            "<sdf version='1.10'>"
            f"<model name='{name}'><static>true</static><link name='link'>"
            f"<visual name='ball'><pose>0 0 {self.marker_z} 0 0 0</pose>"
            f"<geometry><sphere><radius>0.12</radius></sphere></geometry>{mat}</visual>"
            "</link></model></sdf>"
        )

    def _update_marker(self, x, y, arrived=False):
        """生成/更新 Gazebo 里的目标球: 先删旧(必须带 type=MODEL) 再建新。

        注意: /world/<world>/remove 服务如果只给 name 不给 type, Gazebo 会返回 success=True
        却什么都不删; 随后同名 spawn 也不会真正重建 -> 表现为"只有第一个球能出现"。
        """
        if not self.spawn_marker:
            return
        self._marker_gen += 1
        gen = self._marker_gen
        self._pending_marker = (float(x), float(y), bool(arrived), gen)
        if self.del_cli.service_is_ready():
            req = DeleteEntity.Request()
            req.entity.name = self.marker_name
            req.entity.type = Entity.MODEL          # <<< 关键: 不给类型则删除无效
            self.del_cli.call_async(req).add_done_callback(
                lambda _f: self._do_spawn_marker(gen))
        else:
            self._do_spawn_marker(gen)

    def _do_spawn_marker(self, gen):
        if self._pending_marker is None or self._pending_marker[3] != gen:
            return                                       # 已有更新的目标, 丢弃这次
        x, y, arrived, _ = self._pending_marker
        if not self.spawn_cli.service_is_ready():
            if not self._marker_warned:
                self._marker_warned = True
                self.get_logger().warn(
                    f'未找到 /world/{self.marker_world}/create 服务, 跳过 Gazebo 目标球'
                    ' (需重启 start_simulation.sh 启用实体服务桥接; 导航本身不受影响)')
            return
        color = (0.10, 0.45, 1.00) if arrived else (0.10, 1.00, 0.25)
        req = SpawnEntity.Request()
        req.entity_factory.name = self.marker_name
        req.entity_factory.allow_renaming = False
        req.entity_factory.sdf = self._marker_sdf(self.marker_name, color)
        req.entity_factory.pose.position.x = x
        req.entity_factory.pose.position.y = y
        req.entity_factory.pose.position.z = 0.0
        req.entity_factory.pose.orientation.w = 1.0
        req.entity_factory.relative_to = 'world'
        self.spawn_cli.call_async(req).add_done_callback(
            lambda f: self._on_marker_done(f, gen))

    def _on_marker_done(self, fut, gen):
        if gen != self._marker_gen:
            return
        try:
            res = fut.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'Gazebo 目标球生成异常: {e}')
            return
        if res is not None and res.success:
            self._marker_alive = True
            x, y, arrived, _ = self._pending_marker
            self.get_logger().info(
                f'Gazebo 目标球已放到 ({x:+.2f}, {y:+.2f})' + (' [到达, 已变蓝]' if arrived else ''))
        else:
            self.get_logger().warn('Gazebo 目标球生成失败, 已忽略')

    def _publish_marker(self):
        if self.goal is None:
            return
        mk = Marker()
        mk.header.frame_id = 'odom'
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns = 'goal'
        mk.id = 0
        mk.type = Marker.SPHERE
        mk.action = Marker.ADD
        mk.pose.position.x = self.goal[0]
        mk.pose.position.y = self.goal[1]
        mk.pose.position.z = 0.10
        mk.pose.orientation.w = 1.0
        mk.scale.x = mk.scale.y = mk.scale.z = 0.25
        mk.color.r = 0.1
        mk.color.g = 1.0
        mk.color.b = 0.2
        mk.color.a = 0.9
        self.marker_pub.publish(mk)


def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
