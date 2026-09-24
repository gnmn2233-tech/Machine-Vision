#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from pynput import keyboard
import threading
import math
import time


class SmoothYaw:
    """无极丝滑转向器 (与 vlm_camera_rotator.py / obstacle_avoider.py 中的实现保持一致)。

    - 角度永不回卷(不 %2π): 目标角无限累积, revolute 关节可连续转任意圈,
      跨 ±π 边界零突跳 —— 这是"无极转向"的关键。
    - 发布角以 限速+限加速度 的梯形速度规划逼近目标: 恒速转动、远端减速
      缓停, 指令无阶跃 —— 这是"丝滑转向"的关键。
    """

    def __init__(self, max_rate=1.0, accel=5.0, dt=0.02):
        self.target = 0.0        # 目标角 (unwrapped, 无限累积)
        self.pos = 0.0           # 已发布角 (unwrapped)
        self.vel = 0.0           # 当前转动速率 (rad/s)
        self.max_rate = max_rate
        self.accel = accel
        self.dt = dt             # 每 tick 的时间步长 (s)

    def step_delta(self, delta):
        """目标角增量 (不 wrap, 可无限累加)"""
        self.target += delta

    def set_target(self, t):
        self.target = t

    def tick(self):
        """每周期调用一次, 返回下一帧要发布的平滑角度"""
        err = self.target - self.pos
        if abs(err) < 1e-9:
            self.vel = 0.0
            return self.pos
        direction = 1.0 if err > 0.0 else -1.0
        dist = abs(err)
        # 以当前速率减速到 0 所需距离 (v²/2a), 进入该区间开始减速缓停
        stop_dist = self.vel * self.vel / (2.0 * self.accel)
        if dist <= stop_dist:
            v_des = math.sqrt(max(2.0 * self.accel * dist, 0.0))
        else:
            v_des = self.max_rate
        # 加速度限制: 同向加速, 方向反转时快速刹车
        if direction * self.vel >= 0.0:
            self.vel += direction * self.accel * self.dt
        else:
            self.vel -= direction * 2.0 * self.accel * self.dt
        if abs(self.vel) > v_des:
            self.vel = direction * v_des
        self.pos += self.vel * self.dt
        return self.pos


class KeyboardJointController(Node):
    def __init__(self):
        super().__init__('keyboard_joint_controller')
        self.publisher = self.create_publisher(Float64, '/camera_joint_controller/commands', 10)
        self.override_pub = self.create_publisher(Float64, '/avoider_override', 10)
        self.drive_x_pub = self.create_publisher(Float64, '/drive_x/commands', 10)
        self.drive_y_pub = self.create_publisher(Float64, '/drive_y/commands', 10)
        # 订阅真实关节角: 接管避障瞬间从此角度起步, 避免位置跳变
        qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.js_sub = self.create_subscription(JointState, '/joint_states', self.on_joint_states, qos)
        self._real_angle = 0.0
        self._real_seen = False      # 是否收到过真实关节角 (没收到时接管不能用 0 锚位!)
        self._real_warned = False
        self.step_per_tick = 0.02   # 按住时每 tick(50Hz) 的目标角增量 → 恒速 1.0 rad/s
        self.press_step = 0.15      # 点按承诺转动量 (rad): 按住不足此量时, 松开也会转完再停
        self._pressed = None        # 当前按住的方向: 'left' / 'right' / None
        self._press_base = 0.0      # 本次按下瞬间的已发布角 (点按承诺线基准)
        self._move = None           # 当前按住的前进方向: 'up' / 'down' / None
        self._last_key_event = time.time()   # 卡键保险: 最后一次键盘事件时刻
        self._stuck_timeout = 8.0   # 超过该时长无键盘事件仍显示按住 -> 视为松键丢失, 自动复位
        self.MOVE_SPEED = 0.25      # 前进/后退速度 (m/s) —— 先慢一点
        self.DRIVE_LIMIT = 9.0      # prismatic 关节平移边界 (m), 世界地面 20x20
        # x/y 底盘位置平滑器: 位置沿当前朝向分解, 限速 0.25 m/s + 缓起缓停
        self.drive_x = SmoothYaw(max_rate=self.MOVE_SPEED, accel=0.8, dt=0.02)
        self.drive_y = SmoothYaw(max_rate=self.MOVE_SPEED, accel=0.8, dt=0.02)
        self._lock = threading.Lock()
        # 无极丝滑转向: 角度无限累积不 wrap, 发布角限速逼近目标
        self.smooth = SmoothYaw(max_rate=1.0, accel=5.0, dt=0.02)

        # 定时器 50Hz 在主线程中发布平滑后的角度
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info("键盘关节控制器已启动（按住转动 · 松开即停 · 点按一下转一下 · 按键时自动接管避障）")
        self.print_help()

    def on_joint_states(self, msg):
        """记录真实关节角, 接管避障瞬间用 (避免从 0 起步把镜头拉回)。"""
        try:
            idx = list(msg.name).index('base_yaw_joint')
        except ValueError:
            return
        with self._lock:
            self._real_angle = msg.position[idx]
            self._real_seen = True

    def print_help(self):
        print("\n控制说明：")
        print("  左箭头 (←) : 按住持续向左转，松开停止；点按一下转一下")
        print("  右箭头 (→) : 按住持续向右转，松开停止；点按一下转一下")
        print("  上箭头 (↑) : 按住前进，松开停止")
        print("  下箭头 (↓) : 按住后退，松开停止（速度 0.25 m/s）")
        print("  ESC       : 退出")

    def timer_callback(self):
        """定时器回调 (50Hz):
        - 有按键/在转动/在移动 -> 接管: 发布 override=1 + 平滑命令
        - 空闲 -> 静默: 发布 override=0, 不碰关节话题 (让避障独占)
        - 卡键保险: 超过 8s 无键盘事件仍显示按住 -> 松键事件多半丢了, 自动复位
        """
        active = False
        with self._lock:
            if (self._pressed or self._move) and \
               time.time() - self._last_key_event > self._stuck_timeout:
                self.get_logger().warn(
                    f"检测到卡键 ({self._stuck_timeout:.0f}s 无键盘事件), 自动复位, 交还控制权")
                self._pressed = None
                self._move = None
                self.smooth.set_target(self.smooth.pos)
                self.drive_x.set_target(self.drive_x.pos)
                self.drive_y.set_target(self.drive_y.pos)

            if self._pressed == 'left':
                self.smooth.step_delta(+self.step_per_tick)
                active = True
            elif self._pressed == 'right':
                self.smooth.step_delta(-self.step_per_tick)
                active = True

            # 前进/后退: 沿当前朝向(yaw)分解为 x/y 位移增量 (不越过平移边界)
            if self._move:
                v = self.MOVE_SPEED if self._move == 'up' else -self.MOVE_SPEED
                yaw = self.smooth.pos
                dx = math.cos(yaw) * v * 0.02
                dy = math.sin(yaw) * v * 0.02
                nx = self.drive_x.target + dx
                ny = self.drive_y.target + dy
                if abs(nx) <= self.DRIVE_LIMIT and abs(ny) <= self.DRIVE_LIMIT:
                    self.drive_x.step_delta(dx)
                    self.drive_y.step_delta(dy)
                active = True

            # 点按承诺转动中 / 平移缓停中也算 active (不中途交还)
            if abs(self.smooth.target - self.smooth.pos) > 1e-6:
                active = True
            if abs(self.drive_x.target - self.drive_x.pos) > 1e-6 or \
               abs(self.drive_y.target - self.drive_y.pos) > 1e-6:
                active = True

            angle = self.smooth.tick()
            xp = self.drive_x.tick()
            yp = self.drive_y.tick()

        ov = Float64()
        ov.data = 1.0 if active else 0.0
        self.override_pub.publish(ov)

        if active:
            msg = Float64()
            msg.data = angle
            self.publisher.publish(msg)
            xmsg = Float64()
            xmsg.data = xp
            self.drive_x_pub.publish(xmsg)
            ymsg = Float64()
            ymsg.data = yp
            self.drive_y_pub.publish(ymsg)

    def set_angle(self, delta):
        """增加目标角 (无极: 不 wrap, 可无限累加)"""
        with self._lock:
            self.smooth.step_delta(delta)

    def on_press(self, key):
        try:
            self._last_key_event = time.time()
            if key == keyboard.Key.left:
                with self._lock:
                    # key-repeat 判定: 已经按住同方向 (OS 自动重复触发 on_press) 就不再锚定/累加
                    # 否则每次 repeat 都用滞后的真实角重置 smooth, 前进量被反复吞掉 -> "转过去一点被拉回"
                    if self._pressed != 'left':
                        # 锚定接管起点: 收到过真实角 → 从真实角度起步 (零跳变);
                        # 桥断了 → 沿用键盘自身已发布角, 绝不回 0!
                        if self._real_seen:
                            self._press_base = self._real_angle
                            self.smooth.pos = self._real_angle
                            self.smooth.target = self._real_angle
                            self.smooth.vel = 0.0
                        else:
                            self._press_base = self.smooth.pos
                            if not self._real_warned:
                                self._real_warned = True
                                self.get_logger().warn(
                                    '未收到 /joint_states 真实关节角 (检查 joint_state 桥是否带 '
                                    "-r /joint_state:=/joint_states), 沿用键盘自身角度接管")
                        self.smooth.step_delta(+self.press_step)  # 首按转量 (点按/启动)
                    self._pressed = 'left'
            elif key == keyboard.Key.right:
                with self._lock:
                    if self._pressed != 'right':
                        if self._real_seen:
                            self._press_base = self._real_angle
                            self.smooth.pos = self._real_angle
                            self.smooth.target = self._real_angle
                            self.smooth.vel = 0.0
                        else:
                            self._press_base = self.smooth.pos
                            if not self._real_warned:
                                self._real_warned = True
                                self.get_logger().warn(
                                    '未收到 /joint_states 真实关节角 (检查 joint_state 桥是否带 '
                                    "-r /joint_state:=/joint_states), 沿用键盘自身角度接管")
                        self.smooth.step_delta(-self.press_step)
                    self._pressed = 'right'
            elif key == keyboard.Key.up:
                with self._lock:
                    self._move = 'up'
            elif key == keyboard.Key.down:
                with self._lock:
                    self._move = 'down'
            elif key == keyboard.Key.esc:
                self.get_logger().info("ESC 退出，关闭键盘控制器")
                # 退出前关闭节点，触发 shutdown
                rclpy.shutdown()  # 会停止整个 rclpy 上下文
                return False      # 停止监听
        except Exception as e:
            self.get_logger().error(f"按键处理异常: {e}")
        return True

    def on_release(self, key):
        """松开: 正按住该方向则停住。
        转向: 点按承诺线 —— 若本次按下累计还不足 press_step(点按), 松开后仍转完承诺量再停;
              若已转够(真按住), 立即丝滑刹车。
        移动: 松开即斜坡减速至 0。
        """
        try:
            self._last_key_event = time.time()
            direction = +1.0 if key == keyboard.Key.left else \
                        (-1.0 if key == keyboard.Key.right else 0.0)
            if direction != 0.0 and self._pressed == ('left' if direction > 0 else 'right'):
                with self._lock:
                    self._pressed = None
                    min_target = self._press_base + direction * self.press_step
                    if direction * (self.smooth.pos - min_target) >= 0.0:
                        self.smooth.set_target(self.smooth.pos)   # 转够了 → 松开即停
                    else:
                        self.smooth.set_target(min_target)        # 点按 → 转完承诺量再停
            elif key == keyboard.Key.up and self._move == 'up':
                with self._lock:
                    self._move = None
            elif key == keyboard.Key.down and self._move == 'down':
                with self._lock:
                    self._move = None
        except Exception as e:
            self.get_logger().error(f"松键处理异常: {e}")
        return True

def main(args=None):
    rclpy.init(args=args)
    controller = KeyboardJointController()
    # 在单独线程中运行 pynput 监听，主线程 spin
    listener = keyboard.Listener(on_press=controller.on_press, on_release=controller.on_release)
    listener.start()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        listener.stop()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()