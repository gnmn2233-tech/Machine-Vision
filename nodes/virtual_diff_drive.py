#!/usr/bin/env python3
"""速度语义适配层: 把 /cmd_vel 翻译成这台底盘的位置指令。

底盘是世界系 XY 龙门 + 转台 (worlds/robots/robot_with_camera/model.sdf):
  world --drive_x_joint(prismatic, 世界 X, ±10m)--> chassis_x
        --drive_y_joint(prismatic, 世界 Y, ±10m)--> chassis_y
        --base_yaw_joint(revolute)             --> base

它其实是一个**全向平台** (x、y、yaw 三轴独立), 只是上层从来没有速度接口。
本节点补上这一层, 让 nav2 / DWA / MPPI 这类只认 /cmd_vel 的东西可以接:

  /cmd_vel (geometry_msgs/Twist)
      linear.x  前进速度 (m/s), 方向 = 当前车头
      linear.y  >0 时进入全向模式: 直接当车体侧向速度 (世界系合成)
      angular.z 转向角速度 (rad/s), 逆时针为正
  -> /drive_x/commands, /drive_y/commands, /camera_joint_controller/commands
     (JointPositionController 要的是**绝对**位置, 不是增量)

  /joint_states (真值) -> /odom (nav_msgs/Odometry) + TF odom->base_link

两条与位置伺服打交道的经验 (来自 navigate_to_pose.py, 别重蹈覆辙):
  1) 内部累积、不贴实测。指令若每拍都贴着实测 +v*dt, 伺服滞后会把运动
     拖成每拍零点几毫米, 永远追不上;
  2) 但领先量要封顶。累积量离实测超过 lead_max 就重对齐, 否则撞了东西
     之后指令还在往前跑, 车会顶着障碍硬推。

日志: 俄文 (与项目其它节点一致)。
"""
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster

JOINT_X = 'drive_x_joint'
JOINT_Y = 'drive_y_joint'
JOINT_YAW = 'base_yaw_joint'

TOPIC_X = '/drive_x/commands'
TOPIC_Y = '/drive_y/commands'
TOPIC_YAW = '/camera_joint_controller/commands'

DRIVE_LIMIT = 10.0


def wrap_pi(a):
    """把角度折到 (-pi, pi]。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


class VirtualDiffDrive(Node):
    def __init__(self):
        super().__init__('virtual_diff_drive')
        self.declare_parameter('max_v', 0.8)          # m/s
        self.declare_parameter('max_w', 1.5)          # rad/s
        self.declare_parameter('cmd_timeout', 0.5)    # 超过这么久没新指令就停
        self.declare_parameter('lead_max', 0.60)      # 指令领先实测的上限 (m)
        self.declare_parameter('rate', 50.0)          # Hz

        self.max_v = float(self.get_parameter('max_v').value)
        self.max_w = float(self.get_parameter('max_w').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.lead_max = float(self.get_parameter('lead_max').value)
        rate = float(self.get_parameter('rate').value)

        # 实测位姿 (来自 /joint_states 真值, 与 navigate_to_pose.py 同源)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.got_pose = False

        # 指令累积量 (发布出去的目标位姿)
        self.cx = None
        self.cy = None
        self.cyaw = None

        # 当前速度指令
        self.v_fwd = 0.0
        self.v_lat = 0.0
        self.w = 0.0
        self.last_cmd_t = 0.0
        self.have_cmd = False

        # odom 差分用的上一拍
        self._px = None
        self._py = None
        self._pyaw = None
        self._pt = None

        self.pub_x = self.create_publisher(Float64, TOPIC_X, 10)
        self.pub_y = self.create_publisher(Float64, TOPIC_Y, 10)
        self.pub_yaw = self.create_publisher(Float64, TOPIC_YAW, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)

        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)

        self._t_last = time.time()
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            'Скоростной интерфейс включён: /cmd_vel -> позиции приводов, '
            'публикую /odom + TF odom->base_link (v_max=%.2f м/с, w_max=%.2f рад/с)'
            % (self.max_v, self.max_w))

    # ---------- 输入 ----------
    def on_cmd(self, msg):
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        w = float(msg.angular.z)
        n = math.hypot(vx, vy)
        if n > self.max_v:                      # 合成速度限幅, 方向不变
            k = self.max_v / n
            vx *= k
            vy *= k
        self.v_fwd = vx
        self.v_lat = vy if abs(vy) > 0.02 else 0.0
        self.w = max(-self.max_w, min(self.max_w, w))
        self.last_cmd_t = time.time()
        self.have_cmd = True

    def on_joints(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if JOINT_X in pos:
            self.x = float(pos[JOINT_X])
            self.got_pose = True
        if JOINT_Y in pos:
            self.y = float(pos[JOINT_Y])
        if JOINT_YAW in pos:
            self.yaw = float(pos[JOINT_YAW])

    # ---------- 主循环 ----------
    def tick(self):
        now = time.time()
        dt = now - self._t_last
        self._t_last = now
        if dt > 0.15:                           # 掉帧/启动阶段: 别把一大段时间积进来
            dt = 0.02
        if not self.got_pose:
            return

        if self.cx is None:
            self.cx, self.cy, self.cyaw = self.x, self.y, wrap_pi(self.yaw)

        v = self.v_fwd
        vl = self.v_lat
        w = self.w
        if now - self.last_cmd_t > self.cmd_timeout:
            v = vl = w = 0.0                     # 上层不说话了就停, 不要一直跑

        # 转向先积, 再按新的车头方向积位移 —— 与差速车的运动学一致
        self.cyaw = wrap_pi(self.cyaw + w * dt)
        if vl:
            # 全向模式: 车体系 (v, vl) -> 世界系
            self.cx += (math.cos(self.cyaw) * v - math.sin(self.cyaw) * vl) * dt
            self.cy += (math.sin(self.cyaw) * v + math.cos(self.cyaw) * vl) * dt
        else:
            self.cx += math.cos(self.cyaw) * v * dt
            self.cy += math.sin(self.cyaw) * v * dt

        # 领先量封顶: 撞上东西/被挡住时, 指令要跟着停下来
        lead = math.hypot(self.cx - self.x, self.cy - self.y)
        if lead > self.lead_max:
            k = self.lead_max / lead
            self.cx = self.x + (self.cx - self.x) * k
            self.cy = self.y + (self.cy - self.y) * k
        if abs(wrap_pi(self.cyaw - self.yaw)) > 0.9:     # 转太多没跟上 -> 重对齐
            self.cyaw = wrap_pi(self.yaw)

        cx = max(-DRIVE_LIMIT, min(DRIVE_LIMIT, self.cx))
        cy = max(-DRIVE_LIMIT, min(DRIVE_LIMIT, self.cy))
        self.pub_x.publish(Float64(data=cx))
        self.pub_y.publish(Float64(data=cy))
        self.pub_yaw.publish(Float64(data=self.cyaw))

        self._publish_odom(now)

    def _publish_odom(self, now):
        if self._pt is None:                    # 第一拍只建基准, 但位姿照发
            self._px, self._py, self._pyaw, self._pt = self.x, self.y, self.yaw, now
        dt = now - self._pt
        if dt > 1e-6:
            vx = (self.x - self._px) / dt
            vy = (self.y - self._py) / dt
            wz = wrap_pi(self.yaw - self._pyaw) / dt
            self._px, self._py, self._pyaw, self._pt = self.x, self.y, self.yaw, now
        else:
            vx = vy = wz = 0.0

        yaw = wrap_pi(self.yaw)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_link'
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x = vx
        od.twist.twist.linear.y = vy
        od.twist.twist.angular.z = wz
        od.pose.covariance[0] = 0.0004      # 关节真值, 当 2 cm 标准差看
        od.pose.covariance[7] = 0.0004
        od.pose.covariance[35] = 0.0004
        od.twist.covariance[0] = 0.01
        od.twist.covariance[7] = 0.01
        od.twist.covariance[35] = 0.01
        self.pub_odom.publish(od)

        tf = TransformStamped()
        tf.header.stamp = od.header.stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf.sendTransform(tf)


def main():
    rclpy.init()
    node = VirtualDiffDrive()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
