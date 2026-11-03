#!/usr/bin/env python3
"""仿真端到端验收: 在机器人前方摆一个障碍, 让导航自己决定绕不绕。

它只做三件事: 发一个 /navigate_to 目标、按 20 Hz 记实测位姿、算与障碍的最小间距。
判定: 到点 = 通过; 撞上(间距 < 车体半宽) = 失败; 超时没到 = 卡住。
用法: python3 nodes/check_avoid_in_sim.py 3.5 0        (目标坐标)
      python3 nodes/check_avoid_in_sim.py 3.5 0 --no-obstacle   (无障碍基线)
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64

# 与 /tmp/avoid_obstacle.sdf 一致: 中心 (2.0, 0), 尺寸 1.2 x 0.4
OBS_X, OBS_Y, OBS_HX, OBS_HY = 2.0, 0.0, 0.6, 0.2
BODY_HALF = 0.20        # 车体半宽 (m), 间距小于它算擦到
TIMEOUT = 120.0


def dist_to_rect(x, y):
    """点到障碍矩形外边的距离, 在矩形内为负。"""
    dx = abs(x - OBS_X) - OBS_HX
    dy = abs(y - OBS_Y) - OBS_HY
    if dx > 0 and dy > 0:
        return math.hypot(dx, dy)
    return max(dx, dy)


class Checker(Node):
    def __init__(self):
        super().__init__('check_avoid_in_sim')
        self.x = self.y = self.yaw = None
        self.reached = False
        self.min_clear = 99.0
        self.traj = []
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.create_subscription(Bool, '/nav_goal_reached', lambda m: setattr(self, 'reached', bool(m.data)), 10)
        self.create_subscription(Float64, '/nearest_obstacle', lambda m: None, 10)
        self.pub_goal = self.create_publisher(Pose2D, '/navigate_to', 10)

    def on_js(self, msg):
        p = dict(zip(msg.name, msg.position))
        if 'drive_x_joint' in p:
            self.x = p['drive_x_joint']
        if 'drive_y_joint' in p:
            self.y = p['drive_y_joint']
        if 'base_yaw_joint' in p:
            self.yaw = p['base_yaw_joint']
        if None not in (self.x, self.y):
            self.traj.append((self.x, self.y))
            self.min_clear = min(self.min_clear, dist_to_rect(self.x, self.y))


def main():
    gx, gy = float(sys.argv[1]), float(sys.argv[2])
    check_obs = '--no-obstacle' not in sys.argv
    rclpy.init()
    node = Checker()
    t0 = time.time()
    while node.x is None and time.time() - t0 < 15.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.x is None:
        print('✗ 没有 /joint_states, 仿真没起来?')
        return 2
    print(f'起点 ({node.x:+.2f}, {node.y:+.2f}) -> 目标 ({gx:+.2f}, {gy:+.2f})'
          + (', 障碍 (2.00, 0.00) 1.2x0.4' if check_obs else ', 无障碍'))
    g = Pose2D(x=gx, y=gy, theta=0.0)
    best = 99.0
    t0 = time.time()
    # 新起节点的 discovery 要一两秒, 早发的目标会直接丢 (每帧发 5 条也不一定够):
    # 所以头 6 秒一直重发, 之后闭麦, 否则每秒 clear 目标会让车反复重新起步。
    send_until = t0 + 6.0
    while time.time() - t0 < TIMEOUT:
        if time.time() < send_until:
            node.pub_goal.publish(g)
        rclpy.spin_once(node, timeout_sec=0.05)
        d = math.hypot(gx - node.x, gy - node.y)
        if check_obs:
            best = min(best, dist_to_rect(node.x, node.y))
        if node.reached or d < 0.05:
            break
    dt = time.time() - t0
    path = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(node.traj, node.traj[1:]))
    print(f'终点 ({node.x:+.2f}, {node.y:+.2f}), 误差 {d:.3f} m, 用时 {dt:.1f}s')
    print(f'路径长 {path:.2f} m (直线 {math.hypot(gx, gy):.2f} m), 采样 {len(node.traj)} 点')
    if check_obs:
        print(f'与障碍最小间距 {best:.2f} m (车体半宽 {BODY_HALF:.2f} m)')
    ok = d < 0.10 or node.reached
    if check_obs:
        ok = ok and best > BODY_HALF
    print(('✓ 通过: ' if ok else '✗ 失败: ') + ('到点' if d < 0.10 or node.reached else '未到点')
          + ('' if not check_obs else (f', 间距 {best:.2f} m' if best > BODY_HALF else f', 撞了(间距 {best:.2f} m)')))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
