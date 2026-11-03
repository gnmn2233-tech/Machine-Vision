#!/usr/bin/env python3
"""检查 (0,8) 行人场景: 用时/路径/与行人最小间距。
用法: python3 nodes/check_person.py 0 8
"""
import math, sys, time
import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

PERSON_X, PERSON_Y, PERSON_R = 0.006, 4.614, 0.35
BODY_HALF = 0.20
TIMEOUT = 120.0

class Checker(Node):
    def __init__(self):
        super().__init__('check_person')
        self.x = self.y = self.yaw = None
        self.reached = False
        self.min_d = 99.0
        self.path = 0.0
        self.prev = None
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.create_subscription(Bool, '/nav_goal_reached', lambda m: setattr(self, 'reached', bool(m.data)), 10)
        self.pub_goal = self.create_publisher(Pose2D, '/navigate_to', 10)

    def on_js(self, msg):
        p = dict(zip(msg.name, msg.position))
        if 'drive_x_joint' in p: self.x = p['drive_x_joint']
        if 'drive_y_joint' in p: self.y = p['drive_y_joint']
        if 'base_yaw_joint' in p: self.yaw = p['base_yaw_joint']
        if None not in (self.x, self.y):
            if self.prev is not None:
                self.path += math.hypot(self.x - self.prev[0], self.y - self.prev[1])
            self.prev = (self.x, self.y)
            d = math.hypot(self.x - PERSON_X, self.y - PERSON_Y) - PERSON_R - BODY_HALF
            self.min_d = min(self.min_d, d)

def main():
    gx, gy = float(sys.argv[1]), float(sys.argv[2])
    rclpy.init()
    node = Checker()
    t0 = time.time()
    while node.x is None and time.time() - t0 < 15.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.x is None:
        print('NO joint_states')
        return 2
    print('start (%.2f,%.2f) -> (%.2f,%.2f), person (%.2f,%.2f)' % (node.x, node.y, gx, gy, PERSON_X, PERSON_Y))
    # 只在开头补发几次: 节点每收到一次 /navigate_to 就 avoider.reset(), 记住的
    # 障碍方位全作废 —— 每 0.5 s 发一次等于让它永远从零开始 (实测 120 s 里发了
    # 230 次, 车卡在 look 循环只走了 3.3 m)。目标没变就不该打断它。
    # 但发布端要先空转一会儿把 DDS 连接建起来, 否则前几条直接丢 (实测发 3 次全丢,
    # 节点一次 Старт 都没打印、车一步没动)。
    t_boot = time.time()
    while time.time() - t_boot < 2.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    for _ in range(5):
        node.pub_goal.publish(Pose2D(x=gx, y=gy, theta=0.0))
        rclpy.spin_once(node, timeout_sec=1.0)
    t0 = time.time()
    while time.time() - t0 < TIMEOUT:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.reached:
            break
    el = time.time() - t0
    print('time %.1f s' % el)
    print('path %.2f m' % node.path)
    print('min gap to person %+.2f m (neg = inside body)' % node.min_d)
    print('reached: %s' % node.reached)
    return 0 if node.reached else 1

if __name__ == '__main__':
    sys.exit(main())
