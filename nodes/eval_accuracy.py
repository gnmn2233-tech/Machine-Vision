#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ТЗ 4.2.1 精度评测: 系统障碍检测 vs 场景几何真值.

独立脚本 —— 演示时不跑 (simulation.sh / run.sh 完全不碰它), 只在对账时手动执行:

    python3 nodes/eval_accuracy.py --scene mixed --depth 1

两套统计范围 x 两套口径
----------------------
范围:
  all  —— 全 360 度、所有距离都算 (严格口径, 会把「传感器物理上够不到」也算成漏检)
  cov  —— 只算「系统承诺覆盖的量程内」:
           * 雷达扫描面 z = 0.235 m, 高度 >= 它的障碍 360 度都算;
           * 比它矮的 (0.10 m 矮箱) 只有落在深度视野 ±30 度且 <= depth_reach 1.4 m 时才算.
          量程外的方向标「不计分」, 不计入 P/R.
口径:
  sector  —— 感知层: 当帧雷达 ±5 度最近 + 深度弧 (代表「看见了什么」)
  planner —— 决策层: clearance ±20 度窗 + 记忆 (偏保守, 代表「敢不敢走」)

真值: 场景 SDF 的碰撞几何 (box / capsule) 做 2D 射线求交, 不依赖任何传感器。
阈值 0.6 / 1.0 / 2.0 m 各算一遍。
"""
import argparse
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from avoid_planner import (AvoidPlanner, Config as AvoidConfig, compute_sectors,
                           wrap_pi)  # noqa: E402

SAFE = 0.60
THRESH = (0.60, 1.00, 2.00)
STEP = 10.0                     # 评测方向步长 (度)
SPAN_SECTOR = 5.0
SPAN_PLANNER = 20.0
LIDAR_Z = 0.235                 # 雷达扫描面高度: 低于它的障碍雷达打不到
DEPTH_REACH = 2.20              # 深度弧签约定量程, 必须与 Config.depth_reach 一致
DEPTH_HFOV = 30.0               # 深度水平半视场 (度)


# 真值几何: (类型, cx, cy, 半长或半径A, 半宽或半径B, yaw, 高度 m)
OBST = {
    'empty': [],
    'symmetric': [('box', 0.0, 3.2, 0.60, 0.20, 0.0, 0.60)],
    'corridor': [('box', -0.50, 3.0, 0.15, 0.80, 0.0, 0.60),
                 ('box', 0.50, 3.0, 0.15, 0.80, 0.0, 0.60)],
    'low_box': [('box', 0.0, 2.5, 0.40, 0.20, 0.0, 0.10)],
    'mixed': [('box', 0.0, 2.0, 0.40, 0.20, 0.0, 0.10),      # 矮箱
              ('box', 1.2, 3.6, 0.50, 0.20, 0.0, 0.60),      # 横墙
              ('box', 1.8, 5.2, 0.90, 2.30, math.pi / 2, 1.50),  # prius
              ('cyl', 1.4, 6.9, 0.25, 0.0, 0.0, 1.30)],      # 行人
    # U 型死角: 三面墙高 0.6 m (雷达可见), 开口朝 -y, 机器人从 (0,0) 走进去
    'utrap': [('box', 0.0, 3.1, 1.20, 0.10, 0.0, 0.60),
              ('box', -1.1, 2.0, 0.10, 1.10, 0.0, 0.60),
              ('box', 1.1, 2.0, 0.10, 1.10, 0.0, 0.60)],
    'person': [('box', -4.5714, 4.7399, 0.90, 2.30, math.pi / 2, 1.50),
               ('cyl', 0.0059, 4.6140, 0.25, 0.0, 0.0, 1.30),
               ('box', 3.2302, 4.2403, 0.90, 2.30, 0.0, 1.50)],
}


def ray_box(px, py, ang, cx, cy, ha, hb, yaw):
    ca, sa = math.cos(-yaw), math.sin(-yaw)
    dx, dy = px - cx, py - cy
    lx = ca * dx - sa * dy
    ly = sa * dx + ca * dy
    vx = ca * math.cos(ang) - sa * math.sin(ang)
    vy = sa * math.cos(ang) + ca * math.sin(ang)
    tmin, tmax = -1e18, 1e18
    for o, v, h in ((lx, vx, ha), (ly, vy, hb)):
        if abs(v) < 1e-9:
            if abs(o) > h:
                return None
        else:
            t1, t2 = (-h - o) / v, (h - o) / v
            if t1 > t2:
                t1, t2 = t2, t1
            tmin = max(tmin, t1)
            tmax = min(tmax, t2)
    return max(tmin, 0.0) if tmax >= max(tmin, 0.0) else None


def ray_cyl(px, py, ang, cx, cy, r):
    ox, oy = px - cx, py - cy
    dx, dy = math.cos(ang), math.sin(ang)
    b = ox * dx + oy * dy
    c = ox * ox + oy * oy - r * r
    disc = b * b - c
    if disc < 0.0:
        return None
    t = -b - math.sqrt(disc)
    return t if t >= 0.0 else None


def truth(px, py, ang, objs, win_deg, step_deg=1.0):
    """返回 (最近距离, 那个障碍的高度); 没东西 = (inf, None)。"""
    best = math.inf
    bh = None
    n = int(win_deg)
    for d in range(-n, n + 1, int(step_deg)):
        a = ang + math.radians(d)
        for o in objs:
            if o[0] == 'box':
                t = ray_box(px, py, a, o[1], o[2], o[3], o[4], o[5])
            else:
                t = ray_cyl(px, py, a, o[1], o[2], o[3])
            if t is not None and t < best:
                best = t
                bh = o[6]
    return best, bh


def in_range(dist, height, deg_rel):
    """这个方向的真值是否落在「系统签约的量程内」。量程外不计分。"""
    if not math.isfinite(dist) or height is None:
        return True                      # 空方向: 报有就是误报, 计分
    if height >= LIDAR_Z:
        return True                      # 雷达 360 度都该看见
    return abs(deg_rel) <= DEPTH_HFOV and dist <= DEPTH_REACH


class Eval(Node):
    def __init__(self, scene, duration, use_depth=False):
        super().__init__('eval_accuracy')
        self.scene = scene
        self.duration = duration
        self.use_depth = use_depth
        self.objs = OBST[scene]
        self.avoider = AvoidPlanner()
        self.x = self.y = self.yaw = None
        self.scan = None
        self.sectors = None
        self.bucket = {}
        self.frames = 0
        self.sec = []      # (系统距离 or None, 真值距离, 是否计分)
        self.pln = []
        self.t0 = time.time()
        self.done = False
        self.br = None
        try:
            from cv_bridge import CvBridge
            self.br = CvBridge()
        except Exception:
            pass
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(PoseStamped, '/robot_pose', self.on_pose, 10)
        if self.use_depth:
            self.create_subscription(Image, '/depth_camera', self.on_depth, 5)
        self.create_timer(0.2, self.tick)
        self.get_logger().info(
            f'评测开始: 场景 {scene}, {duration:.0f} s, 真值 {len(self.objs)} 个障碍, '
            f'方向 {int(360 / STEP)} 个, 量程过滤=' + ('开' if use_depth or True else '关'))

    def on_scan(self, msg):
        self.scan = (msg.ranges, msg.angle_min, msg.angle_increment)

    def on_pose(self, msg):
        p = msg.pose
        q = p.orientation
        self.x, self.y = p.position.x, p.position.y
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_depth(self, msg):
        if self.br is None:
            return
        try:
            arr = self.br.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self.sectors = compute_sectors(arr, n_sectors=self.avoider.cfg.n_sectors)
        except Exception:
            self.sectors = None

    def _build_bucket(self):
        if not self.scan:
            return
        ranges, amin, ainc = self.scan
        b = {}
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r < 0.10 or r > 12.0:
                continue
            a = amin + i * ainc + self.yaw
            k = int(round(math.degrees(wrap_pi(a))))
            if k not in b or r < b[k]:
                b[k] = r
        self.bucket = b

    def _sector_dist(self, deg_rel, now, span=SPAN_SECTOR):
        ang = wrap_pi(self.yaw + math.radians(deg_rel))
        best = None
        for d in range(-int(span), int(span) + 1):
            k = int(round(math.degrees(wrap_pi(ang + math.radians(d)))))
            v = self.bucket.get(k)
            if v is not None and (best is None or v < best):
                best = v
        dv = self.avoider._depth_abs_min(ang, now, span)
        if dv is not None and (best is None or dv < best):
            best = dv
        return best

    def tick(self):
        if self.done or self.x is None or self.scan is None:
            return
        if time.time() - self.t0 > self.duration:
            return self.finish()
        now = time.time()
        gx = self.x + 8.0 * math.cos(self.yaw)
        gy = self.y + 8.0 * math.sin(self.yaw)
        self.avoider.update(self.x, self.y, self.yaw, (gx, gy),
                            self.sectors, now, scan=self.scan)
        self._build_bucket()
        self.frames += 1
        for a in range(-180, 180, int(STEP)):
            ang_w = self.yaw + math.radians(a)
            ds = self._sector_dist(a, now)
            gs, hs = truth(self.x, self.y, ang_w, self.objs, SPAN_SECTOR)
            self.sec.append((ds, gs, in_range(gs, hs, a)))
            dp = self.avoider.clearance(a, now)
            gp, hp = truth(self.x, self.y, ang_w, self.objs, SPAN_PLANNER, 2.0)
            self.pln.append((dp, gp, in_range(gp, hp, a)))

    def stats(self, pairs, th, cover_only):
        tp = fp = fn = tn = skip = 0
        fp_frames = tp_frames = scored_frames = 0
        frame_fp = frame_tp = frame_scored = False
        for i, (sysd, gt, ok) in enumerate(pairs):
            if i % 36 == 0:
                frame_fp = frame_tp = frame_scored = False
            if cover_only and not ok:
                skip += 1
                if i % 36 == 35:
                    fp_frames += 1 if frame_fp else 0
                    tp_frames += 1 if frame_tp else 0
                    scored_frames += 1 if frame_scored else 0
                continue
            s = sysd is not None and sysd < th
            g = gt < th
            frame_scored = True
            if s and g:
                tp += 1
                frame_tp = True
            elif s and not g:
                fp += 1
                frame_fp = True
            elif g:
                fn += 1
            else:
                tn += 1
            if i % 36 == 35:
                fp_frames += 1 if frame_fp else 0
                tp_frames += 1 if frame_tp else 0
                scored_frames += 1 if frame_scored else 0
        n = tp + fp + fn + tn
        errs = [abs(s - g) for s, g, ok in pairs
                if s is not None and math.isfinite(s) and math.isfinite(g)
                and (ok or not cover_only)]
        return {
            'thresh': th, 'tp': tp, 'fp': fp, 'fn': fn, 'tn': tn, 'n': n,
            'skipped': skip,
            'precision': round(tp / (tp + fp), 4) if tp + fp else None,
            'recall': round(tp / (tp + fn), 4) if tp + fn else None,
            'fp_dir_rate': round(fp / n, 4) if n else 0.0,
            'fp_frames': fp_frames,
            'scored_frames': scored_frames,
            'fp_frame_rate': round(fp_frames / scored_frames, 4) if scored_frames else 0.0,
            'tp_frames': tp_frames,
            'mean_abs_err_m': round(sum(errs) / len(errs), 3) if errs else None,
        }

    def finish(self):
        self.done = True
        res = {'scene': self.scene, 'frames': self.frames,
               'sector': [self.stats(self.sec, t, False) for t in THRESH],
               'planner': [self.stats(self.pln, t, False) for t in THRESH],
               'sector_cov': [self.stats(self.sec, t, True) for t in THRESH],
               'planner_cov': [self.stats(self.pln, t, True) for t in THRESH]}
        out = f'/tmp/eval_{self.scene}.json'
        with open(out, 'w') as f:
            json.dump(res, f, ensure_ascii=False, indent=1)
        lg = self.get_logger()
        lg.info('=' * 72)
        lg.info(f'场景 {self.scene}: {self.frames} 帧 x 36 方向')
        for tag in ('sector', 'sector_cov', 'planner', 'planner_cov'):
            for s in res[tag]:
                p = '  -  ' if s['precision'] is None else f"{s['precision'] * 100:5.1f}%"
                r = '  -  ' if s['recall'] is None else f"{s['recall'] * 100:5.1f}%"
                lg.info(f"  {tag:<12}{s['thresh']:.1f} m: P {p} R {r} "
                        f"| FP帧 {s['fp_frame_rate'] * 100:5.1f}% "
                        f"| TP{s['tp']} FP{s['fp']} FN{s['fn']} 跳过{s['skipped']}")
        lg.info(f'  -> {out}')
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scene', default=os.environ.get('SCENE', 'empty'))
    ap.add_argument('--duration', type=float, default=40.0)
    ap.add_argument('--depth', type=int, default=0)
    args, _ = ap.parse_known_args()
    if args.scene not in OBST:
        print(f'未知场景 {args.scene}, 可选: {list(OBST)}')
        sys.exit(2)
    rclpy.init()
    node = Eval(args.scene, args.duration, bool(args.depth))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
