#!/usr/bin/env python3
"""avoid_planner 的离线几何测试 (不需要 ROS / Gazebo)。

做法: 用「圆形障碍 + 射线测距」模拟深度相机的分扇区净空, 用与
navigate_to_pose.py 完全相同的运动律 (先原地转向对准, 再沿车头直行)
驱动一个点机器人, 看规划器能不能真的绕过去、该停的时候停没停。

跑法: python3 nodes/test_avoid_planner.py
"""
import math
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from avoid_planner import AvoidPlanner, Config, NAV, DODGE, HOLD, INF  # noqa: E402

DT = 0.02
HFOV = 1.047
NSEC = 9
GATE = 0.15          # 与导航节点 heading_gate 一致
V = 0.5              # 与 DEFAULT_V_MAX 一致
A = 1.0
W = 1.0
TURN_GAIN = 2.0


def ray_circle(p, d, c, r):
    ox, oy = c[0] - p[0], c[1] - p[1]
    tca = ox * d[0] + oy * d[1]
    d2 = ox * ox + oy * oy - tca * tca
    if d2 > r * r:
        return INF
    thc = math.sqrt(max(r * r - d2, 0.0))
    t0 = tca - thc
    if t0 > 0:
        return t0
    return 0.0 if tca + thc > 0 else INF


def sense(x, y, yaw, obstacles, n=NSEC, hfov=HFOV, sub=6):
    """模拟深度相机: 视野按角度均分 n 个扇区, 每个扇区取最近障碍距离。"""
    out = []
    for i in range(n):
        a0 = -hfov / 2 + hfov * i / n
        a1 = a0 + hfov / n
        best = INF
        for j in range(sub):
            a = a0 + (a1 - a0) * (j + 0.5) / sub
            h = yaw + a
            d = (math.cos(h), math.sin(h))
            for (cx, cy, r) in obstacles:
                t = ray_circle((x, y), d, (cx, cy), r)
                if t < best:
                    best = t
        out.append(best)
    return tuple(out)


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def simulate(obstacles, start, goal, max_t=90.0, cfg=None, verbose=False):
    """跑一次, 返回 (结果, 轨迹, 最小净空, 状态集合)。"""
    planner = AvoidPlanner(cfg or Config())
    x, y, yaw = start
    t = 0.0
    min_clear = INF
    states = set()
    stuck_t = 0.0
    trail = []
    while t < max_t:
        sectors = sense(x, y, yaw, obstacles)
        # 净空用几何距离量 (转向时车头扫过障碍会让 sectors 变小, 但那不是"靠近")
        if obstacles:
            min_clear = min(min_clear,
                            min(math.hypot(x - cx, y - cy) - r for (cx, cy, r) in obstacles))
        plan = planner.update(x, y, yaw, goal, sectors, t, V)
        states.add(plan.state)
        trail.append((x, y, yaw, plan.state))
        if verbose and abs(t % 2.0) < DT:
            print('  t=%5.1f %-5s %-44s pos=(%.2f,%.2f) yaw=%+4.0f sec=%s'
                  % (t, plan.state, plan.reason[:44], x, y, math.degrees(yaw),
                     ' '.join('%.1f' % s for s in sectors)))
        if not plan.advance:
            # 只转车头, 不发位移 —— 与导航节点接到 advance=False 时的行为一致
            t += DT
            stuck_t += DT
            err0 = wrap(plan.heading - yaw)
            if abs(err0) > GATE:
                w0 = min(W, max(math.sqrt(4.0 * abs(err0)), 0.05))
                yaw += math.copysign(min(w0 * DT, abs(err0)), err0)
            if stuck_t > 8.0:
                return ('held', trail, min_clear, states)
            continue
        stuck_t = 0.0
        dist_goal = math.hypot(goal[0] - x, goal[1] - y)
        if dist_goal < 0.10:
            return ('reached', trail, min_clear, states)
        err = wrap(plan.heading - yaw)
        if abs(err) > GATE:
            w = min(W, max(math.sqrt(4.0 * abs(err)), 0.05))
            yaw += math.copysign(min(w * DT, abs(err)), err)
        else:
            room = max(plan.clear - 0.05, 0.05)
            v = min(V, max(math.sqrt(2.0 * A * room), 0.03))
            if dist_goal < 0.5:
                v = min(v, 0.18)
            step = v * DT
            x += math.cos(yaw) * step
            y += math.sin(yaw) * step
            yaw += max(-W, min(W, TURN_GAIN * err)) * DT
        if any(math.hypot(x - cx, y - cy) < r - 0.03 for (cx, cy, r) in obstacles):
            return ('collision', trail, min_clear, states)
        t += DT
    return ('timeout', trail, min_clear, states)


FAILS = []


def check(name, cond, detail=''):
    print('  [%s] %s %s' % ('PASS' if cond else 'FAIL', name, detail))
    if not cond:
        FAILS.append(name)


def main():
    print('=== 1. 无障碍: 应全程 NAV 直达 ===')
    r, trail, mc, st = simulate([], (0, 0, 0.0), (3.0, 0.0))
    check('到达目标', r == 'reached', r)
    check('全程无避障', st == {NAV}, str(st))

    print('=== 2. 正前方一堵墙 (宽 3m): 应绕到墙端后到达 ===')
    wall = [(-1.2 + 0.42 * i, 2.0, 0.30) for i in range(8)]
    r, trail, mc, st = simulate(wall, (0.0, 0.0, 0.0), (0.0, 4.0), max_t=120)
    check('绕过墙到达', r == 'reached', r)
    check('进入过 DODGE', DODGE in st, str(st))
    check('未碰撞', mc > 0.20, 'min_clear=%.2f' % mc)

    print('=== 3. 走廊尽头 (三面围住, 无出口): 应 HOLD 而不是乱撞 ===')
    # 周围一圈障碍全贴脸 (净空 0.25m): 四面都窄, 就该原地不动
    box = [(0.55 * math.cos(2 * math.pi * i / 10),
            0.55 * math.sin(2 * math.pi * i / 10), 0.30) for i in range(10)]
    r, trail, mc, st = simulate(box, (0.0, 0.0, 0.0), (0.0, 3.0), max_t=20)
    check('停住而不是撞', r in ('held', 'timeout'), r)
    check('触发过 HOLD', HOLD in st, str(st))
    check('几乎没有位移', math.hypot(trail[-1][0], trail[-1][1]) < 0.05,
          'moved=%.3f' % math.hypot(trail[-1][0], trail[-1][1]))
    check('未碰撞', mc > 0.20, 'min_clear=%.2f' % mc)

    print('=== 4. 右侧贴脸左侧畅通: 应向左让 ===')
    ob = [(0.25, 0.90, 0.40), (0.55, 1.60, 0.40)]
    r, trail, mc, st = simulate(ob, (0.0, 0.0, 0.0), (0.0, 3.5))
    check('到达目标', r == 'reached', r)
    check('进入过 DODGE', DODGE in st, str(st))
    check('未碰撞', mc > 0.20, 'min_clear=%.2f' % mc)

    print('=== 5. 硬安全线: 正前方 0.2m 时必须 hold ===')
    p = AvoidPlanner(Config())
    sec = (INF, INF, INF, INF, 0.20, INF, INF, INF, INF)
    plan = p.update(0, 0, 0.0, (0.0, 5.0), sec, 0.0, 0.0)
    check('立即停', (not plan.advance) and plan.state == HOLD, plan.state + ' ' + plan.reason)
    plan = p.update(0, 0, 0.0, (0.0, 5.0), sec, 0.02, 0.0)
    check('持续停', not plan.advance, plan.state)

    print('=== 6. 目标方向够宽: 侧前方有障碍也不带偏 ===')
    sec = (INF, INF, 1.05, INF, INF, INF, INF, INF, INF)
    p = AvoidPlanner(Config())
    plan = p.update(0, 0, math.radians(90), (0.0, 5.0), sec, 0.0, 0.5)
    check('保持朝目标 NAV', plan.state == NAV and abs(wrap(plan.heading - math.radians(90))) < 1e-6,
          plan.state + ' ' + plan.reason)

    print('=== 7. LLM 意见: 只改搜索顺序, 不能突破安全线 ===')
    p = AvoidPlanner(Config())
    p.note_llm(+1, 0.0)
    sec = (INF, INF, INF, INF, 0.50, INF, INF, INF, INF)   # 正前方 0.5m, 两侧都宽
    plan = p.update(0, 0, math.radians(90), (0.0, 5.0), sec, 0.05, 0.0)
    check('按意见向左让', plan.state == DODGE and plan.side == +1,
          '%s side=%d' % (plan.state, plan.side))
    p2 = AvoidPlanner(Config())
    p2.note_llm(+1, 0.0)
    sec2 = (INF, INF, INF, INF, INF, 0.50, 0.50, 0.35, INF)  # 左侧贴脸, 右侧全空
    plan2 = p2.update(0, 0, math.radians(90), (0.0, 5.0), sec2, 0.05, 0.0)
    check('窄侧被否决 -> 向右让', plan2.side == -1, 'side=%d' % plan2.side)

    print('=== 8. 0.5m 窄缝: 不该硬挤 ===')
    narrow = [(-0.6, 1.5, 0.35), (0.6, 1.5, 0.35)]
    r, trail, mc, st = simulate(narrow, (0.0, 0.0, 0.0), (0.0, 3.0), max_t=60)
    check('未挤进去撞上', mc > 0.20, 'min_clear=%.2f %s' % (mc, r))

    print()
    if FAILS:
        print('FAILED: %d 项 -> %s' % (len(FAILS), FAILS))
        return 1
    print('全部通过 ✓')
    return 0


if __name__ == '__main__':
    sys.exit(main())
