#!/usr/bin/env python3
"""随机场景压力测试: 检验避障不是只对 test_avoid_planner.py 的那 8 个场景过拟合。

机器人固定从原点出发, 目标方向/距离、障碍位置/半径都随机。
统计到达率、碰撞率、超时率、全程最小净空。
"""
import math
import random
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_avoid_planner as T


def make_scene(rng):
    ang = rng.uniform(-math.pi, math.pi)
    dist = rng.uniform(4.0, 6.0)
    goal = (dist * math.cos(ang), dist * math.sin(ang))
    n = rng.randint(4, 12)
    obs = []
    while len(obs) < n:
        x = rng.uniform(-6.0, 6.0)
        y = rng.uniform(-6.0, 6.0)
        if math.hypot(x, y) < 0.8:
            continue
        if math.hypot(x - goal[0], y - goal[1]) < 0.8:
            continue
        obs.append((x, y, rng.uniform(0.22, 0.42)))
    return obs, goal


def main(n_seed=30):
    ok = crash = stuck = 0
    worst = 9.9
    worst_seed = None
    for s in range(n_seed):
        rng = random.Random(s)
        obs, goal = make_scene(rng)
        r, trail, mc, st = T.simulate(obs, (0.0, 0.0, 0.0), goal, max_t=90)
        if r == 'reached':
            ok += 1
        elif mc < 0.0:
            crash += 1
        else:
            stuck += 1
        if mc < worst:
            worst, worst_seed = mc, s
        if mc < 0.0:
            print('  seed %d 撞了 (min_clear=%.2f)' % (s, mc))
    print('到达 %d/%d, 碰撞 %d, 未达 %d, 最差净空 %.2f (seed %s)'
          % (ok, n_seed, crash, stuck, worst, worst_seed))
    return 0 if crash == 0 and ok >= n_seed * 0.8 else 1


if __name__ == '__main__':
    sys.exit(main(int(sys.argv[1]) if len(sys.argv) > 1 else 30))
