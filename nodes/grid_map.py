#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""局部占用栅格 + A* —— 给 VFH 一层「该往哪儿走」。

VFH (nodes/avoid_planner.py) 是纯贪心的: 每周期只在车头左右各 100 度里挑
「偏差最小、净空够」的那条线。它在 mixed 场景一次撞上三个死穴:

  1) 车只能沿车头直行 (两个 prismatic 关节, 没有横移), 改变方向只能原地转身;
  2) go_limit/back_limit = 100 度, 「真正开出去的方向最多偏目标 100 度」——
     「先绕到左后方再折回」这种必须先背离目标的走法永远进不了候选;
  3) 36 个 10 度扇区只记「这个方位现在多空」, 不记形状 —— 矮箱右缘 x=0.4
     与墙左缘 x=0.7 之间 0.30 m 的死缝, 沿射线看是 1.78 m 的自由距离,
     VFH 判「可通」, 车开过去才发现过不去, 然后原地转到超时。

这一层只管「大致往哪走」, 安全仍然全交给 VFH:
  * 车边走的滚动栅格, 不建图、不发布、不持久化 —— 车走了就忘;
  * 雷达 450 束直接投影成占用格 (10 Hz)。0.10 m 高的矮箱整个在雷达扫描面
    (z=0.235 m) 之下, 雷达对它一无所见, 深度相机是它唯一的证人:
    当深度兜底报出「正前方 X 米有东西」时, 在车头方向 X 处补一个粘滞点;
  * 按车体等效半径 0.25 m 膨胀: 0.30 m 死缝两侧膨胀后重叠 -> 判死;
    0.70 m 窄缝还剩 0.20 m (2 格) -> 仍可穿, 不丢原有的窄缝能力;
  * 8 邻域 A* (不许斜穿墙角), 取车前方 1.5 m 处一点当方向喂给 AvoidPlanner;
  * 算不出路就返回 None, 调用方原样退回纯 VFH 行为 —— 不引入新的死法。

纯逻辑模块, 不 import rclpy: python3 nodes/grid_map.py 可以直接跑自测。
"""

import heapq
import math

RES = 0.10          # 栅格边长 (m): 0.30 m 死缝在膨胀后要能被判死, 0.70 m 窄缝要留 2 格
INFLATE = 0.25      # 车体等效半径 (m): 半宽 0.20 + 余量 0.05
SOFT_R = 0.50       # 「贴边」软代价半径 (m): 硬膨胀之外再围一圈, 走进去加价
SOFT_COST = 4.0     # 贴边每格加价 (相当于 0.4 m 路程): 宁可绕远, 也别贴着缝硬挤。
                    # 矮箱右缘 0.4 与墙左缘 0.7 那个 0.30 m 死缝, 纯几何上车宽 0.30
                    # 刚好等于缝宽 —— A* 会给出「之字形斜切」的解, 物理上成立但实测
                    # 过不去 (要精确对中、反复转向); 加价之后绕左 10 m 反而更便宜。
SCAN_TTL = 1.5      # 雷达点保留 (s): 一帧 10 Hz, 留 15 帧足够扛抖动, 弃点也很快忘
STICKY_TTL = 8.0    # 深度补的点保留 (s): 车贴脸时反复看到同一个箱子, 不能每帧续命,
                    # 否则绕过之后它还堵着; 8 s 够以 0.25 m/s 走出 2 m
ESCAPE_R = 0.60     # 起点逃生半径 (m): 车贴脸时自己就在膨胀区里, 不让它挪就永远出不来
LOOKAHEAD = 1.5     # 前视距离 (m): 太短会跟着栅格锯齿左右抖, 太长会切墙角
MIN_USE = 0.45      # 路径短于这个长度就别用了, 说明车已经贴到目标/贴到墙
MAX_EXPAND = 8000   # A* 展开上限: 墙后不可达时别把 CPU 全吃掉
H_WEIGHT = 1.4      # 加权 A*: 路径略次一点, 换 3-5 倍的展开数下降 (本层只要方向)

INF = float('inf')
_NEI = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))


class GridMap:
    def __init__(self, res=RES, inflate=INFLATE):
        self.res = float(res)
        self.inflate = float(inflate)
        self.occ = {}            # (ix, iy) -> 过期时刻 (s), 原始观测点
        self.blocked = set()     # 膨胀后的格, rebuild() 重建
        self.soft = set()        # 再往外一圈「贴边区」, 可走但代价高
        self._disk = self._make_disk(self.inflate / self.res)
        self._disk2 = self._make_disk(SOFT_R / self.res)
        self.path_len = 0.0      # 上一次规划的路径长度 (m), 只用于日志
        self.n_occ = 0

    # ---------- 观测 ----------
    @staticmethod
    def _make_disk(r_cells):
        n = int(math.ceil(r_cells))
        lim = r_cells * r_cells
        out = []
        for dx in range(-n, n + 1):
            for dy in range(-n, n + 1):
                if dx * dx + dy * dy <= lim:
                    out.append((dx, dy))
        return tuple(out)

    def _cell(self, x, y):
        r = self.res
        return int(math.floor(x / r)), int(math.floor(y / r))

    def apply_scan(self, ranges, angle_min, angle_inc, x, y, yaw, now,
                   max_range=8.0, min_range=0.12):
        """一帧激光 -> 占用格。世界角 = yaw + 相对角 (ROS 约定, 逆时针为正)。"""
        r0 = self.res
        occ = self.occ
        keep = now + SCAN_TTL
        for i, r in enumerate(ranges):
            if not (min_range <= r <= max_range):
                continue        # inf / nan / 太近的自遮挡
            a = yaw + angle_min + i * angle_inc
            ix = int(math.floor((x + r * math.cos(a)) / r0))
            iy = int(math.floor((y + r * math.sin(a)) / r0))
            occ[(ix, iy)] = keep
        return len(occ)

    def sticky(self, x, y, now, ttl=STICKY_TTL):
        """深度看到「正前方 X 米有东西」: 在那儿补一个点。

        雷达从矮箱头顶飞过, 只有深度相机看得见它。这里一次只加一个新格 ——
        已有就不续命, 否则车一直贴着同一个箱子时那个位置永不销毁。
        半径 0.30 m 的盘: 单个点太细, A* 会从旁边一格溜过去。
        """
        key = self._cell(x, y)
        if key in self.occ:
            return key
        n = int(math.ceil(0.30 / self.res))
        r2 = (0.30 / self.res) ** 2
        for dx in range(-n, n + 1):
            for dy in range(-n, n + 1):
                if dx * dx + dy * dy <= r2:
                    self.occ[(key[0] + dx, key[1] + dy)] = now + ttl
        return key

    def rebuild(self, now):
        """清过期 + 重新膨胀。占用点数在 500-1500 量级, 全量重建比增量维护简单可靠。"""
        occ = self.occ
        if occ:
            dead = [k for k, t in occ.items() if t < now]
            for k in dead:
                del occ[k]
        self.n_occ = len(occ)
        blocked = set()
        soft = set()
        disk = self._disk
        disk2 = self._disk2
        for (ix, iy) in occ:
            for dx, dy in disk:
                blocked.add((ix + dx, iy + dy))
            for dx, dy in disk2:
                soft.add((ix + dx, iy + dy))
        self.blocked = blocked
        self.soft = soft
        self.n_soft = len(soft)
        return len(blocked)

    def clear(self):
        self.occ.clear()
        self.blocked = set()
        self.soft = set()
        self.path_len = 0.0
        self.n_occ = 0

    # ---------- 规划 ----------
    def plan(self, x, y, gx, gy, max_expand=MAX_EXPAND):
        """8 邻域 A*, 返回世界坐标折线 [(x,y), ...] 或 None。

        目标在栅格外面也没关系: 搜索不设硬窗口, 由启发式往目标方向推进,
        展开数超限时退回「已展开格中离目标最近的那个」—— 也就是走到能看到
        的最远处, 车一动下一拍就重规划 (滚动式)。
        """
        r = self.res
        sx, sy = self._cell(x, y)
        tx, ty = self._cell(gx, gy)
        blocked = self.blocked
        soft = self.soft
        occ = self.occ
        esc2 = (ESCAPE_R / r) ** 2

        def free(ix, iy):
            # 真观测到的点任何时候都不许穿; 膨胀区在起点附近放行, 让车能挤出来
            if (ix, iy) in occ:
                return False
            if (ix - sx) ** 2 + (iy - sy) ** 2 <= esc2:
                return True
            return (ix, iy) not in blocked

        openh = [(0.0, 0.0, sx, sy)]
        g = {(sx, sy): 0.0}
        parent = {}
        best = (sx, sy)
        best_h = math.hypot(tx - sx, ty - sy)
        expand = 0
        hw = H_WEIGHT
        while openh and expand < max_expand:
            _, gc, ix, iy = heapq.heappop(openh)
            if gc > g.get((ix, iy), INF) + 1e-9:
                continue        # 懒删除: 堆里留着的旧代价
            expand += 1
            h = math.hypot(tx - ix, ty - iy)
            if h < best_h:
                best_h, best = h, (ix, iy)
            if ix == tx and iy == ty:
                best = (ix, iy)
                break
            for dx, dy in _NEI:
                nx, ny = ix + dx, iy + dy
                if not free(nx, ny):
                    continue
                if dx and dy and (not free(ix + dx, iy) or not free(ix, iy + dy)):
                    continue    # 不许斜着穿墙角: 车头是方的, 不是点
                ng = gc + (1.4142 if (dx and dy) else 1.0)
                if (nx, ny) in soft:
                    ng += SOFT_COST     # 贴着障碍走: 加价, 逼 A* 选更宽的绕行
                if ng < g.get((nx, ny), INF):
                    g[(nx, ny)] = ng
                    parent[(nx, ny)] = (ix, iy)
                    heapq.heappush(openh, (ng + hw * math.hypot(tx - nx, ty - ny),
                                           ng, nx, ny))
        if best == (sx, sy):
            self.path_len = 0.0
            return None
        pts = []
        cur = best
        while cur != (sx, sy):
            pts.append(((cur[0] + 0.5) * r, (cur[1] + 0.5) * r))
            nxt = parent.get(cur)
            if nxt is None:
                break
            cur = nxt
        pts.append((x, y))
        pts.reverse()
        total = 0.0
        px, py = pts[0]
        for qx, qy in pts[1:]:
            total += math.hypot(qx - px, qy - py)
            px, py = qx, qy
        self.path_len = total
        return pts

    def lookahead(self, path, x, y, dist=LOOKAHEAD):
        """沿路径量出 dist 米处的一点。路径太短就不给 —— 车已经贴到墙了。"""
        if not path or len(path) < 2 or self.path_len < MIN_USE:
            return None
        acc = 0.0
        px, py = path[0]
        for qx, qy in path[1:]:
            seg = math.hypot(qx - px, qy - py)
            if acc + seg >= dist:
                k = (dist - acc) / seg if seg > 1e-9 else 0.0
                return px + (qx - px) * k, py + (qy - py) * k
            acc += seg
            px, py = qx, qy
        return px, py


# ---------------- 自测: python3 nodes/grid_map.py ----------------
def _box(gm, x0, y0, x1, y1, step=0.10):
    """矩形内按 step 铺永久占用点 (模拟「雷达已经扫到这片」)。"""
    nx = max(1, int(abs(x1 - x0) / step))
    ny = max(1, int(abs(y1 - y0) / step))
    for i in range(nx + 1):
        for j in range(ny + 1):
            px = x0 + (x1 - x0) * i / nx
            py = y0 + (y1 - y0) * j / ny
            gm.occ[gm._cell(px, py)] = 1e9


def _selftest():
    ok = True

    def case(name, cond, extra=''):
        nonlocal ok
        print(('  ok  ' if cond else '  FAIL ') + name + (('  ' + extra) if extra else ''))
        ok = ok and cond

    print('1) 空地直行: (0,0) -> (0,8)')
    gm = GridMap()
    gm.rebuild(0.0)
    p = gm.plan(0.0, 0.0, 0.0, 8.0)
    aim = gm.lookahead(p, 0.0, 0.0)
    case('有路径', p is not None, 'len=%.2f m' % gm.path_len)
    case('前视点在前方 1.5 m', aim is not None and abs(aim[0]) < 0.2
         and abs(aim[1] - 1.5) < 0.25,
         'aim=(%.2f, %.2f)' % aim if aim else '')

    print('2) mixed 真实几何: 矮箱 + 右墙 + 普锐斯 + 行人, (0,0) -> (1,8)')
    rects = [(-0.4, 1.8, 0.4, 2.2),        # low_box_mid, 0.10 m 高, 雷达看不见
             (0.7, 3.4, 1.7, 3.8),         # wall_right
             (0.75, 4.32, 2.85, 6.08),     # prius hybrid, 横置
             (1.15, 6.65, 1.65, 7.15)]     # walking person
    gm = GridMap()
    for rc in rects:
        _box(gm, *rc)
    gm.rebuild(0.0)
    p = gm.plan(0.0, 0.0, 1.0, 8.0)
    ys = [q[1] for q in p] if p else []

    def inside(pt, margin=0.0):
        for x0, y0, x1, y1 in rects:
            if x0 - margin <= pt[0] <= x1 + margin and y0 - margin <= pt[1] <= y1 + margin:
                return True
        return False

    hit = [q for q in p or [] if inside(q)] if p else []
    case('有路径', p is not None, 'len=%.2f m' % gm.path_len)
    case('不穿进任何障碍', not hit, '%d 个点越界' % len(hit))
    case('最后能到目标附近', bool(ys) and max(ys) > 7.0,
         'max y=%.2f' % max(ys) if ys else '')

    print('2b) 只有矮箱: (0,0) -> (0,8), 必须绕开它')
    gm = GridMap()
    _box(gm, -0.4, 1.8, 0.4, 2.2)
    gm.rebuild(0.0)
    p = gm.plan(0.0, 0.0, 0.0, 8.0)
    xs = [q[0] for q in p] if p else []
    case('绕开了 (最大侧偏 > 0.55 m)', bool(xs) and max(abs(v) for v in xs) > 0.55,
         'max |x|=%.2f' % max(abs(v) for v in xs) if xs else '')

    print('3) 0.70 m 窄缝必须还能穿 (膨胀 0.25 后剩 2 格)')
    gm = GridMap()
    _box(gm, 0.35, 0.5, 20.0, 0.9)         # 横贯整场的隔断, 只在中间留缝
    _box(gm, -20.0, 0.5, -0.35, 0.9)
    gm.rebuild(0.0)
    p = gm.plan(0.0, 0.0, 0.0, 3.0)
    ymax = max((q[1] for q in p), default=-1) if p else -1
    case('穿过去了 (y > 1.2)', ymax > 1.2, 'y max=%.2f' % ymax)

    print('4) 0.30 m 死缝必须判死 (膨胀 0.25 后封口)')
    gm = GridMap()
    _box(gm, 0.15, 0.5, 20.0, 0.9)
    _box(gm, -20.0, 0.5, -0.15, 0.9)
    gm.rebuild(0.0)
    p = gm.plan(0.0, 0.0, 0.0, 3.0)
    ymax = max((q[1] for q in p), default=-1) if p else -1
    case('没穿过去 (y < 1.0)', ymax < 1.0, 'y max=%.2f' % ymax)

    print('5) 贴脸逃生: 车站在膨胀区里也要能规划出去')
    gm = GridMap()
    _box(gm, -0.4, 1.8, 0.4, 2.2)
    gm.rebuild(0.0)
    p = gm.plan(0.0, 1.55, -2.0, 4.0)
    case('有路径', p is not None, 'len=%.2f m' % gm.path_len)

    print('')
    print('全部通过' if ok else '有用例失败')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(_selftest())

