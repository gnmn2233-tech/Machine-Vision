#!/usr/bin/env python3
"""老姿态避障规划器 (VFH 简化版)。

为什么是这个形状
----------------
这台底盘只会「原地转向」和「沿车头直行」(位置伺服, 没有 /cmd_vel, 不横移),
所以避障唯一能输出的东西就是 **一个方向**: 车头该指哪儿, 以及**现在能不能往前挪**。

不用 waypoint(绕行点) 那套 —— 固定绕行点在这台车上必翻车: 它先转向、再直行,
直行到点位时已经贴在障碍前, 而障碍有多宽它事先并不知道。

用 VFH (Vector Field Histogram) 的思路:
  1. 相机视野内每个扇区给出「沿该方向还能走多远」;
  2. 把这些距离按**绝对方位**记进一张极坐标小图 (机器人一转, 视野就跟着扫宽);
  3. 从「正对目标」开始, 以 10 度步长往两边找, 选**最接近目标方向且够宽**的那条;
  4. 视野内全都不够宽时, 不动, 只转 —— 把车头转向「还没看过」的方位去补测;
  5. 全周都测过还是不够宽(走廊尽头/被围死) -> 停住等着, 不硬挤。

关键点: **转向 = 扫描**。这台车的相机与车头同轴, 所以转头就是在测侧方,
不需要额外传感器就能绕开「宽度未知」的障碍 —— 贴到墙前也只会变成「转头找边」。

本模块是纯逻辑 (不依赖 ROS), 便于离线跑几何测试。
"""
from dataclasses import dataclass
import math

# ---- 状态 ----
NAV = 'NAV'        # 朝最终目标走
DODGE = 'DODGE'    # 朝让开障碍的方向走
HOLD = 'HOLD'      # 不走 (可能仍在原地转头观察), 等障碍让开

INF = float('inf')
GRID_DEG = 10.0
N_GRID = int(round(360.0 / GRID_DEG))


def wrap_pi(a):
    """把角度折进 (-pi, pi]。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


@dataclass
class Plan:
    """一次决策的结果 —— 导航节点只认 heading / advance 两个字段。"""
    state: str = NAV
    heading: float = 0.0      # 本周期建议的**绝对**朝向 (rad)
    goal_dir: float = 0.0     # 到目标的直线方向 (rad), 仅用于日志/对比
    side: int = 0             # +1 向左让, -1 向右让, 0 直走
    advance: bool = True      # False -> 只转车头, 不发位移指令
    reason: str = ''
    clear: float = INF        # 选中方向上还能走多远 (m)
    sectors: tuple = ()       # 本周期输入 (调试用)


@dataclass
class Config:
    hfov: float = 1.047       # 深度相机水平视场 (rad), 与 obstacle_avoider 一致
    n_sectors: int = 9        # 视野被切成几个扇区
    safe_dist: float = 0.60   # 一个方向「够宽」的门槛 (m), 含车体与余量
    hard_dist: float = 0.28   # 车头正前方小于它 -> 立刻停 (m)
    margin: float = 0.25      # 走廊纵向 reach 的余量项 (m)。
    side_gap: float = 0.40    # 横向擦边净距 (m): 车体半宽 0.20 + 障碍表面余量 0.20。
                              # 原来判「挡」用的是 margin 0.25 -> 车侧壁离障碍表面只剩 5 cm,
                              # 行人 (capsule 半径 0.25) 擦一下就倒 ((0,8) 实测撞倒两次)。
                              # 0.45 实测能过但假阻挡太多: 车头偏 20 度、横向 0.41 m 的正常
                              # 走廊也被判堵, 车卡在 look 循环里 120 s 只走了 3.3 m。
    lookahead: float = 0.55   # 判「堵」时额外往前看的距离 (m): 转向要 1.5 s, 得早一点动手
                              # (0.85 m 才判堵 -> 车头已贴到障碍侧边, 左右全堵只剩后退)
    window_deg: float = 25.0  # 「正前方」的角度窗口 (deg), 窗口内要求沿射线够远
    mark_ttl: float = 2.5     # 极坐标记忆有效期 (s); 过期视为「没看过」, 会再转过去看
    sticky_ttl: float = 0.45  # 记住「这里有障碍」的最短时间 (s): 期间不许改回「空旷」
    mark_move: float = 0.20   # 车身挪动超过这个距离 (m), 之前按旧位置记的方位就作废
    commit_ttl: float = 2.0   # 选定一个绕行方向后至少坚持这么久 (s), 免得左右横跳
    commit_min: float = 1.2   # 一个绕行方向至少坚持这么久 (s) 才允许被 LLM 改边。
                              # 0 = 不设下限, 相邻两帧就能换边 —— 9B 的回答在
                              # 1-3 s 之间抖动, 那等于把抖动量直接喂进转向。
    commit_max: float = 3.5   # 一个绕行方向最多坚持这么久 (s): 到点必须回头看一眼目标
    commit_span: float = 60.0  # 坚持期间允许沿同一侧把绕行角加大到这么多度
    back_limit: float = 100.0 # 最多允许偏离目标方向多少度 (deg), 超出就当看不见
                              # 130 时车会转头 ±160 去看身后: 看了也不许开过去(go_limit),
                              # 白转一趟再转回来, 日志里一秒换一个方向就是它。
    go_limit: float = 100.0   # 真正**开出去**的方向最多偏目标多少度 (deg): 更偏的只许转头看
                              # 110 度时车会朝侧后方"倒着绕", 一趟 3.5 m 走成 7.2 m (实测);
                              # 85 度则贴到障碍前就没路了 —— 车不能侧移, 贴脸时只剩 ±90 可选。
                              # (90 度能砍掉倒车, 但 3m 宽墙要横移 1.5m+, 会卡死; 110 两头都过)
    look_hold: float = 2.5    # 「转过去看」这个动作要粘住这么久 (s): 转向约 1 rad/s,
                              # 转 90 度要 1.6 s, 每周期重算会让车头原地抖 (实测 ±100 度翻)
    step_deg: float = 10.0    # 方向搜索步长 (deg)
    fuse_depth: bool = False  # 有雷达时是否仍把深度图并进记忆; 默认关, 理由见 _merge_depth
    scan_max: float = 9.0     # 激光雷达读数超过它就算「这一边空旷」(m)。
                              # 量程 12 m, 留 3 m 余量, 免得远处的墙被当成近处障碍。
    llm_ttl: float = 8.0      # 外部(LLM)意见有效期 (s)。4 s 实测不够: 车进了 commit
                              # 分支要绕满 commit_max 3.5 s 才回头看, 加上 LLM 自己
                              # 0.8-2.8 s 的延迟, 意见到能用的那一刻往往已经过期了
                              # (три 次询问/一次回答, DBG 一次都没打印)。
    llm_tie_deg: float = 30.0 # 两侧绕行偏角相差不超过它就认为「几何分不出高下」, 只有
                              # 这时才允许 LLM 的意见改变搜索顺序 (SayCan 式加权); 差得多
                              # 说明几何自己有偏好, 模型不许插手 —— 权重的分母是几何。
                              # 判偏角而不是判净空: 可行方向的净空几乎总是量程上限
                              # (不够远就不可行), 拿它比大小等于恒等, 比不出任何东西。
    near_proj: float = 0.90   # 纵向投影小于它的障碍算「马上要擦肩而过」
    near_gap: float = 0.50    # 这类近距离障碍要求的横向净距 (m), 比 side_gap 严。
                              # 一刀切加大 side_gap 会误伤远处的走廊 (实测 0.45 时
                              # 车头偏 20 度、横向 0.41 m 也判堵, 车卡在 look 循环里
                              # 120 s 只挪 3.3 m); 只对真正要错身的那一段加严, 远处
                              # 仍按 0.40 —— (0,8) 实测最小净距 0.15 m 就来自这一段。
    ring_step: float = 30.0   # 喂给 LLM 的净空环分辨率 (deg): 12 格转一圈


class AvoidPlanner:
    def __init__(self, cfg=None):
        self.cfg = cfg or Config()
        self.state = NAV
        self.side = 0
        self.marks = {}           # 方位格 -> (距离, 时间戳, 是否实测); 缺格 = 没看过
        self.block_why = ''       # 最近一次 _strict 判不通的元凶, 供日志打印
        self._px = None           # 上一次的位姿, 用来判断记忆是否已经错位
        self._py = None
        self._commit = 0.0        # 当前坚持的绕行方向 (相对目标方向的偏角, deg)
        self._commit_t0 = 0.0     # 这个绕行方向是什么时候选定的 (用于限制总坚持时长)
        self._commit_until = 0.0
        self.last_sectors = ()
        self.last_reason = ''
        self._llm_side = 0
        self._llm_until = 0.0
        self._llm_conf = 0.0
        self.pref_src = ''        # 上一拍的选择理由来自谁: '' 几何 / 'llm' 模型的偏好
        self.tie_info = ''        # 最近一次「模型有没有资格开口」的现场, 供日志打印
        self._yaw = 0.0           # 最近一次 update 的车头朝向, 供 clearance() 换算绝对方位
        self._side_hist = []      # (时间, 绕行方向) 最近几秒, 用来发现「来回换边」
        self._look_heading = None # 正在转过去看的那个方位 (rad); 粘住它, 别每帧换
        self._depth_confirm = {}  # 方位格 -> (上帧深度, 连续帧数); 相机读数要连续一致才采信
        # 正前方雷达看不见的低矮障碍 (箱子/台阶/桌腿) 的最近距离; None=没看到
        self._depth_front = None
        self._depth_front_t = 0.0
        self._goal_dir = 0.0       # 本拍目标方位, 深度否决要锚在它上面 (见 _strict)
        # 正前方净空 (m), inf=空旷。上层据此决定「要不要报警」—— 判堵用的 1.40 m
        # 是提前打方向的量, 拿它当警报线会吓人; 真正会撞的线是 safe_dist。
        self.front_gap = INF
        self._look_until = 0.0
        self.scan_ok = False      # 本帧拿到了激光雷达没有
        self._raw_scan = None     # 最近一帧原始雷达 (ranges, angle_min, angle_inc)
        self.narrow_why = ''      # 窄缝判定为什么没成立, 供日志标定 (调试用)
        self._ndbg_t = 0.0
        self._narrow_w = None     # 本拍正对的那条固定窄缝有多宽 (m); None = 不在缝里
        self._narrow_off = 0.0    # 缝中线相对车体中线的横向偏移 (m, 左正)
        self._narrow_d = 1.0      # 到墙口的纵向距离 (m), 用作对中修正的分母

    # ---------------- 外部策略注入 ----------------
    def note_llm(self, side, now, ttl=None, conf=0.0):
        """外部模型给出「往哪边让」的意见: +1 左, -1 右, 0 未知。

        它只影响搜索顺序(先试哪边), 不能把一个不够宽的方向变成够宽 ——
        安全判据始终由几何说了算。conf 是模型自报的置信度, 只进日志不参与判据:
        7B 的置信度校准很差, 拿它当权重反而把偶然的一次蒙对放大成纪律。
        """
        self._llm_side = int(side)
        self._llm_conf = float(conf)
        self._llm_until = now + (self.cfg.llm_ttl if ttl is None else ttl)

    def llm_active(self, now):
        return self._llm_side != 0 and now < self._llm_until

    # ---------------- 对外查询 (给上层日志 / LLM 顾问用) ----------------
    def clearance(self, deg_rel, now, half_span=20.0):
        """相对车头 deg_rel 方向上「还能走多远」(m)。

        返回 None = 这一片还没看过 (相机在车头 ±30 度内, 侧方要靠转头扫到);
        inf = 看过且什么都没有。窗口取 ±half_span 内的**最近**记录 —— 宁可报窄,
        让模型以为这边更堵, 也好过诱它往墙上撞。
        """
        vals = []
        step = max(int(GRID_DEG), 1)
        for d in range(-int(half_span), int(half_span) + 1, step):
            v = self._mark(wrap_pi(self._yaw + math.radians(deg_rel + d)), now)
            if v is not None:
                vals.append(v)
        return min(vals) if vals else None

    def ring(self, now, step=None, half_span=14.0):
        """相对车头一整圈的净空 (m), 从 0 度起按 step 递增 —— 喂给 LLM 的「状态」。

        为什么给 12 个数而不是左中右三个: 模型只拿到「左 1.5 / 右 1.4」时,
        「两边都宽敞」和「两边都宽敞但身后全堵」在它眼里没区别, 只能靠猜
        (实测 7B 对 4 组截然不同的场景一律回 LEFT)。给它一圈, 它才看得出
        这是走廊、是死角, 还是已经贴着墙。half_span 取小于半个步长,
        免得相邻两格读同一个格子的数, 看不出「这里是边界」。
        """
        st = self.cfg.ring_step if step is None else step
        n = max(1, int(round(360.0 / st)))
        out = []
        for i in range(n):
            deg = i * st
            if deg > 180.0:
                deg -= 360.0
            out.append(self.clearance(deg, now, half_span=half_span))
        return tuple(out)

    def flap(self, now, window=2.0, need=2):
        """最近 window 秒内绕行方向换了几次; 达到 need 就说明在左右横跳。"""
        hist = [(t, s) for (t, s) in self._side_hist if now - t <= window]
        if len(hist) < 2:
            return 0
        flips = 0
        for i in range(1, len(hist)):
            if hist[i][1] != hist[i - 1][1]:
                flips += 1
        return flips if flips >= need else 0

    def reset(self):
        self.state = NAV
        self.side = 0
        self.marks.clear()
        self._px = None
        self._py = None
        self._commit = 0.0
        self._commit_until = 0.0
        self._llm_side = 0
        self._llm_until = 0.0
        self._llm_conf = 0.0
        self.pref_src = ''

    # ---------------- 主入口 ----------------
    def update(self, x, y, yaw, goal, sectors, now, speed=0.0, scan=None):
        """每周期调用一次, 返回本周期车头该指哪儿、能不能往前挪。

        x, y, yaw : 实测位姿 (yaw 弧度)
        goal      : 最终目标 (gx, gy)
        sectors   : 深度相机视野内从左到右的净空 (m), inf=该方向空旷
        speed     : 当前速度 (m/s), 目前只用于日志
        scan      : 激光雷达一帧 (ranges, angle_min, angle_increment)。给了就用它 ——
                    360 度同一时刻全覆盖, 不再需要靠转头补盲区
        """
        c = self.cfg
        self.block_why = ''
        self._yaw = yaw
        if self.side:               # 只在「真的在绕」时记一笔, 直线走不算换边
            self._side_hist.append((now, self.side))
            while self._side_hist and now - self._side_hist[0][0] > 2.0:
                self._side_hist.pop(0)
        # 雷达模式下深度图可能还没来/已断, sectors 会是 None
        sec = tuple(float(v) for v in sectors) if sectors else ()
        self.last_sectors = sec
        # 极坐标记忆记的是「旧位置上看到的方位」: 车一挪位置, 这些方位全错位,
        # 会把自己围死 (所有方向都记着贴脸, 于是原地乱摆头)。挪远了就重新开始看。
        self.scan_ok = scan is not None
        self._raw_scan = scan
        # 深度兜底: 雷达装在 z=0.235 m, 比它矮的东西 (0.10 m 高的箱子、台阶、桌腿)
        # 每一束光线都从顶上飞过去 —— 雷达眼里这条路完全通畅, 车就这么顶上去卡死。
        # 深度相机平视、离地 0.28 m, 视野下边缘打在地面 0.65 m 处: 0.65 m 以外
        # 矮物的顶面正常成像 (实测 1.0 m 处读到 1.00, 0.4 m 处读到 0.44, 空旷场全 inf)。
        # 所以这里只取「正前方那几个扇区的最小值」当一路额外信号, 不并进 marks ——
        # 深度只有 +/-30 度视野, 进了记忆就会在车转头的瞬间被当成「这个方向也空了」。
        if sec:
            _vals = [v for v in sec if math.isfinite(v)]
            if _vals:
                _d = min(_vals)
                if _d < c.safe_dist + c.margin + c.lookahead:
                    self._depth_front = _d
                    self._depth_front_t = now
        # 看到后记忆保留 2.5 s: 深度只有 +/-30 度视野, 车一转头箱子就出画。
        # 立刻清空的话, 车会马上「转回去看一眼」-> 又看到 -> 再转走, 左右摆头一步不挪
        # (这个坑在 look_hold 上也踩过一次)。2.5 s 足够以 0.5 m/s 走出 1.2 m 绕开。
        if self._depth_front is not None and now - self._depth_front_t > 2.5:
            self._depth_front = None
            self._depth_front_t = 0.0
        # 正前方净空: 深度优先 (雷达从矮物头顶飞过, 看不见它们), 没有深度就退回雷达。
        self.front_gap = INF
        if self._depth_front is not None:
            self.front_gap = self._depth_front
        else:
            _fm = self._mark(yaw, now)
            if _fm is not None and math.isfinite(_fm):
                self.front_gap = _fm
        if self.scan_ok:
            # 雷达每帧把一整圈重新测一遍: 旧方位记忆没有价值, 更不该按位移外推
            # (外推是给「转头扫出来的稀疏记忆」用的, 用在每秒全刷新的雷达上只会引入误差)
            self._ingest_scan(scan[0], scan[1], scan[2], yaw, now)
            if c.fuse_depth and sec:
                self._merge_depth(yaw, sec, now)
        else:
            if self._px is not None:
                _dxm, _dym = x - self._px, y - self._py
                _moved = math.hypot(_dxm, _dym)
                if _moved > 1.0:
                    self.marks.clear()      # 被搬走/瞬移: 旧方位全不可信
                elif _moved > 0.02:
                    self._shift_marks(_dxm, _dym, now)
            if sec:
                self._ingest(yaw, sec, now)   # 没相机数据时别清空已有记忆
        self._px, self._py = x, y

        gx, gy = goal
        dist_goal = math.hypot(gx - x, gy - y)
        goal_dir = math.atan2(gy - y, gx - x) if dist_goal > 1e-9 else yaw
        self._goal_dir = goal_dir

        near_now = self._mark(yaw, now)
        if near_now is not None and near_now < c.hard_dist:
            # 车头已经贴到障碍上: 先停, 别靠转向硬蹭
            self.state = HOLD
            self._look_heading = None            # 贴脸先停下, 别粘着旧方向硬转
            return self._plan(HOLD, yaw, goal_dir, 0, False,
                              'too close: %.2f m' % near_now, near_now)

        # 窄缝直穿: 两侧是固定结构、中间只剩一条比 side_gap 窄但比车宽的通道时,
        # 按车宽判而不是按行人余量判 (细节见 _narrow)。放在 _choose 之前, 因为
        # _choose 里每一次 _feasible 都会走 _strict, 用的就是这两个字段。
        nr = self._narrow(now)
        self._narrow_w, self._narrow_off = nr if nr else (None, 0.0)
        best = self._choose(goal_dir, now)
        if best is None:
            self.state = HOLD
            return self._plan(HOLD, yaw, goal_dir, 0, False,
                              'blocked all around (front %.2f)'
                              % (near_now if near_now is not None else -1.0),
                              near_now if near_now is not None else INF)

        heading, delta_deg, clear, kind = best
        if self._narrow_w is not None:
            # 窄缝成立 = 缝中线两侧 0.24 m 内实测无物, 这条路确认是通的。
            # 此时**不让 DODGE 参与竞争**: 这车不能横移, 只会沿车头直行,
            # 偏出中线就再也回不来 —— 实测偏 0.19 m 时左墙角点落到放行窗口外 5 mm,
            # _strict 判堵 -> DODGE 往更左打 -> 偏更多 -> 一路甩到 176 度从墙外绕。
            # 所以直接把车头指向缝中线: 越靠近墙口打得越狠, 进了缝 mid 自动趋 0 拉直。
            d0 = max(0.40, min(1.20, self._narrow_d))
            aim = max(-0.45, min(0.45, math.atan2(self._narrow_off, d0)))
            heading = wrap_pi(yaw + aim)
            delta_deg = math.degrees(wrap_pi(heading - goal_dir))
            kind = 'go'
            clear = self._narrow_w
        if kind == 'go':
            self._set_commit(delta_deg, now)
        half = c.step_deg * 0.5
        side = 1 if delta_deg >= half else (-1 if delta_deg <= -half else 0)
        self.side = side
        if kind == 'go':
            self.state = NAV if abs(delta_deg) < 20.0 else DODGE
            return self._plan(self.state, heading, goal_dir, side, True,
                              '%s %+d deg clear=%.2f front=%s'
                              % (self.state, int(delta_deg), clear,
                                 '%.2f' % near_now if near_now is not None else '?'),
                              clear)
        # kind == 'look': 这个方向没测过(或全堵), 只转过去看, 不前进
        self._look_heading = heading            # 粘住, 下几帧继续朝它转
        self._look_until = now + c.look_hold
        self.state = DODGE if abs(wrap_pi(heading - yaw)) > math.radians(8.0) else HOLD
        return self._plan(self.state, heading, goal_dir, side, False,
                          'look %+d deg (clear=%s)'
                          % (int(delta_deg),
                             '%.2f' % clear if math.isfinite(clear) else '-'),
                          clear)

    # ---------------- 极坐标记忆 ----------------
    def _shift_marks(self, dx, dy, now):
        """车挪了一点: 把各方位的「还能走多远」减掉位移在该方向上的投影。

        以前的写法是「挪过 mark_move 就整车清空记忆」, 结果每前进 0.2 m 就失忆一次,
        车得重新转头扫一遍, 于是走走停停、慢得像卡住。按投影衰减两头都躲开了:
        近处的记录自动变近 (本来就该变近), 走过的方向自动作废。
        """
        dead = []
        for idx, (d, stamp, real) in self.marks.items():
            if now - stamp > self.cfg.mark_ttl:
                dead.append(idx)
                continue
            if not math.isfinite(d):
                continue
            ang = math.radians(idx * GRID_DEG)
            nd = d - (dx * math.cos(ang) + dy * math.sin(ang))
            if nd < 0.10:
                dead.append(idx)            # 这个方位已经走过去了
            else:
                self.marks[idx] = (nd, stamp, real)
        for idx in dead:
            self.marks.pop(idx, None)

    def _ingest_scan(self, ranges, angle_min, angle_inc, yaw, now):
        """激光雷达: 一次给全 360 度, 每一格都重写 —— 从此不存在「没看过」的方向。

        为什么必须换成雷达 (实测, 2026-10-27 行人场景):
        深度相机水平视场只有约 60 度 (左右各 30), 车宽 0.4 m, 在 0.6 m 安全距离处
        视野只剩 0.69 m 宽 —— 要判断 1.2 m 宽的障碍能不能绕过, 得退到 1 m 外; 想看清
        侧面只能转头发扫, 而 base_yaw 只有约 1 rad/s, 扫一次 1.5 s, 转过去的瞬间
        之前记的那一片又过期了。日志里的原话是「离行人 0.89 m, 测得到, 却在原地摆头
        20 秒走不过去」。雷达 450 条射线覆盖一圈, 同一时刻的信息是完整的,
        look(只转头不前进) 基本不再触发。

        每格取本帧最小值; 一圈都没打到的格子写 inf (这一边确实空) —— 不写「没看过」,
        因为那会让上层又跑去转头确认。
        """
        c = self.cfg
        n = len(ranges)
        if n == 0:
            return
        best = {}
        covered = set()
        for i in range(n):
            d = ranges[i]
            if d != d:                      # NaN
                continue
            idx = self._idx(wrap_pi(yaw + angle_min + i * angle_inc))
            covered.add(idx)
            if d < 0.05 or d > c.scan_max:
                continue                    # 太近是自己的影子; 太远算空旷
            if idx not in best or d < best[idx]:
                best[idx] = d
        for idx in covered:
            self.marks[idx] = (best.get(idx, INF), now, True)

    def _merge_depth(self, yaw, sectors, now):
        """深度相机: 只补雷达看不见的近处细节, 不覆盖雷达的读数。

        雷达是单线水平扫描 —— 低于扫描平面的东西完全隐身。实测 (2026-10-28):
        正前方 2 m 放一个 0.10 m 高的盒子, 雷达 0±6 度内 0 条回波 (回波还是 4.5 m
        外的行人), 而深度图有 3779 个像素从 2.0-8.0 m 变近到 1.7-2.0 m。
        所以相机不是冗余, 它是这条盲区唯一的来源。

        合并规则 (试过两版, 这版才对):
        * 相机说空旷 (inf) 一律不写 —— 抹掉雷达的有限值等于把真障碍从记忆里删掉。
        * **只填雷达看不见的格子** (雷达记的是 inf / 没记过 / 记的已过期)。
          第一版写成「相机更近就覆盖」, 结果行人场景从 19.2 s 掉到 49.2 s、DODGE
          从 5 次涨到 26 次: 相机在 1.2 m 处就开始报「挡」, 而雷达在 1.2 m 处
          本来还判「够宽」, 两套读数打架, 车在原地犹豫。
        第二版只补雷达的空格 + 连续 3 帧一致才采信, 行人场景 29.8 s 还能过, 但
        **矮障碍场景彻底卡死**: 正前方 0.4x0.4x0.10 m 的盒子, 75 s 只走 0.8 m,
        55 次 DODGE, 挡的方位在 +60/+80/+90/+100 度之间跳、距离在 0.43-1.31 m 之间
        跳, курс 在 +20 与 +170 之间翻 —— 车头左右摆, 一寸不动。
        根因三条: (a) 相机水平视场只有 60 度, 贴近矮物时它占满整幅, 无从判断
        往哪边绕; (b) 滤地平线用的是「比该行地面深度近 25%」, 矮物贴近时与地面
        深度几乎相同, 会被当成地面滤掉, 于是「远看得见、近看不见」; (c) 9 个扇区
        的中心角随车头摆动, 同一个盒子会在相邻扇区之间跳。
        结论: 这套 60 度单目 + 36 格极坐标记忆的架构吃不下稠密近距点云。默认关闭,
        代码留作可选 (fuse_depth=True), 深度相机回到「雷达断线时兜底」的定位。
        真要防低于扫描平面的东西, 加第二层扫描线 (多线雷达) 比融合深度图可靠。
        """
        c = self.cfg
        n = len(sectors)
        if n == 0:
            return
        for i, d in enumerate(sectors):
            if not math.isfinite(d):
                continue
            rel = -c.hfov / 2.0 + c.hfov * (i + 0.5) / n
            idx = self._idx(wrap_pi(yaw + rel))
            old = self.marks.get(idx)
            if old is not None and now - old[1] <= c.mark_ttl and math.isfinite(old[0]):
                continue          # 雷达这一格看得见, 相机不插嘴
            # 单帧读数会跳: 矮盒子在 4 m 外只有几个像素, front 在 inf / 0.5 / 1.47
            # 之间来回变, 直接写进去会让 _strict 的判据跟着抖 (实测 70 s 卡死、
            # 50 次 DODGE)。所以要求连续 3 帧都在近处且互相接近才认。
            prev = self._depth_confirm.get(idx)
            if prev is not None and abs(prev[0] - d) < 0.30:
                cnt = prev[1] + 1
            else:
                cnt = 1
            self._depth_confirm[idx] = (d, cnt)
            if cnt >= 3:
                self.marks[idx] = (d, now, True)

    def _ingest(self, yaw, sectors, now):
        """把本帧各扇区的净空写进绝对方位的格子 (机器人一转, 视野就跟着扫宽)。

        膨胀只补「本帧没测量到」的相邻格 —— 用本帧自己的值覆盖本帧自己的格子,
        否则一个近值会把旁边真正较宽的格子污染掉 (正前方明明 0.5m, 却被旁边
        0.1m 的扇区写成贴脸)。
        """
        c = self.cfg
        n = len(sectors)
        if n == 0:
            return
        measured = {}
        for i, d in enumerate(sectors):
            rel = -c.hfov / 2.0 + c.hfov * (i + 0.5) / n
            idx = self._idx(wrap_pi(yaw + rel))
            if idx not in measured or d < measured[idx]:
                measured[idx] = d
        for idx, d in measured.items():
            old = self.marks.get(idx)
            if (old is not None and now - old[1] <= c.sticky_ttl
                    and math.isfinite(old[0])
                    and (not math.isfinite(d) or d > old[0])):
                # 刚看到过障碍, 这一帧又说空旷 —— 多半是射线擦着障碍**边缘**过去了。
                # 车头轻轻一摆, 同一条射线就在「撞上」与「擦过」之间跳, 判据跟着跳,
                # 车头就每帧换个方向、一步也走不动 (踩过这个坑)。先信那个近的值。
                d = old[0]
            self.marks[idx] = (d, now, True)
        for idx, d in measured.items():
            for off in (-1, 1):
                j = (idx + off) % N_GRID
                if j in measured:
                    continue
                old = self.marks.get(j)
                if old is None or now - old[1] > c.mark_ttl or d < old[0]:
                    # 补出来的格子标成「非实测」: 只用来填视野缝隙, 不参与擦边判据
                    self.marks[j] = (d, now, False)

    def _idx(self, ang):
        return int(round(math.degrees(ang) / GRID_DEG)) % N_GRID

    def _mark(self, ang, now):
        """查某方位上记录到的净空; None = 没看过(或记录过期)。"""
        m = self.marks.get(self._idx(wrap_pi(ang)))
        if m is None or now - m[1] > self.cfg.mark_ttl:
            return None
        return m[0]

    def _feasible(self, ang, now):
        """朝 ang 走安不安全 —— 两条都要满足, 否则返回 None。

        光看「沿该方向的射线够远」是不够的: 射线可以**擦过**障碍边缘, 距离看着
        还有几米, 车却正好贴着障碍皮过去。所以再算一条「走廊」判据:
        把车前端沿 ang 方向放出一段 reach=safe_dist+margin 的矩形走廊, 宽 = 2*margin。
        任何实测障碍点只要落进这个走廊 (纵向投影 proj <= reach 且横向偏移 lat < margin),
        这条路就被挡住。
        远处的障碍不挡 —— 现在不挡不代表未来不挡, 等它近了自然会触发转向;
        要是在 2m 外就否决, 车会在原地转圈永远不敢动 (踩过这个坑)。
        """
        if not self._any_fresh():
            return None
        if not self._strict(ang, now):
            return None
        return self._mark(ang, now)

    def _strict(self, ang, now):
        """只问「已测到的障碍有没有挡住这条走廊」, 不问这个方向本身测过没有。

        分开问是为了「转头去看」的选向: 没测过的方向里, 也得先挑那些**现有障碍
        数据不反对**的, 否则车贴着障碍时会先朝 +10 度这种明显更堵的方向转,
        转完发现不行再转回来, 原地摆头一步不动。
        """
        c = self.cfg
        # 纵向 reach 必须带上提前量: 这车不能侧移, 发现到 0.6 m 才转向时已经来不及
        # (1 rad/s 转 90 度要 1.5 s, 同一时间还前进 0.75 m) —— 把 margin 减掉会直接撞穿。
        # 提前量: safe_dist 只是「不许再近」的底线, 不是「该开始绕」的时机。
        # 0.85 m 才判堵时, 车头已经贴到障碍侧边 (实测 (0.80, 0.06) 左右两侧全被挡死,
        # 只剩后退一条路 -> 于是疯狂摆头 120 秒不动)。留 0.55 m 让车离障碍 1.4 m 就开始绕。
        reach = c.safe_dist + c.margin + c.lookahead
        for idx, (d, stamp, real) in self.marks.items():
            if not real or now - stamp > c.mark_ttl or not math.isfinite(d):
                continue
            delta = wrap_pi(math.radians(idx * GRID_DEG) - ang)
            if abs(delta) >= math.pi / 2.0:
                continue                        # 障碍在身后, 不影响往前开
            proj = d * math.cos(delta)
            lat = d * math.sin(delta)
            if proj > reach:
                continue                        # 还在走廊外, 先不管
            need = c.near_gap if proj < c.near_proj else c.side_gap
            if self._narrow_w is not None:
                # 缝里两壁是固定结构, 没有「人会被蹭倒」这回事: 只要没进车体
                # (半宽 0.20) 就放它过。只对**落在缝范围内**的障碍放宽, 缝外的
                # 行人仍按 near_gap/side_gap 判 —— 别让一条隧道把别处的余量也吃掉。
                # 缝壁本身已经由 _narrow 用原始射线精确量过 (缝宽、缝中间有没有
                # 东西), 这里不能再拿 10 度格心的 lat 重复判一次 —— 那个量化误差
                # 恰好会把 0.35 m 的墙量成 0.24 m, 把好好的通道重新判成死路。
                lo_g = self._narrow_off - self._narrow_w * 0.5 - 0.12
                hi_g = self._narrow_off + self._narrow_w * 0.5 + 0.12
                if lo_g <= lat <= hi_g:
                    continue
            if abs(lat) < need:
                # 记下是谁挡的, 供上层日志打印 (调试"明明空旷却判不通")
                self.block_why = '%+d deg d=%.2f proj=%.2f lat=%+.2f' % (
                    idx * GRID_DEG, d, proj, lat)
                return False                    # 落进走廊 -> 这条路不通
        # 深度兜底: 雷达看不见的低矮障碍。只在「想直着走」时否决 ——
        # 深度视野只有 +/-30 度, 拿它去判 60 度以外的方向是越权 (那些方向深度根本没看,
        # 读到的 inf 是「没看见」不是「空旷」)。绕行方向照旧交给雷达。
        # 锚在**目标方向**而不是当前车头: 车一转到绕行方向, 那个方向就成了「正前方」,
        # 拿车头当参考的话它会立刻否决自己刚选的绕行道 —— 实测 +30/-30/+20/-20 每秒
        # 翻一次、原地摆头 10 s (lb10)。真正要禁的是「朝矮障碍直冲」, 障碍就在目标方向上。
        if (self._depth_front is not None
                and abs(wrap_pi(ang - self._goal_dir)) < math.radians(24.0)):
            self.block_why = 'depth %.2f m' % self._depth_front
            return False
        return True

    def _narrow(self, now):
        """本拍是否正对着一条「固定窄缝」: 两侧各有一段障碍、中间只剩一条比 side_gap
        窄、却比车体宽的通道 (走廊场景那 0.70 m 隧道)。返回 (缝宽, 中线横向偏移) 或 None。

        为什么要单拎出来: side_gap 0.40 是给行人留的 (capsule 半径 0.25, 蹭一下就倒),
        而墙没有半径 —— 车半宽 0.20, 0.70 m 的通道单边还能余 0.15 m, 本来就能过。
        一律按 0.40 判, 整条隧道会被当成死路: 实测车从墙外侧兜弧绕过去, x 最远到
        -1.29 m, 隧道口 (x ∈ ±0.35) 一次都没进过。

        为什么读原始射线而不是 marks: 10 度一格的格心角与真实命中角最多差 5 度,
        在 1.3 m 处就是 0.11 m 的横向误差 —— 与通道单边 0.15 m 的余量同量级。
        实测 0.70 m 的缝被量成 0.47 m, 判据直接失效。原始射线 0.8 度一条, 误差 0.02 m。

        反过来的风险是「单个人被当成通道」, 所以要求两侧**同时**有障碍、缝宽落在
        0.52~1.15 m 之间, 而且缝中线两侧必须真的空着 —— 单侧障碍凑不成缝,
        缝中间站人也直接否决 (行人是个实心柱, 不会给出两条腿之间的假通道)。
        """
        c = self.cfg
        sc = self._raw_scan
        if not sc:
            self.narrow_why = 'no_scan'
            return None
        ranges, amin, ainc = sc
        reach = c.safe_dist + c.margin + c.lookahead
        half_body = 0.24            # 车半宽 0.20 + 余量 0.04: 缝中线两侧必须这么空
        cand = []                   # 走廊范围内所有实测点 (横向, 纵向), 横向 > 0 = 车左侧
        for i in range(len(ranges)):
            d = ranges[i]
            if d != d or d < 0.05 or d > c.scan_max:
                continue
            rel = wrap_pi(amin + i * ainc)  # 雷达 0 度就是车头正前
            if abs(rel) >= math.pi / 2.0:
                continue
            proj = d * math.cos(rel)
            # 用比 reach 更远的门限: 提前 2 m 就认出窄缝, 才有时间把车头对准中线。
            # reach=1.40 时只剩 0.8 m 的调整距离, 实测来不及 (偏 0.19 就先判堵了)。
            if proj > max(reach, 2.20) or proj < 0.05:
                continue
            cand.append((d * math.sin(rel), proj))
        if not cand:
            self.narrow_why = 'no_cand'
            return None
        # 只有「两边同时有东西」才构得成缝。单边障碍绕过去就行。
        # 侧方贴得很近的墙点也要收进来 (proj 可能接近 0), 所以上面滤波只留 proj < 0.05。
        L = [t for t in cand if t[0] >= 0.12]
        R = [t for t in cand if t[0] <= -0.12]
        if not L or not R:
            self.narrow_why = 'oneside L=%d R=%d' % (len(L), len(R))
            return None
        wall_l = min(L, key=lambda t: t[0])[0]      # 左侧最靠中线的那一点
        wall_r = max(R, key=lambda t: t[0])[0]      # 右侧最靠中线的那一点
        width = wall_l - wall_r
        if not (0.52 <= width <= 1.15):
            self.narrow_why = 'width %.2f (L=%.2f R=%.2f)' % (width, wall_l, wall_r)
            return None                     # 太窄过不去 / 太宽按常规判据就够
        mid = (wall_l + wall_r) * 0.5       # 缝中线相对车中线的偏移
        # 关键: 要检查的是「缝中线」，不是「车中线」。车在走廊里偏右 0.17 m 时，
        # 右墙口会落进车中线 ±0.24 而被当成「缝中间有东西」—— 实测就是这么卡死的。
        for lat, proj in cand:
            if abs(lat - mid) < half_body:
                self.narrow_why = 'inner lat=%+.2f mid=%+.2f proj=%.2f' % (lat, mid, proj)
                return None                 # 缝中线正对着障碍: 这不是通道, 是死路
        self._narrow_d = min(t[1] for t in L + R)     # 墙口纵向距离, 越近修正越狠
        self.narrow_why = 'ok %.2f mid=%+.2f d=%.2f' % (width, mid, self._narrow_d)
        return (width, mid)

    def _side_best(self, gp, now, sign):
        """某一侧 (+1 左 / -1 右) **最接近目标**的可行方向: (净空, 偏角 deg) 或 None。

        取最接近目标的那个, 而不是最宽的那个 —— 后者正是 _choose_from_scratch
        按下单顺序要试的全部, 差得远的方向再宽也不会被选中, 拿它比没意义。
        也正因为可行方向必然是「转过来看过、而且离得够远」的, 净空这一栏
        几乎总是同一个量级, 真正能分出高下的是偏角差 (见 _doubtful)。
        """
        c = self.cfg
        for k in range(1, int(c.go_limit // c.step_deg) + 1):
            deg = sign * k * c.step_deg
            d = self._feasible(wrap_pi(gp + math.radians(deg)), now)
            if d is not None:
                return (min(d, c.scan_max), deg)   # inf 折成量程上限, 否则没法比大小
        return None

    def _doubtful(self, gp, now):
        """几何是不是「两边都能过、分不出高下」—— 只有这时候才让 LLM 改搜索顺序。

        SayCan 那种加权在这个离散搜索里就落成这一行: 模型给的是**偏好**不是判据,
        几何一票否决。两边差得明显时让模型改序, 只会把车领到更窄的那一边去。
        """
        lb = self._side_best(gp, now, +1)
        rb = self._side_best(gp, now, -1)
        if lb is None or rb is None:
            return False                    # 只有一边可行 -> 几何自己有答案, 不许插手
        # 两侧「第一个能走的方向」偏角接近 -> 几何没有理由偏好任何一边。
        # 这里只看偏角: 能走的格子必然是转过看过、而且够远的, 净空读数几乎
        # 总是量程上限, 比它恒等于 0 差 —— 真正有信息的是「要转多少度才走得成」。
        if abs(abs(lb[1]) - abs(rb[1])) <= self.cfg.llm_tie_deg:
            return True
        # 两侧净空都顶到量程上限 = 两边都能一路走过去, 只是 ±10° 的扫描顺序
        # 决定谁先被找到。对称墙场景实测 lb=(9.0,+70) rb=(9.0,-30), 偏角差 100°
        # 远超 20°, 可按偏角判就永远平局不了 —— 这种"都空旷"也该算平局。
        cap = self.cfg.scan_max - 1e-6
        return lb[0] >= cap and rb[0] >= cap

    def _any_fresh(self):
        """有没有任何一条还新鲜的观测 —— 全没有说明压根没数据, 不能放行。

        深度也算一路观测: 空旷场雷达一条 marks 都攒不出来 (地面扫不到、矮物也扫不到),
        只认 marks 的话深度判出的「正前方有东西」永远没机会生效。
        """
        return bool(self.marks) or self._depth_front is not None

    def _set_commit(self, deg, now):
        """记下「现在坚持往哪边绕」。同一侧加大角度不算换方向, 不重新计时。"""
        c = self.cfg
        if abs(deg) < c.step_deg * 0.5:
            self._commit = 0.0
            self._commit_until = 0.0
            self._commit_t0 = now
            return
        if self._commit * deg <= 0.0:
            self._commit_t0 = now          # 换了边(或从 0 开始) -> 重新计时
        self._commit = deg
        self._commit_until = now + c.commit_ttl

    def _choose(self, goal_dir, now):
        """先看目标方向; 不通再坚持上一拍的绕行方向; 都不行才从头挑一遍。"""
        c = self.cfg
        # 这两项是「本拍」的诊断, 每拍清空: 留着上一拍的值, 日志就会一直挂着
        # 一次早已过期的平局现场, 看着像模型年年参与。
        self.tie_info = ''
        self.pref_src = ''

        # 0) 目标方向自身通不通? 通就直接回家, 并忘掉之前的绕行方向。
        # 少了这一步, 车会绕着目标转大圈: 绕行方向一旦选定就每帧续期,
        # 于是一直朝"目标方向 +100 度"开, 永远到不了 (踩过这个坑)。
        gp = wrap_pi(goal_dir)
        # 0') 「转过去看」要粘住: 车体转向约 1 rad/s, 转 90 度要 1.6 s, 而这里每周期
        # 重算一次。不粘的话 plan.heading 会每秒翻一次符号 (实测 +100/-90 来回),
        # 车头物理上停在中间两头不到, 一步也走不了 (2026-10-27 行人场景 80 次 DODGE 卡死)。
        # 粘住期间只判两件事: 目标方向恰好通了就走, 否则继续朝同一侧转。
        if self._look_heading is not None:
            if now >= self._look_until:
                self._look_heading = None
            else:
                ok_gp = self._feasible(gp, now)
                if ok_gp is not None:
                    self._look_heading = None
                    self._set_commit(0.0, now)
                    return (gp, 0.0, ok_gp, 'go')
                ang = self._look_heading
                deg = math.degrees(wrap_pi(ang - gp))
                if abs(deg) <= c.go_limit:
                    ok = self._feasible(ang, now)
                    if ok is not None:
                        self._look_heading = None
                        return (ang, deg, ok, 'go')
                    return (ang, deg, INF, 'look')
                self._look_heading = None
        clear0 = self._feasible(gp, now)
        # 试过「绕行方向吃掉整个 commit_ttl、期间不看目标方向」: 车贴着障碍绕得更深,
        # 80 秒 / 7.3 m / 最小间距 0.19 m —— 反而擦上了。锁定方向 != 安全, 已回滚。
        if clear0 is not None:
            self._set_commit(0.0, now)
            return (gp, 0.0, clear0, 'go')
        # 0.5) 目标方向不通时, 先看上一拍选的方向还走不走得通 —— 走得通就接着走。
        # 不加这条会左右横跳: 每帧重新挑, 而两侧往往都"勉强可行",
        # 车头就在两个方向之间来回转, 一步也挪不动 (踩过这个坑)。
        # 但坚持得有上限: 绕行方向每帧续期的话车会一直沿它绕圈、永远不回家,
        # 所以 commit_max 一到就放它回去重新看一眼目标 (踩过这个坑)。
        if (self._commit and now < self._commit_until
                and abs(self._commit) > c.step_deg * 0.5):
            if now - self._commit_t0 >= c.commit_max:
                self._set_commit(0.0, now)
                return self._choose_from_scratch(gp, now)
            # 0.6) 模型说要换边, 而且几何确实判不出高下, 这次坚持也够久了:
            # 放弃这次 commit, 重新从头挑一遍。
            # 没有这条, commit 分支一路 return, 模型的话在绝大多数周期里
            # 根本够不着 —— 实测 (0,8) 一趟问了 16 次、答了 5 次, 进决策 0 次;
            # 它只在 _choose_from_scratch 里才有声音, 而顺利的行程很少走到那里。
            # 仍守着 _doubtful: 几何自己有明显偏好时, 模型不许把它掰弯。
            if (self._llm_side and self._llm_side * self._commit < 0
                    and self.llm_active(now) and now - self._commit_t0 >= c.commit_min
                    and self._doubtful(gp, now)):
                self._set_commit(0.0, now)      # 顺手重置计时, 免得下一帧又被掰回来
                return self._choose_from_scratch(gp, now)
            # 绕行角不够就沿**同一侧**一点点加大, 而不是掉头换边。
            sgn = 1.0 if self._commit > 0 else -1.0
            unseen = None
            for dk in range(0, int(c.commit_span // c.step_deg) + 1):
                deg = self._commit + sgn * dk * c.step_deg
                ang = wrap_pi(goal_dir + math.radians(deg))
                clear = self._feasible(ang, now)
                # 朝侧后方"开出去"是原地打转的头号来源 (Nav2 里叫 prefer_forward):
                # 那边再空旷也只是离目标更远, 允许倒车会白烧几十秒。
                if clear is not None and abs(deg) <= c.go_limit:
                    self._set_commit(deg, now)
                    return (ang, deg, clear, 'go')
                # 待查看方向也不许越过 go_limit: 看了也不让走, 转过去只是白甩头
                if (unseen is None and abs(deg) <= c.go_limit
                        and self._mark(ang, now) is None
                        and self._strict(ang, now)):
                    unseen = (ang, deg)
            if unseen is not None:
                # 这一侧还有没看过的方向, 而且现有障碍数据不反对: 继续转头看这一侧
                self._set_commit(unseen[1], now)
                return (unseen[0], unseen[1], INF, 'look')
            # 这一侧彻底走不通 (死角): 放弃坚持, 交给下面重新挑 —— 另一侧才可能出去
            self._set_commit(0.0, now)

        return self._choose_from_scratch(gp, now)

    def _choose_from_scratch(self, gp, now):
        """不看上一拍的绕行方向, 完全按「离目标近」重挑一遍。"""
        c = self.cfg
        lim = int(c.back_limit // c.step_deg)
        # pref 平时只认几何的 side。LLM 曾经一律不认 —— 实测 9B/7B 判左右基本靠猜
        # (4 组截然不同的图一律回 LEFT)。但那种「猜」有一半是信息不足造成的:
        # 只喂左右两个数, 它分不清走廊和死角。现在喂 12 格净空 + 约束输出之后,
        # 它至少是在**有依据地**选边, 而「两边都 0.75 m 该走哪边」确实不是几何
        # 能回答的问题。所以改成 SayCan 式加权: 只在几何平局 (_doubtful) 时才让它改序,
        # 差得超过 llm_tie_deg 就一律按几何走 —— 判据始终在几何手里。
        pref = self.side
        src = ''
        if self._llm_side and self.llm_active(now):
            # 原来这里有一句 print('DBG ...') 做诊断。现在改成把「这场平局长什么样」
            # 记进 tie_info, 由上层日志按需打印 —— 每个决策周期都往 stdout 写,
            # 一行就把真日志挤出屏幕了。
            _lb = self._side_best(gp, now, +1)
            _rb = self._side_best(gp, now, -1)
            self.tie_info = 'llm=%+d lb=%s rb=%s tie=%s' % (
                self._llm_side, _lb, _rb, self._doubtful(gp, now))
            if self._doubtful(gp, now):
                pref = self._llm_side
                src = '(llm)'
        self.pref_src = 'llm' if src else ''
        order = [0]
        if src:
            # LLM 真的参与时才整侧优先: 它说的是「往哪边绕」, 交替下单的话
            # 「另一侧 40 度 vs 偏好侧 50 度」永远先试 40 度, 它的意见等于没投。
            # 只改先后, 可行性仍由 _feasible 逐条判定 —— 它选错侧也只是多绕。
            s0 = 1 if pref > 0 else -1
            order += [s0 * k for k in range(1, lim + 1)]
            order += [-s0 * k for k in range(1, lim + 1)]
        else:
            for k in range(1, lim + 1):
                if pref < 0:
                    order += [-k, k]
                else:
                    order += [k, -k]

        clear0 = self._feasible(gp, now)
        if clear0 is not None:
            self._set_commit(0.0, now)
            return (gp, 0.0, clear0, 'go')
        # 目标方向压根没看过 -> 先把车头转过去看一眼再决定。
        # 否则会一直闷头按上一拍的绕行方向绕圈, 目标永远在"没看过"的那一侧。
        # 深度已经判过「正前方有东西」时不再"转过去看一眼": 那一眼早就看过了,
        # 而 _mark 只认雷达, 矮箱子永远进不了 marks —— 不改这里的话车会一直朝目标
        # 转头、一步不挪 (现场：雷达空白 + 深度有障碍 = 无限 look)。
        if self._mark(gp, now) is None and self._depth_front is None:
            return (gp, 0.0, INF, 'look')

        # 按「离目标近」的顺序走一遍: 撞上第一个能走的方向就选它;
        # 撞上第一个「还没看过」的方向, 就先转过去看 ——
        # 注意这两件事必须混在同一个顺序里。分开两轮的话, 远处的宽方向 (比如车后)
        # 会先被选中, 车头就掉头往回走了 (踩过这个坑)。
        unseen = None
        for k in order:
            ang = wrap_pi(gp + math.radians(k * c.step_deg))
            clear = self._feasible(ang, now)
            if clear is not None and abs(k * c.step_deg) <= c.go_limit:
                return (ang, float(k) * c.step_deg, clear, 'go' + src)
            # 没看过、但现有障碍数据不反对 -> 候选「转过去看」; 还不行就先记着继续找
            if (unseen is None and abs(k * c.step_deg) <= c.go_limit
                    and self._mark(ang, now) is None and self._strict(ang, now)):
                unseen = (ang, float(k) * c.step_deg)
        if unseen is not None:
            # 挑一个最接近目标的「待查看」方向转过去, 并坚持这一侧, 免得下一帧又换边
            self._set_commit(unseen[1], now)
            return (unseen[0], unseen[1], INF, 'look' + src)

        # 3) 都看过且都窄: 朝最宽处探头看, 但一步也不往前挪
        best_k, best_d = 0, None
        for k in order:
            ang = wrap_pi(gp + math.radians(k * c.step_deg))
            d = self._mark(ang, now)
            if d is None:
                continue
            if best_d is None or d > best_d:
                best_k, best_d = k, d
        if best_d is not None and best_d >= c.hard_dist:
            ang = wrap_pi(gp + math.radians(best_k * c.step_deg))
            return (ang, float(best_k) * c.step_deg, best_d, 'look' + src)
        return None

    def _plan(self, state, heading, goal_dir, side, advance, reason, clear=INF):
        self.last_reason = reason
        return Plan(state=state, heading=wrap_pi(heading), goal_dir=goal_dir,
                    side=side, advance=advance, reason=reason, clear=clear,
                    sectors=self.last_sectors)


# ---------------- 深度图 -> 分扇区净空 ----------------
def _depth_array(depth):
    try:
        import numpy as np
    except ImportError:
        return None
    arr = np.asarray(depth, dtype=np.float32)
    if arr.ndim != 2 or arr.size == 0:
        return None
    return arr


def _obstacle_mask(arr, cam_height, cam_hfov, ground_ratio):
    """剔除地面像素, 只留真正的障碍。相机水平、离地高度固定 -> 每行对应确定的地面深度。"""
    import numpy as np
    h, w = arr.shape[:2]
    rows = np.arange(h, dtype=np.float32)
    focal = (w / 2.0) / math.tan(cam_hfov / 2.0)
    ground = cam_height * focal / np.maximum(rows - h / 2.0, 1e-3)
    mask = np.isfinite(arr) & (arr > 0.0) & (rows[:, None] <= h / 2.0)
    below = rows > h / 2.0
    if below.any():
        mask[below] |= (arr[below] < ground[below, None] * ground_ratio)
    return mask


def _zone_min(arr, mask, percentile, step=4):
    import numpy as np
    if mask.size == 0:
        return INF
    vals = arr[mask][::step]
    vals = vals[np.isfinite(vals) & (vals > 0.0)]
    if vals.size == 0:
        return INF
    return float(np.percentile(vals, percentile))


def compute_sectors(depth, n_sectors=9, cam_height=0.28, cam_hfov=1.047,
                    ground_ratio=0.75, percentile=10.0):
    """深度图 -> 从左到右 n_sectors 个扇区的最近障碍距离 (m), inf=空旷。

    扇区按**角度**均匀切分 (不是按像素列): 针孔相机里列与角度是 tan 关系,
    不换算的话边缘扇区会宽得离谱, 选出来的方向全是偏的。
    """
    arr = _depth_array(depth)
    if arr is None:
        return tuple(INF for _ in range(n_sectors))
    h, w = arr.shape[:2]
    focal = (w / 2.0) / math.tan(cam_hfov / 2.0)
    mask = _obstacle_mask(arr, cam_height, cam_hfov, ground_ratio)
    out = []
    for i in range(n_sectors):
        a0 = -cam_hfov / 2.0 + cam_hfov * i / n_sectors
        a1 = a0 + cam_hfov / n_sectors
        c0 = int(round(w / 2.0 + focal * math.tan(a0)))
        c1 = int(round(w / 2.0 + focal * math.tan(a1)))
        c0 = max(0, min(w, c0))
        c1 = max(c0 + 1, min(w, c1))
        out.append(_zone_min(arr[:, c0:c1], mask[:, c0:c1], percentile))
    return tuple(out)


def compute_zones(depth, cam_height=0.28, cam_hfov=1.047,
                  left_pct=0.4, right_pct=0.6,
                  ground_ratio=0.75, percentile=10.0):
    """深度图 -> (左, 中, 右) 三区最近距离 —— 与 obstacle_avoider 同源。

    保留它是为了和 /obstacle_zones 对得上, 便于互相校验。
    """
    arr = _depth_array(depth)
    if arr is None:
        return (INF, INF, INF)
    h, w = arr.shape[:2]
    mask = _obstacle_mask(arr, cam_height, cam_hfov, ground_ratio)
    xl, xr = int(w * left_pct), int(w * right_pct)
    return (_zone_min(arr[:, :xl], mask[:, :xl], percentile),
            _zone_min(arr[:, xl:xr], mask[:, xl:xr], percentile),
            _zone_min(arr[:, xr:], mask[:, xr:], percentile))
