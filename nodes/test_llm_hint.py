#!/usr/bin/env python3
"""llm_hint 的离线测试 (不连真模型, 用一个假 OpenAI 兼容端点)。

验四件事:
  1. 提示词说了该说的 (车宽/不能横移/三条规则/回一个词), 且不再犯旧提示词的错;
  2. 模型的回答能被解析成一个边 (含 TURN_LEFT / 思考标签 / 胡说八道);
  3. 请求体是对的 —— 尤其 reasoning_effort=none 只给会思考的端点, 图像只在 VLM 分支出现;
  4. 任何失败都只是「没意见」: 不抛异常、不卡住、过期自动作废。

跑法: python3 nodes/test_llm_hint.py
"""
import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from avoid_planner import AvoidPlanner, Config          # noqa: E402
import llm_hint                                          # noqa: E402
from llm_hint import (HintConfig, LlmHint, Scene,        # noqa: E402
                      _prompt, parse_side, parse_json_answer)

FAILS = []
CHECKS = [0]


def check(cond, msg):
    CHECKS[0] += 1
    if not cond:
        FAILS.append(msg)
        print('  FAIL  %s' % msg)


# ---------------- 假端点 ----------------
class Handler(BaseHTTPRequestHandler):
    answer = 'LEFT'
    status = 200
    bodies = []
    hit = 0

    def do_POST(self):
        n = int(self.headers.get('Content-Length', 0))
        Handler.bodies.append(json.loads(self.rfile.read(n)))
        Handler.hit += 1
        if Handler.status != 200:
            self.send_response(Handler.status)
            self.end_headers()
            self.wfile.write(b'{}')
            return
        body = json.dumps({'choices': [{'message': {'content': Handler.answer}}]}).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *a):
        pass


def serve():
    srv = HTTPServer(('127.0.0.1', 0), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, 'http://127.0.0.1:%d/v1' % srv.server_address[1]


def wait_done(h, timeout=5.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if not h._busy:
            return True
        time.sleep(0.01)
    return False


def scene(**kw):
    d = dict(trigger='tie', goal_bearing=15.0, left=1.20, ahead=0.80, right=0.90)
    d.update(kw)
    return Scene(**d)


# ---------------- 1. 解析 ----------------
def test_parse():
    print('[1] 回答解析')
    cases = [
        ('LEFT', 1), ('right', -1), ('STRAIGHT', 0),
        ('TURN_LEFT', 1), ('TURN_RIGHT', -1),
        ('go forward', 0),
        ('  LEFT\n', 1),
        ('The answer is RIGHT.', -1),
        ('think 左边也堵, 但按规则选 LEFT  →', 1),
        ('', 0), ('I am not sure.', 0),
        ('NOTLEFT', 0),
    ]
    for text, want in cases:
        got, word = parse_side(text)
        check(got == want, '解析 %r -> %s (期望 %s)' % (text, got, want))


# ---------------- 2. 提示词 ----------------
def test_prompt():
    print('[2] 提示词')
    p = _prompt(scene(), False)
    check('0.40 m wide' in p, '提示词里要写明车宽 0.40 m')
    check('cannot strafe' in p, '提示词里要写明不能横移')
    check('cannot reverse' in p, '提示词里要写明不能倒车')
    check('+15 deg' in p, '目标方位要写进去')
    check('1.20' in p and '0.80' in p and '0.90' in p, '左中右净空都要写进去')
    check('0.70 m' in p, '硬规则: 净空 < 0.70 m 的一侧不许选')
    check('60 deg of the goal bearing' in p, '硬规则: 最终朝向要靠近目标方位')
    check('LEFT, RIGHT or STRAIGHT' in p, '允许回答直走, 不能只有左右')
    check('you control' not in p.lower(), '不能再写 "You control a robot camera" 这种错主语')
    check('image' not in p.lower(), '不带图时不该提图像')
    p2 = _prompt(scene(trigger='stall'), True)
    check('front camera view' in p2, '带图时要说明图是什么')
    check('not getting any closer' in p2, 'stall 的原因要写进去')
    check(all(ord(c) < 128 for c in p2), '提示词必须是纯英文')


# ---------------- 3. 请求体 ----------------
def test_request():
    print('[3] 请求体')
    srv, base = serve()
    Handler.answer, Handler.status, Handler.bodies = 'RIGHT', 200, []
    cfg = HintConfig(on=True, provider='local', local_url=base, local_key='k',
                     local_model='qwen2.5-7b-instruct-1m', vlm_model='', vision=False)
    h = LlmHint(cfg)
    now = time.time()
    check(h.consider(now, scene()) == 'tie', '触发后应返回触发原因')
    check(wait_done(h), 'worker 应很快结束')
    check(Handler.hit == 1, '应只发一次请求 (实际 %d)' % Handler.hit)
    b = Handler.bodies[0]
    check(b['model'] == 'qwen2.5-7b-instruct-1m', 'model 要按配置传')
    if 'response_format' in b:
        # JSON 模式: 一个词装不进 JSON 外壳, 得给够长度; 输出形状靠 schema 锁,
        # 不靠把 max_tokens 掐到 8 —— 掐太小只会得到半截 JSON 再回落成瞎猜。
        check(b['max_tokens'] >= 32, 'JSON 模式要给够 token, 否则被截成半截 JSON')
        sch = b['response_format'].get('json_schema', {}).get('schema', {})
        check(b['response_format'].get('type') == 'json_schema', '要用 json_schema 锁输出形状')
        check(sch.get('properties', {}).get('side', {}).get('enum')
              == ['LEFT', 'RIGHT', 'STRAIGHT'], 'side 必须三选一, 不许自由发挥')
    else:
        check(b['max_tokens'] <= 16, 'max_tokens 要小 (一个字)')
    check(b['temperature'] == 0, 'temperature 必须 0')
    check('reasoning_effort' not in b,
          'qwen2.5 不思考, 不能传 reasoning_effort (实测会从 0.08s 拖到 8.85s)')
    check(isinstance(b['messages'][0]['content'], str), '不带图时 content 用纯字符串最省事')
    check(h.side(now) == -1, 'RIGHT 应变成 -1 (往右让)')
    check(h.answered == 1 and h.failed == 0, '计数器不对')

    # 带图: 走 VLM 分支
    Handler.bodies, Handler.hit = [], 0
    cfg2 = HintConfig(on=True, provider='local', local_url='http://127.0.0.1:1/v1',
                      vlm_url=base, vlm_model='qwen3.5-9b-vl', vision=True,
                      local_timeout=0.5, vlm_timeout=5.0)
    h2 = LlmHint(cfg2)
    h2.consider(time.time(), scene(trigger='stall', image=b'\xff\xd8jpegbytes'))
    check(wait_done(h2), 'VLM worker 应结束 (本地端点不通, 但 VLM 端点会先被问到)')
    check(Handler.hit == 1, 'VLM 端点优先, 应被问到一次')
    b2 = Handler.bodies[0]
    check(b2['model'] == 'qwen3.5-9b-vl', 'VLM 分支要用 9B 模型')
    check(b2.get('reasoning_effort') == 'none',
          'VLM/带思考模型必须带 reasoning_effort=none, 否则 8 token 全被思考吃光')
    c = b2['messages'][0]['content']
    check(isinstance(c, list) and len(c) == 2, '带图时 content 要是 [文本, 图]')
    check(c[1]['image_url']['url'].startswith('data:image/jpeg;base64,'), '图要 base64 内联')
    check(h2.side(time.time()) == -1, 'VLM 的回答也要能解析')
    srv.shutdown()


# ---------------- 4. 容错 ----------------
def test_robust():
    print('[4] 容错')
    # 端点全挂: 只能「没意见」
    srv, base = serve()
    Handler.status = 500
    cfg = HintConfig(on=True, provider='local', local_url=base, local_timeout=2.0)
    h = LlmHint(cfg)
    t0 = time.time()
    h.consider(time.time(), scene())
    check(wait_done(h, 6.0), 'HTTP 500 后 worker 仍要结束, 不能卡死')
    check(time.time() - t0 < 5.0, '失败要快, 不能拖住 10 Hz 的环')
    check(h.failed == 1, '要记一次失败')
    check(h.side(time.time()) == 0, '失败时不给意见 = 纯几何')
    check('500' in h.last_err or 'Server' in h.last_err or 'HTTP' in h.last_err,
          'last_err 要留下原因 (%r)' % h.last_err)
    srv.shutdown()

    # 关掉开关就完全不发请求
    Handler.status = 200
    Handler.hit = 0
    off = LlmHint(HintConfig(on=False))
    check(off.consider(time.time(), scene()) is None, 'LLM_ON=0 时不该触发')
    check(off.side(time.time()) == 0, 'LLM_ON=0 时永远没意见')
    check(Handler.hit == 0, '关掉后不该有任何请求')

    # 节流: 2.5 s 内第二次不问
    srv2, base2 = serve()
    h3 = LlmHint(HintConfig(on=True, provider='local', local_url=base2, min_interval=2.5))
    now = time.time()
    check(h3.consider(now, scene()) == 'tie', '第一次应问')
    wait_done(h3)
    check(h3.consider(now + 1.0, scene()) is None, '节流期内第二次不该问')
    check(h3.consider(now + 3.0, scene()) == 'tie', '过了节流期应能再问')
    wait_done(h3)

    # 答案 ttl: 4 s 后自动作废
    h4 = LlmHint(HintConfig(on=True, provider='local', local_url=base2))
    h4._side, h4._res_t = 1, 100.0
    check(h4.side(101.0) == 1, 'ttl 内有效')
    check(h4.side(105.0) == 0, 'ttl 后作废')
    srv2.shutdown()

    # 没 requests 也不炸
    saved = llm_hint.requests
    llm_hint.requests = None
    h5 = LlmHint(HintConfig(on=True))
    check(h5.consider(time.time(), scene()) is None, '没有 requests 时安静降级')
    llm_hint.requests = saved


# ---------------- 5. 触发判据的原料 ----------------
def test_feeds():
    print('[5] 给模型的数字')
    p = AvoidPlanner(Config())
    now = 100.0
    p._yaw = 0.0
    # 直接往极坐标格子里种三个方位: 正前 0.80, 左 +60 空旷, 右 -60 1.10
    idx = lambda deg: int(round(deg / 10.0)) % 36
    p.marks = {idx(0): (0.80, now, True),
               idx(60): (math.inf, now, True),
               idx(-60): (1.10, now, True)}
    check(p.clearance(0.0, now) == 0.80, '正前净空')
    check(p.clearance(60.0, now) == math.inf, '左侧空旷应为 inf')
    check(p.clearance(-60.0, now) == 1.10, '右侧净空')
    check(p.clearance(170.0, now) is None, '没看过的方向要返回 None, 不能瞎报')

    # 换边统计
    p._side_hist = [(now - 0.9, 1), (now - 0.6, -1), (now - 0.3, 1)]
    check(p.flap(now) >= 2, '三条记录来回两次应判为横跳 (实际 %s)' % p.flap(now))
    p._side_hist = [(now - 0.9, 1), (now - 0.6, 1), (now - 0.3, 1)]
    check(p.flap(now) == 0, '一直同一边不算横跳')
    p._side_hist = [(now - 5.0, 1), (now - 4.0, -1), (now - 3.0, 1)]
    check(p.flap(now) == 0, '窗口外的旧记录不算')


# ---------------- 6. 12 格净空 ----------------
def test_ring():
    print('[6] 12 格净空与 JSON 输出')
    ring = (1.20, math.inf, 0.90, None, 0.80, 0.70,
            0.60, 0.50, 0.40, 0.30, 0.20, 0.10)
    p = _prompt(scene(ring=ring), False)
    check('Range readings all around' in p, '有一圈净空时不该再只写左中右')
    check('+90=' in p and '-90=' in p, '12 格要把正侧方的角度写出来')
    check('left  +60 deg' not in p, '有一圈时旧的左中右三行不该再出现')
    check(all(ord(c) < 128 for c in p), '提示词必须是纯英文')

    p2 = _prompt(scene(ring=ring, prev_answer='LEFT'), False, json_mode=True)
    check('"side"' in p2 and 'confidence' in p2, 'JSON 模式要把输出格式说清楚')
    check('exactly one word' not in p2, 'JSON 模式不该再要求一个词')
    check('previous answer was LEFT' in p2, '上次的答案要回灌进去')

    # 后退一格: 没有一圈数据时必须退回老三行, 不能崩
    p3 = _prompt(scene(), False)
    # 提示词排版是 '  left  (+60 deg): 2.57', 括号是后来加的 (A/B 实测 P1 排版对齐
    # 多对 5 题), 这里跟着改 —— 旧断言查的是没有括号的老写法, 一直红着。
    check('left  (+60 deg)' in p3 and 'Range readings' not in p3, '没有一圈数据时退回老三行')


# ---------------- 7. JSON 输出解析 ----------------
def test_json_answer():
    print('[7] JSON 输出解析')
    cases = [
        ('{"side": "LEFT", "confidence": 0.9}', 1, 0.9),
        ('{"side":"right","confidence":0.4}', -1, 0.4),
        ('{"side": "STRAIGHT"}', 0, 0.5),
        ('Sure. {"side": "LEFT", "confidence": 7} end', 1, 1.0),
        ('{"side": "UP", "confidence": 0.9}', 0, 0.0),
        ('LEFT', 1, 0.5),
        ('', 0, 0.0),
    ]
    for text, want_side, want_conf in cases:
        s, w, c = parse_json_answer(text)
        check(s == want_side and abs(c - want_conf) < 1e-6,
              '%r -> side=%s conf=%.2f (期望 %s/%.2f)' % (text, s, c, want_side, want_conf))


# ---------------- 8. 加权 ----------------
def test_weighting():
    print('[8] SayCan 加权: 只有几何平局才听模型')
    now = 100.0
    idx = lambda deg: int(round(deg / 10.0)) % 36

    def mk(marks):
        p = AvoidPlanner(Config())
        p._yaw = 0.0
        p.marks = {idx(d): (v, now, True) for d, v in marks.items()}
        return p

    # 两侧绕行代价对称: 几何没有偏好。
    # 正前方那个障碍放 1.60 m 而不是 0.60 m: side_gap/near_gap 收紧后, 0.60 m 的
    # 正前障碍在 30 度方向上的横向偏移只有 0.30 m < near_gap 0.50, 会把两侧一起
    # 否掉 (那是**对的**行为 —— 车贴着障碍往 30 度斜切确实会蹭上), 而这条用例
    # 要测的是「两侧对称」, 不是「贴脸时两侧都堵」。
    p = mk({0: 1.60, 30: 3.00, -30: 3.00})
    check(p._side_best(0.0, now, +1) == (3.00, 30), '左侧取最近可行方向 30 deg')
    check(p._side_best(0.0, now, -1) == (3.00, -30), '右侧取最近可行方向 -30 deg')
    check(p._doubtful(0.0, now), '两岸对称 -> 该让模型表态')

    # 一侧明显更绕: 几何自己就有答案
    p2 = mk({0: 1.60, 30: 3.00, -70: 3.00})
    check(not p2._doubtful(0.0, now), '左转 30 / 右转 70 -> 几何已经分出高下')

    # 只有一边有路
    p3 = mk({0: 1.60, 30: 3.00})
    check(p3._side_best(0.0, now, -1) is None, '右侧没探到可行方向')
    check(not p3._doubtful(0.0, now), '只有一边有路 -> 不听模型')

    # 意见本身带 TTL 与置信度
    p.note_llm(-1, now, ttl=4.0, conf=0.8)
    check(p.llm_active(now + 1.0) and p._llm_side == -1, '意见应先生效')
    check(abs(p._llm_conf - 0.8) < 1e-9, '置信度要存下来进日志')
    check(not p.llm_active(now + 5.0), 'ttl 后意见作废')

    # 关掉加权 = 几何永不平局, 用来跑 A/B 基线
    q = mk({0: 0.60, 30: 3.00, -30: 3.00})
    q.cfg.llm_tie_deg = -1.0
    check(not q._doubtful(0.0, now), 'llm_tie_deg<0 时几何永远不平局 (A/B 基线)')


# ---------------- 9. commit 中途被模型改边 ----------------
def test_commit_switch():
    """模型能不能把车从 commit 分支里拉出来。

    改造前它根本够不着: _choose 一进 commit 分支就 return, 只有 _choose_from_scratch
    里才有模型的声音, 而顺利的行程很少走到那里 —— (0,8) 实测一趟问 16 次、答 5 次,
    进决策 0 次。
    """
    print('[9] commit 中途被模型改边')
    now = 200.0
    idx = lambda deg: int(round(deg / 10.0)) % 36

    def mk(sym=True):
        p = AvoidPlanner(Config())
        p._yaw = 0.0
        p.marks = {idx(d): (v, now, True) for d, v in
                   ({0: 1.10, 30: 3.00, -30: 3.00} if sym else
                    {0: 1.10, 30: 3.00, -70: 3.00}).items()}
        p._set_commit(30.0, now - 2.0)      # 几何已经选了左边 30 度
        p._commit_until = now + 10.0        # commit_ttl 早过了, 这条是「还在坚持」的标记
        return p

    p = mk()
    p.note_llm(-1, now, ttl=8.0)
    plan = p._choose(0.0, now)
    check(plan is not None and plan[3].endswith('(llm)'),
          '模型说换边 -> 该走出 from_scratch 的 (llm) 分支')
    check(plan is not None and plan[1] < 0, '换边之后要朝右 (-30 deg)')

    # 刚选定的方向不许动: 9B 的回答本身在 1-3 s 之间抖动
    q = mk()
    q._commit_t0 = now - 0.5                # 比 commit_min 1.2 短
    q.note_llm(-1, now, ttl=8.0)
    plan = q._choose(0.0, now)
    check(plan is not None and plan[1] > 0, '还不到 commit_min -> 不许改边')

    # 几何自己有明显偏好时, 模型不许把它掰弯
    r = mk(sym=False)                       # 右边要转 70 度, 左边只要 30
    r.note_llm(-1, now, ttl=8.0)
    plan = r._choose(0.0, now)
    check(plan is not None and plan[1] > 0, '几何有偏好 (30 vs 70) -> 不听模型')

    # 意见过期了也不许动
    s = mk()
    s.note_llm(-1, now - 100.0, ttl=8.0)
    plan = s._choose(0.0, now)
    check(plan is not None and plan[1] > 0, '意见过期 -> 按几何走')


def main():
    test_parse()
    test_prompt()
    test_request()
    test_robust()
    test_feeds()
    test_ring()
    test_json_answer()
    test_weighting()
    test_commit_switch()
    print('')
    if FAILS:
        print('FAILED %d / %d' % (len(FAILS), CHECKS[0]))
        for f in FAILS:
            print('  - ' + f)
        return 1
    print('OK  %d проверок' % CHECKS[0])
    return 0


if __name__ == '__main__':
    sys.exit(main())
