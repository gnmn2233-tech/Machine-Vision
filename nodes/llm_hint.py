#!/usr/bin/env python3
"""LLM/VLM 顾问: 几何层拿不定主意时才问一句, 问的只是「往哪边让」。

为什么要它
----------
VFH 几何能算出「所有够宽的方向」, 但常常选不出**唯一**的那个: 左右都够宽、
两边来回换、或者干脆原地犹豫。这种时候缺的不是精度, 是一个偏好。模型擅长的
正是这个 —— 把「目标在哪边、左中右各多宽、刚才是不是在犹豫」一次看完, 给一个边。

为什么不让它定角度
------------------
距离是深度相机量的 (精确到 cm), 让 VLM 看图估距只会更差; 而且一次推理
0.5-3 s, 控制环是 10 Hz。所以分工是 **模型定边, 几何定角**: 模型只回一个词,
角度与安全线全部由 avoid_planner 的几何判据把关 —— 它说往左, 左边不够宽,
车照样不走。

三个触发点 (都从「几何自己在犹豫」出发, 不是定时轮询)
--------------------------------------------------
flap   2 秒内绕行换边 >= 2 次
tie    左右净空都 >= 0.70 m 且相差 < 0.30 m
stall  在开, 但 6 秒内到目标的距离没减少 0.30 m

不阻塞: 推理在 daemon 线程里跑, 结果 4 s 内有效, 过期自动作废回到几何。

本模块是纯逻辑 (不依赖 ROS), 便于离线测: 见 tests_llm_hint.py。
"""
from dataclasses import dataclass
import base64
import json
import math
import os
import re
import threading
import time

try:
    import requests
except Exception:                      # 没装 requests: 自动降级成纯几何
    requests = None

INF = float('inf')

# 触发原因 -> 提示词里的一句自然语言
_WHY = {
    'flap': 'it has switched sides several times in the last 2 seconds',
    'tie': 'both sides are open and almost equally wide',
    'stall': 'it is driving but not getting any closer to the goal',
}

# 模型回一个词; 顺手容错 TURN_LEFT / go forward 这类写法。
# 边界不用 \b: 下划线算 word 字符, "TURN_LEFT" 里的 LEFT 前面没有词边界,
# 会整句匹配不上 —— 用「前一个字符不是大写字母/下划线」来划界。
_ANS = re.compile(r'(?<![A-Z_])(?:TURN[ _-]?)?(LEFT|RIGHT|STRAIGHT|FORWARD|GO)(?![A-Z])')
_SIDE = {'LEFT': 1, 'RIGHT': -1, 'STRAIGHT': 0, 'FORWARD': 0, 'GO': 0}


def _strip_think(text):
    """丢掉推理标签里的内容 —— 否则「左边也堵」这种思考碎句会被当成答案。"""
    if not text:
        return ''
    t = re.sub(r'(?s)<(think|analysis|reasoning)>.*?</\1>', ' ', text)
    t = re.sub(r'(?s)<(think|analysis|reasoning)>.*$', ' ', t)   # 没闭合的, 后面全丢
    return t.strip()


def parse_side(text):
    """模型输出 -> +1 左 / -1 右 / 0 直走或看不懂。"""
    t = _strip_think(text).upper()
    m = _ANS.search(t)
    if not m:
        return 0, ''
    return _SIDE[m.group(1)], m.group(1)


def parse_json_answer(text):
    """模型输出 -> (边, 词, 置信度)。

    约束解码下答案就是这个形状; 解不出来退回老的关键词匹配 —— 老版本 LM Studio
    不认 response_format, 那时候还有一半的答案能捡回来。置信度只进日志,
    不当判据: 7B 的自评置信度校准很差, 拿它加权等于把偶然蒙对放大成纪律。
    """
    raw = _strip_think(text)
    i, j = raw.find('{'), raw.rfind('}')
    if 0 <= i < j:
        try:
            obj = json.loads(raw[i:j + 1])
        except (ValueError, TypeError):
            obj = None
        if isinstance(obj, dict):
            w = str(obj.get('side') or '').strip().upper()
            if w in _SIDE:
                try:
                    conf = float(obj.get('confidence'))
                except (TypeError, ValueError):
                    conf = 0.5
                return _SIDE[w], w, max(0.0, min(1.0, conf))
    side, word = parse_side(raw)
    return side, word, (0.5 if word else 0.0)


def _fmt(v):
    """净空 -> 提示词里的写法: 没测过/none 写成 '-', 空旷写成 'none'。"""
    if v is None:
        return '-'
    if not math.isfinite(v):
        return 'none'
    return '%.2f' % v


fmt_clear = _fmt          # 公开别名: 上层的日志也用同一套写法 (none / - / 0.82)


@dataclass
class Scene:
    """问模型之前, 把「它需要知道的全部」压成这一小包。"""
    trigger: str = ''          # flap / tie / stall
    goal_bearing: float = 0.0  # 目标相对车头 (deg), + 为左
    left: float = None         # 左侧净空 (m), 老的三个数保留: 提示词退化时还要用
    ahead: float = None
    right: float = None
    ring: tuple = ()           # 一整圈净空 (m), 从 0 度起每 ring_step 一格; () = 不喂
    prev_answer: str = ''      # 上一次它自己的答案, 给它一点连续性 (别每 2.5 s 换边)
    image: bytes = None        # JPEG, 仅 VLM 用; None = 这轮不带图
    combo: bool = False        # 图是「前视 + 俯视雷达」并排; False = 单张前视图


@dataclass
class HintConfig:
    """全部来自环境变量, 这样 run.sh 里 export 就能改, 不用改代码。"""
    on: bool = True
    provider: str = 'failover'                 # local / cloud / failover
    local_url: str = 'http://127.0.0.1:1234/v1'
    local_key: str = 'lm-studio'
    local_model: str = 'qwen/qwen3.5-9b'       # 与 vlm_model 保持一致: LM Studio
                                               # 一次只留一个模型在显存, 7B/9B 交替
                                               # 每次都要重载, 实测两边一起超时
    local_timeout: float = 8.0
    cloud_url: str = 'https://api.openai.com/v1'
    cloud_key: str = ''
    cloud_model: str = 'gpt-4o-mini'
    cloud_timeout: float = 12.0
    vision: bool = False                       # 要不要喂图 (VLM)
    vision_mode: str = 'front'                 # front = 只给前视相机图;
                                               # combo = 前视 + 雷达俯视扇区图并排。
                                               # 前视图只有 60 度, 左右哪边开阔不在画面里,
                                               # 实测 9B 对 4 组不同前视图一律回 LEFT;
                                               # 补上俯视雷达后 4 场景判对 3 个。
    vlm_url: str = 'http://127.0.0.1:1234/v1'
    vlm_key: str = 'lm-studio'
    vlm_model: str = ''                        # 空 = 不启用 VLM 分支
    vlm_timeout: float = 20.0
    min_interval: float = 2.5                  # 两次提问之间的最小间隔 (s)
    ttl: float = 4.0                           # 一次意见的有效期 (s)
    max_tokens: int = 8                        # 自由文本模式够用; JSON 模式会临时放大
    ring: bool = False                         # 喂 12 格净空还是老的左中右三个数。
                                               # 实测 (8 场景 x 10 次): 给 12 格明细
                                               # 并不提准 (两者都能满分, 配旧规则时
                                               # 反而从 87.5% 掉到 54%), 默认不喂 ——
                                               # 少 9 个数字, 少一份被读反的机会。
    json_schema: bool = True                   # 是否用 response_format 把输出锁成 JSON

    @classmethod
    def from_env(cls):
        e = os.environ
        def flag(name, dflt):
            v = e.get(name)
            if v is None:
                return dflt
            return v.strip().lower() not in ('0', 'false', 'no', 'off', '')
        def num(name, dflt):
            try:
                return float(e.get(name, dflt))
            except (TypeError, ValueError):
                return float(dflt)
        cfg = cls()
        cfg.on = flag('LLM_ON', True)
        cfg.provider = (e.get('LLM_PROVIDER') or 'failover').strip().lower()
        cfg.local_url = (e.get('LLM_LOCAL_URL') or cfg.local_url).rstrip('/')
        cfg.local_key = e.get('LLM_LOCAL_KEY') or cfg.local_key
        cfg.local_model = e.get('LLM_LOCAL_MODEL') or cfg.local_model
        cfg.local_timeout = num('LLM_LOCAL_TIMEOUT', cfg.local_timeout)
        cfg.cloud_url = (e.get('LLM_CLOUD_URL') or cfg.cloud_url).rstrip('/')
        cfg.cloud_key = e.get('LLM_CLOUD_KEY') or cfg.cloud_key
        cfg.cloud_model = e.get('LLM_CLOUD_MODEL') or cfg.cloud_model
        cfg.cloud_timeout = num('LLM_CLOUD_TIMEOUT', cfg.cloud_timeout)
        cfg.vision = flag('LLM_VISION', False)
        cfg.vision_mode = (e.get('LLM_VISION_MODE') or cfg.vision_mode).strip().lower()
        cfg.vlm_url = (e.get('LLM_VLM_URL') or cfg.vlm_url).rstrip('/')
        cfg.vlm_key = e.get('LLM_VLM_KEY') or cfg.vlm_key
        cfg.vlm_model = e.get('LLM_VLM_MODEL') or cfg.vlm_model
        cfg.vlm_timeout = num('LLM_VLM_TIMEOUT', cfg.vlm_timeout)
        cfg.min_interval = num('LLM_MIN_INTERVAL', cfg.min_interval)
        cfg.ttl = num('LLM_TTL', cfg.ttl)
        cfg.max_tokens = int(num('LLM_MAX_TOKENS', cfg.max_tokens))
        cfg.ring = flag('LLM_RING', cfg.ring)
        cfg.json_schema = flag('LLM_JSON', cfg.json_schema)
        return cfg

    def endpoints(self):
        """要试的端点, 按顺序。带图的 VLM 排在前面, 因为它才是「看得见」的那个。

        no_think: 只有会「思考」的模型才传 reasoning_effort, 见 _wants_no_think。
        """
        ep = []
        if self.vision and self.vlm_model:
            ep.append(('vlm', self.vlm_url, self.vlm_key, self.vlm_model,
                       self.vlm_timeout, True, _wants_no_think('vlm', self.vlm_model)))
        if self.provider in ('local', 'failover', 'ollama'):
            ep.append(('local', self.local_url, self.local_key, self.local_model,
                       self.local_timeout, False,
                       _wants_no_think('local', self.local_model)))
        if self.provider in ('cloud', 'failover', 'openai') and self.cloud_key:
            ep.append(('cloud', self.cloud_url, self.cloud_key, self.cloud_model,
                       self.cloud_timeout, False,
                       _wants_no_think('cloud', self.cloud_model)))
        return ep


def _wants_no_think(tag, model):
    """要不要给这个端点带 'reasoning_effort': 'none'。

    只对「带思考」的模型有用, 且只对它们无害:
      - qwen3 / qwen3.5 默认先吐一大段思考, 8 token 的 max_tokens 会被思考吃光,
        content 为空 -> 解析失败。实测 9B 加上这个字段从 46s(还在想) 变成 0.33s 回 LEFT。
      - qwen2.5 这类不思考的模型, 传了反而变慢 (实测 0.08s -> 8.85s), 所以不能统一传。
    别的关闭手段都试过且无效: chat_template_kwargs.enable_thinking、/no_think、
    顶层 enable_thinking、LM Studio 模型配置的 enableThinking 字段。
    """
    m = (model or '').lower()
    if tag == 'vlm':
        return True                                 # 视觉端就是那个 9B, 默认它会思考
    return any(k in m for k in ('qwen3', 'thinking', 'reasoner', 'qwq', 'deepseek-r1'))


def _prompt(scene, with_image, json_mode=False):
    """提示词: 只说车能做什么、三条硬规则、回一个词。

    以前的写法 (obstacle_avoider.py 里的 _prompt) 有三处会带偏模型, 这里逐条改掉:
      1. 'You control a robot camera' -> 相机会转头, 车不会横移, 主体错了;
      2. 只给「有没有 / 左右分界」-> 现在给真实净空, 模型不用猜;
      3. 只许 TURN_LEFT/TURN_RIGHT -> 空旷时「直走」才是正确答案, 必须可选。
    """
    b = scene.goal_bearing
    why = _WHY.get(scene.trigger, 'it is unsure')
    lines = [
        'You advise the motion planner of a ground robot.',
        'Robot: 0.40 m wide, 0.60 m long. It cannot strafe and cannot reverse.',
        'It turns on the spot, then drives forward along its nose.',
        '',
        'Goal bearing: %+.0f deg relative to the nose (0 = straight ahead, positive = left).' % b,
        '',
    ]
    ring = scene.ring if scene.ring else ()
    if ring:
        # 只给左中右三个数时, 模型分不清「两边都宽敞」和「两边都宽敞但身后全堵」,
        # 信息不足就只能猜 (实测 7B 一律回 LEFT)。给一圈它才看得出走廊/死角/贴墙。
        step = int(round(360.0 / len(ring)))
        cells = []
        for i, v in enumerate(ring):
            deg = i * step
            if deg > 180:
                deg -= 360
            cells.append('%+d=%s' % (deg, _fmt(v)))
        lines.append('Range readings all around (deg from the nose, + = left, '
                     'metres, none = nothing in range):')
        for i in range(0, len(cells), 4):
            lines.append('  ' + '  '.join(cells[i:i + 4]))
    else:
        lines += [
            'Depth-camera free space along each bearing, metres (none = nothing in range):',
            '  left  (+60 deg): %s' % _fmt(scene.left),
            '  ahead    (0 deg): %s' % _fmt(scene.ahead),
            '  right (-60 deg): %s' % _fmt(scene.right),
        ]
    if scene.prev_answer:
        lines += ['',
                  'Your previous answer was %s. Only change side if the readings now '
                  'clearly say so.' % scene.prev_answer]
    lines += [
        '',
        'The planner is unsure which side to detour around the obstacle: %s.' % why,
        'Answer which side it should favour.',
        '',
        'Rules:',
        '- the planner wants a side to steer around the obstacle it is about to hit',
        '- never pick a side whose free space is below 0.70 m',
        '- keep the final heading within 60 deg of the goal bearing',
        '- if the obstacle is ahead of you (ahead is not none while both sides are none,',
        '  or ahead is the smallest of the three), answer LEFT or RIGHT, never STRAIGHT',
        '- if both sides are 0.70 m or more, take the wider one; if they are equally',
        '  wide, take the one closer to the goal bearing',
        '- answer STRAIGHT only when the way ahead is clear and neither side is wider',
    ]
    if with_image:
        # 图是特意标注过的, 得先告诉它图上有什么, 否则它会把刻度线当成障碍物。
        if scene.combo:
            # 单独一张前视图回答不了「左右哪边空」: 60 度画面里根本没有那个信息,
            # 实测模型只能退化成常数 (4 组不同图一律 LEFT)。右边补一张雷达俯视图,
            # 它才知道自己在决定什么。两半要分别讲清楚, 否则会把雷达扇形当成物体。
            lines += ['',
                      'The attached image has TWO panels side by side.',
                      'LEFT panel: the front camera view, 60 deg wide, annotated with bearing',
                      'ticks every 10 deg (-30..+30, + is left), a yellow cross marking the',
                      'nose direction, LEFT/RIGHT labels and the measured free space printed',
                      'along the bottom edge.',
                      'RIGHT panel: a top-down radar map of the whole 360 deg around the',
                      'robot. The robot is the centre; UP in this panel is straight ahead;',
                      'LEFT in this panel is the robot left. Each wedge is coloured by how',
                      'far the nearest obstacle is in that direction: red = under 0.8 m,',
                      'orange under 1.4 m, yellow under 2.5 m, green = more open, grey =',
                      'never measured. A longer wedge means more room to that side.',
                      'Decide which side is genuinely more open from the radar panel; use',
                      'the front panel only to see what the thing ahead actually is.',
                      'Trust the printed numbers over anything you judge by eye.'] 
        else:
            lines += ['',
                      'The attached image is the front camera view, 60 deg wide.',
                      'It is annotated: bearing ticks every 10 deg (-30..+30, + is left), a',
                      'yellow cross marking the nose direction, LEFT/RIGHT labels, a green',
                      'arrow showing the goal bearing, and the measured free space in metres',
                      'printed along the bottom edge.',
                      'Trust those printed numbers over anything you judge by eye: do not',
                      'estimate distances from the picture yourself.']
    if json_mode:
        lines += ['', 'Answer as JSON: {"side": "LEFT" | "RIGHT" | "STRAIGHT", '
                      '"confidence": number between 0 and 1}']
    else:
        lines += ['', 'Reply with exactly one word: LEFT, RIGHT or STRAIGHT.']
    return '\n'.join(lines)


# 把「一个词」锁成一个形状: 模型在 decode 阶段就没法吐出解释文字,
# 解析失败率从「时有时无」变成确定性 0 (llama.cpp 侧等价于 GBNF grammar)。
_SIDE_SCHEMA = {
    'type': 'json_schema',
    'json_schema': {
        'name': 'detour_side',
        'strict': True,
        'schema': {
            'type': 'object',
            'properties': {
                'side': {'type': 'string', 'enum': ['LEFT', 'RIGHT', 'STRAIGHT']},
                'confidence': {'type': 'number'},
            },
            'required': ['side', 'confidence'],
            'additionalProperties': False,
        },
    },
}


def _payload(model, prompt, image, max_tokens, no_think=True, json_mode=False):
    if image:
        content = [
            {'type': 'text', 'text': prompt},
            {'type': 'image_url',
             'image_url': {'url': 'data:image/jpeg;base64,'
                                  + base64.b64encode(image).decode('ascii')}},
        ]
    else:
        content = prompt
    p = {
        'model': model,
        'messages': [{'role': 'user', 'content': content}],
        'max_tokens': max_tokens,
        'temperature': 0,
        'stream': False,
    }
    if json_mode:
        # JSON 比一个词长得多, 8 token 装不下 -> 会被截成半截 JSON, 反而更糟。
        p['max_tokens'] = max(int(max_tokens), 48)
        p['response_format'] = _SIDE_SCHEMA
    if no_think:
        # LM Studio 0.4.8+ 认这个键 (取值 none/minimal/low/medium/high/xhigh)。
        # 注意别用 chat_template_kwargs.enable_thinking: LM Studio 下实测无效, 模型照旧思考。
        p['reasoning_effort'] = 'none'
    return p


class LlmHint:
    """异步问一次, 给一个边的偏好。任何失败都只是「没意见」, 绝不拖住控制环。"""

    def __init__(self, cfg=None, log=None):
        self.cfg = cfg or HintConfig.from_env()
        self.log = log
        self._lock = threading.Lock()
        self._busy = False
        self._side = 0
        self._conf = 0.0
        self._res_t = -1e9
        self._last_ask = -1e9
        self._last_warn = -1e9
        self.asked = 0
        self.answered = 0
        self.failed = 0
        self.last_reason = ''
        self.last_err = ''
        self.last_latency = 0.0

    # ---------------- 对外 ----------------
    def warmup(self, image=None):
        """后台先问一句废话, 把模型拉热 —— 别让第一次真提问撞上冷启动。

        实测 (LM Studio, 本机): 9B 带图第一次 7.66 s, 第二次起 0.29 s;
        7B 换入内存后第一次 1.9 s, 之后 0.07 s。这几秒该花在启动时, 不该花在要撞墙的时候。
        """
        def run():
            for tag, base, key, model, timeout, want_img, no_think in self.cfg.endpoints():
                if want_img and not image:
                    continue
                try:
                    self._post(base, key, model, 'ok', image,
                               max(timeout, 180.0), no_think)
                except Exception:                   # noqa: BLE001  预热失败无所谓
                    pass
        threading.Thread(target=run, daemon=True).start()

    def consider(self, now, scene):
        """满足触发条件就问一次 (不等待)。返回本次触发的原因, 没触发返回 None。"""
        c = self.cfg
        if not c.on or requests is None:
            return None
        if not scene or not scene.trigger:
            return None
        with self._lock:
            if self._busy:
                return None
            if now - self._last_ask < c.min_interval:
                return None
            self._busy = True
            self._last_ask = now
            self.asked += 1
        threading.Thread(target=self._worker, args=(scene,), daemon=True).start()
        return scene.trigger

    def side(self, now):
        """当前有效的意见: +1 左 / -1 右 / 0 没意见 (过期、还没回、或失败)。"""
        if not self.cfg.on:
            return 0
        with self._lock:
            if now - self._res_t > self.cfg.ttl:
                return 0
            return self._side

    def conf(self, now):
        """这条意见自报的置信度 (0-1), 只进日志。过期返回 0。"""
        if not self.cfg.on:
            return 0.0
        with self._lock:
            if now - self._res_t > self.cfg.ttl:
                return 0.0
            return self._conf

    def status(self):
        return ('llm: спрошено %d, ответов %d, сбоев %d%s'
                % (self.asked, self.answered, self.failed,
                   (', последний: %s' % self.last_reason) if self.last_reason else ''))

    # ---------------- 内部 ----------------
    def _worker(self, scene):
        t0 = time.time()
        side, reason, err, conf = 0, '', '', 0.0
        try:
            side, reason, conf = self._ask(scene)
        except Exception as e:                      # noqa: BLE001 —— 兜底, 绝不外抛
            err = '%s: %s' % (type(e).__name__, e)
        finally:
            with self._lock:
                self._side = side
                self._conf = conf
                self._res_t = time.time()
                self._busy = False
                self.last_latency = time.time() - t0
                if err:
                    self.failed += 1
                    self.last_err = err
                else:
                    self.answered += 1
                    self.last_reason = reason
        if self.log is None:
            return
        if err:
            now = time.time()
            if now - self._last_warn > 10.0:        # 模型挂了不要刷屏
                self._last_warn = now
                self.log('llm недоступна, работаю по геометрии: %s' % err, 'warn')
        else:
            # 成功也报一声: 模型回 STRAIGHT 时 side=0, 上层不会打任何日志,
            # 「答了但弃权」与「根本没答」必须能从日志里分开。
            self.log('llm ответ: %s, %.1f с' % (reason, self.last_latency), 'info')

    def _ask(self, scene):
        """按顺序试端点; 全挂抛异常。任一端点给出明确答案就停。"""
        prompt = _prompt(scene, bool(scene.image), self.cfg.json_schema)
        errs = []
        for tag, base, key, model, timeout, want_img, no_think in self.cfg.endpoints():
            image = scene.image if (want_img and scene.image) else None
            if want_img and not image:
                continue                            # 没有图就别白问 VLM
            try:
                text = self._post(base, key, model, prompt, image, timeout, no_think)
            except Exception as e:                  # noqa: BLE001
                errs.append('%s: %s' % (tag, e))
                continue
            side, word, conf = parse_json_answer(text)
            if word:
                return side, '%s -> %s (%.2f)' % (tag, word, conf), conf
            errs.append('%s: empty answer (%r)' % (tag, (text or '')[:40]))
        raise RuntimeError('; '.join(errs) or 'no endpoint available')

    def _post(self, base, key, model, prompt, image, timeout, no_think=False):
        url = base + '/chat/completions'
        headers = {'Content-Type': 'application/json'}
        if key:
            headers['Authorization'] = 'Bearer ' + key
        payload = _payload(model, prompt, image, self.cfg.max_tokens,
                           no_think=no_think, json_mode=self.cfg.json_schema)
        r = requests.post(url, data=json.dumps(payload), headers=headers, timeout=timeout)
        # 两级退让: 不认 reasoning_effort 的网关不少, 不认 response_format 的更多。
        # 每退一级都重问一次, 而不是直接放弃 —— 退回纯文本时关键词解析还能捡回答案。
        for _ in range(2):
            if r.status_code not in (400, 422):
                break
            if 'reasoning_effort' in payload:
                payload.pop('reasoning_effort', None)
            elif 'response_format' in payload:
                payload.pop('response_format', None)
                payload['max_tokens'] = self.cfg.max_tokens
            else:
                break
            r = requests.post(url, data=json.dumps(payload), headers=headers, timeout=timeout)
        r.raise_for_status()
        data = r.json()
        return data['choices'][0]['message'].get('content') or ''
