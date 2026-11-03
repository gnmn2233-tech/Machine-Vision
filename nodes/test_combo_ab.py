
import sys, os, math, time, json, base64, re, urllib.request
sys.path.insert(0, '/root/gz_ros/nodes')
import numpy as np, cv2

t0 = time.time()
# ---- 1. 抓一帧真实相机 + 一帧真实雷达 ----
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
rclpy.init()
g = Node('grab_ab'); br = CvBridge(); got = {}
def on_img(m):
    if 'img' not in got: got['img'] = br.imgmsg_to_cv2(m, 'bgr8')
def on_scan(m):
    if 'scan' not in got: got['scan'] = (np.array(m.ranges, np.float64), m.angle_min, m.angle_increment)
g.create_subscription(Image, '/camera', on_img, 10)
g.create_subscription(LaserScan, '/scan', on_scan, 10)
while len(got) < 2 and time.time() - t0 < 15:
    rclpy.spin_once(g, timeout_sec=0.2)
frame = got.get('img'); scan = got.get('scan')
print('抓帧:', None if frame is None else frame.shape, 'scan:', scan is not None)
g.destroy_node(); rclpy.shutdown()
if frame is None or scan is None: sys.exit('没有仿真数据')
cv2.imwrite('/tmp/real_frame.png', frame)

# ---- 2. 造两个对称场景, 只改雷达 ----
from avoid_planner import AvoidPlanner, Config
import importlib.util
spec = importlib.util.spec_from_file_location('ntp', '/root/gz_ros/nodes/navigate_to_pose.py')
ntp = importlib.util.module_from_spec(spec); spec.loader.exec_module(ntp)
import llm_hint as LH

def planner(left_m, right_m, ahead_m=2.0):
    p = AvoidPlanner(Config())
    n = 450; amin = -math.pi; inc = 2 * math.pi / n
    now = time.time(); rr = []
    for i in range(n):
        ang = amin + i * inc
        d = left_m if math.sin(ang) > 0 else right_m
        if abs(ang) < math.radians(12): d = ahead_m
        rr.append(d)
    p._ingest_scan(rr, amin, inc, 0.0, now)
    return p, now

def render_combo(p, now, gb, l, r):
    fake = {'avoider': p}
    radar = ntp.Navigator._radar_panel(type('F', (), {'avoider': p})(), now)
    front = ntp.Navigator._annotate(None, frame, gb, l, 2.0, r)
    cv2.putText(front, 'FRONT VIEW', (6, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (235,235,235), 1)
    f2 = cv2.resize(front, (radar.shape[1], radar.shape[0]))
    seam = np.full((radar.shape[0], 4, 3), 96, np.uint8)
    return np.hstack([f2, seam, radar])

def render_front(gb, l, r):
    return ntp.Navigator._annotate(None, frame, gb, l, 2.0, r)

import requests
def ask(img, prompt):
    if img is None:
        payload = LH._payload('qwen/qwen3.5-9b', prompt, None,
                              cfg.max_tokens, no_think=True,
                              json_mode=cfg.json_schema)
    else:
        ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        payload = LH._payload('qwen/qwen3.5-9b', prompt, buf.tobytes(),
                              cfg.max_tokens, no_think=True,
                              json_mode=cfg.json_schema)
    s = time.time()
    try:
        r = requests.post('http://127.0.0.1:1234/v1/chat/completions',
                          data=json.dumps(payload),
                          headers={'Content-Type': 'application/json'}, timeout=90)
        txt = r.json()['choices'][0]['message'].get('content') or ''
    except Exception as e:
        txt = 'ERR %s' % e
    return txt, time.time() - s

cfg = LH.HintConfig()
cases = [('左空右堵', 4.0, 0.8), ('右空左堵', 0.8, 4.0)]
def render_noleak(p, now, gb, l, r):
    img = render_combo(p, now, gb, l, r).copy()
    H, W = img.shape[:2]
    cv2.rectangle(img, (0, H - 34), (80, H), (18, 18, 18), -1)
    cv2.rectangle(img, (W - 82, H - 34), (W, H), (18, 18, 18), -1)
    return img

def render_small(gb, l, r):
    return cv2.resize(render_front(gb, l, r), (336, 252))

res = {}
for rnd in range(3):
    for mode in ('real448',):
        for name, l, r in cases:
            p, now = planner(l, r)
            img = (None if mode == 'text' else
                   ntp.Navigator._annotate(
                       None, cv2.resize(frame, (448, 336)), 0.0, l, 2.0, r)
                   if mode == 'real448' else render_front(0.0, l, r))
            scene = LH.Scene(trigger='tie', goal_bearing=0.0, left=l, ahead=2.0, right=r,
                             ring=(), prev_answer='', image=b'x', combo=(mode == 'combo'))
            prompt = LH._prompt(scene, mode != 'text', cfg.json_schema)
            txt, dt = ask(img, prompt)
            mm = re.search(r'"side"\s*:\s*"(\w+)"', txt or '')
            side = mm.group(1) if mm else (txt or '')[:24].replace('\n', ' ')
            res.setdefault((mode, name), []).append((side, round(dt, 2)))
            if rnd == 0 and img is not None:
                cv2.imwrite('/tmp/ab_%s_%s.jpg'
                            % (mode, 'L' if l > r else 'R'), img)
print('=== 3 轮汇总 (左空右堵 应为 LEFT, 右空左堵 应为 RIGHT) ===')
for (mode, name), v in res.items():
    print('%-6s %-8s %s' % (mode, name, v))
print('总耗时 %.1f s' % (time.time() - t0))
