#!/usr/bin/env python3
"""深度相机自检脚本。

用法:
  python3 ~/gz_ros/check_depth.py                      # 默认检查 12 秒
  python3 ~/gz_ros/check_depth.py --ros-args -p duration:=5.0

检查内容:
  1. 话题是否有数据在流: /camera, /depth_camera, /depth_camera/camera_info (点云可选)
  2. 帧率 / 分辨率 / 编码 / 坐标系 (应为 camera_link)
  3. 深度值是否真实: 有效像素比例、最小/中位/最大深度
  4. 左 / 中 / 右三个扇区的最小深度 (导航避障用的就是这个"正前方")
  5. camera_info 内参 (算水平 FOV, 判断是不是那台 60 度的相机)

退出码: 0=通过  1=有问题
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

TOPICS = {
    'rgb': '/camera',
    'depth': '/depth_camera',                 # 本仓库: 深度图 32FC1, 单位米
    'info': '/depth_camera/camera_info',
    'points': '/depth_camera/points',         # 可选: 本仓库默认不桥接点云
}
OPTIONAL = ('points',)                        # 缺失不算失败


class DepthCheck(Node):
    def __init__(self, duration):
        super().__init__('check_depth')
        self.duration = duration
        self.count = {k: 0 for k in TOPICS}
        self.data = {}
        self.t0 = time.time()

        self.create_subscription(Image, TOPICS['rgb'], lambda m: self.on_img(m, 'rgb'), 10)
        self.create_subscription(Image, TOPICS['depth'], lambda m: self.on_img(m, 'depth'), 10)
        self.create_subscription(CameraInfo, TOPICS['info'], self.on_info, 10)
        self.create_subscription(PointCloud2, TOPICS['points'], self.on_pc, 10)

    # ---------- 回调 ----------
    def on_img(self, m, key):
        self.count[key] += 1
        if key == 'depth' and 'depth' not in self.data:
            self.data['depth'] = self.analyse_depth(m)

    def on_info(self, m):
        self.count['info'] += 1
        if 'k' not in self.data:
            self.data['k'] = list(m.k)[:4]        # fx, 0, cx, 0
            self.data['size'] = (m.width, m.height)

    def on_pc(self, m):
        self.count['points'] += 1
        if 'pc' not in self.data:
            self.data['pc'] = (m.width, m.height, len(m.data), m.header.frame_id)

    # ---------- 分析 ----------
    @staticmethod
    def analyse_depth(m):
        row = m.step // 4
        arr = np.frombuffer(m.data, dtype=np.float32)
        if row <= 0 or arr.size < row * m.height:
            return None
        img = arr.reshape(m.height, row)[:, :m.width]
        valid = np.isfinite(img) & (img > 0.05)
        fin = img[valid]
        out = {
            'size': (m.width, m.height), 'enc': m.encoding, 'frame': m.header.frame_id,
            'valid_px': int(valid.sum()), 'total_px': int(img.size),
            'center': float(img[m.height // 2, m.width // 2]),
            'sector': {},
        }
        if fin.size:
            out['min'] = float(fin.min())
            out['median'] = float(np.median(fin))
            out['max'] = float(fin.max())
        h, w = img.shape
        band = img[int(h * 0.40):int(h * 0.85), :]
        for name, c0, c1 in (('left', 0.05, 0.35), ('front', 0.35, 0.65), ('right', 0.65, 0.95)):
            s = band[:, int(w * c0):int(w * c1)]
            s = s[np.isfinite(s) & (s > 0.05)]
            out['sector'][name] = float(np.percentile(s, 5)) if s.size >= 20 else None
        return out


def main():
    rclpy.init()
    duration = 12.0
    tmp = rclpy.create_node('param_probe')       # 读参数用
    tmp.declare_parameter('duration', 12.0)
    duration = float(tmp.get_parameter('duration').value)
    tmp.destroy_node()

    n = DepthCheck(duration)
    print(f'深度相机自检中... 采样 {duration:.0f} 秒\n')
    try:
        while time.time() - n.t0 < duration:
            rclpy.spin_once(n, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('\n(被中断, 下面是已采集到的数据)')
    el = time.time() - n.t0
    ok = True

    print('=' * 62)
    print('1) 话题数据流')
    for key, topic in TOPICS.items():
        c = n.count[key]
        opt = key in OPTIONAL
        flag = 'OK ' if c > 0 else ('跳过' if opt else 'FAIL')
        if c == 0 and not opt:
            ok = False
        note = '  (可选, 未桥接正常)' if opt and c == 0 else ''
        print(f'   [{flag}] {topic:34s} {c:5d} 帧  ≈ {c / el:4.1f} Hz{note}')

    print('\n2) 深度图属性')
    d = n.data.get('depth')
    if d:
        print(f'   分辨率 : {d["size"][0]}x{d["size"][1]}')
        print(f'   编码   : {d["enc"]}   (应为 32FC1 = 32位浮点, 单位米)')
        print(f'   坐标系 : {d["frame"]!r}')
        if d['frame'] != 'camera_link':
            print('          (本仓库深度相机没设 gz_frame_id, 这个 frame 不在位置感知的 TF 树里;')
            print('           Image 显示不需要 TF, 只有点云/三维对齐才需要)')
    else:
        ok = False
        print('   FAIL: 没收到深度图')

    print('\n3) 深度值是否真实')
    if d and 'median' in d:
        pct = 100.0 * d['valid_px'] / max(d['total_px'], 1)
        print(f'   有效像素: {d["valid_px"]}/{d["total_px"]} ({pct:.1f}%)  其余为 inf(超量程/天空)')
        print(f'   深度范围: min={d["min"]:.2f}  中位={d["median"]:.2f}  max={d["max"]:.2f} m')
        print(f'   画面正中心: {d["center"]:.2f} m')
        if d['max'] - d['min'] < 1e-6:
            ok = False
            print('   FAIL: 深度全是一个值, 传感器没有真正工作')
        else:
            print('   OK: 深度值有正常起伏, 数据是真实的')
    else:
        ok = False
        print('   FAIL: 没有有效深度值')

    print('\n4) 左/中/右扇区最小深度 (导航避障判断"正前方"用的)')
    if d:
        for name, cn in (('left', '左  '), ('front', '正前'), ('right', '右  ')):
            v = d['sector'].get(name)
            print(f'   {cn}: {"(无有效值)" if v is None else f"{v:.2f} m"}')

    print('\n5) 相机内参')
    if 'k' in n.data:
        fx, _, cx, _ = n.data['k']
        w = n.data.get('size', (320, 240))[0]
        fov = 2 * math.atan(w / (2 * fx)) if fx else 0
        print(f'   fx={fx:.1f}  cx={cx:.1f}  水平 FOV = {math.degrees(fov):.1f} deg (SDF 配置 60.0)')
    else:
        print('   (没收到 camera_info)')

    print('\n6) 点云 (可选)')
    pc = n.data.get('pc')
    if pc:
        print(f'   {pc[0]}x{pc[1]}  data={pc[2]}B  frame={pc[3]!r}')
    else:
        print('   (未桥接点云 —— 本仓库 start_simulation.sh 默认注释掉点云桥, 正常)')

    print('=' * 62)
    print('结论: ' + ('全部通过 ✅ 深度相机工作正常' if ok else
                     '有问题 ❌ 见上面 FAIL 项'))
    print('\n提示: 若话题数据流全为 0, 多半是仿真没重启(改了 SDF 必须重启 gz sim);')
    print('      本仓库深度图话题是 /depth_camera (不是 /depth_camera/depth_image)。')

    try:
        n.destroy_node()
    except Exception:  # noqa: BLE001
        pass
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
