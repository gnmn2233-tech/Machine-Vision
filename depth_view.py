#!/usr/bin/env python3
"""深度相机可视化: 深度图伪彩色 + YOLO 检测框/目标距离 + 左中右分区最近距离。

输出:
  /depth_view   sensor_msgs/Image (bgr8)   —— 用 RViz 的 Image 显示, 或 rqt_image_view /depth_view

参数:
  background     底图: depth(默认) 或 rgb
  colormap       深度伪彩色表: turbo(默认)/jet/hot/bone/rainbow/viridis
  min_range      最近显示距离 (m), 默认 0.30
  max_range      最远显示距离 (m), 默认 10.0
  scale          放大倍数, 默认 2 (320x240 -> 640x480, 字看得清)
  band           取"各区最近距离"的行带, 默认 "0.40,0.85" (= 导航避障用的行带)
                 想只看物体、不看地面, 可改成 "0.40,0.60"
  box_stat       框内取值: min(最近点) / median(默认)
  show_mask      true(默认) 用分割模型的 mask 多边形填充实心剪影 (需 yolov8*-seg 模型)
  mask_alpha     剪影不透明度, 默认 0.85
  save_dir       非空则每隔 save_interval 秒存一张 png (留空=不存)
  save_interval  存图间隔秒, 默认 5.0
  show_window    true 会在本机弹一个 cv2 窗口 (需要图形界面)

分区与导航一致: 列 35% / 65% 分左右, 行 40%~85% 为避障行带; 顶部显示各区行带最小深度。
"""
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from yolo_msgs.msg import DetectionArray

CMAPS = {
    'turbo': cv2.COLORMAP_TURBO,
    'jet': cv2.COLORMAP_JET,
    'hot': cv2.COLORMAP_HOT,
    'bone': cv2.COLORMAP_BONE,
    'rainbow': cv2.COLORMAP_RAINBOW,
    'viridis': cv2.COLORMAP_VIRIDIS,
}
CLASS_COLORS = {
    'car': (60, 220, 60), 'truck': (60, 220, 60), 'bus': (60, 220, 60),
    'person': (200, 255, 60), 'bicycle': (0, 200, 255), 'motorcycle': (0, 160, 255),
}
ZONE_SPLIT = (0.35, 0.65)      # 与 goal_navigator 的分区一致
BAND = (0.40, 0.85)            # 避障行带
GREEN = (0, 255, 0)
CYAN = (255, 255, 0)
WHITE = (255, 255, 255)


class DepthView(Node):
    def __init__(self):
        super().__init__('depth_view')
        p = self.declare_parameter
        self.bg = str(p('background', 'depth').value).lower()
        self.cmap_name = str(p('colormap', 'turbo').value).lower()
        self.min_r = float(p('min_range', 0.30).value)
        self.max_r = float(p('max_range', 10.0).value)
        self.scale = max(1, int(p('scale', 2).value))
        try:
            b0, b1 = str(p('band', '0.40,0.85').value).split(',')
            self.band = (float(b0), float(b1))
        except Exception:  # noqa: BLE001
            self.band = BAND
        self.box_stat = str(p('box_stat', 'median').value).lower()
        self.show_mask = bool(p('show_mask', True).value)
        self.mask_alpha = float(p('mask_alpha', 0.85).value)
        self.save_dir = str(p('save_dir', '').value)
        self.save_interval = float(p('save_interval', 5.0).value)
        self.show_window = bool(p('show_window', False).value)
        # 深度图话题: 本仓库用 /depth_camera (32FC1, 米); 旧版 rgbd 相机则是 /depth_camera/depth_image
        self.depth_topic = str(p('depth_topic', '/depth_camera').value)
        self.rgb_topic = str(p('rgb_topic', '/camera').value)

        self.depth = None          # (h, w) float32, m
        self.rgb = None            # (H, W, 3) bgr8
        self.rgb_size = (640, 480)
        self.dets = []
        self._last_save = 0.0

        self.cmap = CMAPS.get(self.cmap_name, cv2.COLORMAP_TURBO)
        self.create_subscription(Image, self.depth_topic, self.on_depth, 10)
        self.create_subscription(Image, self.rgb_topic, self.on_rgb, 10)
        self.create_subscription(DetectionArray, '/yolo/detections', self.on_dets, 10)
        self.pub = self.create_publisher(Image, '/depth_view', 10)
        self.create_timer(0.05, self.render)       # 20 Hz

        if self.save_dir:
            os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info(
            f'深度可视化启动: 底图={self.bg} 色表={self.cmap_name} 掩膜={self.show_mask} '
            f'范围={self.min_r}~{self.max_r}m 放大={self.scale}x'
            + (f' 存图={self.save_dir}' if self.save_dir else ''))
        self.get_logger().info('看画面: rqt_image_view /depth_view  或 RViz 加 Image 显示, 话题选 /depth_view')

    # ---------------- 回调 ----------------
    def on_depth(self, m):
        row = m.step // 4
        a = np.frombuffer(m.data, dtype=np.float32)
        if row <= 0 or a.size < row * m.height:
            return
        self.depth = a.reshape(m.height, row)[:, :m.width]

    def on_rgb(self, m):
        if self.rgb is None or m.encoding not in ('rgb8', 'bgr8'):
            self.rgb_size = (m.width, m.height)
        if m.encoding == 'rgb8':
            self.rgb = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)[:, :, ::-1].copy()
        elif m.encoding == 'bgr8':
            self.rgb = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3).copy()
        self.rgb_size = (m.width, m.height)

    def on_dets(self, m):
        self.dets = [d for d in m.detections if getattr(d, 'score', 0.0) >= 0.4]

    # ---------------- 工具 ----------------
    def zone_values(self):
        """返回 左/中/右 三个区在避障行带内的最小深度 (m), 无效为 None"""
        if self.depth is None:
            return (None, None, None)
        h, w = self.depth.shape
        band = self.depth[int(h * self.band[0]):int(h * self.band[1]), :]
        out = []
        for c0, c1 in ((0.0, ZONE_SPLIT[0]), (ZONE_SPLIT[0], ZONE_SPLIT[1]), (ZONE_SPLIT[1], 1.0)):
            z = band[:, int(w * c0):int(w * c1)]
            v = z[np.isfinite(z) & (z > 0.05)]
            out.append(float(np.percentile(v, 5)) if v.size >= 20 else None)
        return tuple(out)

    def box_distance(self, d):
        """取检测框内的深度代表值 (m)"""
        if self.depth is None:
            return None
        h, w = self.depth.shape
        rw, rh = self.rgb_size
        bb = d.bbox
        cx, cy = bb.center.position.x, bb.center.position.y
        bw, bh = bb.size.x, bb.size.y
        x0 = int(max(0, (cx - bw / 2.0) * w / rw))
        x1 = int(min(w, (cx + bw / 2.0) * w / rw))
        y0 = int(max(0, (cy - bh / 2.0) * h / rh))
        y1 = int(min(h, (cy + bh / 2.0) * h / rh))
        if x1 <= x0 or y1 <= y0:
            return None
        roi = self.depth[y0:y1, x0:x1]
        v = roi[np.isfinite(roi) & (roi > 0.05)]
        if v.size < 10:
            return None
        return float(v.min()) if self.box_stat == 'min' else float(np.median(v))

    def mask_polygon(self, d, w, h):
        """把分割 mask 的多边形点换算到(放大后的)深度图坐标系"""
        mk = d.mask
        try:
            n = len(mk.data)
        except Exception:  # noqa: BLE001
            return None
        if not self.show_mask or mk.width <= 0 or mk.height <= 0 or n < 3:
            return None
        sx = w * self.scale / float(mk.width)
        sy = h * self.scale / float(mk.height)
        pts = np.array([[int(p.x * sx), int(p.y * sy)] for p in mk.data], dtype=np.int32)
        return pts if pts.shape[0] >= 3 else None

    @staticmethod
    def class_color(name):
        return CLASS_COLORS.get(name, (255, 160, 60))

    # ---------------- 绘制 ----------------
    def render(self):
        if self.depth is None:
            return
        h, w = self.depth.shape
        # 1) 底图
        if self.bg == 'rgb' and self.rgb is not None:
            base = cv2.resize(self.rgb, (w, h), interpolation=cv2.INTER_LINEAR)
        else:
            span = max(self.max_r - self.min_r, 1e-6)
            n = np.clip((self.depth - self.min_r) / span, 0.0, 1.0)
            base = cv2.applyColorMap((n * 255.0).astype(np.uint8), self.cmap)
            base[~np.isfinite(self.depth)] = 0          # 无效(无穷远) -> 黑
            base[self.depth <= 0.05] = 0

        vis = cv2.resize(base, (w * self.scale, h * self.scale), interpolation=cv2.INTER_NEAREST)

        # 2) 分区竖线
        H, W = vis.shape[:2]
        for s in ZONE_SPLIT:
            x = int(W * s)
            cv2.line(vis, (x, 0), (x, H), GREEN, 1)

        # 3) 顶部各区最小深度
        lz, cz, rz = self.zone_values()
        fmt = lambda v: '--' if v is None else f'{v:.2f}m'
        cv2.putText(vis, f'L {fmt(lz)}', (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 2, cv2.LINE_AA)
        cv2.putText(vis, f'C {fmt(cz)}', (int(W * 0.42), 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 2, cv2.LINE_AA)
        t = f'R {fmt(rz)}'
        (tw, _), _ = cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.putText(vis, t, (W - tw - 8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 2, cv2.LINE_AA)

        # 4) YOLO 检测框 + 目标距离
        rw, rh = self.rgb_size
        for d in self.dets:
            color = self.class_color(d.class_name)
            # 6a) 分割剪影 (实心)
            poly = self.mask_polygon(d, w, h)
            if poly is not None:
                ov = vis.copy()
                cv2.fillPoly(ov, [poly], color)
                cv2.addWeighted(ov, self.mask_alpha, vis, 1.0 - self.mask_alpha, 0.0, vis)
                cv2.polylines(vis, [poly], True, color, 1)
            # 6b) 检测框 + 目标距离
            bb = d.bbox
            cx, cy = bb.center.position.x, bb.center.position.y
            bw, bh = bb.size.x, bb.size.y
            x0 = int((cx - bw / 2.0) * w / rw * self.scale)
            x1 = int((cx + bw / 2.0) * w / rw * self.scale)
            y0 = int((cy - bh / 2.0) * h / rh * self.scale)
            y1 = int((cy + bh / 2.0) * h / rh * self.scale)
            cv2.rectangle(vis, (x0, y0), (x1, y1), WHITE, 1)
            dist = self.box_distance(d)
            label = f'{d.class_name} {"?" if dist is None else f"{dist:.2f}m"}'
            cv2.putText(vis, label, (x0 + 2, max(y0 - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)

        # 5) 发布 / 存图 / 显示
        out = Image()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'depth_camera_link'
        out.height, out.width = vis.shape[:2]
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = out.width * 3
        out.data = vis.tobytes()
        self.pub.publish(out)

        now = time.time()
        if self.save_dir and now - self._last_save >= self.save_interval:
            self._last_save = now
            fn = os.path.join(self.save_dir, time.strftime('depth_view_%Y%m%d_%H%M%S.png'))
            cv2.imwrite(fn, vis)
            self.get_logger().info(f'已存图: {fn}')
        if self.show_window:
            cv2.imshow('depth_view', vis)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DepthView()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.show_window:
            cv2.destroyAllWindows()
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():            # 被 kill/Ctrl+C 时上下文可能已经关了, 别重复 shutdown
            rclpy.shutdown()


if __name__ == '__main__':
    main()
