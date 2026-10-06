"""深度相机测距探针: 把 /depth_camera 变成人看得懂的"哪里有多远"。

用途: 验证深度相机 + gz 桥是否正常, 并直观看到障碍物距离。

  ros2 run obstacle_avoider depth_probe                     # 只打印三区距离
  ros2 run obstacle_avoider depth_probe --view              # 额外开窗口(热力图, 越红越近)
  ros2 run obstacle_avoider depth_probe --publish           # 不开窗, 只把彩色图发到 /depth_camera/colorized
  ros2 run obstacle_avoider depth_probe --publish --quiet   # 同上且完全静默(挂在 RViz2 场景里用)

看图的两种方式:
  1. --view 直接在机器上弹 OpenCV 窗口(最省事);
  2. --publish 后用 rqt_image_view / RViz2 订阅 /depth_camera/colorized
     (原始 /depth_camera 是 32FC1 浮点(米), 普通看图工具显示不出来, 所以这里发一份 8UC3 彩色版)。

测距逻辑与 obstacle_avoider 节点保持同一套:
  1. 相机高度固定 -> 每个像素行有确定的"地面期望深度", 比它更近的像素才算障碍(矮障碍不会被地面淹没)。
  2. 按 左(<40%) / 中(40%~60%) / 右(>60%) 三区, 各取低百分位最近距离; 无近处障碍输出 inf。
  3. yolo.sh 在跑时订阅 /yolo/detections, 每个检测框单独算距离(框内像素中位数), 画成 "person 4.33m" 标签。
"""
import argparse
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from yolo_msgs.msg import DetectionArray

DEPTH_TOPIC = '/depth_camera'
VIS_TOPIC = '/depth_camera/colorized'   # 彩色化深度图(8UC3 bgr8), 给人看用
DET_TOPIC = '/yolo/detections'          # 每个检测框一个距离(需要 yolo.sh 在跑)
DET_TIMEOUT = 2.0                       # 检测结果超过这么久没更新就不画(避免留旧框)
# 相机光心离地高度(米); 相机位置/装配变了要同步改(与 obstacle_avoider.py 一致)。
CAM_HEIGHT = 0.28
CAM_HFOV = 1.047          # 相机水平视场 (与 SDF 中 horizontal_fov 一致)
GROUND_RATIO = 0.75       # 像素深度 < 地面期望 * 该比例 -> 判为障碍
DEPTH_PERCENTILE = 10.0   # 每区取该百分位作为最近距离(抗离群像素)
LEFT_PERCENT = 0.4
RIGHT_PERCENT = 0.6
VIEW_MAX_RANGE = 10.0     # 窗口可视化的最大距离(米), 更远一律算很远


def zone_dist(depth, mask):
    """区域内障碍像素的最近距离(低百分位); 无有效像素返回 inf。"""
    vals = depth[mask][::4]
    vals = vals[np.isfinite(vals) & (vals > 0.0)]
    if vals.size == 0:
        return math.inf
    return float(np.percentile(vals, DEPTH_PERCENTILE))


def obstacle_mask(depth):
    """剔地面: 上半画面任何有效回波都算障碍; 下半画面要比地面期望更近才算。"""
    h, w = depth.shape[:2]
    rows = np.arange(h, dtype=np.float32)
    fy = (w / 2.0) / math.tan(CAM_HFOV / 2.0)
    ground = CAM_HEIGHT * fy / np.maximum(rows - h / 2.0, 1e-3)
    mask = np.isfinite(depth) & (depth > 0.0) & (rows[:, None] <= h / 2.0)
    below = rows > h / 2.0
    mask[below] |= depth[below] < ground[below, None] * GROUND_RATIO
    return mask


def fmt(d):
    return '  inf(空旷)' if not math.isfinite(d) else f'{d:6.2f} m'


class DepthProbe(Node):
    def __init__(self, view=False, publish=False, quiet=False):
        super().__init__('depth_probe')
        self.view = view
        self.publish = publish
        self.quiet = quiet
        self.bridge = CvBridge()
        self.frames = 0
        self.last_print = 0.0
        self.last_frame_t = 0.0
        self.hz = 0.0
        self.latest_dets = []     # [(class_name, cx, cy, w, h), ...]
        self.det_stamp = 0.0
        qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Image, DEPTH_TOPIC, self.on_depth, qos)
        self.det_sub = self.create_subscription(
            DetectionArray, DET_TOPIC, self.on_detections, qos)
        self.vis_pub = None
        if view or publish:
            self.vis_pub = self.create_publisher(Image, VIS_TOPIC, 5)
        if not quiet:
            self.get_logger().info(
                f'等待 {DEPTH_TOPIC} ... '
                f'(没数据请检查深度相机是否在 SDF 里 / 桥是否在跑)')
            if publish:
                self.get_logger().info(
                    f'彩色深度图发布中: {VIS_TOPIC} '
                    f'(ros2 run rqt_image_view rqt_image_view {VIS_TOPIC})')

    def on_detections(self, msg):
        """记下最新一帧 YOLO 检测框(像素坐标), 深度帧到了再逐个算距离。"""
        dets = []
        for det in msg.detections:
            name = getattr(det, 'class_name', '') or str(getattr(det, 'class_id', '?'))
            b = det.bbox
            dets.append((name, b.center.position.x, b.center.position.y,
                         b.size.x, b.size.y))
        self.latest_dets = dets
        self.det_stamp = time.time()

    def _det_dists(self, depth, mask):
        """每个检测框一个距离: 框内的障碍像素取中位数(避开框里的地面/背景)。"""
        if time.time() - self.det_stamp > DET_TIMEOUT:
            return []
        h, w = depth.shape[:2]
        out = []
        for name, cx, cy, bw, bh in self.latest_dets:
            x0 = max(int(cx - bw / 2.0), 0)
            x1 = min(int(cx + bw / 2.0), w - 1)
            y0 = max(int(cy - bh / 2.0), 0)
            y1 = min(int(cy + bh / 2.0), h - 1)
            if x1 <= x0 or y1 <= y0:
                continue
            vals = depth[y0:y1, x0:x1][mask[y0:y1, x0:x1]]
            vals = vals[np.isfinite(vals) & (vals > 0.0)]
            dist = float(np.median(vals)) if vals.size else math.inf
            out.append((x0, y0, x1, y1, name, dist))
        return out

    def on_depth(self, msg):
        try:
            depth = np.asarray(self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1'),
                               dtype=np.float32)
        except Exception as e:
            self.get_logger().warn(f'深度图解码失败: {e}')
            return
        if depth.ndim != 2 or depth.size == 0:
            return
        h, w = depth.shape[:2]
        mask = obstacle_mask(depth)
        xl, xr = int(w * LEFT_PERCENT), int(w * RIGHT_PERCENT)
        zones = (
            zone_dist(depth[:, :xl], mask[:, :xl]),
            zone_dist(depth[:, xl:xr], mask[:, xl:xr]),
            zone_dist(depth[:, xr:], mask[:, xr:]),
        )
        self.frames += 1
        now = time.time()
        if self.last_frame_t > 0:
            dt = now - self.last_frame_t
            if dt > 0:
                hz = 1.0 / dt
                self.hz = hz if self.hz == 0.0 else (self.hz * 0.9 + hz * 0.1)
        self.last_frame_t = now

        dets = self._det_dists(depth, mask)
        if not self.quiet and now - self.last_print >= 1.0:
            self.last_print = now
            valid = float(np.mean(np.isfinite(depth) & (depth > 0.0))) * 100.0
            objects = ' | '.join(
                f'{name} {d:.2f}m' if math.isfinite(d) else f'{name} --'
                for _, _, _, _, name, d in dets)
            self.get_logger().info(
                f'{w}x{h} {self.hz:.1f}Hz 有效像素 {valid:.0f}%  '
                f'最近障碍 {fmt(min(zones))} | 左 {fmt(zones[0])} '
                f'中 {fmt(zones[1])} 右 {fmt(zones[2])}'
                + (f' | 物体: {objects}' if objects else ''))

        if self.view or self.publish:
            vis = self.colorize(depth, mask, zones, xl, xr, dets)
            if self.vis_pub is not None:
                self.vis_pub.publish(self.bridge.cv2_to_imgmsg(vis, encoding='bgr8'))
            if self.view:
                cv2.imshow('depth_camera (closer = red)', vis)
                cv2.waitKey(1)

    def colorize(self, depth, mask, zones, xl, xr, dets=()):
        """热力图: 黑=无回波/地面, 蓝->红 = 近; 绿线是分区界; 有检测就画框标距离。"""
        d = np.where(np.isfinite(depth) & (depth > 0.0), depth, VIEW_MAX_RANGE)
        d = np.clip(d, 0.0, VIEW_MAX_RANGE)
        norm = ((VIEW_MAX_RANGE - d) / VIEW_MAX_RANGE * 255.0).astype(np.uint8)
        vis = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
        vis[~np.isfinite(depth)] = (0, 0, 0)                 # 无回波 -> 黑
        vis[~mask] = (vis[~mask] * 0.25).astype(np.uint8)    # 地面/远背景 -> 压暗
        cv2.line(vis, (xl, 0), (xl, vis.shape[0] - 1), (0, 255, 0), 1)
        cv2.line(vis, (xr, 0), (xr, vis.shape[0] - 1), (0, 255, 0), 1)
        for i, (label, z) in enumerate(zip(('L', 'C', 'R'), zones)):
            x = {0: 4, 1: vis.shape[1] // 2 - 24, 2: vis.shape[1] - 64}[i]
            text = f'{label} {z:.2f}m' if math.isfinite(z) else f'{label} --'
            cv2.putText(vis, text, (x, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 2, cv2.LINE_AA)
        # 每个检测框一行标签: 类别 + 它自己的距离
        for x0, y0, x1, y1, name, dist in dets:
            cv2.rectangle(vis, (x0, y0), (x1, y1), (255, 255, 255), 1)
            text = f'{name} {dist:.2f}m' if math.isfinite(dist) else f'{name} --'
            ty = y0 - 6 if y0 > 20 else y1 + 16
            cv2.putText(vis, text, (x0, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(vis, text, (x0, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (255, 255, 0), 1, cv2.LINE_AA)
        return vis


def main():
    ap = argparse.ArgumentParser(description='深度相机测距探针')
    ap.add_argument('--view', action='store_true', help='开窗口可视化深度热力图')
    ap.add_argument('--publish', action='store_true',
                    help=f'把彩色深度图发布到 {VIS_TOPIC} (不开窗, 给 rqt_image_view/RViz2 看)')
    ap.add_argument('--quiet', action='store_true', help='不打印任何信息(只安静发布)')
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = DepthProbe(view=args.view, publish=args.publish, quiet=args.quiet)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if args.view:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()