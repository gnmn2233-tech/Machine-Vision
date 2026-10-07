#!/usr/bin/env python3
"""机器人 / 相机位置感知节点。

数据来源 (均为 Gazebo 真值):
  /joint_states  sensor_msgs/JointState   drive_x_joint / drive_y_joint / base_yaw_joint
  /pose_info     tf2_msgs/TFMessage       PosePublisher 输出的连杆/传感器世界位姿 (校验用)

运动学 (与 robot_with_camera.sdf 严格一致):
  world --drive_x_joint(prismatic +x)--> chassis_x
        --drive_y_joint(prismatic +y)--> chassis_y
        --base_yaw_joint(revolute +z)--> base --camera_joint(fixed z=0.175)--> cylinder(相机)
  相机传感器在 cylinder 上的位姿: x=+0.1, z=+0.28
  => 机器人 base 世界位姿 = (x, y, 0.075, yaw)
  => 相机   世界位姿 = base + R(yaw)*(0.1, 0, 0.455)   (高度恒为 0.53 m)
  说明: 朝向在 base_yaw_joint 上, 模型根连杆 chassis_x 只平移不旋转,
        所以不用 gz 的 OdometryPublisher (它发布模型根位姿, yaw 恒为 0)。

发布:
  /robot_pose   geometry_msgs/PoseStamped  frame=odom       机器人位姿
  /camera_pose  geometry_msgs/PoseStamped  frame=odom       相机光心位姿
  /odom         nav_msgs/Odometry                           标准里程计接口 (yaw 已修正)
  /robot_path   nav_msgs/Path                               轨迹 (RViz 画线)
  TF: odom -> base_link -> camera_link -> camera_optical_frame
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

# ---- 几何常量 (单位 m)。用 Gazebo 的 /pose_info 实测校准, 不是照抄 SDF 数字 ----
# 实测运动学链: robot_with_camera -> base      (0, 0, 0.075)
#               robot_with_camera -> cylinder  (0, 0, 0)      <- camera_joint 的 z=0.175 被
#                                                                cylinder 自身的 <pose>0 0 0</pose> 覆盖
#               cylinder -> camera             (0.1, 0, 0.28)
# 所以相机在模型坐标系的 z = 0.28 m; 相对 base 连杆的 z = 0.28 - 0.075 = 0.205 m
BASE_Z = 0.075                                   # base 连杆相对模型原点的高度
CAM_X = 0.1                                      # 相机在模型/连杆系里的 x 偏移
CAM_Z_MODEL = 0.28                               # 相机在模型原点上方的真实高度
CAM_Z_IN_BASE = CAM_Z_MODEL - BASE_Z             # 0.205
CAM_Z_WORLD = CAM_Z_MODEL                        # 0.28

ODOM_FRAME = 'odom'
BASE_FRAME = 'base_link'
CAMERA_FRAME = 'camera_link'
OPTICAL_FRAME = 'camera_optical_frame'

JOINT_X = 'drive_x_joint'
JOINT_Y = 'drive_y_joint'
JOINT_YAW = 'base_yaw_joint'

PATH_MIN_STEP = 0.01                             # 轨迹点最小间距 (m)
PATH_MAX_POSES = 5000


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class AngleUnwrapper:
    """把周期角展开成连续角: 连续转多圈也不跳变。"""

    def __init__(self):
        self.last_raw = None
        self.turns = 0.0
        self.value = 0.0

    def update(self, raw):
        if self.last_raw is None:
            self.value = raw
        else:
            d = raw - self.last_raw
            if d > math.pi:
                self.turns -= 2.0 * math.pi
            elif d < -math.pi:
                self.turns += 2.0 * math.pi
            self.value = self.turns + raw
        self.last_raw = raw
        return self.value


class RobotPoseMonitor(Node):
    def __init__(self):
        super().__init__('robot_pose_monitor')

        self.x = 0.0
        self.y = 0.0
        self.yaw = AngleUnwrapper()
        self.got_joints = False
        self._last_log = 0.0

        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.camera_pub = self.create_publisher(PoseStamped, '/camera_pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.path_pub = self.create_publisher(Path, '/robot_path', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.path = Path()
        self.path.header.frame_id = ODOM_FRAME

        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 10)
        self.create_timer(0.05, self.on_timer)       # 20 Hz

        self.get_logger().info(
            '位置感知节点已启动: /joint_states -> /robot_pose /camera_pose /odom /robot_path + TF')

    # ---------------- 关节状态 (真值) ----------------
    def on_joint_state(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if JOINT_X in pos:
            self.x = float(pos[JOINT_X])
        if JOINT_Y in pos:
            self.y = float(pos[JOINT_Y])
        if JOINT_YAW in pos:
            self.yaw.update(float(pos[JOINT_YAW]))
        if not self.got_joints:
            self.got_joints = True
            self.get_logger().info(f'已收到关节状态, 可用关节: {sorted(pos.keys())}')

    # ---------------- 周期发布 ----------------
    def on_timer(self):
        if not self.got_joints:
            return

        yaw = self.yaw.value
        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quat(yaw)

        cam_x = self.x + CAM_X * math.cos(yaw)
        cam_y = self.y + CAM_X * math.sin(yaw)

        # ---- /robot_pose ----
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = ODOM_FRAME
        ps.pose.position.x = self.x
        ps.pose.position.y = self.y
        ps.pose.position.z = BASE_Z
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self.pose_pub.publish(ps)

        # ---- /camera_pose ----
        cs = PoseStamped()
        cs.header.stamp = now
        cs.header.frame_id = ODOM_FRAME
        cs.pose.position.x = cam_x
        cs.pose.position.y = cam_y
        cs.pose.position.z = CAM_Z_WORLD
        cs.pose.orientation.x = qx
        cs.pose.orientation.y = qy
        cs.pose.orientation.z = qz
        cs.pose.orientation.w = qw
        self.camera_pub.publish(cs)

        # ---- /odom ----
        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = ODOM_FRAME
        od.child_frame_id = BASE_FRAME
        od.pose.pose = ps.pose
        od.pose.covariance[0] = 1e-4
        od.pose.covariance[7] = 1e-4
        od.pose.covariance[35] = 1e-4
        self.odom_pub.publish(od)

        # ---- TF: odom -> base_link -> camera_link -> camera_optical_frame ----
        tf_base = TransformStamped()
        tf_base.header.stamp = now
        tf_base.header.frame_id = ODOM_FRAME
        tf_base.child_frame_id = BASE_FRAME
        tf_base.transform.translation.x = self.x
        tf_base.transform.translation.y = self.y
        tf_base.transform.translation.z = BASE_Z
        tf_base.transform.rotation.x = qx
        tf_base.transform.rotation.y = qy
        tf_base.transform.rotation.z = qz
        tf_base.transform.rotation.w = qw

        tf_cam = TransformStamped()
        tf_cam.header.stamp = now
        tf_cam.header.frame_id = BASE_FRAME
        tf_cam.child_frame_id = CAMERA_FRAME
        tf_cam.transform.translation.x = CAM_X
        tf_cam.transform.translation.y = 0.0
        tf_cam.transform.translation.z = CAM_Z_IN_BASE
        tf_cam.transform.rotation.w = 1.0

        tf_opt = TransformStamped()
        tf_opt.header.stamp = now
        tf_opt.header.frame_id = CAMERA_FRAME
        tf_opt.child_frame_id = OPTICAL_FRAME
        ox, oy, oz, ow = rpy_to_quat(-math.pi / 2.0, 0.0, -math.pi / 2.0)
        tf_opt.transform.rotation.x = ox
        tf_opt.transform.rotation.y = oy
        tf_opt.transform.rotation.z = oz
        tf_opt.transform.rotation.w = ow

        self.tf_broadcaster.sendTransform([tf_base, tf_cam, tf_opt])

        # ---- /robot_path ----
        if (not self.path.poses) or math.hypot(
                self.path.poses[-1].pose.position.x - self.x,
                self.path.poses[-1].pose.position.y - self.y) > PATH_MIN_STEP:
            self.path.header.stamp = now
            self.path.poses.append(ps)
            if len(self.path.poses) > PATH_MAX_POSES:
                self.path.poses = self.path.poses[-PATH_MAX_POSES:]

        # ---- 1 Hz: 重复发布轨迹(后启动的 RViz 也能收到) + 日志 ----
        t = self.get_clock().now().nanoseconds / 1e9
        if t - self._last_log >= 1.0:
            self._last_log = t
            if self.path.poses:
                self.path_pub.publish(self.path)
            self.get_logger().info(
                f'x={self.x:+.2f} y={self.y:+.2f} yaw={math.degrees(yaw):+.1f} deg'
                f' | 相机({cam_x:+.2f}, {cam_y:+.2f}, {CAM_Z_WORLD:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = RobotPoseMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
