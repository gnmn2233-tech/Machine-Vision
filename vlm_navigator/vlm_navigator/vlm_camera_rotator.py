import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import base64
import asyncio
import aiohttp
import re
import math


class SmoothYaw:
    """无极丝滑转向器 (与 keyboard_joint_controller.py / obstacle_avoider.py 保持一致)。

    - 角度永不回卷: 目标角无限累积, revolute 关节可连续转任意圈, 跨 ±π 零突跳。
    - 发布角以 限速+限加速度 的梯形速度规划逼近目标: 恒速转动、远端减速缓停, 无阶跃。
    """

    def __init__(self, max_rate=1.0, accel=5.0, dt=0.02):
        self.target = 0.0
        self.pos = 0.0
        self.vel = 0.0
        self.max_rate = max_rate
        self.accel = accel
        self.dt = dt

    def step_delta(self, delta):
        self.target += delta

    def set_target(self, t):
        self.target = t

    def tick(self):
        err = self.target - self.pos
        if abs(err) < 1e-9:
            self.vel = 0.0
            return self.pos
        direction = 1.0 if err > 0.0 else -1.0
        dist = abs(err)
        stop_dist = self.vel * self.vel / (2.0 * self.accel)
        if dist <= stop_dist:
            v_des = math.sqrt(max(2.0 * self.accel * dist, 0.0))
        else:
            v_des = self.max_rate
        if direction * self.vel >= 0.0:
            self.vel += direction * self.accel * self.dt
        else:
            self.vel -= direction * 2.0 * self.accel * self.dt
        if abs(self.vel) > v_des:
            self.vel = direction * v_des
        self.pos += self.vel * self.dt
        return self.pos


class VLMCameraRotator(Node):
    def __init__(self):
        super().__init__('vlm_camera_rotator')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/yolo/dbg_image', self.image_cb, 10)
        self.cmd_pub = self.create_publisher(Float64, '/camera_joint_controller/commands', 10)
        # 接管兼容: 键盘/避障说话时本节点静默, 不抢命令话题
        self.ov_sub = self.create_subscription(Float64, '/avoider_override', self.on_override, 10)
        self._overridden = False

        self.loop = asyncio.get_event_loop()
        self.processing = False
        self.action_pattern = re.compile(r'\b(left|right|stop|forward)\b', re.IGNORECASE)
        # 无极丝滑转向: 角度无限累积不 wrap, 发布角限速逼近目标
        self.smooth = SmoothYaw(max_rate=0.9, accel=4.0, dt=0.02)

        # 定时器 50Hz: 仅在"未被接管 且 有未完成的转动"时发布 (空闲完全静默)
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info("VLM Camera Rotator 节点已启动（无极丝滑转向 · override 兼容）")

    def on_override(self, msg):
        ov = msg.data > 0.5
        if ov and not self._overridden:
            self._overridden = True
            self.get_logger().info('键盘/避障接管: VLM 转动暂停')
        elif not ov and self._overridden:
            self._overridden = False
            self.get_logger().info('接管释放: VLM 转动恢复')

    def timer_callback(self):
        """定时器回调: 未被接管且目标未达成时才推进并发布; 空闲静默, 绝不抢话题。"""
        if self._overridden:
            return
        if abs(self.smooth.target - self.smooth.pos) < 1e-9:
            return
        msg = Float64()
        msg.data = self.smooth.tick()
        self.cmd_pub.publish(msg)

    def image_cb(self, msg):
        if self.processing:
            return
        self.processing = True

        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        _, buffer = cv2.imencode('.jpg', cv_img)
        b64_str = base64.b64encode(buffer).decode('utf-8')

        asyncio.run_coroutine_threadsafe(self.invoke_llava(b64_str), self.loop)

    async def invoke_llava(self, b64_image):
        prompt = (
            "You are a camera rotation controller for a robot. "
            "The image shows YOLO detection boxes of objects. "
            "If an obstacle appears on the LEFT side, turn the camera RIGHT. "
            "If an obstacle appears on the RIGHT side, turn LEFT. "
            "If the path is clear, reply 'forward'. "
            "If you are unsure or too close to an obstacle, reply 'stop'. "
            "Reply with ONLY ONE WORD: left, right, forward, or stop."
        )
        payload = {
            "model": "llava:7b",
            "prompt": prompt,
            "images": [b64_image],
            "stream": False
        }
        try:
            async with aiohttp.ClientSession() as session:
                async with session.post('http://localhost:11434/api/generate', json=payload,
                                        timeout=aiohttp.ClientTimeout(total=15)) as resp:
                    data = await resp.json()
                    response_text = data.get('response', '')
                    action = self.extract_action(response_text)
                    self.get_logger().info(f"LLaVA: '{response_text}' -> 动作: {action}")
                    self.publish_joint_command(action)
        except Exception as e:
            self.get_logger().error(f"调用Ollama时出错: {e}")
            self.publish_joint_command('stop')
        finally:
            self.processing = False

    def extract_action(self, text):
        match = self.action_pattern.search(text.lower())
        return match.group(1) if match else 'stop'

    def publish_joint_command(self, action):
        """只更新目标角 (无极累加), 实际发布交给 50Hz 定时器平滑输出"""
        rotation_step = 0.3

        if action == 'left':
            self.smooth.step_delta(rotation_step)
        elif action == 'right':
            self.smooth.step_delta(-rotation_step)
        # forward 或 stop 保持当前目标角

def main(args=None):
    rclpy.init(args=args)
    node = VLMCameraRotator()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
