
import rclpy, cv2, time, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/snap.png'
rclpy.init()
n = Node('snap')
br = CvBridge()
got = {'img': None}
qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
def cb(m):
    if got['img'] is None:
        got['img'] = br.imgmsg_to_cv2(m, desired_encoding='bgr8')
n.create_subscription(Image, '/camera', cb, qos)
t0 = time.time()
while got['img'] is None and time.time()-t0 < 20:
    rclpy.spin_once(n, timeout_sec=0.3)
if got['img'] is not None:
    cv2.imwrite(out, got['img'])
    print('saved', out, got['img'].shape)
else:
    print('no image')
