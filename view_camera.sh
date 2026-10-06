#!/bin/bash

# 相机可视化: 起 RViz2, 里面同时看彩色相机和深度相机
#   彩色图       /camera
#   深度彩色图   /depth_camera/colorized  (在 RViz2 里 Add -> Image, Topic 填它)
# 深度彩色图由本脚本后台的 depth_probe --publish 安静发布, 不弹窗、不打日志。
# 静态 TF: RViz2 的 Image display 需要 TF, 把两个相机 frame 挂到 map 下(只求可查, 不用于对齐)。

source /opt/ros/jazzy/setup.bash
source ~/gz_ros/obstacle_ws/install/setup.bash 2>/dev/null

echo "========================================="
echo "  相机图像查看 - 使用 RViz2"
echo "========================================="

# 检查 rviz2 包是否已安装
if ! ros2 pkg list | grep -q rviz2; then
    echo "✗ RViz2 未安装，请先执行："
    echo "  sudo apt install ros-jazzy-rviz2"
    exit 1
fi

cleanup() {
    kill $DEPTH_PUB_PID $TF_CAM_PID $TF_DEPTH_PID 2>/dev/null
    pkill -9 -f '[d]epth_probe' 2>/dev/null
    pkill -9 -f '[s]tatic_transform_publisher' 2>/dev/null
    exit
}
trap cleanup SIGINT SIGTERM

# 静态 TF: 让 RViz2 的 Image display 能找到相机 frame
ros2 run tf2_ros static_transform_publisher \
    --frame-id map --child-frame-id robot_with_camera/cylinder/camera \
    >/dev/null 2>&1 &
TF_CAM_PID=$!

ros2 run tf2_ros static_transform_publisher \
    --frame-id map --child-frame-id robot_with_camera/cylinder/depth_camera \
    >/dev/null 2>&1 &
TF_DEPTH_PID=$!

# 深度彩色图发布 (静默后台; RViz2 里 Add -> Image -> /depth_camera/colorized)
ros2 run obstacle_avoider depth_probe --publish --quiet &
DEPTH_PUB_PID=$!

sleep 2
if ! kill -0 $DEPTH_PUB_PID 2>/dev/null; then
    echo "⚠️  深度彩色图发布器没能起来, 先编译再重跑:"
    echo "    cd ~/gz_ros/obstacle_ws && colcon build --packages-select obstacle_avoider"
else
    # 用 rclpy 查 ROS 图(比 ros2 topic list 可靠), 最多等 8 秒(等桥把话题建出来):
    GRAPH=$(timeout 20 python3 - <<'PY' 2>/dev/null
import time, rclpy
rclpy.init()
n = rclpy.create_node('view_camera_check')
found, t0 = set(), time.time()
while time.time() - t0 < 8 and not {'/depth_camera', '/depth_camera/colorized'} <= found:
    found |= {x[0] for x in n.get_topic_names_and_types()}
    time.sleep(0.5)
print('COLORIZED=1' if '/depth_camera/colorized' in found else 'COLORIZED=0')
print('DEPTH=1' if '/depth_camera' in found else 'DEPTH=0')
n.destroy_node()
rclpy.shutdown()
PY
)
    if echo "$GRAPH" | grep -q 'COLORIZED=1'; then
        echo "✓ 深度彩色图已在发布: /depth_camera/colorized"
    else
        echo "⚠️  发布器进程在, 但图上没等到 /depth_camera/colorized (等了 8 秒)"
        echo "    先重跑一次 all.sh; 还不行就在 RViz2 里手动找它(Add 对话框关掉重开)"
    fi
    if ! echo "$GRAPH" | grep -q 'DEPTH=1'; then
        echo "⚠️  也没等到 /depth_camera 原始话题 —— 深度桥没起来, 彩色图会没画面"
        echo "    排查: 重跑 bash start_simulation.sh, 确认 4 个桥都启动"
    fi
fi

echo "RViz2 里看图:  Add -> Image -> Topic"
echo "  彩色相机     /camera"
echo "  深度(彩色)   /depth_camera/colorized   (越红越近, 黑=没回波)"
echo "  ※ Add 对话框是打开那一刻的快照, 后起的话题要关掉重新 Add 才看得到"
echo ""

# 启动 RViz2
ros2 run rviz2 rviz2