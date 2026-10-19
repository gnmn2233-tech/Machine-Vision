#!/bin/bash

# 相机可视化: 起 RViz2, 里面同时看彩色相机和深度相机
#   彩色图       /camera
#   深度彩色图   /depth_camera/colorized  (在 RViz2 里 Add -> Image, Topic 填它)
# 深度彩色图由本脚本后台的 depth_probe --publish 安静发布, 不弹窗、不打日志。
# 静态 TF: RViz2 的 Image display 需要 TF, 把两个相机 frame 挂到 map 下(只求可查, 不用于对齐)。

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
source "$ROOT/obstacle_ws/install/setup.bash" 2>/dev/null

echo "========================================="
echo "  Камера - RViz2"
echo "========================================="

# 检查 rviz2 包是否已安装
if ! ros2 pkg list | grep -q rviz2; then
    echo "✗ RViz2 не установлен, выполните:"
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

if ! kill -0 $DEPTH_PUB_PID 2>/dev/null; then
    echo "⚠️  Публикатор глубины не стартовал, соберите и повторите:"
    echo "    cd "$ROOT/obstacle_ws" && colcon build --packages-select obstacle_avoider"
else
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
        echo "✓ Глубина публикуется: /depth_camera/colorized"
    else
        echo "⚠️  Публикатор жив, топика /depth_camera/colorized нет (8 с)"
        echo "    Перезапустите all.sh; иначе ищите топик вручную в RViz2 (переоткройте Add)"
    fi
    if ! echo "$GRAPH" | grep -q 'DEPTH=1'; then
        echo "⚠️  /depth_camera тоже нет - мост глубины не поднялся, карта пустая"
        echo "    Проверка: bash start_simulation.sh, все 4 моста должны быть запущены"
    fi
fi

# 启动 RViz2 (robot_pose.rviz: Fixed Frame=odom + Path(/robot_path) + Odometry + TF)
ros2 run rviz2 rviz2 -d "$ROOT/config/robot_pose.rviz"
