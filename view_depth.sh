#!/bin/bash

# 加载 ROS 2 Jazzy 环境 (yolo_msgs 在 yolo 工作区里)
source /opt/ros/jazzy/setup.bash
if [ -f "$HOME/gz_ros/yolo_ros2_ws/install/setup.bash" ]; then
    source "$HOME/gz_ros/yolo_ros2_ws/install/setup.bash"
fi

echo "========================================="
echo "  深度相机可视化 (伪彩色 + 检测框/距离 + 左右中分区)"
echo "========================================="

NODE="$HOME/gz_ros/depth_view.py"
if [ ! -f "$NODE" ]; then
    echo "✗ 找不到 $NODE"
    exit 1
fi

cleanup() {
    kill $NODE_PID 2>/dev/null
    exit
}
trap cleanup INT TERM

# 参数原样透传, 例如:
#   bash view_depth.sh --ros-args -p scale:=3 -p save_dir:=$HOME/gz_ros/depth_shots
python3 "$NODE" "$@" &
NODE_PID=$!
sleep 2

if ros2 pkg executables rqt_image_view 2>/dev/null | grep -q 'rqt_image_view rqt_image_view'; then
    echo "打开 rqt_image_view —— 在窗口左上角下拉框里把话题选成 /depth_view"
    echo "(rqt_image_view 不在 PATH 里, 必须用 ros2 run 启动)"
    echo "(关掉这个窗口, 本脚本也会退出)"
    ros2 run rqt_image_view rqt_image_view
    cleanup
else
    echo "!! 没找到 rqt_image_view"
    echo "   Ubuntu 24.04 安装: sudo apt install ros-jazzy-rqt-image-view"
    echo "   或者用 RViz2: Add -> Image -> Topic 选 /depth_view"
    echo "   可视化节点仍在运行, Ctrl+C 退出..."
    wait $NODE_PID
fi
