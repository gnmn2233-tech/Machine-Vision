#!/bin/bash

# 加载 ROS 2 Jazzy 环境
source /opt/ros/jazzy/setup.bash

echo "========================================="
echo "  相机图像查看 - 使用 RViz2"
echo "========================================="

# 检查 rviz2 包是否已安装
if ! ros2 pkg list | grep -q rviz2; then
    echo "✗ RViz2 未安装，请先执行："
    echo "  sudo apt install ros-jazzy-rviz2"
    exit 1
fi

# 启动 RViz2
ros2 run rviz2 rviz2