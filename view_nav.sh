#!/bin/bash

# 加载 ROS 2 Jazzy 环境
source /opt/ros/jazzy/setup.bash

echo "========================================="
echo "  导航可视化 - RViz2 (位姿/轨迹/点云/目标点)"
echo "========================================="

if ! ros2 pkg list | grep -q rviz2; then
    echo "✗ RViz2 未安装，请先执行："
    echo "  sudo apt install ros-jazzy-rviz2"
    exit 1
fi

CONFIG="$HOME/gz_ros/robot_nav.rviz"
if [ ! -f "$CONFIG" ]; then
    echo "✗ 找不到配置文件: $CONFIG"
    exit 1
fi

rviz2 -d "$CONFIG"
