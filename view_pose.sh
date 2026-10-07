#!/bin/bash

# 加载 ROS 2 Jazzy 环境
source /opt/ros/jazzy/setup.bash

echo "========================================="
echo "  机器人/相机位置感知 - RViz2 可视化"
echo "========================================="

if ! ros2 pkg list | grep -q rviz2; then
    echo "✗ RViz2 未安装，请先执行："
    echo "  sudo apt install ros-jazzy-rviz2"
    exit 1
fi

# 先用 /robot_pose_monitor.py 发布 /robot_pose /camera_pose /odom /robot_path + TF
rviz2 -d "$HOME/gz_ros/robot_pose.rviz"
