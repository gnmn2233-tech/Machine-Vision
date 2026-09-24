#!/bin/bash

# =========================================
#  1. 基础环境设置
# =========================================
WORKSPACE_DIR=~/gz_ros/yolo_ros2_ws

# 检查目录是否存在
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "✗ 错误：工作空间目录 $WORKSPACE_DIR 不存在！"
    exit 1
fi

cd "$WORKSPACE_DIR"

# =========================================
#  2. 激活虚拟环境与 ROS 2
# =========================================
echo "Activating virtual environment..."
# 确保虚拟环境路径正确
if [ -f ".venv/bin/activate" ]; then
    source .venv/bin/activate
else
    echo "✗ 错误：虚拟环境未找到 (.venv)"
    exit 1
fi

echo "Sourcing ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash

# =========================================
#  3. 刷新环境与启动
# =========================================
# 这一步至关重要：重新加载 install 目录，确保能立即找到刚才编译生成的包
echo "Sourcing workspace..."
source install/setup.bash

# 设置 Python 路径 (保留你原有的设置)
export PYTHONPATH="$WORKSPACE_DIR/.venv/lib/python3.12/site-packages:$PYTHONPATH"

# 设置参数
CAMERA_TOPIC="/camera"

echo "========================================="
echo "  启动 YOLO 节点..."
echo "  话题: $CAMERA_TOPIC"
echo "========================================="

# 启动命令
# 注意：这里使用了绝对路径指向 launch 文件
ros2 launch /root/gz_ros/yolo_ros2_ws/src/yolo_ros/yolo_bringup/launch/yolov8.launch.py \
  input_image_topic:=$CAMERA_TOPIC