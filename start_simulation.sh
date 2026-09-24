#!/bin/bash

echo "========================================="
echo "  Gazebo Harmonic 相机仿真启动脚本"
echo "========================================="

# 加载 ROS 2 Jazzy 环境 (脚本自己 source, 任何终端直接跑, 不依赖外部已 source)
source /opt/ros/jazzy/setup.bash

export GZ_SIM_RESOURCE_PATH="$HOME/gz_ros/mod:$GZ_SIM_RESOURCE_PATH"

echo "正在清理旧进程..."
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "ros_gz_bridge" 2>/dev/null
pkill -9 -f "keyboard_joint_controller.py" 2>/dev/null
# 清 VLM 残留节点
pkill -9 -f "vlm_camera_rotator" 2>/dev/null
# 清避障残留节点 (避免多发布者抢关节话题)
pkill -9 -f "obstacle_avoider" 2>/dev/null
# 清 yolo 残留 (检测节点不碰关节, 但避免重复实例)
pkill -9 -f "yolo_ros2" 2>/dev/null
sleep 1

echo "启动 Gazebo..."
gz sim -r "$HOME/gz_ros/robot_with_camera.sdf" &
GAZEBO_PID=$!

echo "等待 Gazebo 启动..."
sleep 5

if ! kill -0 $GAZEBO_PID 2>/dev/null; then
    echo "✗ Gazebo 启动失败"
    exit 1
fi
echo "✓ Gazebo 启动成功 (PID: $GAZEBO_PID)"

echo "启动键盘关节控制器..."
python3 "$HOME/gz_ros/keyboard_joint_controller.py" &
CONTROLLER_PID=$!

# ---- 同名桥接：图像和信息 ----
# 相机图像（同名话题：Gazebo /camera → ROS /camera）
ros2 run ros_gz_bridge parameter_bridge \
  /camera@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args -r __node:=bridge_image &

# 相机信息（同名话题：Gazebo /camera_info → ROS /camera_info）
ros2 run ros_gz_bridge parameter_bridge \
  /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
  --ros-args -r __node:=bridge_info &

# 关节状态（Gazebo /joint_state → ROS /joint_states，用于避障同步真实关节角）
# 注意: 必须带 -r /joint_state:=/joint_states 重映射 —— 键盘/避障订阅的是复数话题 /joint_states
ros2 run ros_gz_bridge parameter_bridge \
  '/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model' \
  --ros-args -r __node:=bridge_joint_state -r '/joint_state:=/joint_states' &

# 关节命令（ROS → Gazebo，方向 ]，控制相机旋转）
ros2 run ros_gz_bridge parameter_bridge \
  /camera_joint_controller/commands@std_msgs/msg/Float64]gz.msgs.Double \
  --ros-args -r __node:=bridge_joint_command &

# 底盘驱动（ROS Float64 位置 → Gazebo prismatic 关节，控制前进/后退）
ros2 run ros_gz_bridge parameter_bridge \
  /drive_x/commands@std_msgs/msg/Float64]gz.msgs.Double \
  --ros-args -r __node:=bridge_drive_x &

ros2 run ros_gz_bridge parameter_bridge \
  /drive_y/commands@std_msgs/msg/Float64]gz.msgs.Double \
  --ros-args -r __node:=bridge_drive_y &

echo ""
echo "========================================="
echo "  ✅ 仿真启动完成！"
echo "========================================="
echo "图像话题 (ROS): /camera"
echo "相机信息话题 : /camera_info"
echo "关节命令话题 : /camera_joint_controller/commands"
echo "底盘驱动话题 : /drive_x/commands /drive_y/commands (Float64 位置)"
echo ""
echo "查看图像:"
echo "  ros2 run rqt_image_view rqt_image_view /camera"
echo ""
echo "手动转动关节测试:"
echo "  gz topic -t /camera_joint_controller/commands -m gz.msgs.Double -p \"data: 1.0\""
echo "========================================="

cleanup() {
    echo "正在停止所有进程..."
    kill $GAZEBO_PID 2>/dev/null
    kill $CONTROLLER_PID 2>/dev/null
    kill $BRIDGE_IMAGE_PID 2>/dev/null
    kill $BRIDGE_INFO_PID 2>/dev/null
    kill $BRIDGE_JOINT_PID 2>/dev/null
    pkill -9 -f "gz sim" 2>/dev/null
    pkill -9 -f "ros_gz_bridge" 2>/dev/null
    exit
}
trap cleanup SIGINT

wait