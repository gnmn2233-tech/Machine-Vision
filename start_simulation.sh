#!/bin/bash

echo "========================================="
echo "  Симуляция камеры Gazebo Harmonic"
echo "========================================="

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 加载 ROS 2 Jazzy 环境 (脚本自己 source, 不依赖外部)
source /opt/ros/jazzy/setup.bash

export GZ_SIM_RESOURCE_PATH="$ROOT/mod:$GZ_SIM_RESOURCE_PATH"

echo "Очистка..."
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "ros_gz_bridge" 2>/dev/null
pkill -9 -f "keyboard_joint_controller.py" 2>/dev/null
# 清 VLM 残留节点
pkill -9 -f "vlm_camera_rotator" 2>/dev/null
# 清避障残留节点 (避免多发布者抢关节话题)
pkill -9 -f "obstacle_avoider" 2>/dev/null
# 清 yolo 残留 (避免重复实例)
pkill -9 -f "yolo_ros2" 2>/dev/null

echo "Запуск Gazebo..."
gz sim -r "$ROOT/worlds/robot_with_camera.sdf" &
GAZEBO_PID=$!

if ! kill -0 $GAZEBO_PID 2>/dev/null; then
    echo "✗ Gazebo не стартовал"
    exit 1
fi
echo "✓ Gazebo запущен (PID: $GAZEBO_PID)"

echo "Запуск контроллера шарниров..."
python3 "$ROOT/nodes/keyboard_joint_controller.py" &
CONTROLLER_PID=$!

# ---- 同名桥接：图像和信息 ----
# 相机图像（同名话题：Gazebo /camera → ROS /camera）
ros2 run ros_gz_bridge parameter_bridge \
  /camera@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args -r __node:=bridge_image &

# 相机信息（同名话题：Gazebo /camera/camera_info → ROS /camera/camera_info）
ros2 run ros_gz_bridge parameter_bridge \
  /camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
  --ros-args -r __node:=bridge_info &

# ---- 深度相机：机器人"看得见距离" ----
# 深度图（Gazebo /depth_camera → ROS /depth_camera）：32FC1 浮点, 单位米, 未命中为 inf
ros2 run ros_gz_bridge parameter_bridge \
  '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image' \
  --ros-args -r __node:=bridge_depth &

# 深度内参（Gazebo 固定把内参发在 /camera_info，这里 remap 到 /depth_camera/camera_info）
ros2 run ros_gz_bridge parameter_bridge \
  '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo' \
  --ros-args -r __node:=bridge_depth_info -r '/camera_info:=/depth_camera/camera_info' &

# 深度点云（可选：约 30 万点/帧，需要做三维重建时再取消注释）
# ros2 run ros_gz_bridge parameter_bridge \
#   '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked' \
#   --ros-args -r __node:=bridge_depth_points &

# 关节状态（Gazebo /joint_state → ROS /joint_states，用于避障同步真实关节角）
# 必须 remap 成复数话题 /joint_states（键盘/避障订阅的是它）
ros2 run ros_gz_bridge parameter_bridge \
  '/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model' \
  --ros-args -r __node:=bridge_joint_state -r '/joint_state:=/joint_states' &

# 关节命令（ROS → Gazebo，控制相机旋转）
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
echo "  ✅ Симуляция готова!"
echo "========================================="
echo "Изображение (ROS): /camera"
echo "Параметры камеры : /camera/camera_info"
echo "Глубина          : /depth_camera (32FC1, м, inf=нет данных)"
echo "Параметры глубины: /depth_camera/camera_info"
echo "Команды шарниров : /camera_joint_controller/commands"
echo "Привод шасси     : /drive_x/commands /drive_y/commands (Float64)"
echo ""
echo "Изображение:"
echo "  ros2 run rqt_image_view rqt_image_view /camera"
echo ""
echo "Глубина / дистанция:"
echo "  RViz2: Add -> Image -> /depth_camera/colorized   # цветная карта глубины, публикует view_camera.sh"
echo "  bash view_camera.sh                               # RViz2 + цветная карта глубины"
echo "  ros2 run obstacle_avoider depth_probe             # дистанции слева/по центру/справа"
echo "  ros2 topic echo /nearest_obstacle                 # ближайшее препятствие (м), inf = свободно"
echo ""
echo "Поворот шарниров вручную:"
echo "  gz topic -t /camera_joint_controller/commands -m gz.msgs.Double -p \"data: 1.0\""
echo "========================================="

cleanup() {
    echo "Стоп: все процессы..."
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
