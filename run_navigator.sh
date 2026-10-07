#!/bin/bash

# 加载 ROS 2 Jazzy 环境
source /opt/ros/jazzy/setup.bash

echo "========================================="
echo "  A->B 导航节点 (go-to-goal)"
echo "========================================="

if ! ros2 node list 2>/dev/null | grep -q robot_pose_monitor; then
    echo "!! 没检测到 /robot_pose_monitor 节点"
    echo "   导航需要位姿, 请先在另一个终端运行:"
    echo "     python3 $HOME/gz_ros/robot_pose_monitor.py"
    echo ""
fi

if ! ros2 topic list 2>/dev/null | grep -q '/depth_camera$'; then
    echo "!! 没检测到深度图话题 /depth_camera"
    echo "   深度急停会失效(仍可纯导航), 请确认 start_simulation.sh 已运行"
    echo "   (旧版 rgbd 相机的话加 -p depth_topic:=/depth_camera/depth_image)"
    echo ""
fi

if ! ros2 service list 2>/dev/null | grep -q "/world/default/create"; then
    echo "!! 没检测到 /world/default/create 服务"
    echo "   Gazebo 世界里不会出现目标球(导航本身不受影响),"
    echo "   需要重启 start_simulation.sh 以启用实体服务桥接"
    echo ""
fi

echo "发目标: RViz 里用 '2D Goal Pose' 点一下, 或:"
echo "  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\"
echo "    '{header: {frame_id: odom}, pose: {position: {x: 3.0, y: 2.0}, orientation: {w: 1.0}}}'"
echo ""

# 可通过命令行覆盖参数, 例如:
#   bash run_navigator.sh --ros-args -p max_speed:=0.3 -p stop_dist:=0.6
NODE="$HOME/gz_ros/goal_navigator.py"
if [ ! -f "$NODE" ]; then
    echo "✗ 找不到节点文件: $NODE"
    exit 1
fi

python3 "$NODE" "$@"
