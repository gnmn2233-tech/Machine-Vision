#!/usr/bin/env bash
# 一键关掉仿真及其所有配套进程（没窗口 / 窗口卡死时用）
#   bash stop.sh
echo "==> 关闭导航与避障节点"
pkill -9 -f navigate_to_pose.py 2>/dev/null
pkill -9 -f obstacle_avoider 2>/dev/null

echo "==> 关闭话题桥"
pkill -9 -f ros_gz_bridge 2>/dev/null
pkill -9 -f parameter_bridge 2>/dev/null

echo "==> 关闭 YOLO 与 RViz"
pkill -9 -f yolo 2>/dev/null
pkill -9 -f rviz2 2>/dev/null

echo "==> 关闭 Gazebo（先正常结束，2 秒后强杀残留）"
pkill -f "gz sim" 2>/dev/null
sleep 2
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "gz-sim" 2>/dev/null
sleep 1

echo "==> 剩余相关进程:"
left=$(pgrep -af "gz sim|gz-sim|ros_gz_bridge|navigate_to_pose|obstacle_avoider|rviz2" | cut -c1-70)
if [ -z "$left" ]; then echo "    (无，已全部关闭)"; else echo "$left"; fi
