#!/bin/bash
# ============================================
#  目标点导航: 输入坐标 -> 机器人移动到坐标处
# ============================================
# 用法:
#   bash run_navigate.sh                      交互模式, 之后每行输入 "x y [角度deg]" 出发
#   bash run_navigate.sh 2.0 -1.0             直接走到 (2, -1), 朝向不变
#   bash run_navigate.sh 2.0 -1.0 --yaw 90    走到 (2, -1) 并转到 90 度
#   bash run_navigate.sh 1 0 2 1 0 0          多航点, 依次走 (1,0) -> (2,1) -> (0,0)
#   bash run_navigate.sh -p v_max:=0.8 3 -3   改速度上限 (m/s)
#
# 运动方式和键盘控制一致: 先原地转向对准目标方向, 再沿车头前进, 不横移。
# 终端里也能随时输入 stop 取消; 或用话题:
#   ros2 topic pub --once /nav_cancel std_msgs/msg/Bool "{data: true}"
#
# 前提: 另一个终端已经 bash start_simulation.sh (仿真 + 话题桥)
# 不要 set -u: /opt/ros/jazzy/setup.bash 内部引用未定义变量会直接退出
set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH="$ROOT/mod:${GZ_SIM_RESOURCE_PATH:-}"

if ! pgrep -f "gz sim" >/dev/null 2>&1; then
    echo "✗ 没检测到 Gazebo 仿真进程。请先另开终端运行: bash start_simulation.sh"
    exit 1
fi
if ! pgrep -f "ros_gz_bridge" >/dev/null 2>&1; then
    echo "✗ 没检测到 ros_gz_bridge。请先另开终端运行: bash start_simulation.sh"
    exit 1
fi

# 清残留导航节点, 避免多个发布者抢关节话题
pkill -9 -f "navigate_to_pose.py" 2>/dev/null
sleep 0.5

exec python3 -u "$ROOT/nodes/navigate_to_pose.py" "$@"
