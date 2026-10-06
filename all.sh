#!/bin/bash

# all.sh - 
#   1) start_simulation.sh   仿真 + 键盘 + 全部话题桥(含深度相机)
#   2) yolo.sh               YOLO 检测
#   3) view_camera.sh        RViz2 + 静默发布 /depth_camera/colorized + 静态 TF
# Ctrl+C 

cd "$(dirname "$0")" || exit 1

echo "========================================="
echo "  初始化"
echo "========================================="

source /opt/ros/jazzy/setup.bash
source "$HOME/gz_ros/obstacle_ws/install/setup.bash" 2>/dev/null
source "$HOME/gz_ros/yolo_ros2_ws/install/setup.bash" 2>/dev/null
export GZ_SIM_RESOURCE_PATH="$HOME/gz_ros/mod:$GZ_SIM_RESOURCE_PATH"

# 清掉上次运行留下的所有残留进程
echo "正在清理旧进程..."
for pat in '[g]z sim' '[p]arameter_bridge' '[r]viz2' '[d]epth_probe' \
           '[s]tatic_transform_publisher' '[k]eyboard_joint_controller' \
           '[o]bstacle_avoider' '[y]olo_ros2' '[v]lm_camera_rotator'; do
    pkill -9 -f "$pat" 2>/dev/null
done

# 基本检查
if ! command -v ros2 >/dev/null 2>&1; then
    echo "✗ 找不到 ros2, 请确认 ROS 2 Jazzy 已安装"
    exit 1
fi
if [ ! -f "$HOME/gz_ros/robot_with_camera.sdf" ]; then
    echo "✗ 找不到 $HOME/gz_ros/robot_with_camera.sdf"
    exit 1
fi
# obstacle_avoider: 没编译过 或 源码比 install 新 -> 自动编译
PKG_SRC="$HOME/gz_ros/obstacle_ws/src/obstacle_avoider/obstacle_avoider"
pkg_install_dir() {
    ls -d "$HOME/gz_ros/obstacle_ws/install/obstacle_avoider/lib/python3"*/site-packages/obstacle_avoider 2>/dev/null | head -1
}
build_pkg() {
    echo "编译 obstacle_avoider ..."
    if (cd "$HOME/gz_ros/obstacle_ws" && colcon build --packages-select obstacle_avoider) \
            > /tmp/all_sh_build.log 2>&1; then
        echo "✓ 编译完成"
        source "$HOME/gz_ros/obstacle_ws/install/setup.bash" 2>/dev/null
    else
        echo "✗ 编译失败, 最后 20 行日志:"
        tail -20 /tmp/all_sh_build.log
        exit 1
    fi
}
PKG_INST=$(pkg_install_dir)
if [ -z "$PKG_INST" ]; then
    echo "obstacle_ws 还没编译过, 自动编译..."
    build_pkg
    PKG_INST=$(pkg_install_dir)
elif [ -n "$(find "$PKG_SRC" -name '*.py' -newer "$PKG_INST" -print -quit 2>/dev/null)" ]; then
    echo "检测到源码比已安装的版本新, 自动重编译..."
    build_pkg
fi
echo "✓ 环境就绪, 残留已清理"
echo ""

# ---- 依次启动三路 ----
echo "[1/3] 仿真 + 键盘 + 话题桥..."
bash start_simulation.sh &
SIM_PID=$!
sleep 3

echo "[2/3] YOLO 检测..."
bash yolo.sh &
YOLO_PID=$!
sleep 3

echo "[3/3] RViz2 相机可视化..."
bash view_camera.sh &
VIEW_PID=$!

cleanup() {
    echo ""
    echo "收尾: 停止所有子进程..."
    kill $SIM_PID $YOLO_PID $VIEW_PID 2>/dev/null
    for _ in 1 2; do
        for pat in '[d]epth_probe' '[s]tatic_transform_publisher' '[r]viz2' \
                   '[p]arameter_bridge' '[g]z sim' '[k]eyboard_joint_controller' \
                   '[o]bstacle_avoider' '[y]olo' '[v]lm_camera_rotator'; do
            pkill -9 -f "$pat" 2>/dev/null
        done
        sleep 1
    done
    exit 0
}
trap cleanup SIGINT SIGTERM

wait