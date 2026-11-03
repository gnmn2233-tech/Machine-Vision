#!/bin/bash

# all.sh - 一键启动
#   1) start_simulation.sh   仿真 + 键盘 + 全部话题桥(含深度相机)
#   2) yolo.sh               YOLO 检测
#   3) view_camera.sh        RViz2 + 静默发布 /depth_camera/colorized + 静态 TF
# Ctrl+C 一次性收干净

cd "$(dirname "${BASH_SOURCE[0]}")" || exit 1
ROOT="$PWD"

echo "========================================="
echo "  Инициализация"
echo "========================================="

source /opt/ros/jazzy/setup.bash
source "$ROOT/obstacle_ws/install/setup.bash" 2>/dev/null
source "$ROOT/yolo_ros2_ws/install/setup.bash" 2>/dev/null
export GZ_SIM_RESOURCE_PATH="$ROOT/mod:$GZ_SIM_RESOURCE_PATH"

# 清掉上次运行留下的所有残留进程
echo "Очистка..."
for pat in '[g]z sim' '[g]z-[s]erver' '[g]zclient' '[r]uby.*gz.sim' \
           '[p]arameter_bridge' '[r]viz2' '[d]epth_probe' \
           '[s]tatic_transform_publisher' '[k]eyboard_joint_controller' \
           '[o]bstacle_avoider' '[y]olo_ros2' '[v]lm_camera_rotator' \
           '[r]obot_pose_monitor'; do
    pkill -9 -f "$pat" 2>/dev/null
done
# gz 的旧会话日志/锁(不动 gui.config 等用户配置)
rm -f /tmp/gz-sim-*.log /tmp/gz-*.lock 2>/dev/null

# 基本检查
if ! command -v ros2 >/dev/null 2>&1; then
    echo "✗ Нет ros2 (ROS 2 Jazzy)"
    exit 1
fi
if [ ! -f "$ROOT/worlds/robot_with_camera.sdf" ]; then
    echo "✗ Нет $ROOT/worlds/robot_with_camera.sdf"
    exit 1
fi
# obstacle_avoider: 没编译过 或 源码比 install 新 -> 自动编译
PKG_SRC="$ROOT/obstacle_ws/src/obstacle_avoider/obstacle_avoider"
pkg_install_dir() {
    ls -d "$ROOT/obstacle_ws/install/obstacle_avoider/lib/python3"*/site-packages/obstacle_avoider 2>/dev/null | head -1
}
build_pkg() {
    echo "Сборка obstacle_avoider..."
    if (cd "$ROOT/obstacle_ws" && colcon build --packages-select obstacle_avoider) \
            > /tmp/all_sh_build.log 2>&1; then
        echo "✓ Собрано"
        source "$ROOT/obstacle_ws/install/setup.bash" 2>/dev/null
    else
        echo "✗ Ошибка сборки, лог (20):"
        tail -20 /tmp/all_sh_build.log
        exit 1
    fi
}
PKG_INST=$(pkg_install_dir)
if [ -z "$PKG_INST" ]; then
    echo "obstacle_ws не собран, собираю..."
    build_pkg
    PKG_INST=$(pkg_install_dir)
elif [ -n "$(find "$PKG_SRC" -name '*.py' -newer "$PKG_INST" -print -quit 2>/dev/null)" ]; then
    echo "Исходники новее, пересобираю..."
    build_pkg
fi
echo "✓ Готово, остатки очищены"

# 显示检查: Gazebo/RViz 需要可用的 X server; WSLg 未运行时 Qt 直接 abort, 用户看不到任何窗口
display_ok() {
    [ -n "$DISPLAY" ] || return 1
    command -v xdpyinfo >/dev/null 2>&1 || return 0
    xdpyinfo -display "$DISPLAY" >/dev/null 2>&1
}
if display_ok; then
    echo "✓ Дисплей: $DISPLAY"
else
    echo "⚠️  Дисплей недоступен (DISPLAY=$DISPLAY) — окна Gazebo и RViz не откроются."
    echo "    Симуляция пойдёт без окон. Исправление: в PowerShell выполните"
    echo "    'wsl --shutdown', подождите 10 с и откройте WSL заново."
fi
echo ""

# ---- 选择仿真场景 ----
SCENES_DIR="$ROOT/worlds/scenes"
SCENE_LIST=$(ls "$SCENES_DIR"/*.sdf 2>/dev/null | xargs -r -n1 basename | sed 's/\.sdf$//')
if [ -n "$SCENE" ] && [ -f "$SCENES_DIR/$SCENE.sdf" ]; then
    export WORLD_FILE="$SCENES_DIR/$SCENE.sdf"
elif [ -t 0 ] && [ -n "$SCENE_LIST" ]; then
    echo "Доступные сцены (worlds/scenes):"
    i=1
    for s in $SCENE_LIST; do printf "  %d) %s\n" "$i" "$s"; i=$((i+1)); done
    printf "Выберите сцену (номер или имя, Enter = person): "
    read -r pick
    pick="${pick:-person}"
    if [ -f "$SCENES_DIR/$pick.sdf" ]; then
        export WORLD_FILE="$SCENES_DIR/$pick.sdf"
    else
        sel=$(echo "$SCENE_LIST" | sed -n "${pick}p")
        export WORLD_FILE="$SCENES_DIR/${sel:-person}.sdf"
    fi
else
    export WORLD_FILE="$SCENES_DIR/person.sdf"
fi
echo "Сцена: $(basename "$WORLD_FILE" .sdf)"
echo ""

# ---- 依次启动四路 ----
echo "[1/4] Симуляция + клавиатура + мосты..."
bash "$ROOT/start_simulation.sh" &
SIM_PID=$!

echo "[2/4] YOLO..."
bash "$ROOT/yolo.sh" &
YOLO_PID=$!

echo "[3/4] RViz2 (камера + траектория)..."
bash "$ROOT/view_camera.sh" &
VIEW_PID=$!

echo "[4/4] Позиционирование (робот/камера + TF)..."
python3 "$ROOT/nodes/robot_pose_monitor.py" > /tmp/pose_monitor.log 2>&1 &
POSE_PID=$!
if kill -0 $POSE_PID 2>/dev/null; then
    echo "✓ Позиционирование: /robot_pose /camera_pose /odom /robot_path + TF"
    echo "  Траектория - в RViz2: Path(/robot_path), Fixed Frame=odom"
    echo "  Над роботом: x / y / yaw (/robot_label)"
else
    echo "⚠️  Позиционирование не стартовало, лог (5):"
    tail -5 /tmp/pose_monitor.log
fi

cleanup() {
    echo ""
    echo "Стоп: дочерние процессы..."
    kill $SIM_PID $YOLO_PID $VIEW_PID $POSE_PID 2>/dev/null
    for _ in 1 2; do
        for pat in '[d]epth_probe' '[s]tatic_transform_publisher' '[r]viz2' \
                   '[p]arameter_bridge' '[g]z sim' '[k]eyboard_joint_controller' \
                   '[o]bstacle_avoider' '[y]olo' '[v]lm_camera_rotator' \
                   '[r]obot_pose_monitor'; do
            pkill -9 -f "$pat" 2>/dev/null
        done
        sleep 1
    done
    exit 0
}
trap cleanup SIGINT SIGTERM

wait
