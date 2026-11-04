#!/bin/bash
# =============================================================================
#  simulation.sh —— 起仿真 (本项目唯一的仿真入口)
#
#    bash simulation.sh                带界面: 仿真 + 键盘 + 全部话题桥 + YOLO
#                                      + RViz + 定位, Ctrl+C 一次收干净
#    bash simulation.sh mixed          同上, 直接指定场景 (worlds/scenes/mixed.sdf)
#    bash simulation.sh headless       无界面: gz -s + 单进程桥, 给 run.sh 用
#    bash simulation.sh view           只起 RViz 相机视图 (彩色 + 深度彩色)
#    bash simulation.sh yolo           只起 YOLO 检测
#    bash simulation.sh test           场景回归: person lowbox sym mixed 全跑
#    bash simulation.sh test mixed     只跑混合场景
#    TX=0 TY=6 bash simulation.sh test 回归时把目标点改近
#
#  停止: bash run.sh stop   (导航 / 桥 / 仿真 / YOLO / RViz 一起关)
# =============================================================================

cd "$(dirname "${BASH_SOURCE[0]}")" || exit 1
ROOT="$PWD"

source /opt/ros/jazzy/setup.bash
source "$ROOT/obstacle_ws/install/setup.bash" 2>/dev/null
source "$ROOT/yolo_ros2_ws/install/setup.bash" 2>/dev/null
export GZ_SIM_RESOURCE_PATH="$ROOT/mod:$ROOT/worlds/robots:${GZ_SIM_RESOURCE_PATH:-}"

# ---------- 清残留 ----------
clean_all() {
    echo "Очистка..."
    for pat in '[g]z sim' '[g]z-[s]erver' '[g]zclient' '[r]uby.*gz.sim' \
               '[p]arameter_bridge' '[r]viz2' '[d]epth_probe' \
               '[s]tatic_transform_publisher' '[k]eyboard_joint_controller' \
               '[o]bstacle_avoider' '[y]olo_ros2' '[v]lm_camera_rotator' \
               '[r]obot_pose_monitor' '[n]avigate_to_pose'; do
        pkill -9 -f "$pat" 2>/dev/null
    done
    rm -f /tmp/gz-sim-*.log /tmp/gz-*.lock 2>/dev/null
}

# ---------- 墙面贴图: 缺失时自动下载 (CC0 砖墙 albedo, 供 worlds 里的 PBR 材质引用) ----------
# 贴图放 mod/wall_textures/ (已随 GZ_SIM_RESOURCE_PATH 导出), SDF 里写
# model://wall_textures/brick_diffuse.jpg 即可, 不写死在绝对路径, 换机器也能用。
ensure_wall_textures() {
    local f="$ROOT/mod/wall_textures/brick_diffuse.jpg"
    [ -s "$f" ] && return 0
    mkdir -p "$(dirname "$f")"
    echo "==> Загрузка текстуры кирпичной стены..."
    for url in         "https://cdn.jsdelivr.net/gh/mrdoob/three.js@dev/examples/textures/brick_diffuse.jpg"         "https://threejs.org/examples/textures/brick_diffuse.jpg"; do
        if curl -fsSL --connect-timeout 10 --max-time 90 -o "$f.part" "$url" 2>/dev/null && [ -s "$f.part" ]; then
            mv "$f.part" "$f"
            echo "✓ Текстура: $(du -h "$f" | cut -f1)"
            return 0
        fi
    done
    rm -f "$f.part"
    echo "⚠️  Текстура не скачана — стены будут серыми (положите файл в $f)"
    return 1
}

# ---------- 场景选择: $1 > 环境变量 SCENE > 交互 > person ----------
pick_scene() {
    local want="${1:-${SCENE:-}}"
    local dir="$ROOT/worlds/scenes"
    local list
    list=$(ls "$dir"/*.sdf 2>/dev/null | xargs -r -n1 basename | sed "s/\.sdf$//")
    if [ -z "$want" ] && [ -t 0 ] && [ -n "$list" ]; then
        echo "Доступные сцены (worlds/scenes):"
        local i=1
        for s in $list; do printf "  %d) %s\n" "$i" "$s"; i=$((i+1)); done
        printf "Выберите сцену (номер или имя, Enter = person): "
        read -r want
        want="${want:-person}"
        # 输入的是序号就翻译成名字
        if ! [ -f "$dir/$want.sdf" ]; then
            local sel; sel=$(echo "$list" | sed -n "${want}p")
            [ -n "$sel" ] && want="$sel"
        fi
    fi
    want="${want:-person}"
    if [ -f "$dir/$want.sdf" ]; then
        export WORLD_FILE="$dir/$want.sdf"
    elif [ -f "$want" ]; then
        export WORLD_FILE="$want"          # 直接给了 sdf 路径
    else
        echo "✗ 没有场景: $want (可选: $(echo $list | tr " " "/"))"
        exit 1
    fi
    echo "Сцена: $(basename "$WORLD_FILE" .sdf)"
}

check_display() {
    if [ -n "$DISPLAY" ] && { ! command -v xdpyinfo >/dev/null 2>&1 || xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; }; then
        echo "✓ Дисплей: $DISPLAY"
    else
        echo "⚠️  Дисплей недоступен (DISPLAY=$DISPLAY) — окна не откроются."
        echo "    Исправление: в PowerShell 'wsl --shutdown', подождать 10 с, снова открыть WSL."
    fi
}

# obstacle_avoider 没编译过 / 源码更新过 -> 自动编译 (view 与 GUI 模式要用它)
ensure_pkg() {
    local src="$ROOT/obstacle_ws/src/obstacle_avoider/obstacle_avoider"
    local inst
    inst=$(ls -d "$ROOT/obstacle_ws/install/obstacle_avoider/lib/python3"*/site-packages/obstacle_avoider 2>/dev/null | head -1)
    if [ -z "$inst" ] || [ -n "$(find "$src" -name "*.py" -newer "$inst" -print -quit 2>/dev/null)" ]; then
        echo "Сборка obstacle_avoider..."
        if (cd "$ROOT/obstacle_ws" && colcon build --packages-select obstacle_avoider) > /tmp/sim_build.log 2>&1; then
            echo "✓ Собрано"
            source "$ROOT/obstacle_ws/install/setup.bash" 2>/dev/null
        else
            echo "✗ Ошибка сборки, лог (20):"
            tail -20 /tmp/sim_build.log
            return 1
        fi
    fi
    return 0
}

# ---------- 桥: 带界面版 (9 个独立桥, 与旧 start_simulation.sh 一致) ----------
bridges_gui() {
    ros2 run ros_gz_bridge parameter_bridge \
      /camera@sensor_msgs/msg/Image@gz.msgs.Image \
      --ros-args -r __node:=bridge_image &
    ros2 run ros_gz_bridge parameter_bridge \
      /camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
      --ros-args -r __node:=bridge_info &
    ros2 run ros_gz_bridge parameter_bridge \
      '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image' \
      --ros-args -r __node:=bridge_depth &
    # 深度内参: Gazebo 固定把内参发在 /camera_info, 这里 remap 到 /depth_camera/camera_info
    ros2 run ros_gz_bridge parameter_bridge \
      '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo' \
      --ros-args -r __node:=bridge_depth_info -r '/camera_info:=/depth_camera/camera_info' &
    # 2D 雷达: 360 度单线, 深度相机只有 60 度视场, 避障的空档全靠它
    ros2 run ros_gz_bridge parameter_bridge \
      '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' \
      --ros-args -r __node:=bridge_scan &
    # 关节状态: Gazebo /joint_state -> ROS /joint_states (复数, 键盘/导航订阅的是它)
    ros2 run ros_gz_bridge parameter_bridge \
      '/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model' \
      --ros-args -r __node:=bridge_joint_state -r '/joint_state:=/joint_states' &
    ros2 run ros_gz_bridge parameter_bridge \
      /camera_joint_controller/commands@std_msgs/msg/Float64]gz.msgs.Double \
      --ros-args -r __node:=bridge_joint_command &
    # 底盘: ROS 位置指令 -> Gazebo prismatic 关节
    ros2 run ros_gz_bridge parameter_bridge \
      /drive_x/commands@std_msgs/msg/Float64]gz.msgs.Double \
      --ros-args -r __node:=bridge_drive_x &
    ros2 run ros_gz_bridge parameter_bridge \
      /drive_y/commands@std_msgs/msg/Float64]gz.msgs.Double \
      --ros-args -r __node:=bridge_drive_y &
}

# ---------- 桥: 无界面版 (单进程, 只用导航需要的话题) ----------
bridges_headless() {
    ros2 run ros_gz_bridge parameter_bridge \
      '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
      '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' \
      '/camera@sensor_msgs/msg/Image[gz.msgs.Image' \
      '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo' \
      '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image' \
      '/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model' \
      '/camera_joint_controller/commands@std_msgs/msg/Float64]gz.msgs.Double' \
      '/drive_x/commands@std_msgs/msg/Float64]gz.msgs.Double' \
      '/drive_y/commands@std_msgs/msg/Float64]gz.msgs.Double' \
      --ros-args -r '/joint_state:=/joint_states'
}

usage() {
    sed -n "2,15p" "$ROOT/simulation.sh" | sed "s/^# \{0,1\}//"
    exit 1
}

CMD="${1:-run}"
SCENE_ARG=""
case "$CMD" in
    run|gui) shift; SCENE_ARG="${1:-}" ;;
    headless|sim) shift; SCENE_ARG="${1:-}" ;;
    view|rviz) shift ;;
    yolo) shift ;;
    test) shift ;;
    -h|--help|help) usage ;;
    *) # 第一个参数不是子命令: 当场景名处理 (bash simulation.sh mixed)
       SCENE_ARG="$CMD"; CMD=run ;;
esac

# 墙贴图先备好 (幂等: 有文件就跳过), 只对真正起仿真的分支生效
case "$CMD" in run|gui|headless|sim|test) ensure_wall_textures ;; esac

# =========================== 带界面全量 ===========================
if [ "$CMD" = run ] || [ "$CMD" = gui ]; then
    echo "========================================="
    echo "  Симуляция + YOLO + RViz"
    echo "========================================="
    clean_all
    command -v ros2 >/dev/null 2>&1 || { echo "✗ Нет ros2 (ROS 2 Jazzy)"; exit 1; }
    pick_scene "$SCENE_ARG"
    check_display
    ensure_pkg || exit 1
    echo ""
    echo "[1/5] Gazebo (с окном)..."
    gz sim -r "$WORLD_FILE" &
    SIM_PID=$!
    sleep 5
    echo "[2/5] Клавиатура..."
    python3 "$ROOT/nodes/keyboard_joint_controller.py" &
    KB_PID=$!
    echo "[3/5] Мосты..."
    bridges_gui
    echo "[4/5] YOLO..."
    bash "$ROOT/simulation.sh" yolo &
    YOLO_PID=$!
    echo "[5/5] RViz + позиционирование..."
    bash "$ROOT/simulation.sh" view &
    VIEW_PID=$!
    python3 "$ROOT/nodes/robot_pose_monitor.py" > /tmp/pose_monitor.log 2>&1 &
    POSE_PID=$!
    echo ""
    echo "✅ Готово. Изображение: /camera | Глубина: /depth_camera"
    echo "   Навигация: bash run.sh 0 8     Стоп: bash run.sh stop"
    echo ""
    cleanup() {
        echo ""; echo "Стоп: все процессы..."
        kill $SIM_PID $KB_PID $YOLO_PID $VIEW_PID $POSE_PID 2>/dev/null
        clean_all
        exit 0
    }
    trap cleanup SIGINT SIGTERM
    wait
    exit 0
fi

# =========================== 无界面 ===========================
if [ "$CMD" = headless ] || [ "$CMD" = sim ]; then
    clean_all
    pick_scene "$SCENE_ARG"
    echo "Запуск Gazebo (без окна)..."
    setsid nohup gz sim -r -s "$WORLD_FILE" > /tmp/gz_headless.log 2>&1 &
    sleep 10
    echo "Запуск мостов..."
    setsid nohup bash -c 'source /opt/ros/jazzy/setup.bash; exec ros2 run ros_gz_bridge parameter_bridge \
      "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
      "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" \
      "/camera@sensor_msgs/msg/Image[gz.msgs.Image" \
      "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
      "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image" \
      "/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model" \
      "/camera_joint_controller/commands@std_msgs/msg/Float64]gz.msgs.Double" \
      "/drive_x/commands@std_msgs/msg/Float64]gz.msgs.Double" \
      "/drive_y/commands@std_msgs/msg/Float64]gz.msgs.Double" \
      --ros-args -r "/joint_state:=/joint_states"' > /tmp/bridges_headless.log 2>&1 &
    echo "Ждём позиционирование..."
    ok=0
    for i in $(seq 1 60); do
        if timeout 3 ros2 topic echo /joint_states --once >/dev/null 2>&1; then
            ok=$((ok+1)); [ $ok -ge 2 ] && { echo "✓ ready (после $i попыток)"; break; }
        else
            ok=0
        fi
        sleep 2
    done
    echo "Дальше: bash run.sh 0 8   (остановка: bash run.sh stop)"
    exit 0
fi

# =========================== 只起 RViz ===========================
if [ "$CMD" = view ] || [ "$CMD" = rviz ]; then
    ros2 pkg list 2>/dev/null | grep -q rviz2 || { echo "✗ RViz2 не установлен: sudo apt install ros-jazzy-rviz2"; exit 1; }
    ensure_pkg || exit 1
    ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id robot_with_camera/cylinder/camera >/dev/null 2>&1 &
    TF1=$!
    ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id robot_with_camera/cylinder/depth_camera >/dev/null 2>&1 &
    TF2=$!
    ros2 run obstacle_avoider depth_probe --publish --quiet &
    DP=$!
    cleanup_view() { kill $TF1 $TF2 $DP 2>/dev/null; pkill -9 -f '[d]epth_probe' 2>/dev/null; pkill -9 -f '[s]tatic_transform_publisher' 2>/dev/null; exit; }
    trap cleanup_view SIGINT SIGTERM
    exec ros2 run rviz2 rviz2 -d "$ROOT/config/robot_pose.rviz"
fi

# =========================== 只起 YOLO ===========================
if [ "$CMD" = yolo ]; then
    [ -d "$ROOT/yolo_ros2_ws" ] || { echo "✗ Нет $ROOT/yolo_ros2_ws"; exit 1; }
    cd "$ROOT/yolo_ros2_ws" || exit 1
    [ -f ".venv/bin/activate" ] || { echo "✗ Нет .venv в yolo_ros2_ws"; exit 1; }
    source .venv/bin/activate
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    export PYTHONPATH="$PWD/.venv/lib/python3.12/site-packages:$PYTHONPATH"
    exec ros2 launch yolo_bringup yolov8.launch.py input_image_topic:=/camera
fi

# =========================== 场景回归 ===========================
if [ "$CMD" = test ]; then
    GX=${TX:-0}; GY=${TY:-8}; NAV_T=${NAV_T:-90}
    world_of() {
        case "$1" in
            person) echo "$ROOT/worlds/scenes/person.sdf" ;;
            lowbox) echo "$ROOT/worlds/scenes/low_box.sdf" ;;
            sym)    echo "$ROOT/worlds/scenes/symmetric.sdf" ;;
            mixed)  echo "$ROOT/worlds/scenes/mixed.sdf" ;;
            *)      echo "" ;;
        esac
    }
    run_case() {
        local tag=$1 world extra=""
        world=$(world_of "$tag")
        [ -z "$world" ] && { echo "未知场景: $tag (可选 person/lowbox/sym/mixed)"; return; }
        [ "$tag" = mixed ] && extra="-p latency_report_every:=200"
        clean_all
        sleep 2
        setsid nohup gz sim -r -s "$world" > /tmp/gz_$tag.log 2>&1 &
        sleep 10
        # 直接起单进程桥 (不走 simulation.sh headless 的等待逻辑, 避免嵌套)
        setsid nohup bash -c 'source /opt/ros/jazzy/setup.bash; exec ros2 run ros_gz_bridge parameter_bridge \
          "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
          "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" \
          "/camera@sensor_msgs/msg/Image[gz.msgs.Image" \
          "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
          "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image" \
          "/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model" \
          "/camera_joint_controller/commands@std_msgs/msg/Float64]gz.msgs.Double" \
          "/drive_x/commands@std_msgs/msg/Float64]gz.msgs.Double" \
          "/drive_y/commands@std_msgs/msg/Float64]gz.msgs.Double" \
          --ros-args -r "/joint_state:=/joint_states"' > /tmp/br_$tag.log 2>&1 &
        local ok=0
        for i in $(seq 1 60); do
            if timeout 3 ros2 topic echo /joint_states --once >/dev/null 2>&1; then
                ok=$((ok+1)); [ $ok -ge 2 ] && { echo "[$tag] joint ready after $i"; break; }
            else
                ok=0
            fi
            sleep 2
        done
        sleep 5
        timeout $NAV_T bash "$ROOT/run.sh" $GX $GY $extra > /tmp/nav_$tag.log 2>&1
        echo "[$tag] NAV_EXIT=$?"
        grep -E "Прибыл|Застрял|Таймаут" /tmp/nav_$tag.log | tail -1
    }
    tags=("$@")
    [ ${#tags[@]} -eq 0 ] && tags=(person lowbox sym mixed)
    for tag in "${tags[@]}"; do run_case "$tag"; done
    echo ALL_DONE
    exit 0
fi

usage
