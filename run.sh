#!/bin/bash
# ============================================
#  机器人总启动: 导航(内嵌避障) / 独立避障节点
# ============================================
# 用法:
#   bash run.sh                      交互模式, 之后每行输入 "x y [角度deg]" 出发
#   bash run.sh 2.0 -1.0             直接走到 (2, -1), 朝向不变
#   bash run.sh 2.0 -1.0 --yaw 90    走到 (2, -1) 并转到 90 度
#   bash run.sh 1 0 2 1 0 0          多航点, 依次走 (1,0) -> (2,1) -> (0,0)
#   bash run.sh -p v_max:=0.8 3 -3   改速度上限 (m/s)
#   bash run.sh 0 8 -p latency_report_every:=200
#                                    顺手采一次响应时间统计 (ТЗ 4.2.2): 采满 200 个连续
#                                    周期打印一次分位数, 随即自动停止采样。默认 0 = 不测
#                                    (一次 180 s 行程打 75 行会把日志淹掉, 实测)
#   bash run.sh 0 8 -p stall_abort:=30
#                                    卡死早停: 30 s 挪不动 0.25 m 就打印 ✗ 并以退出码 2
#                                    结束 (默认 45 s, 0 = 关掉, 一直试)
#   bash run.sh avoid                老的独立避障节点 (obstacle_avoider)
#   bash run.sh stop                 一键关掉仿真 + 桥 + 节点 (原 stop.sh)
#
# 导航模式下避障是**内嵌**的: nodes/navigate_to_pose.py 里的 AvoidPlanner,
# 离目标 > 0.30 m 时全程生效, 进入 0.30 m 交给终点停靠。
# ⚠ 两个模式互斥: 都会发布 /camera_joint_controller 和底盘话题, 同时开会互相抢。
#
# 运动方式和键盘控制一致: 先原地转向对准目标方向, 再沿车头前进, 不横移。
# 终端里也能随时输入 stop 取消; 或用话题:
#   ros2 topic pub --once /nav_cancel std_msgs/msg/Bool "{data: true}"
#
# 前提: 另一个终端已经 bash start_simulation.sh (仿真 + 话题桥)
# 不要 set -u: /opt/ros/jazzy/setup.bash 内部引用未定义变量会直接退出
set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---- 停止: 原来单独的 stop.sh, 并进来了 (脚本太多, 一个入口够用) -----------
if [ "${1:-}" = stop ] || [ "${1:-}" = kill ] || [ "${1:-}" = down ]; then
    echo "==> 关闭导航与避障节点"
    pkill -9 -f navigate_to_pose.py 2>/dev/null
    pkill -9 -f obstacle_avoider 2>/dev/null
    echo "==> 关闭话题桥"
    pkill -9 -f ros_gz_bridge 2>/dev/null
    pkill -9 -f parameter_bridge 2>/dev/null
    echo "==> 关闭 YOLO 与 RViz"
    pkill -9 -f yolo 2>/dev/null
    pkill -9 -f rviz2 2>/dev/null
    echo "==> 关闭 Gazebo"
    pkill -f "gz sim" 2>/dev/null
    sleep 2
    pkill -9 -f "gz sim" 2>/dev/null
    pkill -9 -f "gz-sim" 2>/dev/null
    sleep 1
    left=$(pgrep -af "gz sim|gz-sim|ros_gz_bridge|navigate_to_pose|obstacle_avoider|rviz2" | cut -c1-70)
    if [ -z "$left" ]; then echo "    (无残留, 已全部关闭)"; else echo "$left"; fi
    exit 0
fi

source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH="$ROOT/mod:${GZ_SIM_RESOURCE_PATH:-}"

# ---- 模式: 默认导航, 第一个参数是 avoid/old/avoider 时走老避障节点 ----------
MODE=nav
case "${1:-}" in
    avoid|old|avoider) MODE=avoid; shift ;;
    stop|kill|down)  MODE=stop; shift ;;
esac

if ! pgrep -f "gz sim" >/dev/null 2>&1; then
    echo "✗ Gazebo не запущен. В другом терминале: bash start_simulation.sh"
    exit 1
fi

# == 桥保活: 缺哪个补哪个 (按 ROS 侧话题是否存在检查) ==
ensure_bridge() {
    # $1=ROS 侧话题(检查用)  $2..=桥参数 (含 remap); 话题已存在则跳过
    local topic="$1"; shift
    if timeout 5 ros2 topic list 2>/dev/null | grep -qx "$topic"; then
        return 0
    fi
    local name="bridge_${topic#/}"
    name="${name//\//_}_$$"          # 唯一节点名, 避免与已有桥同名被杀
    echo "Мост: $topic ($*) -> узел $name"
    ros2 run ros_gz_bridge parameter_bridge "$@" --ros-args -r "__node:=$name" &
}

ensure_bridge /joint_states '/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model' -r '/joint_state:=/joint_states'
ensure_bridge /camera_joint_controller/commands '/camera_joint_controller/commands@std_msgs/msg/Float64]gz.msgs.Double'
# 深度图桥: 避障靠它拿距离; 断了会退回「只转头」的纯 YOLO 规则
ensure_bridge /depth_camera '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image'
if [ "$MODE" = nav ]; then
    ensure_bridge /camera '/camera@sensor_msgs/msg/Image@gz.msgs.Image'
    ensure_bridge /drive_x/commands '/drive_x/commands@std_msgs/msg/Float64]gz.msgs.Double'
    ensure_bridge /drive_y/commands '/drive_y/commands@std_msgs/msg/Float64]gz.msgs.Double'
    sleep 2                        # 刚补起的桥要几帧才把话题挂上来
fi

# ---- LLM/VLM 顾问 ----------------------------------------------------------
# 避障本身是纯几何的, 不问模型也能开。这里配的是「几何自己拿不定主意时问一句」:
#   抖动(2 秒内换边两次) / 平局(左右一样宽) / 停滞(说在走却挪不动, 这条才带图)
# 分工是「模型定边, 几何定角」: 它只回 LEFT/RIGHT/STRAIGHT 改搜索顺序,
# 够不够宽、能不能开永远由深度图算 —— 模型说错话也撞不了。
# 没有 LM Studio 也能跑: 连不上就静默降级成纯几何, 不拖慢控制环。
export LLM_ON=${LLM_ON:-1}
export LLM_PROVIDER=${LLM_PROVIDER:-failover}
# 文本端: 回得最快, 热后 ~0.07-0.3 秒
export LLM_LOCAL_URL=${LLM_LOCAL_URL:-http://127.0.0.1:1234/v1}
export LLM_LOCAL_KEY=${LLM_LOCAL_KEY:-lm-studio}
# 和视觉端用**同一个**模型: LM Studio 一次只留一个模型在显存里, 文本端用 7B、
# 带图用 9B 的话, 每问一次图就把 9B 加载进来, 下一次纯文本又换回 7B —— 实测
# 第一次带图请求 20 s 超时, 紧接着的纯文本 8 s 也超时, 两边全灭。
# 9B 纯文本热后 0.26-0.68 s, 带图 + JSON 约束 0.85-0.91 s, 换掉 7B 不亏。
export LLM_LOCAL_MODEL=${LLM_LOCAL_MODEL:-qwen/qwen3.5-9b}
export LLM_LOCAL_TIMEOUT=${LLM_LOCAL_TIMEOUT:-8}
# 视觉端: 所有触发都喂一张**标注过**的画面 (方位刻度/车头十字/目标箭头/净空数字),
# 依据 Prompt-in-Image (arXiv 2508.01678, Qwen2.5-VL 受益) + Grid-Augmented Vision
# 注意 9B 会先思考: llm_hint 只对这类模型带 reasoning_effort=none (实测 46 秒 -> 0.3 秒)
export LLM_VISION=${LLM_VISION:-1}
# front = 只喂前视相机图; combo = 前视标注图 + 雷达俯视扇区图并排 (672x252).
# 为什么: 前视图只有 60 度, 「左右哪边更空」这个信息根本不在画面里, 实测 9B 对 4 组
# 截然不同的图一律回 LEFT; 补上俯视雷达图后 4 个场景判对 3 个。多花约 0.6 s。
export LLM_VISION_MODE=${LLM_VISION_MODE:-front}
export LLM_VLM_URL=${LLM_VLM_URL:-http://127.0.0.1:1234/v1}
export LLM_VLM_KEY=${LLM_VLM_KEY:-lm-studio}
export LLM_VLM_MODEL=${LLM_VLM_MODEL:-qwen/qwen3.5-9b}
# 60 s 是留给「模型被换出显存」后的重新加载: 热态正常 0.85-0.91 s, 冷加载几十秒
# 只发生一次, 而卡在 20 s 等于第一次永远失败 (实测踩过)
export LLM_VLM_TIMEOUT=${LLM_VLM_TIMEOUT:-60}
export LLM_MIN_INTERVAL=${LLM_MIN_INTERVAL:-2.5}
export LLM_TTL=${LLM_TTL:-4.0}
# 云端兜底: 本地异常/超时才切过去, 填了 key 才启用
export LLM_CLOUD_URL=${LLM_CLOUD_URL:-https://api.deepseek.com/v1}
export LLM_CLOUD_KEY=${LLM_CLOUD_KEY:-}
export LLM_CLOUD_MODEL=${LLM_CLOUD_MODEL:-deepseek-chat}
export LLM_CLOUD_TIMEOUT=${LLM_CLOUD_TIMEOUT:-30}

# ---- 启动 ------------------------------------------------------------------
# 清残留: 两个节点抢同一批话题, 留着一定会打架
pkill -9 -f "navigate_to_pose.py" 2>/dev/null
pkill -9 -f "obstacle_avoider/lib/obstacle_avoider/obstacle_avoider" 2>/dev/null   # 只杀节点本身, 别误杀 depth_probe(彩色深度图发布)

if [ "$MODE" = avoid ]; then
    echo "========================================="
    echo "  Объезд препятствий (obstacle_avoider)"
    echo "========================================="
    source "$ROOT/yolo_ros2_ws/install/setup.bash" 2>/dev/null
    source "$ROOT/obstacle_ws/install/setup.bash"
    exec ros2 run obstacle_avoider obstacle_avoider
fi

exec python3 -u "$ROOT/nodes/navigate_to_pose.py" "$@"
