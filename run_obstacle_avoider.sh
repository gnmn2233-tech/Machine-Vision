#!/bin/bash

echo "========================================="
echo "  启动避障节点 (obstacle_avoider)"
echo "========================================="

# 加载环境
source /opt/ros/jazzy/setup.bash
source ~/gz_ros/yolo_ros2_ws/install/setup.bash
source ~/gz_ros/obstacle_ws/install/setup.bash

# 键盘控制器共存: 平时静默、按方向键自动接管避障、松开交还 —— 不需要杀它

# == 桥保活: 确保关键桥在跑 ==
# 按"ROS 侧话题是否存在"检查
ensure_bridge() {
    # $1=ROS 侧话题(检查用)  $2..=桥参数 (含 remap); 话题已存在则跳过
    local topic="$1"; shift
    if timeout 5 ros2 topic list 2>/dev/null | grep -qx "$topic"; then
        return 0
    fi
    local name="bridge_${topic#/}"
    name="${name//\//_}_$$"          # 唯一节点名, 避免与已有桥同名被杀
    echo "补起桥: $topic ($*) -> 节点 $name"
    ros2 run ros_gz_bridge parameter_bridge "$@" --ros-args -r "__node:=$name" &
    sleep 3
}
ensure_bridge /camera_joint_controller/commands '/camera_joint_controller/commands@std_msgs/msg/Float64]gz.msgs.Double'
ensure_bridge /joint_states '/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model' -r '/joint_state:=/joint_states'
# 深度图桥: 节点靠它拿距离; 断了也不致命(会退回 YOLO 规则), 但最好保活
ensure_bridge /depth_camera '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image'

# LLM 后端: 本地优先 + 云端兜底 (failover)
#  主: 本地 LM Studio qwen2.5-7b-instruct-1m (GPU 推理 ~0.5s, 零成本)
#  备: 云端 OpenAI 兼容 API —— 本地异常/超时才自动切过去; 配置 key 即启用
export LLM_PROVIDER=failover
export LLM_LOCAL_URL=http://127.0.0.1:1234/v1
export LLM_LOCAL_KEY=lm-studio              # LM Studio 不校验 key, 占位即可
export LLM_LOCAL_MODEL=qwen2.5-7b-instruct-1m
export LLM_LOCAL_TIMEOUT=30
export LLM_CLOUD_URL=https://api.deepseek.com/v1
export LLM_CLOUD_KEY=                       
export LLM_CLOUD_MODEL=deepseek-chat
export LLM_CLOUD_TIMEOUT=30

# 运行节点
ros2 run obstacle_avoider obstacle_avoider