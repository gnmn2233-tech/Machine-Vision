#!/bin/bash
bash /root/gz_ros/up.sh ${WORLD:-/root/gz_ros/worlds/scenes/corridor.sdf}
source /opt/ros/jazzy/setup.bash
cd /root/gz_ros
export TAG=${TAG:-c2}
# 等桥真正通了再开始: gz 侧的传感器是 lazy 的, 桥进程起来了不代表 gz 已经在发,
# 而导航节点一旦启动就只在等 /joint_states, 等不到就整个不进入控制循环 (c6 就这么空跑 180 s)。
for t in /clock /scan /joint_states; do
  ok=0
  for i in $(seq 1 30); do
    if timeout 3 ros2 topic echo "$t" --once >/dev/null 2>&1; then ok=1; break; fi
    sleep 1
  done
  echo "[ready] $t ok=$ok"
done
LLM_ON=${LLM_ON:-0} timeout 180 bash run.sh ${GX:-0} ${GY:-8} > /tmp/nav_$TAG.log 2>&1
echo "EXIT=$? LOG=/tmp/nav_$TAG.log"
