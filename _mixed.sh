#!/bin/bash
# 混合障碍场景回归: 启动仿真 -> 桥 -> 等定位稳定 -> 导航到 (0,8)
# 卡住 45 s 由导航自己认输 (退出码 2); 外层 90 s 硬超时兜底。
cd /root/gz_ros
source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH="/root/gz_ros/mod:/root/gz_ros/worlds/robots:${GZ_SIM_RESOURCE_PATH:-}"
pkill -9 -f 'gz sim -r' 2>/dev/null
pkill -9 -f parameter_bridge 2>/dev/null
pkill -9 -f navigate_to_pose 2>/dev/null
sleep 3
setsid nohup gz sim -r -s /root/gz_ros/worlds/scenes/mixed.sdf > /tmp/gz_mixed.log 2>&1 &
sleep 10
setsid nohup bash start_bridges_headless.sh > /tmp/br_mixed.log 2>&1 &
ok=0
for i in $(seq 1 60); do
  if timeout 3 ros2 topic echo /joint_states --once >/dev/null 2>&1; then
    ok=$((ok+1)); [ $ok -ge 2 ] && { echo "[mixed] joint ready after $i"; break; }
  else
    ok=0
  fi
  sleep 2
done
sleep 5
timeout ${NAV_T:-90} bash run.sh 0 8 -p latency_report_every:=200 > /tmp/nav_mixed.log 2>&1
echo "[mixed] NAV_EXIT=$?"
grep -E 'Прибыл|Застрял|Таймаут|一次性测量' /tmp/nav_mixed.log | tail -4
echo ALL_DONE
