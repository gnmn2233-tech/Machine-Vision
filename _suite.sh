#!/bin/bash
# 场景回归: 每个场景单独起一次 gz, 跑到 (0,8); 卡住 45 s 由导航自己认输 (退出码 2),
# 这里再套一层 90 s 硬超时兜底。用法: bash _suite.sh  (改目标: TX=0 TY=6 bash _suite.sh)
GX=${TX:-0}
GY=${TY:-8}
NAV_T=${NAV_T:-90}
run_case() {
  local world=$1 tag=$2 gx=$3 gy=$4
  cd /root/gz_ros
  source /opt/ros/jazzy/setup.bash
  export GZ_SIM_RESOURCE_PATH="/root/gz_ros/mod:/root/gz_ros/worlds/robots:${GZ_SIM_RESOURCE_PATH:-}"
  pkill -9 -f 'gz sim -r' 2>/dev/null; pkill -9 -f parameter_bridge 2>/dev/null; pkill -9 -f navigate_to_pose 2>/dev/null
  sleep 3
  setsid nohup gz sim -r -s "$world" > /tmp/gz_$tag.log 2>&1 &
  sleep 10
  setsid nohup bash start_bridges_headless.sh > /tmp/br_$tag.log 2>&1 &
  # 连续 2 次取到关节数据才发车, 否则导航的超时会被"等定位"白吃掉 (实测遇到过 147 s)
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
  timeout $NAV_T bash run.sh $gx $gy > /tmp/nav_$tag.log 2>&1
  echo "[$tag] NAV_EXIT=$?"
  grep -E 'Прибыл|Застрял|Таймаут' /tmp/nav_$tag.log | tail -2
}
run_case /root/gz_ros/worlds/scenes/person.sdf person $GX $GY
run_case /root/gz_ros/worlds/scenes/low_box.sdf lowbox $GX $GY
run_case /root/gz_ros/worlds/scenes/symmetric.sdf sym $GX $GY
echo ALL_DONE
