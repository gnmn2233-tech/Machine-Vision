#!/bin/bash
# ============================================================
#  场景回归: 一次跑完 (或只跑指定的几个)
#    bash _test.sh                  # 四个场景全跑: person lowbox sym mixed
#    bash _test.sh mixed person     # 只跑这两个
#    TX=0 TY=6 bash _test.sh        # 目标改近一点 (默认 0 8)
#  判定: 导航自己卡住 45 s 就打印 ✗ 并以退出码 2 退出;
#        这里再套 90 s 硬超时兜底, 所以每个场景最多等 90 s。
#  日志: /tmp/nav_<tag>.log   (gz: /tmp/gz_<tag>.log, 桥: /tmp/br_<tag>.log)
# ============================================================
GX=${TX:-0}
GY=${TY:-8}
NAV_T=${NAV_T:-90}
cd "$(dirname "${BASH_SOURCE[0]}")" || exit 1
ROOT=$(pwd)
source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH="$ROOT/mod:$ROOT/worlds/robots:${GZ_SIM_RESOURCE_PATH:-}"

world_of() {
  case "$1" in
    person)  echo "$ROOT/worlds/scenes/person.sdf" ;;
    lowbox)  echo "$ROOT/worlds/scenes/low_box.sdf" ;;
    sym)     echo "$ROOT/worlds/scenes/symmetric.sdf" ;;
    mixed)   echo "$ROOT/worlds/scenes/mixed.sdf" ;;
    *)       echo "" ;;
  esac
}

# 等位姿真正可用才发车: 只取到一次就发车, 导航的 90 s 会被"等 /joint_states"白吃掉 (实测 147 s)
wait_pose() {
  local tag=$1 ok=0
  for i in $(seq 1 60); do
    if timeout 3 ros2 topic echo /joint_states --once >/dev/null 2>&1; then
      ok=$((ok+1)); [ $ok -ge 2 ] && { echo "[$tag] joint ready after $i"; return 0; }
    else
      ok=0
    fi
    sleep 2
  done
  echo "[$tag] 等不到 /joint_states, 跳过"
  return 1
}

run_case() {
  local tag=$1 world extra=""
  world=$(world_of "$tag")
  [ -z "$world" ] && { echo "未知场景: $tag (可选 person/lowbox/sym/mixed)"; return; }
  [ "$tag" = mixed ] && extra="-p latency_report_every:=200"
  pkill -9 -f 'gz sim -r' 2>/dev/null
  pkill -9 -f parameter_bridge 2>/dev/null
  pkill -9 -f navigate_to_pose 2>/dev/null
  sleep 3
  setsid nohup gz sim -r -s "$world" > /tmp/gz_$tag.log 2>&1 &
  sleep 10
  setsid nohup bash "$ROOT/start_bridges_headless.sh" > /tmp/br_$tag.log 2>&1 &
  wait_pose "$tag" || return
  sleep 5
  timeout $NAV_T bash "$ROOT/run.sh" $GX $GY $extra > /tmp/nav_$tag.log 2>&1
  local rc=$?
  echo "[$tag] NAV_EXIT=$rc"
  grep -E 'Прибыл|Застрял|Таймаут' /tmp/nav_$tag.log | tail -1
}

for tag in "${@:-person lowbox sym mixed}"; do
  run_case "$tag"
done
echo ALL_DONE
