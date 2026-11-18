#!/bin/bash
# ============================================================
#  切换 light.sdf 里的信号灯 (运行时, 不用重启仿真)
#
#  场景里红绿两颗亮球各自盖住一颗固定黑球:
#    红灯位 z=1.17  绿灯位 z=0.73
#  要亮哪颗就把哪颗移到它自己的位置, 另一颗沉到地下 (z=-50)。
#  于是"红灯亮时绿灯是黑的"这件事由固定黑球保证, 不需要改材质。
#
#  GUI 里点按钮也能切: worlds/scenes/light.sdf 里挂了 TrafficLight 面板插件,
#  场景窗口右侧栏最下面那排 "红圆 / 绿圆 / STOP|GO"。本脚本是无界面
#  (gz sim -s) 或命令行场合的等价操作。
#
#  用法:
#    bash tools/light.sh red      红灯
#    bash tools/light.sh green    绿灯
#    bash tools/light.sh          不带参数 = 红
# ============================================================
set -e
COLOR="${1:-red}"
case "$COLOR" in
  red|RED|r)     UP=lamp_red;   UZ=1.17; DOWN=lamp_green ;;
  green|GREEN|g) UP=lamp_green; UZ=0.73; DOWN=lamp_red ;;
  *) echo "用法: bash tools/light.sh red|green"; exit 1 ;;
esac

move() {   # $1=模型名  $2=z 坐标
  gz service -s /world/default/set_pose \
    --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
    --req "name: \"$1\", position: {x: 0.7, y: 6.0, z: $2}" >/dev/null 2>&1 \
    && echo "  ✓ $1 -> z=$2" || echo "  ✗ $1 失败 (仿真在跑吗?)"
}

echo "信号灯 -> $COLOR"
move "$UP" "$UZ"
move "$DOWN" -50.0
