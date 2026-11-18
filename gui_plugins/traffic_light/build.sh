#!/bin/bash
# ============================================================
#  编译红绿灯 GUI 插件, 并装到 gz-gui 的默认插件搜索路径
#  (~/.gz/gui/plugins), 这样 worlds/scenes/light.sdf 里写
#  <plugin filename="TrafficLight"> 就能直接加载, 不用设环境变量。
#
#  改过 TrafficLight.cc / .qml 之后跑一遍即可, 重开仿真窗口生效。
# ============================================================
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"

mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release >/dev/null
make -j"$(nproc)"

DEST="$HOME/.gz/gui/plugins"
mkdir -p "$DEST"
cp -f libTrafficLight.so "$DEST/"
echo "✓ 已编译并安装: $DEST/libTrafficLight.so"
