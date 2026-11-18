#!/usr/bin/env python3
"""信号灯切换守护：订阅 /traffic/cmd，改场景里的灯球位置。

什么时候才需要它: 无界面 (gz sim -s) 或想用一句话切灯时。带界面的场景
已经自带按钮面板 (gui_plugins/traffic_light, 直接调 set_pose)，不经过这里。

用法:
    python3 tools/traffic_daemon.py            # 常驻，收到就切
    gz topic -p /traffic/cmd -t gz.msgs.StringMsg -m 'data: "toggle"'   # 模拟点按钮

消息约定:
    "red" / "green"  —— 切到指定颜色
    "toggle" / 其他   —— 翻转当前颜色
"""
import subprocess
import sys

import gz.transport13 as gz
from gz.msgs10.stringmsg_pb2 import StringMsg

UZ = {"red": 1.17, "green": 0.73}      # 两颗亮球各自的灯位高度
NAME = {"red": "lamp_red", "green": "lamp_green"}
state = {"color": "red"}


def move(model, z):
    subprocess.run(
        ["gz", "service", "-s", "/world/default/set_pose",
         "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
         "--timeout", "3000",
         "--req", 'name: "%s", position: {x: 0.7, y: 6.0, z: %s}' % (model, z)],
        capture_output=True)


def apply(color):
    other = "green" if color == "red" else "red"
    move(NAME[color], UZ[color])          # 要亮的移到灯位
    move(NAME[other], -50.0)              # 另一颗沉到地下, 露出底下的黑球
    state["color"] = color
    print(f"信号灯 -> {color}", flush=True)


def on_msg(msg):
    cmd = str(msg.data).strip().lower()
    if cmd in ("red", "green"):
        apply(cmd)
    elif cmd in ("toggle", "", "switch"):
        apply("green" if state["color"] == "red" else "red")
    else:
        print(f"忽略命令: {cmd!r}", flush=True)


def main():
    node = gz.Node()
    if not node.subscribe(StringMsg, "/traffic/cmd", on_msg):
        print("订阅失败: /traffic/cmd", file=sys.stderr)
        return 1
    print("信号灯守护已启动, 监听 /traffic/cmd (red / green / toggle)", flush=True)
    apply("red")
    import time
    while True:
        time.sleep(0.5)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        time.sleep(0.5)
