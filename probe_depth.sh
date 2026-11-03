#!/bin/bash
cd /root/gz_ros
bash /root/gz_ros/up.sh /root/gz_ros/worlds/scenes/low_box_probe.sdf
source /opt/ros/jazzy/setup.bash
for i in $(seq 1 30); do timeout 3 ros2 topic echo /clock --once >/dev/null 2>&1 && break; sleep 1; done
echo "=== cam at +0.1m ahead of center; box front edge y=2.30 ==="
for Y in 0.8 1.0 1.2 1.4 1.6 1.8 1.95 2.05; do
  gz service -s /world/default/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 --req "name: \"robot_with_camera\", position: {x: 0, y: $Y, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}" >/dev/null 2>&1
  sleep 1.5
  D=$(python3 -c "print(round(2.30 - ($Y + 0.10), 2))")
  echo -n "y=$Y dist=$D "
  timeout 15 python3 /tmp/probe_depth.py 2>/dev/null | tr "\n" " "
  echo
done