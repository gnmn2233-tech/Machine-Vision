#!/bin/bash
cd /root/gz_ros
source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH="/root/gz_ros/mod:/root/gz_ros/worlds/robots:${GZ_SIM_RESOURCE_PATH:-}"
pkill -9 -f 'gz sim' 2>/dev/null; pkill -9 -f parameter_bridge 2>/dev/null; sleep 2
export WORLD_FILE="${WORLD_FILE:-/root/gz_ros/worlds/scenes/corridor.sdf}"
setsid nohup gz sim -r -s "$WORLD_FILE" > /tmp/gzserver.log 2>&1 &
sleep 14
setsid nohup bash start_bridges_headless.sh > /tmp/bridges.log 2>&1 &
sleep 16
timeout 200 bash run.sh "${GX:-0}" "${GY:-8}" > "/tmp/nav_${TAG:-run}.log" 2>&1
echo "NAV_EXIT=$?"
