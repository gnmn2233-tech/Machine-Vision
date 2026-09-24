#!/bin/bash
# 依次启动仿真、YOLO、相机查看，并保持后台运行

bash start_simulation.sh &
SIM_PID=$!
sleep 3   

bash yolo.sh &
YOLO_PID=$!
sleep 3

bash view_camera.sh &
VIEW_PID=$!

trap "kill $SIM_PID $YOLO_PID $VIEW_PID 2>/dev/null; exit" SIGINT SIGTERM
wait