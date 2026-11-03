#!/bin/bash
source /opt/ros/jazzy/setup.bash
exec ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' \
  '/camera@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo' \
  '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model' \
  '/camera_joint_controller/commands@std_msgs/msg/Float64]gz.msgs.Double' \
  '/drive_x/commands@std_msgs/msg/Float64]gz.msgs.Double' \
  '/drive_y/commands@std_msgs/msg/Float64]gz.msgs.Double' \
  --ros-args -r '/joint_state:=/joint_states'
