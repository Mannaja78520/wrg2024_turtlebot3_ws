#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/wrg2024_turtlebot3_ws/install/setup.bash
ros2 launch wrg_core bot_bringup.launch.py
