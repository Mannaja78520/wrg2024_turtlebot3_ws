#!/bin/bash

# Source ROS 2 and workspace setup files
source /opt/ros/humble/setup.bash
source /home/cmu/microros_ws/install/local_setup.bash
source /home/cmu/turtlebot3_ws/install/setup.bash
source /home/cmu/wrg2024_turtlebot3_ws/robot_ws/install/setup.bash

# Export necessary environment variables
export ROS_DOMAIN_ID=30
export LDS_MODEL=LDS-02
export TURTLEBOT3_MODEL=burger
export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=burger

# Path to your Node.js server script
NODE_SERVER_SCRIPT="/home/cmu/wrg2024_turtlebot3_ws/ros2-web-control/web-control/server.js"
NODE_ROS2_WEBBRIDGE_SCRIPT="/home/cmu/wrg2024_turtlebot3_ws/ros2-web-control/ros2-web-bridge/bin/rosbridge.js"

# Start the ROS 2 launch files with a 1-second delay between each

# ros2 launch wrg_core bot_bringup.launch.py
# sleep 10

echo "Starting turtlebot3_bringup.launch.py..."
ros2 launch turtlebot3_bringup turtlebot3_bringup.launch.py &
sleep 3

# echo "Starting rosbridge_websocket_launch.xml..."
# ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
# sleep 3

echo "Starting state.launch.py..."
ros2 launch wrg_core state.launch.py &
sleep 3

echo "Starting navigate.py..."
ros2 launch wrg_core navigate.py &
sleep 3

echo "Starting microros.launch.py..."
ros2 launch wrg_core microros.launch.py &
sleep 3

# Start the Node.js server in the background
echo "Starting Node.js server..."
node "$NODE_SERVER_SCRIPT" &
node "$NODE_ROS2_WEBBRIDGE_SCRIPT" &

# Keep the script running (e.g., to prevent Docker container from exiting)
# tail -f /dev/null

