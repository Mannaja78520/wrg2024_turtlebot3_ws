import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mannaja/wrg2024_turtlebot3_ws/ros2-web-control/install/gyp'
