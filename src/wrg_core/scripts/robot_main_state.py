#!/usr/bin/env python3
import rclpy
import math
import time

from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import (
    String,
    Float32MultiArray,
    Bool,
)
from geometry_msgs.msg import Twist

class RobotMainState(Node):
    def __init__(self):
        super().__init__("robot_main_state_node")
        
        self.pub_ip = self.create_publisher(
            Float32MultiArray,
            "robot/target_nav_ip",
            qos_profile=qos.qos_profile_system_default,
        )
        self.pub_goal = self.create_publisher(
            Float32MultiArray,
            "robot/target_nav_goal",
            qos_profile=qos.qos_profile_system_default,
        )
        self.sub_state = self.create_subscription(
            String,
            "robot/state",
            self.sub_state_callback,
            qos_profile=qos.qos_profile_system_default,
        )
        
        self.sent_timer = self.create_timer(0.05, self.timer_callback)

        self.robot_state = String()
        self.retry = "none"
        self.move = "none"

        self.robot_main_state = 0 

    def timer_callback(self):
        msg_ip = Float32MultiArray()
        msg_goal = Float32MultiArray()
        if self.robot_state == "IDLE":
            msg_ip.data = [0.0, 0.0, 0.0]
            self.pub_ip.publish(msg_ip)
            
        elif self.robot_state == "START":
            msg_goal.data = [1.0, 0.0, 0.0]
            self.pub_goal.publish(msg_goal)
            
def main():
    rclpy.init()
    sub = RobotMainState()
    rclpy.spin(sub)
    rclpy.shutdown()


if __name__ == "__main__":
    main()