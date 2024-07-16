#!/usr/bin/env python3

import rclpy
import time
import math

from rclpy import qos
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import (
    Bool,
    Float32MultiArray
)

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler

from utilize import To_Radians

class Navigate(Node):
    def __init__(self):
        super().__init__('navigate_node')

        # Initialize the navigator
        self.navigator = BasicNavigator()

        # Wait until Nav2 is active
        self.navigator.waitUntilNav2Active()
        # Create publisher to indicate navigation goal completion
        self.nav_goal_finished = self.create_publisher(
            Bool,
            "finished",
            qos_profile=qos.qos_profile_system_default,
        )
        
        self.sub_ip = self.create_subscription(
            Float32MultiArray,
            "robot/target_nav_ip",
            self.sub_ip_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        
        self.sub_goal = self.create_subscription(
            Float32MultiArray,
            "robot/target_nav_goal",
            self.sub_goal_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        
        self.__previous_target_goal = Float32MultiArray()
        self.__previous_target_ip = Float32MultiArray()
        self.sent_timer = self.create_timer(0.05, self.timer_callback)
        
    def sub_ip_callback(self, initial_pose):
        if initial_pose.data == self.__previous_target_ip:
            return
        ip = self.get_initial_pose(
            initial_pose.data[0], initial_pose.data[1], initial_pose.data[2]
        )
        self.navigator.setInitialPose(ip)
        self.__previous_target_ip = initial_pose.data
        
    def get_initial_pose(self, x, y, yaw):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = x
        initial_pose.pose.position.y = y
        initial_pose.pose.position.z = 0.0
        (
            initial_pose.pose.orientation.x,
            initial_pose.pose.orientation.y,
            initial_pose.pose.orientation.z,
            initial_pose.pose.orientation.w,
        ) = quaternion_from_euler(0, 0, math.radians(yaw * -1))
        return initial_pose

    def sub_goal_callback(self, goal):
        if goal.data == self.__previous_target_goal:
            return
        target = self.get_point(goal.data[0], goal.data[1], goal.data[2])
        self.navigator.goToPose(target)
        self.__previous_target_goal = goal.data

    def get_point(self, x: float, y: float, yaw: float):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        (
            goal_pose.pose.orientation.x,
            goal_pose.pose.orientation.y,
            goal_pose.pose.orientation.z,
            goal_pose.pose.orientation.w
        ) = quaternion_from_euler(0.0, 0.0, To_Radians(yaw))
        return goal_pose
        
    def timer_callback(self):
        finished_msg = Bool()
        finished_msg.data = self.navigator.isTaskComplete()
        self.nav_goal_finished.publish(finished_msg)

def main():
    rclpy.init()
    navigator_node = Navigate()
    # navigator_node.goto_pos(1.0, 1.0, 45.0)
    rclpy.spin(navigator_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
