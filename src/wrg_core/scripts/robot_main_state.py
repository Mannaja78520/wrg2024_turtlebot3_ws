#!/usr/bin/env python3
import rclpy
import math
import time

from rclpy import qos
from rclpy.node import Node
import rclpy.parameter
from std_msgs.msg import (
    String,
    Bool,
    Int8MultiArray,
    Float32MultiArray,
)
from geometry_msgs.msg import Twist
import numpy as np

class RobotMainState(Node):
    def __init__(self):
        super().__init__("robot_main_state_node")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('waypoints_start_to_challenge_zone.start', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('waypoints_start_to_challenge_zone.sub', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('waypoints_start_to_challenge_zone.challenge_zone', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('lists_challenge_zone.room1', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('lists_challenge_zone.room2', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('lists_challenge_zone.room3', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('lists_challenge_zone.room4', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('lists_challenge_zone.room5', rclpy.Parameter.Type.DOUBLE_ARRAY),
            ]
        )
        
        self.waypoint_names = [
        'start', 'sub', 'challenge_zone', 'room1', 'room2', 'room3', 'room4', 'room5'
        ]
        
        self.waypoints = {}
        for name in self.waypoint_names:
            param_name = f'waypoints_start_to_challenge_zone.{name}' if 'room' not in name else f'lists_challenge_zone.{name}'
            self.waypoints[name] = np.array(self.get_parameter(param_name).get_parameter_value().double_array_value, dtype=np.float32)
        
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
        self.pub_cancle_nav = self.create_publisher(
            Bool,
            "robot/cancel_nav",
            qos_profile=qos.qos_profile_system_default,
        )
        self.sub_goal_state = self.create_subscription(
            Bool,
            "goal/state",
            self.sub_goal_state_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_challenge_location = self.create_subscription(
            Int8MultiArray,
            "challenge_location",
            self.sub_challenge_location_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        
        
        self.robot_state: String = String()
        self.robot_last_state: String = String()
        self.retry:String = "none"
        self.robot_main_state: int = 0 
        self.__previous_goal_state: bool = Bool()
        self.challenge_room = []
        self.sent_timer = self.create_timer(0.05, self.timer_callback)
        
    def sub_goal_state_callback(self, goal_state: bool = False) -> None:
        if self.__previous_goal_state != goal_state:
            if goal_state:
                self.robot_main_state += 1
            self.__previous_goal_state = goal_state

    def sub_challenge_location_callback(self, rooms: Int8MultiArray = None) -> None:
        if rooms is not None:
            waypoint_map = {
                1: self.waypoints['room1'],
                2: self.waypoints['room2'],
                3: self.waypoints['room3'],
                4: self.waypoints['room4'],
                5: self.waypoints['room5']
            }
            
            new_challenge_room = []
            
            for room in rooms.data:
                if room in waypoint_map:
                    new_challenge_room.append(waypoint_map[room])
                    
            if len(new_challenge_room) == 4:
                # If we have exactly 4 waypoints, predict the last waypoint
                predicted_last_waypoint = self.predict_last_waypoint(new_challenge_room, waypoint_map)
                new_challenge_room.append(predicted_last_waypoint)
                
            self.challenge_room = new_challenge_room

    def predict_last_waypoint(self, current_waypoints, waypoint_map):
        """
        Predict the last waypoint based on the given waypoints.
        
        Parameters:
        current_waypoints (list of numpy arrays): List of current waypoints.
        waypoint_map (dict): Dictionary mapping room numbers to waypoints.

        Returns:
        numpy array: Predicted last waypoint.
        """
        current_set = set([tuple(wp) for wp in current_waypoints])
        for key, value in waypoint_map.items():
            if tuple(value) not in current_set:
                return value

    def timer_callback(self):
        msg_ip = Float32MultiArray()
        msg_goal = Float32MultiArray()
        if self.robot_state == "INIT":
            msg_ip.data = self.waypoints['start']
            self.pub_ip.publish(msg_ip)
        elif self.robot_state == "START":
            if self.robot_main_state == 0:
                msg_goal.data = np.concatenate((self.waypoints['sub'], self.waypoints['challenge_zone'])).tolist()        
                self.pub_goal.publish(msg_goal)
            if self.robot_main_state == 1:
                for room_number in range(5):
                    return
                
        # nana = [self.room1_waypoint, self.room2_waypoint]
        # print(nana)
        # msg_goal.data = [self.room1_waypoint]
            
def main():
    rclpy.init()
    sub = RobotMainState()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()