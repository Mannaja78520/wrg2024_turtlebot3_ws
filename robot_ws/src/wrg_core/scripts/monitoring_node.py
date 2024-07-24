import rclpy
from rclpy.node import Node
from rclpy import qos
from ament_index_python.packages import get_package_share_directory
from launch import LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from std_msgs.msg import String
import os
import subprocess

class MonitoringNode(Node):
    def __init__(self):
        super().__init__('monitoring_node')
        self.sub_robot_state = self.create_subscription(
            String,
            "robot_state",
            self.sub_robot_state_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.launch_service = LaunchService()
        self.launch_descriptions = {
            'navigate.launch.py': self.create_launch_description('wrg_core', 'navigate.launch.py'),
            'robot.launch.py': self.create_launch_description('turtlebot3_bringup', 'robot.launch.py')
        }
        self.active_launches = {}

        # Kill existing nodes and start initial launch files
        self.kill_existing_nodes()
        self.start_launch_file('navigate.launch.py')
        # self.start_launch_file('robot.launch.py')

    def kill_existing_nodes(self):
        self.get_logger().info('Checking for existing nodes...')
        # List the nodes
        result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
        nodes = result.stdout.splitlines()
        
        # Kill nodes associated with the launch files
        for node in nodes:
            # if 'navigate_node' in node or 'Init TurtleBot3 Node Main' in node:
            if 'navigate_node' in node:
                self.get_logger().info(f'Killing node: {node}')
                subprocess.run(['ros2', 'node', 'kill', node])

    def sub_robot_state_callback(self, msg: String):
        if msg.data == 'None':
            self.restart_launch_file('navigate.launch.py')
        # if msg.data == 'Init':
        #     self.restart_launch_file('robot.launch.py')

    def start_launch_file(self, launch_file_name):
        self.get_logger().info(f'Starting launch file: {launch_file_name}')
        launch_description = self.launch_descriptions[launch_file_name]
        self.launch_service.include_launch_description(launch_description)
        self.launch_service.run()

    def restart_launch_file(self, launch_file_name):
        self.get_logger().info(f'Restarting launch file: {launch_file_name}')
        # Shut down the current launch service and start a new one
        self.launch_service.shutdown()
        self.launch_service = LaunchService()
        self.start_launch_file(launch_file_name)

    def create_launch_description(self, package_name, launch_file_name):
        launch_file_dir = os.path.join(get_package_share_directory(package_name), 'launch')
        return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([launch_file_dir, f'/{launch_file_name}']),
                launch_arguments={}.items()
            )
        ])

def main(args=None):
    rclpy.init(args=args)
    monitoring_node = MonitoringNode()
    try:
        rclpy.spin(monitoring_node)
    except KeyboardInterrupt:
        pass
    finally:
        monitoring_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
