#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

def main(args=None):
    rclpy.init(args=args)

    node = Node('bool_service_client')
    servo = node.create_client(SetBool, '/servo_service')

    while not servo.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for service /servo_service to be available...')

    request = SetBool.Request()
    request.data = True

    future = servo.call_async(request)

    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        response = future.result()
        node.get_logger().info(f'Success: {response.success}, Message: {response.message}')
    else:
        node.get_logger().error('Service call failed')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
