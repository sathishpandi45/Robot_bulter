#!/usr/bin/python3
import rclpy
from rclpy.node import Node

class RobotButler(Node):
    def __init__(self):
        super().__init__("robot_butler")
        self.get_logger().info('Hello Ros2')

def main(args=None):
    rclpy.init(args=args)
    robot_butler = RobotButler()
    rclpy.spin(robot_butler)
    rclpy.shutdown()

if __name__== '__main__':
    main()