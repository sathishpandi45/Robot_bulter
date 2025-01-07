#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from enum import Enum

class Robotstate(Enum):
    IDLE = "Idle"
    MOVE_TO_KITCHEN = "Moving to Kitchen"
    WAITING = "Waiting for Confirmation"
    DELIVERING = "Delivering Food"
    RETURNING_HOME = "Returning Home"

    

class RobotButler(Node):
    def __init__(self):
        super().__init__("robot_butler")
        self.navigation_client = ActionClient(self,NavigateToPose,'navigate_to_pose')

    def send_goal(self,pose):
        goal_msg =NavigateToPose.Goal()
        goal_msg.pose = pose
        self.navigation_client.wait_for_server()
        self.navigation_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self,future):
        self.get_logger().info('Goal sent sucess')

    def set_state(self, new_state):
        self.get_logger().info(f"State changed to: {new_state}")
        self.state = new_state

    WAYPOINT = {
        'home': [0.0,0.0,0.0],
        'kitchen': [1.0,2.0,1.57],
        'table1': [2.0,-2.0,3.14],
        'table2': [4.0,-2.0,3.14],
        'table3': [6.0,-2.0,3.14],

    }
    def get_waypoint(name):
        return WAYPOINT[name]
    
def main(args=None):
    rclpy.init(args=args)
    robot_butler = RobotButler()
    rclpy.spin(robot_butler)
    rclpy.shutdown()

if __name__== '__main__':
    main()