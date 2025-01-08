#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from enum import Enum

class Robotstate(Enum):
    IDLE = "Idle"
    MOVE_TO_KITCHEN = "Moving to Kitchen"
    MOVE_TO_TABLE = "Moving to Table"
    WAITING = "Waiting for Confirmation"
    DELIVERING = "Delivering Food"
    RETURNING_HOME = "Returning Home"

WAYPOINTS = {
    'home': [0.0,0.0,0.0],
    'kitchen': [1.0,2.0,1.57],
    'table1': [2.0,-2.0,3.14],
    'table2': [4.0,-2.0,3.14],
    'table3': [6.0,-2.0,3.14],
}   

class RobotButler(Node):
    def __init__(self):
        super().__init__("robot_butler")
        self.state = Robotstate.IDLE
        self.orders = []
        self.get_logger().info("Initializing Robot Butler Node...")
        self.navigation_client = ActionClient(self, NavigateToPose,'navigate_to_pose')
    
        self.subscription = self.create_subscription(
            String, 'order_topic', self.order_callback,10
        )
        
        self.cancel_subscription = self.create_subscription(
            String, 'cancel_order', self.cancel_order_callback, 10
        )

        self.status_publisher = self.create_publisher(String,'robot_status',10)
        self.status_publisher.publish(String(data="Moving to kitchen"))

        self.task_complete_publisher = self.create_publisher(String, 'task_complete', 10)
        
    def get_waypoint(self,name):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = WAYPOINTS[name][0]
        pose.pose.position.y = WAYPOINTS[name][1]
        pose.pose.orientation.z = WAYPOINTS[name][2]
        return pose
    
    def send_goal(self,pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.navigation_client.wait_for_server()
        self.navigation_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self,future):
        self.get_logger().info('Goal sent sucess')

    def order_callback(self, msg):
        self.get_logger().info(f"Order received: {msg.data}")
        self.orders.append(msg.data)
        self.process_order()

    def cancel_order_callback(self, msg):
        if not self.orders:
            self.get_logger().info(f"No orders to cancel. Ignoring cancellation for: {msg.data}")
            return

        self.get_logger().info(f"Cancellation received for: {msg.data}")
        if self.orders[0] == msg.data:
            self.get_logger().info(f"Canceling current order: {msg.data}")
            self.cancel_task()
        else:
            self.get_logger().info(f"Order {msg.data} not in progress or already completed.")


    def wait_for_confirmation(self, duration,timeout_callback):
        self.timer = self.create_timer(duration,timeout_callback)

    def handle_kitchen_timeout(self):
        self.get_logger().info("Timeout,Returing to home!")
        self.set_state(Robotstate.RETURNING_HOME)
        self.send_goal(self.get_waypoint('home'))

    def handle_table_timeout(self):
        self.get_logger().info("Timeout,Returing to kitchen!")
        self.set_state(Robotstate.MOVE_TO_KITCHEN)
        self.send_goal(self.get_waypoint('kitchen'))

    def cancel_task(self):
        if self.state == Robotstate.MOVE_TO_KITCHEN:
            self.get_logger().info("Task canceled,Returning to home.")
            self.set_state(Robotstate.RETURNING_HOME)
            self.send_goal(self.get_waypoint('home'))
        elif self.state == Robotstate.MOVE_TO_TABLE:
            self.get_logger().info("Task canceled,Returning to Kitchen.")
            self.set_state(Robotstate.MOVE_TO_KITCHEN)
            self.send_goal(self.get_waypoint('kitchen'))

        if self.orders:
            self.orders.pop(0)

    def process_order(self):
        if not self.orders:
            return
        
        current_order = self.orders.pop(0)
        self.process_single_order(current_order)

    def process_single_order(self,table):
        self.set_state(Robotstate.MOVE_TO_KITCHEN)
        self.send_goal(self.get_waypoint('kitchen'))
        self.wait_for_confirmation(10.0,self.handle_kitchen_timeout)

        self.set_state(Robotstate.DELIVERING)
        self.send_goal(self.get_waypoint(table))
        self.wait_for_confirmation(10.0,self.handle_table_timeout)
        self.task_complete_publisher.publish(String(data=f"Task complete for {table}"))

        self.set_state(Robotstate.RETURNING_HOME)
        self.send_goal(self.get_waypoint('home'))

    def set_state(self, new_state):
        self.get_logger().info(f"State changed to: {new_state}")
        self.state = new_state

    
def main(args=None):
    rclpy.init(args=args)
    robot_butler = RobotButler()
    rclpy.spin(robot_butler)
    rclpy.shutdown()

if __name__== '__main__':
    main()