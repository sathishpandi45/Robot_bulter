Robot Butler

The Robot Butler Project is a ROS 2-based system designed to simulate a robot capable of delivering items to predefined locations (e.g., tables in a restaurant) and handling task cancellations. The project leverages the ROS 2 Humble framework, Nav2 for navigation, and Python for logic implementation.

 Features

- Waypoint Navigation: The robot moves between predefined waypoints (e.g., home, kitchen, and tables).
- Order Processing: Receives and processes orders dynamically.
- Task Cancellation: Allows active tasks to be canceled.
- Timeout Handling: Handles timeout scenarios during delivery.
- Task Completion Notification: Publishes notifications when tasks are completed.

 Topics

 Subscribed Topics

1. /order_topic

   - Type: std_msgs/String
   - Description: Receives orders specifying the target location (e.g., table1).

2. /cancel_order

   - Type: std_msgs/String
   - Description: Receives cancellation requests for specific orders.

 Published Topics

1. /robot_status

   - Type: std_msgs/String
   - Description: Publishes the current status of the robot.

2. /task_complete

   - Type: std_msgs/String
   - Description: Notifies when a task is completed.

 Waypoints

    The robot uses the following predefined waypoints:

    | Location | X   | Y    | Theta |
    | -------- | --- | ---- | ----- |
    | Home     | 0.0 | 0.0  | 0.0   |
    | Kitchen  | 1.0 | 2.0  | 1.57  |
    | Table 1  | 2.0 | -2.0 | 3.14  |
    | Table 2  | 4.0 | -2.0 | 3.14  |
    | Table 3  | 6.0 | -2.0 | 3.14  |


 File Structure

    robot_butler/
    ├── package.xml
    ├── setup.py
    ├── resource/
    ├── robot_butler/
    │   ├── __init__.py
    │   └── main.py
    └── setup.cfg


 Usage

 Running the Robot Butler Node

Start the robot butler node using the following command:


    ros2 run robot_butler main


 Sending Orders

To send an order to the robot:


    ros2 topic pub /order_topic std_msgs/String "data: 'table1'"


 Canceling Orders

To cancel an active order:


    ros2 topic pub /cancel_order std_msgs/String "data: 'table1'"


 Monitoring Status

Use the following command to monitor the robot's status:


    ros2 topic echo /robot_status





