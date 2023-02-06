# Import the ROS required modules
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Import the required ROS interfaces
from nav2_msgs.action import Wait, Spin

# Import the builtin 'Duration' message
from builtin_interfaces.msg import Duration

import math

# Import this to type the functions
from typing import List

class BehaviorInterface:

    def __init__(self, parent_node: Node):
        """! Initialize the controller server interface.
        @param parent_node "Node" node instance that will be used to initialize
            the ROS-related attributes of class.
        """
        self.node = parent_node
        self.logger = parent_node.get_logger()
        self.spin_server_client = ActionClient(parent_node, Spin, "spin")
        self.wait_server_client = ActionClient(parent_node, Wait, "wait")

    def call_spin_action_client(self, target_yaw: float, time_allowance: Duration):
        """! Call the Spin behavior server client
        @param target_yaw "float" Target angle to spin
        @param time_allowance "Duration" Time to evoke the behavior, if it takes 
            more is considered as a failure
        """

        # Define the Spin goal
        spin_goal = Spin.Goal()
        spin_goal.target_yaw = target_yaw
        spin_goal.time_allowance = time_allowance

        # Send the goal to the server
        future = self.spin_server_client.send_goal_async(spin_goal)

        # Wait until the future completes
        rclpy.spin_until_future_complete(self.node, future)

        # Check if the goal was accepted
        if not future.result().accepted:
            self.logger.error("The controller server goal was rejected by server!")
            return
        else:
            self.logger.info("Spinning...")

        # Return the action result
        future = future.result().get_result_async()

        # Wait unitl the future completes
        rclpy.spin_until_future_complete(self.node, future)

        # Return the result
        return future.result()

    def call_wait_action_client(self, time: Duration):
        """! Call the Wait behavior server client
        @param time "Duration" Wait time (s)
        """

        # Define the Wait goal
        wait_goal = Wait.Goal()
        wait_goal.time = time

        # Send the goal to the server
        future = self.wait_server_client.send_goal_async(wait_goal)

        # Wait until the future completes
        rclpy.spin_until_future_complete(self.node, future)

        # Check if the goal was accepted
        if not future.result().accepted:
            self.logger.error("The controller server goal was rejected by server!")
            return
        else:
            self.logger.info("Waiting...")

        # Return the action result
        future = future.result().get_result_async()

        # Wait unitl the future completes
        rclpy.spin_until_future_complete(self.node, future)

        # Return the result
        return future.result()

def test_behavior_server(args=None):
    
    # Initialize rclpy
    rclpy.init(args=args)

    # Initialize the test node and the controller interface
    test_node = Node("behavior_server_test")
    behavior_server_interface = BehaviorInterface(test_node)

    # Call the action server to spin the robot. Turn 90 degrees to the right.
    deg = math.radians(-90.0)
    time_allowance = Duration()
    time_allowance.sec = 10

    result = behavior_server_interface.call_spin_action_client(deg, time_allowance)

    # Call the action server to wait. Wait for two seconds
    time = Duration()
    time.sec = 2
    result = behavior_server_interface.call_wait_action_client(time)

    # Call the action server to spin the robot. Turn 90 degrees to the left.
    deg = math.radians(90.0)

    result = behavior_server_interface.call_spin_action_client(deg, time_allowance)

    # Kill them all
    rclpy.shutdown()


if __name__ == "__main__":
    test_behavior_server()
