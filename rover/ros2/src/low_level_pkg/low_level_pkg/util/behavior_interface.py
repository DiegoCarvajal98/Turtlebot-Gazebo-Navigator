# Import the ROS required modules
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Import the required ROS interfaces
from nav2_msgs.action import Wait, Spin, FollowPath, ComputePathToPose
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Import the builtin 'Duration' message
from builtin_interfaces.msg import Duration

import math

# Import this to type the functions
from typing import List

# Import yaml reading module
import yaml

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
        self.planner_server_client = ActionClient(parent_node, ComputePathToPose, "compute_path_to_pose")
        self.controller_server_client = ActionClient(parent_node, FollowPath, 'follow_path')
        self.audio_publisher = self.node.create_publisher(String, "/device/speaker/command", 10)
        self.start_navigation_server = self.node.create_service(Trigger, "start_navigation", self.start_navigation_callback)


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
    
    
    def call_control_action_client(self, path: Path, controller_id: str, goal_checker_id: str):
        """! Call the controller server action to follow a given path.
        @param frame_id "str" name of the frame id in which the points are.
        @param poses "List[PoseStamped]" list of PoseStamped elements that make
            up the path.
        @param controller_id "str" name of the controller that will be used.
        @param goal_checker_id "str" name of the goal checker that will be used.
        """
        # Check if controller server is not available
        if not self.controller_server_client.wait_for_server(timeout_sec=1.0):
            self.logger.error("Controller server is not available!")
            return

        # FollowWaypoints goal message
        action_goal = FollowPath.Goal()
        action_goal.path = path

        action_goal.controller_id = controller_id
        action_goal.goal_checker_id = goal_checker_id

        # Send the goal to the server
        future = self.controller_server_client.send_goal_async(action_goal)

        # Wait until the future completes
        rclpy.spin_until_future_complete(self.node, future)

        # Check if the goal was accepted
        if not future.result().accepted:
            self.logger.error("The controller server goal was rejected by server!")
            return

        # Return the action result
        future = future.result().get_result_async()

        # Wait unitl the future completes
        rclpy.spin_until_future_complete(self.node, future)

        # Return the result
        return future.result()
    
    
    def call_plan_action_client(self, goal: PoseStamped, start: PoseStamped, 
                                planner_id: str, use_start: bool):
        """! Call the planner server action to calculate a path.

        @param goals "List[PoseStamped]" list of PoseStamped elements to plan 
            the path
        @param start "PoseStamped" PoseStamped element that defines the start 
            of the path
        @param planner_id "str" Planner to be used
        @param use_start "bool" If set to False, start from the actual position 
            of the robot, else, start from the start param pose

        """
        # Check if controller server is not available
        if not self.planner_server_client.wait_for_server(timeout_sec=1.0):
            self.logger.error("Planner server is not available!")
            return

        frame_id = "map"

        # Goal definition
        action_goal = ComputePathToPose.Goal()

        action_goal.goal = goal
        action_goal.goal.header.frame_id = frame_id
        action_goal.goal.header.stamp = self.node.get_clock().now().to_msg()

        action_goal.start = start

        action_goal.start.header.stamp = self.node.get_clock().now().to_msg()
        action_goal.start.header.frame_id = frame_id

        # self.node.get_logger().info("%f, %f" % (start.pose.position.x, start.pose.position.y))

        action_goal.planner_id = planner_id

        action_goal.use_start = use_start

        # Send the goal to the server
        self.node.get_logger().info("Sending path goal")
        future = self.planner_server_client.send_goal_async(action_goal)

        # Wait until the future completes
        rclpy.spin_until_future_complete(self.node, future)

        # Check if the goal was accepted
        if not future.result().accepted:
            self.logger.error("The planner server goal was rejected by server!")
            return
        else:
            self.logger.info("Path goal accepted")

        # Return the action result
        future = future.result().get_result_async()

        # Wait until the future completes
        rclpy.spin_until_future_complete(self.node, future)

        self.logger.info("Result from path goal")
        # Return the action result
        return future.result()
    
    
    def start_navigation_callback(self, req, resp):
        sites = ["restaurant", "customer", "parking"]

        for i in sites:
            goal, spin_angle, track_name, wait_time = self.read_waypoints(i)

            #Compute path to restaurant
            start = PoseStamped()
            planner_id = ""
            use_start = False

            self.logger.info("Planning path")
            result = self.call_plan_action_client(goal, start, planner_id, use_start)

            # Execute path to restaurant
            path = result.result.path
            controller_id = "FollowPath"
            goal_checker = "goal_checker"

            result = self.call_control_action_client(path, controller_id, goal_checker)

            # Publish audio
            self.logger.info("Sound for: %s" % i)
            audio_msg = String()
            audio_msg.data = track_name
            self.audio_publisher.publish(audio_msg)

            # Wait
            self.logger.info("Wait %d seconds" % wait_time)
            time = Duration
            time.sec = int(wait_time)
            self.call_wait_action_client(time)

        return resp
    
    
    def read_waypoints(self, site: str):
        """! Function that generates a waypoints from the yaml file.
        @return "str" Waypoint pose.
        @return "float" Spin angle.
        @return "str" Name of the audio track.
        @return "int" Wait time.
        """

        with open("/workspace/rover/ros2/src/tb_bringup/config/waypoints.yaml") as f:
            waypoints = yaml.safe_load(f)
            waypoints = waypoints["waypoints"]

        pose = PoseStamped()

        # Define goal pose
        pose.pose.position.x = waypoints[site]["pose"]["x"]
        pose.pose.position.y = waypoints[site]["pose"]["y"]

        spin_angle = 0.0

        track_name = waypoints[site]["track"]

        wait_time = waypoints[site]["wait_time"]

        return pose, spin_angle, track_name, wait_time
    

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

    rclpy.spin(test_node)

    # Kill them all
    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = Node("behavior_node")
    behavior = BehaviorInterface(node)

    rclpy.spin(behavior)

    behavior.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    test_behavior_server()
    main()
