# Import the ROS required modules
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Import the required ROS interfaces
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped

# Import this to type the functions
from typing import List

# These imports are only used in the tests
# Import the get package function
import yaml
from ament_index_python.packages import get_package_share_directory


class PlannerInterface:

    def __init__(self, parent_node: Node):
        """! Initialize the planner server interface.
        @param parent_node "Node" node instance that will be used to initialize
            the ROS-related attributes of class.
        """
        self.node = parent_node
        self.logger = parent_node.get_logger()
        self.planner_server_client = ActionClient(parent_node, ComputePathToPose, "compute_path_to_pose")

    def call_action_client(self, frame_id: str, goal: PoseStamped, start: PoseStamped, planner_id: str, use_start: bool):
        """! Call the planner server action to calculate a path.

        @param goals "List[PoseStamped]" list of PoseStamped elements to plan the path
        @param start "PoseStamped" PoseStamped element that defines the start of the path
        @param planner_id "str" Planner to be used
        @param use_start "bool" If set to False, start from the actual position of the robot,
            else, start from the start param pose

        """
        # Check if controller server is not available
        if not self.planner_server_client.wait_for_server(timeout_sec=1.0):
            self.logger.error("Planner server is not available!")
            return

        # Goal definition
        action_goal = ComputePathToPose.Goal()

        action_goal.goal = goal

        self.node.get_logger().info("%f, %f" % (goal.pose.position.x, goal.pose.position.y))

        action_goal.goal.header.stamp = self.node.get_clock().now().to_msg()
        action_goal.goal.header.frame_id = frame_id

        action_goal.start = start

        self.node.get_logger().info("%f, %f" % (start.pose.position.x, start.pose.position.y))

        action_goal.start.header.stamp = self.node.get_clock().now().to_msg()
        action_goal.start.header.frame_id = frame_id

        action_goal.planner_id = planner_id

        action_goal.use_start = use_start

        # Send the goal to the server
        future = self.planner_server_client.send_goal_async(action_goal)

        # Wait until the future completes
        rclpy.spin_until_future_complete(self.node, future)

        # Check if the goal was accepted
        if not future.result().accepted:
            self.logger.error("The planner server goal was rejected by server!")
            return

        # Return the action result
        future = future.result().get_result_async()

        # Wait until the future completes
        rclpy.spin_until_future_complete(self.node, future)

        # Return the action result
        return future.result()


def read_waypoints():
    """! Function that generates a waypoints to test the planner server.
    @return "str" name of the frame for the path.
    @return "List[PoseStamped]" list of PoseStamped elements with the goal
        waypoints.
    @return "PoseStamped" PoseStamped element with the start of the path
    @return "str" name of the used path planner.
    @return "bool" if set to False, use robot's current pose as path start.
    """

    frame_id = "map"

    with open("/workspace/rover/ros2/src/tb_bringup/config/waypoints.yaml") as f:
        waypoints = yaml.safe_load(f)
        waypoints = waypoints["waypoints"]

    keys = list(waypoints.keys())

    goal = PoseStamped()

    # Define goal pose
    goal.pose.position.x = waypoints[keys[0]]["pose"]["x"]
    goal.pose.position.y = waypoints[keys[0]]["pose"]["y"]

    start = PoseStamped()
    # Define start pose
    start.pose.position.x = waypoints[keys[1]]["pose"]["x"]
    start.pose.position.y = waypoints[keys[1]]["pose"]["y"]

    # Define planner_id
    planner_id = ""

    # Use start pose for path planning
    use_start = True

    return frame_id, goal, start, planner_id, use_start


def test_planner_server(args=None):
    
    # Initialize rclpy
    rclpy.init(args=args)

    # Initialize the test node and the controller interface
    test_node = Node("planner_server_test")
    planner_server_interface = PlannerInterface(test_node)

    # Generate the goal, start, planner ID and user start values
    frame_id, goals, start, planner_id, use_start = read_waypoints()

    # Call the action client with the path
    result = planner_server_interface.call_action_client(frame_id, goals, start, planner_id, use_start)

    # Kill them all
    rclpy.shutdown()

if __name__ == "__main__":
    test_planner_server()