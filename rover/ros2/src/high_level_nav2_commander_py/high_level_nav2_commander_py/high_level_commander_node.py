import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

# Import ROS interfaces
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Bool

import ament_index_python

import yaml

class Nav2CommanderNode(Node):
    def __init__(self, node_name):
        Node.__init__(self, node_name=node_name)
        self.get_logger().info("Hi! I'm the nav2 commander node!")
        self.navigate_action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.audio_publisher = self.create_publisher(String, "device/speaker/command", 10)
        self.create_subscription(Bool, "navigation/start", self.navigation_start_callback)
        self.create_subscription(Bool, "navigation/stop", self.navigation_stop_callback)

    def read_waypoints(self, pkg: str, folder: str):
        """! Function to read the waypoints from a yaml file
        @param pkg "str" Package where the yaml file is located
        @param folder "str" Yaml file location folder within the package

        @return "dict" Waypoints from the yaml file
        """
        dir = ament_index_python.get_package_prefix(pkg)
        dir = dir + folder
        
        with open(dir) as f:
            waypoints = yaml.safe_load(f)["waypoints"]

        return waypoints

def main():
    rclpy.init()
    nav2_commander = Nav2CommanderNode("nav2_commander_py")

    executor = MultiThreadedExecutor()
    executor.add_node(nav2_commander)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
