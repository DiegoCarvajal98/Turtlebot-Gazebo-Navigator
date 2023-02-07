import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

# Import ROS interfaces
from nav2_msgs.action import NavigateToPose

import ament_index_python

import yaml

class Nav2CommanderNode(Node):
    def __init__(self, node_name):
        Node.__init__(self, node_name=node_name)
        self.get_logger().info("Hi! I'm the nav2 commander node!")
        

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
