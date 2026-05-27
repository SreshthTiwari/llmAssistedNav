import rclpy
from rclpy.node import Node

class NavNode(Node):
    def __init__(self):
        super().__init__("nav_node")
        self.get_logger().info("NavNode started")

def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()