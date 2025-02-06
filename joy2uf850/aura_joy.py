# scripts/aura_joy.py

import rclpy
from rclpy.node import Node

class AuraJoyNode(Node):
    def __init__(self):
        super().__init__('aura_joy_node')
        self.get_logger().info('AURA Joy Node is running!')

def main(args=None):
    rclpy.init(args=args)
    node = AuraJoyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


