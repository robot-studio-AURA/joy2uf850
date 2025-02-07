# scripts/aura_joy.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy

class AuraJoyNode(Node):
    def __init__(self):
        super().__init__('aura_joy_node')
        
        # Publisher for MoveIt Servo TwistStamped commands
        self.servo_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        
        # Subscriber for joystick inputs
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Frame of reference for the Twist commands (use your robot's planning frame)
        self.frame_id = 'base_link'
        
        # Joystick axes mapping
        self.left_joystick_x = 0  # Left joystick horizontal axis
        self.left_joystick_y = 1  # Left joystick vertical axis
        self.right_joystick_x = 3 # Right joystick horizontal axis
        self.right_joystick_y = 4 # Right joystick vertical axis
        self.right_trigger = 5    # Right trigger axis (assumes value goes negative when pressed)

    def joy_callback(self, joy_msg):
        # Create a TwistStamped message for MoveIt Servo
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.frame_id
        
        # Map joystick axes to Cartesian velocities and angular velocities
        twist.twist.linear.x = joy_msg.axes[self.left_joystick_x] * 0.1  # Scale for x velocity
        twist.twist.linear.y = joy_msg.axes[self.left_joystick_y] * 0.1  # Scale for y velocity
        
        twist.twist.linear.z = (joy_msg.axes[self.right_trigger] - 1) * 0.1 / 2   # Scale for z velocity
        
        twist.twist.angular.x = joy_msg.axes[self.right_joystick_y] * 0.1  # Scale for pitch
        twist.twist.angular.y = joy_msg.axes[self.right_joystick_x] * 0.1  # Scale for roll
        
        # Publish the TwistStamped command to MoveIt Servo
        self.servo_pub.publish(twist)
        
        # Log the values (optional, for debugging)
        self.get_logger().info(f"Published Twist: {twist}")

def main(args=None):
    rclpy.init(args=args)
    node = AuraJoyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




