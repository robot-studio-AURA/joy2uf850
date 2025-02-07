# scripts/aura_joy.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy

class AuraJoyNode(Node):
    def __init__(self):
        super().__init__('aura_joy_node')
        
        # Subscribe to joystick input
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',  # Joystick topic
            self.joy_callback,
            10
        )

        # Publisher for Cartesian velocity commands
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_server/delta_twist_cmds',  # Topic for MoveIt Servo
            10
        )

        # State variable for Z-axis spring behavior
        self.z_velocity = 0.0

    def joy_callback(self, msg):
        """Callback to process joystick input and publish TwistStamped commands."""
        twist = TwistStamped()

        # Map left joystick to X and Y positions
        twist.twist.linear.x = msg.axes[1]  # Left stick vertical (up/down)
        twist.twist.linear.y = msg.axes[0]  # Left stick horizontal (left/right)

        # Map right joystick to pitch and roll
        twist.twist.angular.y = msg.axes[4]  # Right stick vertical (pitch)
        twist.twist.angular.x = msg.axes[3]  # Right stick horizontal (roll)

        # Map right trigger (RT) to Z-axis movement with spring behavior
        right_trigger = msg.axes[5]  # Right trigger value (-1 when fully pressed, 1 when released)
        
        if right_trigger < 1.0:  # If RT is pressed, move down
            self.z_velocity = -1.0 * (1.0 - right_trigger)  # Scale Z velocity based on trigger pressure
        else:  # If RT is released, move back up (spring behavior)
            self.z_velocity = 0.5  # Constant upward velocity

        twist.twist.linear.z = self.z_velocity

        # Publish the TwistStamped command
        self.twist_pub.publish(twist)

        # Log for debugging purposes
        self.get_logger().info(f"Published Twist: Linear=({twist.twist.linear.x}, {twist.twist.linear.y}, {twist.twist.linear.z}), "
                               f"Angular=({twist.twist.angular.x}, {twist.twist.angular.y}, {twist.twist.angular.z})")

def main(args=None):
    rclpy.init(args=args)
    node = AuraJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()




