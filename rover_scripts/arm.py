#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
import serial

class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')
        
        # Publisher for button states - required by publish_button_states method
        self.button_publisher = self.create_publisher(Int8MultiArray, '/ESP32_GIZ/led_state_topic', 10)
        
        # Configuration variables
        self.communication_mode = 'ROS2'  # Communication mode: 'ROS2' or 'SATEL'
        self.serial_port = None  # Serial port for SATEL mode (None = not configured)
        
        self.get_logger().info('Control Node started')
        
        # Store timer reference and call publish_button_states once after a short delay
        self.timer = self.create_timer(0.1, self.one_time_publish)

    def one_time_publish(self):
        """
        One-time callback to publish button states and then destroy the timer.
        """
        # Call the button states method once
        self.publish_button_states(kill_switch=1, autonomy=0, manual=0)
        self.publish_button_states(kill_switch=1, autonomy=0, manual=0)
        self.publish_button_states(kill_switch=1, autonomy=0, manual=0)
        self.publish_button_states(kill_switch=1, autonomy=0, manual=0)
        self.publish_button_states(kill_switch=1, autonomy=0, manual=0)
        
        
        self.get_logger().info('Published button states once')
        
        # Destroy the timer so it doesn't repeat
        self.timer.destroy()

    def publish_button_states(self, kill_switch, autonomy, manual):
        """
        Publishes robot state information.
        Modified to send hardcoded values [0, 0, 1].
        
        Args:
            kill_switch: Kill switch state (0 or 1)
            autonomy: Autonomy mode state (0 or 1)
            manual: Manual control state (0 or 1)
        """
        if self.communication_mode == 'ROS2':
            msg = Int8MultiArray()
            msg.data = [1, 0, 0]  # Hardcoded values as requested
            self.button_publisher.publish(msg)

def main(args=None):
    """
    Main function - initializes ROS2 and runs the node.
    Executes immediately when script is run.
    """
    # Initialize ROS2 communication
    rclpy.init(args=args)
    
    # Create the control node instance
    node = ControlNode()
    
    try:
        # Run the node (blocking call - handles callbacks and timers)
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        node.get_logger().info('Shutting down...')
    finally:
        # Clean shutdown sequence
        node.destroy_node()
        rclpy.shutdown()

# Entry point - runs when script is executed directly
if __name__ == '__main__':
    main()
