#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8MultiArray  # Added import for button states
import serial

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # Publisher for robot velocity commands - required by publish_cmd_vel method
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        
        # Publisher for button states - required by publish_button_states method
        self.button_publisher = self.create_publisher(Int8MultiArray, '/ESP32_GIZ/led_state_topic', 10)
        
        # Configuration variables used by publish_cmd_vel method
        self.communication_mode = 'ROS2'  # Communication mode: 'ROS2' or 'SATEL'
        self.serial_port = None           # Serial port for SATEL mode (None = not configured)
        self.speed_factor = 1.0           # Global speed multiplier (0.0 to 1.0)
        self.max_linear_speed = 1.0       # Maximum linear velocity in m/s
        self.max_angular_speed = 1.0      # Maximum angular velocity in rad/s
        
        # Timer for demonstration - calls demo every 1 second
        self.create_timer(0.03, self.demo_callback)
        
        self.get_logger().info('Control Node started')
    
    def publish_cmd_vel(self, axes=None, buttons=None):
        """
        Main method - converts gamepad input to robot velocity commands.
        
        Args:
            axes: List of gamepad axis values (triggers, sticks)
            buttons: List of gamepad button states (0 or 1)
        """
        # Safety check: if no input data, send zero velocity (stop robot)
        if axes is None or buttons is None:
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            if self.communication_mode == 'ROS2':
                self.cmd_vel_publisher.publish(twist_msg)
            elif self.communication_mode == 'SATEL':
                self.send_serial_frame("DV", 128, 128)  # 128 = neutral value
            return

        # Validate input data - need at least 6 axes and 6 buttons
        if len(axes) < 6 or len(buttons) < 6:
            self.get_logger().error("Za mało danych z gamepada!")
            return

        # Check reverse mode buttons (shoulder buttons 4 and 5)
        reverse_mode_left = buttons[5]   # Left shoulder button
        reverse_mode_right = buttons[4]  # Right shoulder button

        # Process trigger values: convert from [-1,1] to [0,1] and apply reverse mode
        left_trigger = (axes[5] + 1) / 2 * (-1 if reverse_mode_left else 1)
        right_trigger = (axes[2] + 1) / 2 * (-1 if reverse_mode_right else 1)

        # Calculate maximum trigger value (unused but kept from original)
        max_trigger = max(left_trigger, right_trigger)

        # Create velocity command message
        twist_msg = Twist()
        # Linear velocity: average of both triggers for forward/backward motion
        twist_msg.linear.x = self.max_linear_speed * (left_trigger + right_trigger) / 2 * self.speed_factor
        # Angular velocity: difference between triggers for turning motion
        twist_msg.angular.z = self.max_angular_speed * (left_trigger - right_trigger) / 2 * self.speed_factor

        # Convert velocity values to bytes for serial communication
        x_byte = self.float_to_byte(twist_msg.linear.x)
        z_byte = self.float_to_byte(twist_msg.angular.z)

        # Send command based on communication mode
        if self.communication_mode == 'ROS2':
            self.cmd_vel_publisher.publish(twist_msg)
        elif self.communication_mode == 'SATEL':
            self.send_serial_frame("DV", x_byte, z_byte)
    
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
            msg.data = [0, 0, 1]  # Hardcoded values as requested
            self.button_publisher.publish(msg)
    
    def float_to_byte(self, value):
        """
        Convert float value from [-1.0, 1.0] range to byte [0, 254].
        Used for SATEL serial communication protocol.
        
        Args:
            value: Float value to convert
            
        Returns:
            int: Byte value (0-254)
        """
        # Clamp value to valid range
        value = max(-1.0, min(1.0, value))
        # Map [-1,1] to [1,255] with 128 as neutral
        return int((value*127.0)+128)
    
    def send_serial_frame(self, mark, *bytes):
        """
        Send data frame via serial port for SATEL communication mode.
        Frame format: $[MARK][DATA_BYTES][CHECKSUM]#
        
        Args:
            mark: Two-character command identifier (e.g., "DV" for drive)
            *bytes: Variable number of data bytes to send
        """
        # Skip if no serial port configured
        if self.serial_port is None:
            return
            
        try:
            # Calculate checksum as sum of data bytes modulo 256
            sum_val = sum(bytes)
            checksum = sum_val % 256

            # Build frame: start marker + command + data + checksum + end marker
            frame = bytearray()
            frame.extend(b"$")                    # Start marker
            frame.extend(mark.encode('utf-8'))    # Command identifier
            for byte in bytes:                    # Data bytes
                frame.append(byte)
            frame.append(checksum)                # Checksum
            frame.extend(b"#")                    # End marker

            # Send frame via serial port
            self.serial_port.write(frame)
            
        except Exception as e:
            self.get_logger().error(f"Serial frame error: {e}")
    
    def demo_callback(self):
        """
        Demonstration callback that shows both methods working.
        Creates sample gamepad input data and calls both methods.
        """
        # Sample gamepad input: simulates right trigger at 50% and left trigger at -30%
        sample_axes = [0.0, 0.0, -1.0, 0.0, 0.0, -1.0]  # axes[2]=right_trigger, axes[5]=left_trigger
        sample_buttons = [0, 0, 0, 0, 0, 0]              # No buttons pressed (no reverse mode)
        
        # Call the main velocity method from Snippet.txt
        self.publish_cmd_vel(sample_axes, sample_buttons)
        
        # Call the button states method (parameters are ignored due to hardcoded values)
        self.publish_button_states(kill_switch=0, autonomy=0, manual=1)
        
        # Log for demonstration
        self.get_logger().info('Demo: Publishing velocity command and button states')

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