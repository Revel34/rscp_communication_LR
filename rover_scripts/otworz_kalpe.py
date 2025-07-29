import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import time

#!/usr/bin/env python3


class ServoAnglePublisher(Node):
    def __init__(self):
        super().__init__('servo_angle_publisher')
        self.publisher_ = self.create_publisher(
            Int32MultiArray, 
            '/ESP32_GIZ/servo_angles_topic', 
            10
        )
        
    def publish_angles(self):
        msg = Int32MultiArray()
        msg.data = [84, 90, 90, 180]
        
        start_time = time.time()
        while time.time() - start_time < 5.0:
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: {msg.data}')
            time.sleep(0.1)  # Publikuj co 100ms

def main(args=None):
    rclpy.init(args=args)
    
    node = ServoAnglePublisher()
    node.publish_angles()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()