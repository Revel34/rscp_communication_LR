#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32,Empty
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from geographiclib.geodesic import Geodesic
import argparse
import sys

class SimpleGPSController(Node):
    def __init__(self, target_lat=None, target_lon=None, move_distance=4.0):
        super().__init__('gps_point_node')
        self.geod = Geodesic.WGS84
        self.correction_angular_speed = 0.25
        self.max_heading_error_deg = 10.0
        self.linear_speed = 0.3
        self.reached_target_radius = 0.5 # in meters
        self.target_latitude = target_lat
        self.target_longitude = target_lon
        self.current_latitude = 0.0
        self.current_longitude = 0.0
        self.current_heading = 0.0  # in deg (0-360)
        self.reached_goal = False
        self.move_distance = move_distance
        self.target_set = False
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.led_pub = self.create_publisher(Int32MultiArray, 'P32_GIZ/led_state_topic', 10)
        self.create_subscription(NavSatFix, 'gps/fix', self.gps_callback, 10)
        self.create_subscription(Float32, 'heading', self.heading_callback, 10)
        self.timer = self.create_timer(0.1, self.CheckSome)

    def set_target_point(self):
        if self.target_latitude is not None and self.target_longitude is not None:
            self.get_logger().info(f"Target coordinates set from args: [{self.target_latitude}, {self.target_longitude}]")
            self.target_set = True
        else:
            # Move N meters straight ahead from current position
            g3 = self.geod.Direct(
                self.current_latitude,
                self.current_longitude,
                self.current_heading,
                self.move_distance
            )
            self.target_latitude = g3['lat2']
            self.target_longitude = g3['lon2']
            self.get_logger().info(f"No coordinates given, moving {self.move_distance} meters straight to [{self.target_latitude}, {self.target_longitude}]")
            self.target_set = True

    def CheckSome(self):
        if not self.target_set and (self.current_latitude != 0.0 and self.current_longitude != 0.0 and self.current_heading != 0.0):
            self.set_target_point()
            self.get_logger().info("Setting target point...")
            return
        if not self.target_set:
            self.get_logger().info("Waiting for GPS fix...")
            return
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        if not self.reached_goal:
            g1 = self.geod.Inverse(self.current_latitude, self.current_longitude, self.target_latitude, self.target_longitude)
            distance = g1['s12']
            if (distance < self.reached_target_radius):  
                self.get_logger().info(f"Current target location within {self.reached_target_radius} m. Stopping and exiting.")
                self.reached_goal = True
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd_vel)
                led_msg = Int32MultiArray()
                led_msg.data = [1, 0, 0]  # Czerwony LED
                self.led_pub.publish(led_msg)
                self.get_logger().info("Goal reached, will shutdown from main loop.")
            else:
                self.get_logger().info("Dist. to target: [{} m].".format(g1['s12']))
                desired_course = g1['azi1']
                heading_error = (desired_course - self.current_heading + 360) % 360
                if heading_error > 180:
                    heading_error -= 360
                if abs(heading_error) > self.max_heading_error_deg:
                    cmd_vel.angular.z = self.correction_angular_speed if heading_error < 0 else -self.correction_angular_speed
                    cmd_vel.linear.x = 0.0
                else:
                    cmd_vel.linear.x = self.linear_speed
                    cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd_vel)
                self.get_logger().info(f"[NAV] Desired course: {desired_course:.2f}°, Current heading: {self.current_heading:.2f}°, Heading error: {heading_error:.2f}°")
        return
    
    def gps_callback(self, msg):
        self.current_latitude = msg.latitude
        self.current_longitude = msg.longitude
        
    def heading_callback(self, msg):
        self.current_heading = msg.data
        
def main(args=None):
    parser = argparse.ArgumentParser(description='Move robot to a GPS point or 2 meters straight ahead.')
    parser.add_argument('--lat', type=float, help='Target latitude')
    parser.add_argument('--lon', type=float, help='Target longitude')
    parser.add_argument('--distance', type=float, default=4.0, help='Distance to move straight if no coordinates given (meters)')
    parsed_args, unknown = parser.parse_known_args()
    rclpy.init(args=args)
    controller = SimpleGPSController(target_lat=parsed_args.lat, target_lon=parsed_args.lon, move_distance=parsed_args.distance)
    try:
        while rclpy.ok() and not controller.reached_goal:
            rclpy.spin_once(controller, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    controller.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

if __name__ == '__main__':
    main()
