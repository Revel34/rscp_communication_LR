#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32,Empty
from geometry_msgs.msg import Twist
from geographiclib.geodesic import Geodesic
import argparse
import sys
from rscp_helper import rscp_send

class SimpleGPSController(Node):
    def __init__(self, target_lat=None, target_lon=None, move_distance=4.0, spiral_radius=2.0):
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
        self.spiral_radius = spiral_radius
        self.spiral_points = []
        self.current_spiral_index = 0
        self.in_spiral_mode = False
        self.spiral_center_lat = None
        self.spiral_center_lon = None
        self.single_saved_coordinates_buffer = [None, None]
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
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
    
    def create_spiral_points(self, center_lat, center_lon, radius):
        """Create 10 spiral points around a center point"""
        spiral_points = []
        min_radius = 2.0  # Minimalna odległość spirali to 2 metry
        
        for i in range(10):
            # Zwiększający się promień spirali od 2m do maksymalnego promienia
            current_radius = min_radius + (radius - min_radius) * (i / 9.0)
            # Kąt dla każdego punktu (36 stopni * i, tworząc spiralę)
            angle = 36.0 * i
            
            # Oblicz współrzędne punktu używając geod.Direct
            point = self.geod.Direct(center_lat, center_lon, angle, current_radius)
            spiral_points.append((point['lat2'], point['lon2']))
            
            # Zapisz współrzędne 5. punktu (indeks 4)
            if i == 4:
                self.single_saved_coordinates_buffer[0] = point['lat2']
                self.single_saved_coordinates_buffer[1] = point['lon2']
                self.get_logger().info(f"Saved 5th spiral point coordinates: {point['lat2']:.8f}, {point['lon2']:.8f}")
            
        return spiral_points

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
        
        if self.in_spiral_mode:
            # Tryb spirali - nawiguj do kolejnych punktów spirali
            if self.current_spiral_index < len(self.spiral_points):
                current_target_lat, current_target_lon = self.spiral_points[self.current_spiral_index]
                
                g1 = self.geod.Inverse(self.current_latitude, self.current_longitude, current_target_lat, current_target_lon)
                distance = g1['s12']
                
                if distance < self.reached_target_radius:
                    self.get_logger().info(f"Reached spiral point {self.current_spiral_index + 1}/10")
                    self.current_spiral_index += 1
                    
                    if self.current_spiral_index >= len(self.spiral_points):
                        self.get_logger().info("Completed entire spiral! Mission finished.")
                        
                        # Wyślij współrzędne 5. punktu spirali przez RSCP
                        if self.single_saved_coordinates_buffer[0] is not None and self.single_saved_coordinates_buffer[1] is not None:
                            coordinates_string = str(self.single_saved_coordinates_buffer[0]) + str(self.single_saved_coordinates_buffer[1])
                            rscp_send('gps', latitude=self.single_saved_coordinates_buffer[0], longitude=self.single_saved_coordinates_buffer[1])
                            self.get_logger().info(f"Sent 5th spiral point coordinates via RSCP: {coordinates_string}")
                        
                        self.reached_goal = True
                        cmd_vel.linear.x = 0.0
                        cmd_vel.angular.z = 0.0
                        self.cmd_vel_pub.publish(cmd_vel)
                        return
                else:
                    # Nawiguj do obecnego punktu spirali
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
                    self.get_logger().info(f"[SPIRAL] Point {self.current_spiral_index + 1}/10, Distance: {distance:.2f}m")
            return
        
        if not self.reached_goal:
            g1 = self.geod.Inverse(self.current_latitude, self.current_longitude, self.target_latitude, self.target_longitude)
            distance = g1['s12']
            if (distance < self.reached_target_radius):  
                self.get_logger().info(f"Current target location within {self.reached_target_radius} m. Starting spiral pattern.")
                
                # Zapisz centrum spirali
                self.spiral_center_lat = self.target_latitude
                self.spiral_center_lon = self.target_longitude
                
                # Utwórz punkty spirali
                self.spiral_points = self.create_spiral_points(
                    self.spiral_center_lat, 
                    self.spiral_center_lon, 
                    self.spiral_radius
                )
                
                self.get_logger().info(f"Created spiral with {len(self.spiral_points)} points, radius: {self.spiral_radius}m")
                
                # Wyloguj wszystkie punkty spirali
                for i, (lat, lon) in enumerate(self.spiral_points):
                    self.get_logger().info(f"Spiral point {i+1}: lat={lat:.8f}, lon={lon:.8f}")
                
                # Przejdź w tryb spirali
                self.in_spiral_mode = True
                self.current_spiral_index = 0
                
                # Zatrzymaj się na chwilę
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd_vel)
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
    parser = argparse.ArgumentParser(description='Move robot to a GPS point or 2 meters straight ahead, then create spiral pattern.')
    parser.add_argument('--lat', type=float, help='Target latitude')
    parser.add_argument('--lon', type=float, help='Target longitude')
    parser.add_argument('--distance', type=float, default=4.0, help='Distance to move straight if no coordinates given (meters)')
    parser.add_argument('--spiral-radius', type=float, default=5.0, help='Radius of the spiral pattern (meters)')
    parsed_args, unknown = parser.parse_known_args()
    
    rclpy.init(args=args)
    controller = SimpleGPSController(
        target_lat=parsed_args.lat, 
        target_lon=parsed_args.lon, 
        move_distance=parsed_args.distance,
        spiral_radius=parsed_args.spiral_radius
    )
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
