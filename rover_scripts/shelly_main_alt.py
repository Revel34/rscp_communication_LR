#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32,Empty
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray,String,Float32MultiArray
from geographiclib.geodesic import Geodesic
from archimedean_spiral import ArchimedeanSpiral
from circle_anon import CircleAnon
from rscp_helper import rscp_send
import math as mt
import time as tm
# from collections import deque
import sys
import argparse

class SimpleGPSController(Node):
    def __init__(self, radius):
        self.counter = 0
        super().__init__('gps_spiral_node')
        # if True waypoints are generated on archimedean spiral, False is boxy spiral.
        self.shell = False
        self.theta_step = mt.pi*(1/8)
        self.radius = radius
        print(f"Value of radius passed to node: {radius}")
        #for altitude controll
        self.single_saved_coordinates_buffer = [0.0,0.0,0.0]
        self.current_altitude = 0
        self.end_reached = False
        self.publish_highest_all_the_time = False
        # ANGULAR SPEED SIGN CHANGES ROTATION DIRECTION
        self.correction_angular_speed = 0.25
        self.max_heading_error_deg = 15.0
        self.linear_speed = 0.25
        self.reached_target_radius = 0.5 # in meters
        #WGS84
        self.origin_point_buffer = []
        self.geod = Geodesic.WGS84
        #initial radius 5m
        self.shelly_spiral = ArchimedeanSpiral(self.radius,self.reached_target_radius,1,1/(2*mt.pi),-(mt.pi/2), mt.pi/8)
        self.circle_anon_spiral = CircleAnon(self.radius,self.reached_target_radius,self.theta_step)
        self.r_azi_buffer = self.shelly_spiral.azi_theta_buffer() if self.shell else self.circle_anon_spiral.azi_theta_buffer()
        self.r_buffer = self.shelly_spiral.geo_r_buffer() if self.shell else self.circle_anon_spiral.geo_r_buffer()
        self.waypoint_index = -1
        self.shelly_flag = False
        self.max_r = 5.0
        self.r = 0.5
        self.target_latitude = 0.0
        self.target_longitude = 0.0
        self.current_latitude = 0.0
        self.current_longitude = 0.0
        self.current_heading = 0.0  # (0-360) degrees
        self.reached_goal = True
        # waypoints buffer waypoints_buffer[i]=[target_lat_i,target_lon_i]
        self.waypoints_buffer = []
        self.waypoint_count = 0

        self.info_string = "\n\n    To activate the shelly run:\
                                  \n        ros2 topic pub SHELLY/activate_shelly std_msgs/msg/Empty {} -1\
                                  \n    To reset it:\
                                  \n        ros2 topic pub SHELLY/reset_shelly std_msgs/msg/Empty {} -1\
                                  \n    To return to the post activation origin point:\
                                  \n        ros2 topic pub SHELLY/return_to_current_origin_point std_msgs/msg/Empty {} -1\
                                  \n    To switch shelly ON/OFF: \
                                  \n        ros2 topic pub SHELLY/switch_shelly_onoff std_msgs/msg/Empty {} -1\
                                  \n    To clear highest point buffer:\
                                  \n        ros2 topic pub SHELLY/clear_highest_buffer std_msgs/msg/Empty {} -1\
                                  \n    To switch always on highest point publisher ON/OFF: \
                                  \n        ros2 topic pub SHELLY/switch_highest_point_aon_pub std_msgs/msg/Empty {} -1\
                                  \n    Pub. topics: SHELLY/info -> debug info topic"
        self.string_msg = String()
        
        # SUBS
        self.create_subscription(NavSatFix, 'gps/fix', self.gps_callback, 10)
        self.create_subscription(Float32, 'heading', self.heading_callback, 10)
        self.create_subscription(Float32MultiArray,'SHELLY/shelly_spiral_radius_and_err',self.ShellyRadius, 10)
        self.create_subscription(Empty,'SHELLY/activate_shelly',self.ShellyGo, 10)
        self.create_subscription(Empty,'SHELLY/reset_shelly',self.ShellyStop, 10)
        self.create_subscription(Empty,'SHELLY/switch_shelly_onoff',self.ShellySwitch, 10)
        self.create_subscription(Empty,'SHELLY/return_to_current_origin_point',self.ReturnToOrigin, 10)
        self.create_subscription(Empty,'SHELLY/switch_highest_point_aon_pub',self.SwitchHPP, 10)
        self.create_subscription(Empty,'SHELLY/clear_highest_buffer',self.ClearHPB, 10)

        # self.create_subscription(Float32MultiArray,'SHELLY/cos',self.Radio, 10)
        # radius,target_reached_error,activate_shelly,reset_shelly,shelly_switch_on_off,highest_always_on,clear_highest_buffer
        # [5,0.5,0,0,0,0,0] len = 7 => dwa pierwsze to float34 a reszta 5 to po otrzymaniu wiadomosci o wartosci 1 sie aktywuje opcja

        # PUBS
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        # self.led_pub = self.create_publisher(Int32MultiArray, 'P32_GIZ/led_state_topic', 10)
        self.highest_place_pub = self.create_publisher(Float32MultiArray, 'SHELLY/highest_place_array', 10)

        self.info_pub = self.create_publisher(String, 'SHELLY/info', 10)
        # timer callback
        self.timer = self.create_timer(0.1, self.CheckSome)
        self.timer_info = self.create_timer(10, self.Info)
        self.get_logger().info(self.info_string)
    
    # def parse_radius(self,radius):
        # self.radius = radius

    # def Radio(self,msg):
    #     radio_menu = msg.data
    #     empty_msg_dummy = Empty()
    #     try:
    #         if (msg.data[0] != 0.0) and (msg.data[1] != 0.0):
    #             self.radius = msg.data[0]
    #             self.reached_target_radius = msg.data[1]
    #             self.shelly_spiral = ArchimedeanSpiral(self.radius,self.reached_target_radius,1,1/(2*mt.pi),-(mt.pi/2), mt.pi/8)
    #         elif (msg.data[0] == 0.0) and (msg.data[1] != 0.0):
    #             self.reached_target_radius = msg.data[1]
    #             self.shelly_spiral = ArchimedeanSpiral(self.radius,self.reached_target_radius,1,1/(2*mt.pi),-(mt.pi/2), mt.pi/8)
    #         elif (msg.data[0] != 0.0) and (msg.data[1] == 0.0):
    #             self.radius = msg.data[0]
    #             self.shelly_spiral = ArchimedeanSpiral(self.radius,self.reached_target_radius,1,1/(2*mt.pi),-(mt.pi/2), mt.pi/8)
    #     except IndexError:
    #         self.string_msg.data = "IndexError: Resend radio message: \"{data:[radius, target_reached_error, activate_shelly, reset_shelly, shelly_switch_on_off,highest_always_on, clear_highest_buffer]}\""
    #         self.info_pub.publish(self.string_msg)
    #         self.string_msg.data = "Radio message, sending 1 is pushing a button, 0 does nothing even for first two: \"{data:[float, float, 0/1, 0/1, 0/1, 0/1, 0/1]}\""
    #         self.info_pub.publish(self.string_msg)
    #     try:
    #         for i in range(2,len(radio_menu)):
    #             if i==2: #activate_shelly
    #                 if radio_menu[i]==1:
    #                     self.ShellyGo(empty_msg_dummy)
    #             if i==3: #reset_shelly
    #                 if radio_menu[i]==1:
    #                     self.ShellyStop(empty_msg_dummy)
    #             if i==4: #switch_shelly
    #                 if radio_menu[i]==1:
    #                     self.ShellySwitch(empty_msg_dummy)
    #             if i==5: #highest_pub_switch_shelly
    #                 if radio_menu[i]==1:
    #                     self.SwitchHPP(empty_msg_dummy)
    #             if i==6: #clear_highest_buffer
    #                 if radio_menu[i]==1:
    #                     self.ClearHPB(empty_msg_dummy)
    #     except IndexError:
    #         self.string_msg.data = "IndexError: Resend radio message: \"{data:[radius, target_reached_error, activate_shelly, reset_shelly, shelly_switch_on_off,highest_always_on, clear_highest_buffer]}\""
    #         self.info_pub.publish(self.string_msg)
    #         self.string_msg.data = "Radio message, sending 1 is pushing a button, 0 does nothing even for first two: \"{data:[float, float, 0/1, 0/1, 0/1, 0/1, 0/1]}\""
    #         self.info_pub.publish(self.string_msg)
    #     return
    
    def Info(self):
        if not self.shelly_flag:
            self.get_logger().info(self.info_string)
        return
    
    def ClearHPB(self,msg):
        self.single_saved_coordinates_buffer = [0.0,0.0,0.0]
        self.string_msg.data = f"HIGHEST POINT BUFFER CLEARED"
        self.info_pub.publish(self.string_msg)
        self.string_msg.data = f"single_saved_coordinates_buffer=[lat,lon,alt]: [{self.single_saved_coordinates_buffer[0]},{self.single_saved_coordinates_buffer[1]},{self.single_saved_coordinates_buffer[2]}]"
        self.info_pub.publish(self.string_msg)

        return

    def ShellyRadius(self,msg):
        try:
            self.reached_target_radius = msg.data[1]
            self.shelly_spiral = ArchimedeanSpiral(msg.data[0],msg.data[1],1,(3/2)/(2*mt.pi),-(mt.pi/2), mt.pi/8)
            self.string_msg.data = f"SET [RADIUS, REACHED TARGET RADIUS]: [{msg.data[0]},{msg.data[1]}]"
            self.info_pub.publish(self.string_msg)
        except IndexError:
            try:
                self.shelly_spiral = ArchimedeanSpiral(msg.data[0],self.reached_target_radius,1,(3/2)/(2*mt.pi),-(mt.pi/2), mt.pi/8)
                self.string_msg.data = f"SET [RADIUS, REACHED_TARGET_RADIUS]: [ {msg.data[0]} , {self.reached_target_radius} ]"
                self.info_pub.publish(self.string_msg)
            except IndexError:
                self.string_msg.data = "Data was not valid, resend either \"{data:[radius]}\" or \"{data:[radius,reached_target_radius]}\""
                self.info_pub.publish(self.string_msg)
        return

    def SwitchHPP(self,msg):
        if not self.publish_highest_all_the_time:
            self.publish_highest_all_the_time = True
        else:
            self.publish_highest_all_the_time = False
        return
    
    def ReturnToOrigin(self,msg):
        self.ResetShelly()
        if len(self.origin_point_buffer) == 0:
            self.string_msg.data = f"NO ORIGIN POINT SET - CAN'T RETURN TO NOTHING"
        else:
            self.waypoints_buffer.append(self.origin_point_buffer)
            self.string_msg.data = f"waypoint_buffer = origin_point => {self.waypoints_buffer}"
            self.shelly_flag =True
        self.info_pub.publish(self.string_msg)
        self.string_msg.data = f"RETURNING TO THE ORIGIN POINT..."
        self.info_pub.publish(self.string_msg)
        return
    
    def ShellyGo(self,msg):
        self.ResetShelly()
        self.shelly_flag = True
        self.end_reached = False
        self.waypoints_buffer = self.GenerateWaypoints()
        if self.shell:
            self.get_logger().info("\n\n ARCHIMEDEAN SPIRAL STARTED.\n")
        else:
            self.get_logger().info("\n\n BOXY SPIRAL STARTED.\n")
        str_msg ="SHELLY STARTED, WAYPOINTS:"
        self.string_msg.data = str_msg
        self.info_pub.publish(self.string_msg)
        self.get_logger().info(str_msg)
        for i in self.waypoints_buffer:
            self.string_msg.data ="{}: {}".format(self.waypoints_buffer.index(i),i)
            self.info_pub.publish(self.string_msg)
            tm.sleep(0.1)
        str_msg = f"waypoint_count = {self.waypoint_count}"
        self.string_msg.data = str_msg
        self.info_pub.publish(self.string_msg)
        str_msg = f"shelly_flag = {self.shelly_flag}"
        self.string_msg.data = str_msg
        self.info_pub.publish(self.string_msg)
        self.string_msg.data = f"origin_point has been saved => {self.origin_point_buffer}"
        self.info_pub.publish(self.string_msg)

        return
    
    def ResetShelly(self):
        self.shelly_flag = False
        self.end_reached = False
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        # Wyślij sygnał LED
        # led_msg = Int32MultiArray()
        # led_msg.data = [1, 0, 0]  # Czerwony LED
        # self.led_pub.publish(led_msg)
        self.waypoint_index = -1
        self.waypoint_count = 0
        self.reached_goal = True
        self.waypoints_buffer.clear()
        
        str_msg = "WAYPOINTS BUFFER CLEARED:"
        self.string_msg.data = str_msg
        self.info_pub.publish(self.string_msg)
        self.get_logger().info(str_msg)

        str_msg = f"shelly_flag = {self.shelly_flag}"
        self.string_msg.data = str_msg
        self.info_pub.publish(self.string_msg)

        str_msg = f"waypoint_buffer = {self.waypoints_buffer}"
        self.string_msg.data = str_msg
        self.info_pub.publish(self.string_msg)


        str_msg = f"waypoint_index = {self.waypoint_index}"
        self.string_msg.data = str_msg
        self.info_pub.publish(self.string_msg)

        str_msg = f"waypoint_count = {self.waypoint_count}"
        self.string_msg.data = str_msg
        self.info_pub.publish(self.string_msg)

        str_msg = f"cmd_vel linear:{cmd_vel.linear.x} angular:{cmd_vel.angular.z}"
        self.string_msg.data = str_msg
        self.info_pub.publish(self.string_msg)

        # str_msg = f"led_msg = {led_msg.data}"
        # self.string_msg.data = str_msg
        # self.info_pub.publish(self.string_msg)

        return
    
    def ShellySwitch(self,msg):
        if self.shelly_flag:
            self.shelly_flag = False
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
            str_msg = f"SHELLY HALTED..."
            self.string_msg.data = str_msg
            self.info_pub.publish(self.string_msg)
            str_msg = f"cmd_vel linear:{cmd_vel.linear.x} angular:{cmd_vel.angular.z}"
            self.string_msg.data = str_msg
            self.info_pub.publish(self.string_msg)
        else:
            self.shelly_flag = True
            str_msg = f"...SHELLY SWITCHED ON"
            self.string_msg.data = str_msg
            self.info_pub.publish(self.string_msg)

        return

    def ShellyStop(self,msg):
        # self.shelly_flag = False
        # self.waypoint_index = -1
        # self.waypoint_count = 0
        # self.reached_goal = True
        # self.waypoints_buffer.clear()
        # str_msg = " ".join('{}'.format(k) for k in self.waypoints_buffer)
        # str_msg = "\n\n WAYPOINTS BUFFER CLEARED:\n"+str_msg
        # self.string_msg.data = str_msg
        # self.info_pub.publish(self.string_msg)
        # self.get_logger().info(str_msg)
        self.ResetShelly()
        return

    def CheckSome(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        highest_place_msg = Float32MultiArray()

        if self.publish_highest_all_the_time:
            highest_place_msg.data = [self.single_saved_coordinates_buffer[0],
                                      self.single_saved_coordinates_buffer[1],
                                      self.single_saved_coordinates_buffer[2]]
            # publish at the end of the run
            self.highest_place_pub.publish(highest_place_msg)
            self.string_msg.data = f"[lat,lon,alt]:[{self.single_saved_coordinates_buffer[0]},{self.single_saved_coordinates_buffer[1]},{self.single_saved_coordinates_buffer[2]}]"
            self.info_pub.publish(self.string_msg)

        if self.end_reached:
            highest_place_msg.data = [self.single_saved_coordinates_buffer[0],
                                      self.single_saved_coordinates_buffer[1],
                                      self.single_saved_coordinates_buffer[2]]
            # publish at the end of the run
            # self.highest_place_pub.publish(highest_place_msg)
            # self.string_msg.data = f"[lat,lon,alt]:[{self.single_saved_coordinates_buffer[0]},{self.single_saved_coordinates_buffer[1]},{self.single_saved_coordinates_buffer[2]}]"
            # self.info_pub.publish(self.string_msg)
            #--------------Send to rscp----------------------------#
            rscp_send("gps",latitude=self.single_saved_coordinates_buffer[0],longitude=self.single_saved_coordinates_buffer[1])
            #------------------------------------------------------#
            sys.exit()
        
        if self.shelly_flag:
            if  self.reached_goal:
                # get next waypoint
                self.waypoint_index += 1
                # unpack waypoints buffer waypoints_buffer[i]=[target_lat_i,target_lon_i]
                self.target_latitude = self.waypoints_buffer[self.waypoint_index][0]
                self.target_longitude = self.waypoints_buffer[self.waypoint_index][1]
                self.reached_goal = False 
            else:
                g1 = self.geod.Inverse(
                    self.current_latitude,
                    self.current_longitude,
                    self.target_latitude,
                    self.target_longitude
                    )
                distance = g1['s12']
                if (distance < self.reached_target_radius):
                    # if the last waypoint is reached, turn off the shelly.
                    if self.waypoints_buffer[self.waypoint_index] == self.waypoints_buffer[-1]:
                        self.ResetShelly()
                        self.get_logger().info(f"\n\n TARGET LOCATION WITHIN: {g1['s12']}m,\
                            \n TARGET REACHED RADIUS: {self.reached_target_radius}m.\
                            \n STATUS: LAST TARGET REACHED - CLOSING SHELLY LOOP\n")
                        str_msg = f"LAST TARGET REACHED -> SHELLY RESET"
                        self.string_msg.data = str_msg
                        self.info_pub.publish(self.string_msg)
                        self.end_reached = True
                    else:
                        self.get_logger().info(f"\n\n TARGET LOCATION WITHIN: {g1['s12']}m,\
                                               \n TARGET REACHED RADIUS: {self.reached_target_radius}m.\
                                               \n STATUS: TARGET REACHED - MOVING TO NEXT ONE\n")
                        str_msg = f"TARGET REACHED -> MOVING TO NEXT ONE"
                        self.string_msg.data = str_msg
                        self.info_pub.publish(self.string_msg)

                        
                        self.reached_goal = True
                        # STOP ROVER
                        cmd_vel.linear.x = 0.0
                        cmd_vel.angular.z = 0.0
                        self.cmd_vel_pub.publish(cmd_vel)
                        # RED LED SIGNAL
                        # led_msg = Int32MultiArray()
                        # led_msg.data = [1, 0, 0]  # RED
                        # self.led_pub.publish(led_msg) 
                else:
                    self.get_logger().info(f"\n\n TARGET LOCATION WITHIN: {g1['s12']}m,\
                                           \n TARGET REACHED RADIUS: {self.reached_target_radius}m.\
                                           \n STATUS: TARGET NOT REACHED\n")
                    bearing = g1['azi1']
                    heading_error = (bearing - self.current_heading + 720) % 360
                    if heading_error > 180:
                        heading_error -= 360
                    if abs(heading_error) > self.max_heading_error_deg:  # Jeśli różnica większa niż 45 stopni, skręć
                        cmd_vel.angular.z = self.correction_angular_speed if heading_error < 0 else -self.correction_angular_speed
                        cmd_vel.linear.x = 0.0
                    else:  # Jedź prosto
                        cmd_vel.linear.x = self.linear_speed  # Stała prędkość do przodu
                        cmd_vel.angular.z = 0.0
                    self.cmd_vel_pub.publish(cmd_vel)
                    self.get_logger().info(f"\n\n BEARING: {bearing:.2f},\
                                            \n HEADING: {self.current_heading:.2f},\
                                            \n HEADING ERROR: {heading_error:.2f}\n")
        return
    # def GenerateShellyWaypoints(self):
        # wp_buffer = []
        # o_lat = self.current_latitude
        # o_lon = self.current_longitude
        # for i in range(len(self.r_buffer)):
            # bearing = (self.current_heading + self.r_azi_buffer[i] + 360) % 360
            # wp_buffer.append(
                # self.GetTargetCoords(
                    # o_lat, o_lon,  
                    # bearing, 
                    # self.r_buffer[i]
                    # )
                # )
        # return wp_buffer

    def GenerateWaypoints(self):
        wp_buffer = []
        self.origin_point_buffer = [self.current_latitude,self.current_longitude]
        for i in range(len(self.r_buffer)):
            bearing = (self.current_heading + self.r_azi_buffer[i] + 360) % 360
            wp_buffer.append(
                self.GetTargetCoords(
                    self.origin_point_buffer[0], self.origin_point_buffer[1],  
                    bearing, 
                    self.r_buffer[i]
                    )
                )
        return wp_buffer
    
    def GetTargetCoords(self, lat1, lon1, azi , r):
        self.waypoint_count += 1
        waypoint_buffer_tmp = []
        g4 = self.geod.Direct(
            lat1,
            lon1,
            azi,
            r
            )
        waypoint_buffer_tmp.append(g4['lat2'])
        waypoint_buffer_tmp.append(g4['lon2'])
        return waypoint_buffer_tmp
    
    def gps_callback(self, msg):
        self.current_latitude = msg.latitude
        self.current_longitude = msg.longitude
        self.current_altitude = msg.altitude
        
        #save highest point
        if self.current_altitude > self.single_saved_coordinates_buffer[2]:
            self.single_saved_coordinates_buffer[0] = self.current_latitude
            self.single_saved_coordinates_buffer[1] = self.current_longitude
            self.single_saved_coordinates_buffer[2] = self.current_altitude


        
    def heading_callback(self, msg):
        self.current_heading = msg.data
        
def main(args=None):
    #_______________________
    parser = argparse.ArgumentParser(
        description="Shelly script with 1 arg=radius in float"
    )
    parser.add_argument("-r","--radius", default=5, type=float, help="Radius arg in meters")
    anon_args = parser.parse_args()    
    #------------------------
    rclpy.init(args=args)
    controller = SimpleGPSController(radius=anon_args.radius)
    controller.ShellyGo(Empty())
    rclpy.spin(controller)
    while rclpy.ok():
        pass
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
