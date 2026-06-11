#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from pyproj import Proj
from std_msgs.msg import Float32MultiArray
from morai_ros2_msgs.msg import GPSMessage

class GPSToUTM(Node):
    def __init__(self):
        super().__init__('GPS_to_UTM')
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.gps_sub = self.create_subscription(GPSMessage, "/gps", self.gps_callback, qos_profile)
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        self.utm_msg = Float32MultiArray()
        self.is_gps_data = False

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # os.system('clear')
        if not self.is_gps_data:
            print("[1] can't subscribe '/gps' topic... \n    please check your GPS sensor connection")

        self.is_gps_data = False

    def gps_callback(self, gps_msg):
        self.is_gps_data = True
        latitude = gps_msg.latitude
        longitude = gps_msg.longitude
        altitude = gps_msg.altitude
        
        utm_xy = self.proj_UTM(longitude, latitude)
        utm_x = utm_xy[0]
        utm_y = utm_xy[1]
        
        map_x = utm_x - gps_msg.east_offset
        map_y = utm_y - gps_msg.north_offset
        
        os.system('clear')
        print(f''' 
        ----------------[ GPS data ]----------------
            latitude    : {latitude}
            longitude   : {longitude}
            altitude    : {altitude}

                         |
                         | apply Projection (utm 52 zone)
                         V

        ------------------[ utm ]-------------------
              utm_x     : {utm_x}
              utm_y     : {utm_y}

                         |
                         | apply offset (east and north)
                         V
          
        ------------------[ map ]-------------------
        simulator map_x : {map_x}
        simulator map_y : {map_y}
        ''')


def main(args=None):
    rclpy.init(args=args)
    gps_to_utm = GPSToUTM()
    
    try:
        rclpy.spin(gps_to_utm)
    except KeyboardInterrupt:
        pass
    finally:
        gps_to_utm.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()