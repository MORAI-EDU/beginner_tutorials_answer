#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from morai_ros2_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from pyproj import Proj

class GPSIMUParser(Node):
    def __init__(self):
        super().__init__('GPS_IMU_parser')
        
        qos_profile = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.gps_sub = self.create_subscription(GPSMessage, "/gps", self.navsat_callback, qos_profile)
        self.imu_sub = self.create_subscription(Imu, "/Imu", self.imu_callback, qos_profile)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 1)
        
        self.x, self.y = None, None
        self.is_imu = False
        self.is_gps = False

        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = '/odom'
        self.odom_msg.child_frame_id = '/base_link'

        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        os.system('clear')
        if self.is_imu and self.is_gps:
            self.convertLL2UTM()
            self.odom_pub.publish(self.odom_msg)
            print(f"odom_msg is now being published at '/odom' topic!\n")
            print('-----------------[ odom_msg ]---------------------')
            print(self.odom_msg.pose)

        if not self.is_imu:
            print("[1] can't subscribe '/imu' topic... \n    please check your IMU sensor connection")
        if not self.is_gps:
            print("[2] can't subscribe '/gps' topic... \n    please check your GPS sensor connection")
        
        self.is_gps = False
        self.is_imu = False

    def navsat_callback(self, gps_msg):
        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude
        self.e_o = gps_msg.east_offset
        self.n_o = gps_msg.north_offset
        self.is_gps = True

    def convertLL2UTM(self):  
        xy_zone = self.proj_UTM(self.lon, self.lat)

        if self.lon == 0 and self.lat == 0:
            self.x = 0.0
            self.y = 0.0
        else:
            self.x = xy_zone[0] - self.e_o
            self.y = xy_zone[1] - self.n_o

        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0.0

    def imu_callback(self, data):
        if data.orientation.w == 0:
            self.odom_msg.pose.pose.orientation.x = 0.0
            self.odom_msg.pose.pose.orientation.y = 0.0
            self.odom_msg.pose.pose.orientation.z = 0.0
            self.odom_msg.pose.pose.orientation.w = 1.0
        else:
            self.odom_msg.pose.pose.orientation.x = data.orientation.x
            self.odom_msg.pose.pose.orientation.y = data.orientation.y
            self.odom_msg.pose.pose.orientation.z = data.orientation.z
            self.odom_msg.pose.pose.orientation.w = data.orientation.w

        self.is_imu = True

def main(args=None):
    rclpy.init(args=args)
    gps_imu_parser = GPSIMUParser()
    
    try:
        rclpy.spin(gps_imu_parser)
    except KeyboardInterrupt:
        pass
    finally:
        gps_imu_parser.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()