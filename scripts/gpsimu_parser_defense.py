#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy
from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from math import pi

class GPSIMUParser:
    def __init__(self):
        rospy.init_node('GPS_IMU_parser', anonymous=True)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = '/odom'
        self.odom_msg.child_frame_id = '/base_link'

        self.is_imu = False
        self.is_gps = False
        
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.gps_callback)

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.is_imu and self.is_gps:
                self.odom_msg.header.stamp = rospy.Time.now()
                self.odom_pub.publish(self.odom_msg)
                rate.sleep()

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

    def convert_gps_to_sim(self, lat, lon):
        x = (1.74366642 * lat) + (111705.27046915 * lon) + (2.74602510)
        y = (110234.58048352 * lat) + (-23.11916045 * lon) + (-4.08750102)
        
        return x, y

    def gps_callback(self, gps_msg):
        unity_x, unity_y = self.convert_gps_to_sim(gps_msg.latitude, gps_msg.longitude)
        self.odom_msg.pose.pose.position.x = unity_x
        self.odom_msg.pose.pose.position.y = unity_y
        self.odom_msg.pose.pose.position.z = 0.0

        self.is_gps = True

if __name__ == '__main__':
    try:
        GPSIMUParser()
    except rospy.ROSInterruptException:
        pass
