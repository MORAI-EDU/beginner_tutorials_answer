#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion

class IMUParser(Node):
    def __init__(self):
        super().__init__('imu')
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        self.image_sub = self.create_subscription(Imu, "/Imu", self.callback, qos_profile)
        self.is_imu = False
        
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        os.system('clear')
        if not self.is_imu:
            print("[1] can't subscribe '/imu' topic... \n    please check your IMU sensor connection")

        self.is_imu = False

    def callback(self, data):
        self.is_imu = True
        quaternion = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        
        roll_deg = roll / math.pi * 180
        pitch_deg = pitch / math.pi * 180
        yaw_deg = yaw / math.pi * 180

        os.system('clear')
        print(f'''
        --------------[ IMU data ]---------------
             Roll  (deg) = {roll_deg}
             Pitch (deg) = {pitch_deg}
             Yaw   (deg) = {yaw_deg}
        -----------------------------------------
        ''')
        self.prev_time = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)
    imu_parser = IMUParser()
    
    try:
        rclpy.spin(imu_parser)
    except KeyboardInterrupt:
        pass
    finally:
        imu_parser.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()