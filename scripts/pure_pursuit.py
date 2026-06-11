#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from math import cos, sin, sqrt, pow, atan2
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from morai_ros2_msgs.msg import CtrlCmd
import numpy as np
from tf_transformations import euler_from_quaternion

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        self.path_sub = self.create_subscription(Path, "local_path", self.path_callback, qos_profile)
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_callback, qos_profile)
        self.ctrl_cmd_pub = self.create_publisher(CtrlCmd, 'ctrl_cmd', 1)
        
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longl_cmd_type = 2

        self.is_path = False
        self.is_odom = False

        self.forward_point = Point()
        self.current_position = Point()
        self.is_look_forward_point = False
        self.vehicle_length = 4.0
        self.lfd = 5.0
        self.max_steer = 32.51 * 3.14 / 180.0
        
        self.vehicle_yaw = 0.0
        self.path = Path()

        self.timer = self.create_timer(1.0 / 15.0, self.timer_callback)

    def timer_callback(self):
        if self.is_path and self.is_odom:
            vehicle_position = self.current_position
            self.is_look_forward_point = False

            translation = [vehicle_position.x, vehicle_position.y]

            # Global to Local 좌표 변환 행렬
            t = np.array([
                [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
                [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
                [0, 0, 1]
            ])

            det_t = np.array([
                [t[0][0], t[1][0], -(t[0][0]*translation[0] + t[1][0]*translation[1])],
                [t[0][1], t[1][1], -(t[0][1]*translation[0] + t[1][1]*translation[1])],
                [0, 0, 1]
            ])

            local_path_point = [0, 0, 1] # 초기화
            
            for num, i in enumerate(self.path.poses):
                path_point = i.pose.position

                global_path_point = [path_point.x, path_point.y, 1]
                local_path_point = det_t.dot(global_path_point)            
                if local_path_point[0] > 0:
                    dis = sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2))
                    if dis >= self.lfd:
                        self.forward_point = path_point
                        self.is_look_forward_point = True
                        break

            theta = atan2(local_path_point[1], local_path_point[0])

            if self.is_look_forward_point:
                # Pure Pursuit 공식 적용
                self.ctrl_cmd_msg.front_steer = float(atan2((2 * self.vehicle_length * sin(theta)), self.lfd))
                self.ctrl_cmd_msg.velocity = 15.0

                os.system('clear')
                print("-------------------------------------")
                print(" steering (deg) = ", self.ctrl_cmd_msg.front_steer * 180 / 3.14)
                print(" velocity (kph) = ", self.ctrl_cmd_msg.velocity)
                print("-------------------------------------")

                # 시뮬레이터 정책에 따라 steering_angle 정규화
                self.ctrl_cmd_msg.front_steer = -self.ctrl_cmd_msg.front_steer / self.max_steer
            else:
                # os.system('clear')
                print("no found forward point")
                self.ctrl_cmd_msg.front_steer = 0.0
                self.ctrl_cmd_msg.velocity = 0.0

            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

        else:
            # os.system('clear')
            if not self.is_path:
                print("[1] can't subscribe '/local_path' topic...")
            if not self.is_odom:
                print("[2] can't subscribe '/odom' topic...")

        self.is_path = False
        self.is_odom = False

    def path_callback(self, msg):
        self.is_path = True
        self.path = msg  

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y

def main(args=None):
    rclpy.init(args=args)
    test_track = PurePursuit()
    
    try:
        rclpy.spin(test_track)
    except KeyboardInterrupt:
        pass
    finally:
        test_track.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()