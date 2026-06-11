#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory

from morai_ros2_msgs.msg import EgoVehicleStatus, CtrlCmd
from nav_msgs.msg import Path


class PurePursuit(Node):
    def __init__(self):
        super().__init__('lane_follower')
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        self.status_sub = self.create_subscription(
            EgoVehicleStatus, 
            "/ego_vehicle_status", 
            self.status_callback, 
            qos_profile
        )
        
        self.lpath_sub = self.create_subscription(
            Path, 
            '/lane_path', 
            self.lane_path_callback, 
            qos_profile
        )
        
        self.cmd_pub = self.create_publisher(CtrlCmd, '/ctrl_cmd', 1)
        
        self.is_status = False
        self.is_lpath = False
        
        self.is_look_forward_point = False
        self.vehicle_length = 2.0
        self.lfd = 20.0
        self.min_lfd = 2.0
        self.max_lfd = 50.0
        self.max_steer = 32.51 * 3.14 / 180.0

        self.lpath = None
        self.ctrl_msg = CtrlCmd()
        self.current_vel = 0.0

        # 10Hz 제어 루프
        self.timer = self.create_timer(1.0 / 10.0, self.timer_callback)

    def timer_callback(self):
        os.system('clear')
        
        if self.lpath:            
            self.calc_acc(20.0 / 3.6)    
            self.steering_angle()
            self.cmd_pub.publish(self.ctrl_msg)
            
        if self.is_status and self.is_lpath:
            print(f'''
                   lane_follower is processing...
                -------------------------------------
                   accel      : {self.ctrl_msg.accel}
                   brake      : {self.ctrl_msg.brake}
                   steering   : {self.ctrl_msg.front_steer}
                   velocity   : {self.ctrl_msg.velocity}
                   acceleration : {self.ctrl_msg.acceleration}
                ''')
            print('if nothing happens... please check [F4] Cmd Control Network')
        else:
            if not self.is_status:
                print("[1] can't subscribe '/ego_vehicle_status' topic... \n    please check connection")
            if not self.is_lpath:
                print("[2] can't subscribe '/lane_path' topic... \n    need lane_fitting.py")

        self.is_status = False
        self.is_lpath = False

    def status_callback(self, data):
        self.is_status = True
        self.current_vel = data.velocity.x

    def lane_path_callback(self, msg):
        self.is_lpath = True
        self.lpath = msg

    def steering_angle(self):
        self.is_look_forward_point = False
        
        # path_point를 초기화하여 루프가 돌지 않았을 때 발생할 수 있는 에러 방지
        path_point = None

        for i in self.lpath.poses:
            path_point = i.pose.position
            if path_point.x > 0:
                dis_i = np.sqrt(np.square(path_point.x) + np.square(path_point.y))
                if dis_i >= self.lfd:
                    self.is_look_forward_point = True
                    break
        
        # path_point가 유효한 경우에만 조향각 계산
        if path_point is not None:
            theta = math.atan2(path_point.y, path_point.x)

            if self.is_look_forward_point:
                steering = math.atan2((2 * self.vehicle_length * math.sin(theta)), self.lfd)
                self.ctrl_msg.front_steer = -float(steering) / self.max_steer
            else: 
                self.ctrl_msg.front_steer = 0.0
                print("no found forward point")
        else:
            self.ctrl_msg.front_steer = 0.0

    def calc_acc(self, target_vel):
        err = target_vel - self.current_vel
        control_input = 1.0 * err

        if control_input > 0:
            self.ctrl_msg.accel = float(control_input)
            self.ctrl_msg.brake = 0.0
        else:
            self.ctrl_msg.accel = 0.0
            self.ctrl_msg.brake = float(-control_input)


def main(args=None):
    rclpy.init(args=args)
    
    # 패키지 경로를 가져와서 json 파일 로드 
    # (원본 코드에 있었으나 실제 사용은 되지 않고 있습니다. 유지해 둡니다.)
    current_path = get_package_share_directory('beginner_tutorials')
    json_path = os.path.join(current_path, 'sensor', 'sensor_params.json')
    
    with open(json_path, 'r') as fp:
        sensor_params = json.load(fp)

    lane_follower = PurePursuit()
    
    try:
        rclpy.spin(lane_follower)
    except KeyboardInterrupt:
        pass
    finally:
        lane_follower.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()