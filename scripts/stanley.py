#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
from math import cos, sin, pi, sqrt, pow, atan2
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from morai_ros2_msgs.msg import CtrlCmd, EgoVehicleStatus
from tf_transformations import euler_from_quaternion


class StanleyController(Node):
    def __init__(self):
        super().__init__('stanly_controller')
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        self.path_sub = self.create_subscription(Path, "local_path", self.path_callback, qos_profile)
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_callback, qos_profile)
        self.status_sub = self.create_subscription(EgoVehicleStatus, "ego_vehicle_status", self.status_callback, qos_profile)
        
        self.ctrl_cmd_pub = self.create_publisher(CtrlCmd, 'ctrl_cmd', 1)
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longl_cmd_type = 2
        
        self.is_path = False
        self.is_odom = False
        self.is_status = False
        
        self.forward_point = Point()
        self.current_position = Point()
        self.target_vel = 15.0
        self.current_vel = 0.0
        self.is_look_forward_point = False
        self.k = 0.9 # Stanley 제어 게인 (Stanley constant)
        self.v_t = 1.0 # 횡오차 계산 시 분모가 0이 되는 것을 방지하기 위한 속도 상수
        self.max_steer = 32.51 * 3.14 / 180.0
        
        self.vehicle_yaw = 0.0
        self.path = Path()

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        if self.is_path and self.is_odom and self.is_status:
            vehicle_position = self.current_position
            self.is_look_forward_point = False
            translation = [vehicle_position.x, vehicle_position.y]
            
            # 글로벌 좌표계를 로컬(차량) 좌표계로 변환하는 행렬 생성
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
            
            temp = np.zeros((2,2)) # 차량과 가장 가까운 경로점의 로컬 좌표를 저장하기 위한 배열
            dis_min = 10000.0
            j = 0
            temp_global = [0, 0, 1]
            
            # 차량에서 가장 가까운 경로점(웨이포인트)을 찾기 위해 모든 경로 순회
            for num, i in enumerate(self.path.poses):
                path_point = i.pose.position
                global_path_point = [path_point.x, path_point.y, 1]
                local_path_point = det_t.dot(global_path_point)
                
                if local_path_point[0] < 0:
                    continue
                
                dis = sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2))
                if dis <= dis_min:
                    dis_min = dis
                    j = num
                    temp[0][0] = local_path_point[0]
                    temp[0][1] = local_path_point[1]
                    temp_global = global_path_point
                    self.is_look_forward_point = True
                
                # 경로의 헤딩(Yaw)을 구하기 위해 바로 다음 경로점의 로컬 좌표 저장
                if num == j + 1:
                    temp[1][0] = local_path_point[0]
                    temp[1][1] = local_path_point[1]
                    
            if self.is_look_forward_point:
                # 차량과 가장 가까운 경로점을 이용해 헤딩 오차(Heading Error) 계산
                heading_error = atan2(temp[1][1] - temp[0][1], temp[1][0] - temp[0][0])
                
                # 차량과 가장 가까운 경로점을 이용해 횡방향 오차(Cross Track Error) 계산
                cte = sin(self.vehicle_yaw) * (temp_global[0] - vehicle_position.x) - cos(self.vehicle_yaw) * (temp_global[1] - vehicle_position.y)
                crosstrack_error = -atan2(self.k * cte, self.current_vel + self.v_t)

                # 헤딩 오차와 횡방향 오차를 합산하여 최종 조향각 산출
                steering_angle = heading_error + crosstrack_error
                
                # 조향 한계값을 차량의 물리적 최대 조향각(40도) 범위로 제한(clip)
                steering_angle = np.clip(steering_angle, -pi/6, pi/6)
                self.ctrl_cmd_msg.front_steer = float(steering_angle)
                self.ctrl_cmd_msg.velocity = self.target_vel
                
                os.system('clear')
                print("-------------------------------------")
                print(" steering (deg) = ", self.ctrl_cmd_msg.front_steer * 180 / pi)
                print(" velocity (kph) = ", self.ctrl_cmd_msg.velocity)
                print("-------------------------------------")
                
                # 조향각 정책 반영 (방향 반전 및 최대 조향각 기준 정규화)
                self.ctrl_cmd_msg.front_steer = -self.ctrl_cmd_msg.front_steer / self.max_steer
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            else:
                # os.system('clear')
                print("can't find local_forward_point")
        else:
            # os.system('clear')
            if not self.is_path:
                print("[1] can't subscribe '/local_path' topic...")
            if not self.is_odom:
                print("[2] can't subscribe '/odom' topic...")
            if not self.is_status:
                print("[3] can't subscribe '/Ego_topic' topic")
                
        self.is_path = False
        self.is_odom = False

    # ROS 토픽 수신을 위한 콜백 함수들
    def status_callback(self, msg):
        self.is_status = True
        self.current_vel = msg.velocity.x
        
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
    test_track = StanleyController()
    
    try:
        rclpy.spin(test_track)
    except KeyboardInterrupt:
        pass
    finally:
        test_track.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()