#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from math import cos, sin, pi, sqrt, pow, atan2
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from morai_ros2_msgs.msg import CtrlCmd, EgoVehicleStatus
import numpy as np
from tf_transformations import euler_from_quaternion

from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory

# advanced_purepursuit 은 차량의 차량의 종 횡 방향 제어 예제입니다.
# Purpusuit 알고리즘의 Look Ahead Distance 값을 속도에 비례하여 가변 값으로 만들어 횡 방향 주행 성능을 올립니다.
# 횡방향 제어 입력은 주행할 Local Path (지역경로) 와 차량의 상태 정보 Odometry 를 받아 차량을 제어 합니다.
# 종방향 제어 입력은 목표 속도를 지정 한뒤 목표 속도에 도달하기 위한 Throttle control 을 합니다.
# 종방향 제어 입력은 longl_cmd_type 1(Throttle control) 이용합니다.

# 노드 실행 순서 
# 1. subscriber, publisher 선언
# 2. 속도 비례 Look Ahead Distance 값 설정
# 3. 좌표 변환 행렬 생성
# 4. Steering 각도 계산
# 5. PID 제어 생성
# 6. 도로의 곡률 계산
# 7. 곡률 기반 속도 계획
# 8. 제어입력 메세지 Publish

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        self.global_path = Path()
        self.global_path.header.frame_id = 'map'
        
        pkg_path = get_package_share_directory('beginner_tutorials')
        full_path = os.path.join(pkg_path, 'path', 'kcity.txt')
        
        file = open(full_path, 'r')
        lines = file.readlines()
        for line in lines:
            tmp = line.split()
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(tmp[0])
            read_pose.pose.position.y = float(tmp[1])
            read_pose.pose.orientation.w = 1.0
            self.global_path.poses.append(read_pose)
        file.close()

        qos_profile = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE
        )
        #TODO: (1) subscriber, publisher 선언
        self.path_sub = self.create_subscription(Path, "lattice_path", self.path_callback, qos_profile)
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_callback, qos_profile)
        self.status_sub = self.create_subscription(EgoVehicleStatus, "ego_vehicle_status", self.status_callback, qos_profile) 
        
        self.ctrl_cmd_pub = self.create_publisher(CtrlCmd, "ctrl_cmd", 1)

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longl_cmd_type = 1

        self.is_path = False
        self.is_odom = False 
        self.is_status = False
        self.velocity_planned = False

        self.is_look_forward_point = False

        self.forward_point = Point()
        self.current_postion = Point()

        self.vehicle_length = 4.0
        self.lfd = 8.0
        self.min_lfd = 5.0
        self.max_lfd = 30.0
        self.lfd_gain = 0.78
        self.target_velocity = 40.0
        self.vehicle_yaw = 0.0
        self.max_steer = 40.0 * pi / 180.0
        self.prev_wp = None

        self.pid = PidControl()
        self.vel_planning = VelocityPlanning(self.target_velocity / 3.6, 0.15)
        
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        if not self.velocity_planned:
            self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 50)
            self.velocity_planned = True
            self.get_logger().info('Velocity planning complete!')

        if self.is_path and self.is_odom and self.is_status:
            self.current_waypoint = self.get_current_waypoint(self.status_msg, self.global_path)
            self.target_velocity = self.velocity_list[self.current_waypoint] * 3.6

            steering = self.calc_pure_pursuit()
            if self.is_look_forward_point:
                self.ctrl_cmd_msg.front_steer = -float(steering) / self.max_steer

            output = self.pid.pid(self.target_velocity, self.status_msg.velocity.x * 3.6)
            if output > 0.0:
                self.ctrl_cmd_msg.accel = float(output)
                self.ctrl_cmd_msg.brake = 0.0
            else:
                self.ctrl_cmd_msg.accel = 0.0
                self.ctrl_cmd_msg.brake = float(-output)

            #TODO: (8) 제어입력 메세지 Publish
            print(f"Target vel: {self.target_velocity:.4f}, Steering: {self.ctrl_cmd_msg.front_steer:.4f}")
            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
        else:
            os.system('clear')
            if not self.is_path:
                print("[1] can't subscribe '/local_path' topic...")
            if not self.is_odom:
                print("[2] can't subscribe '/odom' topic...")
            if not self.is_status:
                print("[3] can't subscribe '/Ego_topic' topic...")

        self.is_path = False
        self.is_odom = False
        self.is_status = False

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
        self.current_postion.x = msg.pose.pose.position.x
        self.current_postion.y = msg.pose.pose.position.y

    def status_callback(self, msg):
        self.is_status = True
        self.status_msg = msg    
    
    def get_current_waypoint(self, ego_status, global_path):
        min_dist = float('inf')
        current_waypoint = -1

        if self.prev_wp is None:
            local_poses = global_path.poses
        else:
            first_point = self.prev_wp - 10
            second_point = self.prev_wp + 50
            local_poses = global_path.poses[first_point:second_point]

        for i, pose in enumerate(local_poses):
            dx = ego_status.position.x - pose.pose.position.x
            dy = ego_status.position.y - pose.pose.position.y

            dist = sqrt(pow(dx, 2) + pow(dy, 2))
            if min_dist > dist:
                min_dist = dist
                current_waypoint = i

        if self.prev_wp is not None:
            current_waypoint += self.prev_wp - 10
        self.prev_wp = current_waypoint
        return current_waypoint

    def calc_pure_pursuit(self):
        #TODO: (2) 속도 비례 Look Ahead Distance 값 설정
        self.lfd = (self.status_msg.velocity.x) * self.lfd_gain
        
        if self.lfd < self.min_lfd: 
            self.lfd = self.min_lfd
        elif self.lfd > self.max_lfd:
            self.lfd = self.max_lfd
            
        vehicle_position = self.current_postion
        self.is_look_forward_point = False

        translation = [vehicle_position.x, vehicle_position.y]

        #TODO: (3) 좌표 변환 행렬 생성
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
        
        local_path_point = [0, 0, 1]

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
        
        #TODO: (4) Steering 각도 계산
        theta = atan2(local_path_point[1], local_path_point[0])
        steering = atan2((2 * self.vehicle_length * sin(theta)), self.lfd)

        return steering


class PidControl:
    def __init__(self):
        self.p_gain = 0.3
        self.i_gain = 0.001
        self.d_gain = 0.01
        self.prev_error = 0.0
        self.i_control = 0.0
        self.controlTime = 0.03

    def pid(self, target_vel, current_vel):
        error = target_vel - current_vel

        #TODO: (5) PID 제어 생성
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error - self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output


class VelocityPlanning:
    def __init__ (self, car_max_speed, road_friction):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friction

    def curvedBaseVelocity(self, gloabl_path, point_num):
        out_vel_plan = []

        for i in range(0, point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num, len(gloabl_path.poses) - point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = gloabl_path.poses[i+box].pose.position.x
                y = gloabl_path.poses[i+box].pose.position.y
                x_list.append([-2*x, -2*y, 1])
                y_list.append((-x*x) - (y*y))

            #TODO: (6) 도로의 곡률 계산
            x_matrix = np.array(x_list)
            y_matrix = np.array(y_list)
            x_trans = x_matrix.T

            a_matrix = np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a = a_matrix[0]
            b = a_matrix[1]
            c = a_matrix[2]
            r = sqrt(a*a + b*b - c)

            #TODO: (7) 곡률 기반 속도 계획
            v_max = sqrt(r * 9.8 * self.road_friction)

            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(gloabl_path.poses) - point_num, len(gloabl_path.poses) - 10):
            out_vel_plan.append(30)

        for i in range(len(gloabl_path.poses) - 10, len(gloabl_path.poses)):
            out_vel_plan.append(0)

        return out_vel_plan


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