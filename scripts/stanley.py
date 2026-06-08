#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, os
import numpy as np
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from tf.transformations import euler_from_quaternion


class stanly_controller :
    def __init__(self):
        # 토픽 구독(Subscribe) 및 발행(Publish) 설정
        rospy.init_node('stanly_controller', anonymous=True)
        rospy.Subscriber("local_path", Path, self.path_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("Ego_topic", EgoVehicleStatus, self.status_callback)
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd',CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 2
        self.is_path = False
        self.is_odom = False
        self.is_status = False
        self.forward_point = Point()
        self.current_postion = Point()
        self.target_vel = 20.0
        self.current_vel = 0.0
        self.is_look_forward_point = False
        self.vehicle_length = 3.0
        self.k = 0.8 # Stanley 제어 게인 (Stanley constant)
        self.v_t = 1 # 횡오차 계산 시 분모가 0이 되는 것을 방지하기 위한 속도 상수
        self.prev_steering_angle = 0

        # --- 조향 정책 반영을 위한 파라미터 추가 ---
        self.max_steer_deg = 40.0  # 차량의 최대 조향각 (40도)
        self.max_steering_angle = self.max_steer_deg * pi / 180 
        # ------------------------------------------

        rate = rospy.Rate(15) # 15hz
        while not rospy.is_shutdown():
            if self.is_path ==True and self.is_odom==True and self.is_status :
                vehicle_position=self.current_postion
                self.is_look_forward_point= False
                translation=[vehicle_position.x, vehicle_position.y]
                
                # 글로벌 좌표계를 로컬(차량) 좌표계로 변환하는 행렬 생성
                t=np.array([
                        [cos(self.vehicle_yaw), -sin(self.vehicle_yaw),translation[0]],
                        [sin(self.vehicle_yaw),cos(self.vehicle_yaw),translation[1]],
                        [0                    ,0                    ,1            ]])
                det_t=np.array([
                       [t[0][0],t[1][0],-(t[0][0]*translation[0]+t[1][0]*translation[1])],
                       [t[0][1],t[1][1],-(t[0][1]*translation[0]+t[1][1]*translation[1])],
                       [0      ,0      ,1                                               ]])
                temp = np.zeros((2,2)) # 차량과 가장 가까운 경로점의 로컬 좌표를 저장하기 위한 배열
                dis_min = 10000
                j = 0
                
                # 차량에서 가장 가까운 경로점(웨이포인트)을 찾기 위해 모든 경로 순회
                for num,i in enumerate(self.path.poses) :
                    path_point=i.pose.position
                    global_path_point=[path_point.x,path_point.y,1]
                    local_path_point=det_t.dot(global_path_point)
                    if local_path_point[0] < 0:
                        continue
                    dis = sqrt(pow(local_path_point[0],2)+pow(local_path_point[1],2))
                    if dis <= dis_min :
                        dis_min = dis
                        j = num
                        temp[0][0] = local_path_point[0]
                        temp[0][1] = local_path_point[1]
                        temp_global = global_path_point
                        self.is_look_forward_point = True
                    
                    # 경로의 헤딩(Yaw)을 구하기 위해 바로 다음 경로점의 로컬 좌표 저장
                    if num == j + 1 :
                        temp[1][0] = local_path_point[0]
                        temp[1][1] = local_path_point[1]
                if self.is_look_forward_point :
                    
                    # 차량과 가장 가까운 경로점을 이용해 헤딩 오차(Heading Error) 계산
                    heading_error = atan2(temp[1][1] - temp[0][1], temp[1][0] - temp[0][0])
                    
                    # 차량과 가장 가까운 경로점을 이용해 횡방향 오차(Cross Track Error) 계산
                    cte = sin(self.vehicle_yaw)*(temp_global[0] - vehicle_position.x) - cos(self.vehicle_yaw)*(temp_global[1] - vehicle_position.y)
                    crosstrack_error = -atan2(self.k * cte, self.current_vel + self.v_t)

                    # 헤딩 오차와 횡방향 오차를 합산하여 최종 조향각 산출
                    steering_angle = heading_error + crosstrack_error
                    
                    # 조향 한계값을 차량의 물리적 최대 조향각(40도) 범위로 제한(clip)
                    steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
                    
                    # 동일 조향각 정책 반영 (방향 반전 및 최대 조향각 기준 정규화)
                    normalized_steer = - (steering_angle / self.max_steering_angle)
                    self.ctrl_cmd_msg.front_steer = np.clip(normalized_steer, -1.0, 1.0)
                    
                    self.ctrl_cmd_msg.velocity = self.target_vel
                    
                    os.system('clear')
                    print("-------------------------------------")
                    print(f" Raw Steer Angle(deg) : {steering_angle * 180 / pi:.2f} deg")
                    print(f" Normalized Steer(-1~1): {self.ctrl_cmd_msg.front_steer:.4f}")
                    print(f" velocity (kph)         : {self.ctrl_cmd_msg.velocity}")
                    print("-------------------------------------")
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                else :
                    os.system('clear')
                    print("can't find local_forward_point")
            else :
                os.system('clear')
                if not self.is_path:
                    print("[1] can't subscribe '/local_path' topic...")
                if not self.is_odom:
                    print("[2] can't subscribe '/odom' topic...")
                if not self.is_status:
                    print("[3] can't subscribe '/Ego_topic' topic")
            self.is_path = self.is_odom = False
            rate.sleep()
            
    # ROS 토픽 수신을 위한 콜백 함수들
    def status_callback(self, msg):
        self.is_status = True
        self.current_vel = msg.velocity.x
    def path_callback(self,msg):
        self.is_path=True
        self.path=msg
    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

if __name__ == '__main__':
    try:
        test_track=stanly_controller()
    except rospy.ROSInterruptException:
        pass