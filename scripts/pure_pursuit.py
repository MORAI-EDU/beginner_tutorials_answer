#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, os
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd
import numpy as np
from tf.transformations import euler_from_quaternion

class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)
        rospy.Subscriber("local_path", Path, self.path_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd',CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg=CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType=2

        self.is_path=False
        self.is_odom=False

        self.forward_point=Point()
        self.current_postion=Point()
        self.is_look_forward_point=False
        
        # --- 파라미터 수정 구간 ---
        self.vehicle_length = 3.0  # 휠베이스(축거)
        self.lfd = 5.0             # Look-forward distance
        self.max_steer_deg = 40.0  # 최대 조향각
        self.max_steering_angle = self.max_steer_deg * pi / 180 
        # -----------------------

        rate = rospy.Rate(15)
        while not rospy.is_shutdown():

            if self.is_path and self.is_odom:
                vehicle_position=self.current_postion
                self.is_look_forward_point= False

                translation=[vehicle_position.x, vehicle_position.y]

                # Global to Local 좌표 변환 행렬
                t=np.array([
                        [cos(self.vehicle_yaw), -sin(self.vehicle_yaw),translation[0]],
                        [sin(self.vehicle_yaw),cos(self.vehicle_yaw),translation[1]],
                        [0                    ,0                    ,1            ]])

                det_t=np.array([
                       [t[0][0],t[1][0],-(t[0][0]*translation[0]+t[1][0]*translation[1])],
                       [t[0][1],t[1][1],-(t[0][1]*translation[0]+t[1][1]*translation[1])],
                       [0      ,0      ,1                                               ]])

                for num,i in enumerate(self.path.poses) :
                    path_point=i.pose.position
                    global_path_point=[path_point.x,path_point.y,1]
                    local_path_point=det_t.dot(global_path_point)            
                    
                    if local_path_point[0]>0 :
                        dis=sqrt(pow(local_path_point[0],2)+pow(local_path_point[1],2))
                        if dis>= self.lfd :
                            self.forward_point=path_point
                            self.is_look_forward_point=True
                            break
                
                if self.is_look_forward_point :
                    theta=atan2(local_path_point[1],local_path_point[0])
                    
                    # Pure Pursuit 공식 적용
                    steering_angle = atan2((2*self.vehicle_length*sin(theta)),self.lfd)
                    
                    # 시뮬레이터 방향에 따라 steering_angle 앞의 부호를 조정합니다.
                    normalized_steer = - (steering_angle / self.max_steering_angle)
                    
                    self.ctrl_cmd_msg.front_steer = np.clip(normalized_steer, -1.0, 1.0)
                    self.ctrl_cmd_msg.velocity = 20.0

                    os.system('clear')
                    print("-------------------------------------")
                    print(" steering (deg) = ", self.ctrl_cmd_msg.front_steer * 180/3.14)
                    print(" velocity (kph) = ", self.ctrl_cmd_msg.velocity)
                    print("-------------------------------------")
                else : 
                    print("Searching for forward point...")
                    self.ctrl_cmd_msg.front_steer=0.0
                    self.ctrl_cmd_msg.velocity=0.0
                
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            self.is_path = self.is_odom = False
            rate.sleep()

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
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass