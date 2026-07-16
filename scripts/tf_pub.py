#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# tf 는 물체의 위치와 자세 데이터를 좌표계로 나타내는 예제입니다.

# 노드 실행 순서 
# 1. Callback 함수 생성
# 2. 브로드캐스터 생성 및 Ego 상태 tf 브로드캐스팅

class Ego_listener(Node):
    def __init__(self):
        super().__init__('status_listener')
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        self.subscription = self.create_subscription(Odometry, "odom", self.odom_callback, qos_profile)
        self.br = TransformBroadcaster(self)


    #TODO: (1) Callback 함수 생성
    def odom_callback(self, msg):
        self.is_odom = True

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.orientation_x = msg.pose.pose.orientation.x
        self.orientation_y = msg.pose.pose.orientation.y
        self.orientation_z = msg.pose.pose.orientation.z
        self.orientation_w = msg.pose.pose.orientation.w

        #TODO: (2) 브로드캐스터 생성 및 Ego 상태 tf 브로드캐스팅
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "Ego"
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 1.0
        
        t.transform.rotation.x = self.orientation_x
        t.transform.rotation.y = self.orientation_y
        t.transform.rotation.z = self.orientation_z
        t.transform.rotation.w = self.orientation_w

        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        tl = Ego_listener()
        rclpy.spin(tl)
    except KeyboardInterrupt:
        pass
    finally:
        if 'tl' in locals():
            tl.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()