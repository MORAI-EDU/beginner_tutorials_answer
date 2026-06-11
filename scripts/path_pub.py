#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from math import sqrt, pow
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from ament_index_python.packages import get_package_share_directory

class PathPub(Node):
    def __init__(self):
        super().__init__('path_pub')
        
        # 센서 데이터 수신을 위한 best_effort QoS 설정
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_callback, qos_profile)
        self.global_path_pub = self.create_publisher(Path, '/global_path', 1)
        self.local_path_pub = self.create_publisher(Path, '/local_path', 1)
        
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = 'map'
        
        self.is_odom = False
        self.local_path_size = 50

        # 패키지 경로 로드
        pkg_path = get_package_share_directory('beginner_tutorials')
        full_path = os.path.join(pkg_path, 'path', 'kcity.txt')
        
        with open(full_path, 'r') as f:
            lines = f.readlines()
            for line in lines:
                tmp = line.split()
                read_pose = PoseStamped()
                read_pose.pose.position.x = float(tmp[0])
                read_pose.pose.position.y = float(tmp[1])
                read_pose.pose.orientation.w = 1.0
                self.global_path_msg.poses.append(read_pose)

        self.timer = self.create_timer(1.0 / 20.0, self.timer_callback)

    def timer_callback(self):
        if self.is_odom:
            local_path_msg = Path()
            local_path_msg.header.frame_id = 'map'
            
            current_time = self.get_clock().now().to_msg()
            local_path_msg.header.stamp = current_time
            self.global_path_msg.header.stamp = current_time
            
            x = self.x
            y = self.y
            min_dis = float('inf')
            current_waypoint = -1
            
            for i, waypoint in enumerate(self.global_path_msg.poses):
                distance = sqrt(pow(x - waypoint.pose.position.x, 2) + pow(y - waypoint.pose.position.y, 2))
                if distance < min_dis:
                    min_dis = distance
                    current_waypoint = i
            
            if current_waypoint != -1:
                end_waypoint = current_waypoint + self.local_path_size
                if end_waypoint > len(self.global_path_msg.poses):
                    end_waypoint = len(self.global_path_msg.poses)
                    
                for num in range(current_waypoint, end_waypoint):
                    tmp_pose = PoseStamped()
                    tmp_pose.pose.position.x = self.global_path_msg.poses[num].pose.position.x
                    tmp_pose.pose.position.y = self.global_path_msg.poses[num].pose.position.y
                    tmp_pose.pose.orientation.w = 1.0
                    local_path_msg.poses.append(tmp_pose)

                print(x, y)
                self.global_path_pub.publish(self.global_path_msg)
                self.local_path_pub.publish(local_path_msg)

    def odom_callback(self, msg):
        self.is_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

def main(args=None):
    rclpy.init(args=args)
    test_track = PathPub()
    
    try:
        rclpy.spin(test_track)
    except KeyboardInterrupt:
        pass
    finally:
        test_track.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()