#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from math import sqrt
from turtlesim.msg import Pose
from ament_index_python.packages import get_package_share_directory

class PathMaker(Node):
    def __init__(self, pkg_name, path_name):
        super().__init__('path_maker')
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        # /turtle1/pose 토픽 구독
        self.subscriber = self.create_subscription(Pose, "/turtle1/pose", self.status_callback, qos_profile)
        
        # 초기화
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.is_status = False
        self.status_msg = Pose()
        
        # 패키지 경로 로드 & 파일 쓰기 모드
        pkg_path = get_package_share_directory(pkg_name)
        full_path = os.path.join(pkg_path, f"{path_name}.txt")
        self.f = open(full_path, 'w')
        
        self.get_logger().info(f"File will be saved to: {full_path}")
        
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        if self.is_status:
            self.path_make()

    def path_make(self):
        x = self.status_msg.x
        y = self.status_msg.y
        distance = sqrt(pow(x - self.prev_x, 2) + pow(y - self.prev_y, 2))
        
        # 이전 waypoint와의 거리가 0.3 이상이어야 기록
        if distance > 0.3:
            data = '{0}\t{1}\n'.format(x, y)
            self.f.write(data)
            self.prev_x = x
            self.prev_y = y
            print("write : ", x, y)

    def status_callback(self, msg):
        self.is_status = True
        self.status_msg = msg

def main(args=None):
    rclpy.init(args=args)
    p_m = PathMaker("beginner_tutorials", "turtle_path")
    
    try:
        rclpy.spin(p_m)
    except KeyboardInterrupt:
        pass
    finally:
        p_m.f.close()
        p_m.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()