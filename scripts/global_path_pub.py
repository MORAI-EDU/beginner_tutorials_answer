#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from ament_index_python.packages import get_package_share_directory

class read_path_pub(Node):
    def __init__(self):
        super().__init__('read_path_pub')
        
        self.global_path_pub = self.create_publisher(Path, '/global_path', 5)

        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = 'map'
        
        pkg_path = get_package_share_directory('beginner_tutorials')
        full_path = os.path.join(pkg_path, 'path', 'R_KR_PR_K-city_2025.txt')
        
        self.f = open(full_path, 'r')
        lines = self.f.readlines()

        for line in lines:
            tmp = line.split()
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(tmp[0])
            read_pose.pose.position.y = float(tmp[1])
            read_pose.pose.position.z = float(tmp[2])
            read_pose.pose.orientation.w = 1.0
            self.global_path_msg.poses.append(read_pose)
        
        self.f.close()

        self.timer = self.create_timer(1.0 / 10.0, self.timer_callback)

    def timer_callback(self):
        self.global_path_msg.header.stamp = self.get_clock().now().to_msg()
        self.global_path_pub.publish(self.global_path_msg)


def main(args=None):
    rclpy.init(args=args)
    test_track = read_path_pub()
    
    try:
        rclpy.spin(test_track)
    except KeyboardInterrupt:
        pass
    finally:
        test_track.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()