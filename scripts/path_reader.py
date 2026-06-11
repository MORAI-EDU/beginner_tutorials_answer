#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory

class PathReader(Node):
    def __init__(self, pkg_name, file_name):
        super().__init__('path_reader')
        self.path_pub = self.create_publisher(Path, '/global_path', 1)
        
        pkg_path = get_package_share_directory(pkg_name)
        self.file_path = pkg_path
        
        self.global_path = self.read_txt(file_name)
        
        self.timer = self.create_timer(1.0, self.timer_callback)

    def read_txt(self, file_name):
        full_file_name = os.path.join(self.file_path, file_name)
        openFile = open(full_file_name, 'r')
        out_path = Path()
        out_path.header.frame_id = 'map'
        
        lines = openFile.readlines()
        for i in lines:
            tmp = i.split()
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(tmp[0])
            read_pose.pose.position.y = float(tmp[1])
            read_pose.pose.position.z = 0.0
            read_pose.pose.orientation.x = 0.0
            read_pose.pose.orientation.y = 0.0
            read_pose.pose.orientation.z = 0.0
            read_pose.pose.orientation.w = 1.0 
            out_path.poses.append(read_pose)

        openFile.close()
        return out_path

    def timer_callback(self):
        self.global_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.global_path)


def main(args=None):
    rclpy.init(args=args)
    p_r = PathReader("beginner_tutorials", "turtle_path.txt")
    
    try:
        rclpy.spin(p_r)
    except KeyboardInterrupt:
        pass
    finally:
        p_r.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()