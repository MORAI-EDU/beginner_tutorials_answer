#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from math import sqrt, pow
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


class path_pub(Node):

    def __init__(self):
        super().__init__('path_pub')
        
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.global_path_sub = self.create_subscription(Path, "/global_path", self.global_Path_callback, 10)

        self.local_path_pub = self.create_publisher(Path, '/local_path', 1)
        
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'
        
        self.is_status = False
        self.local_path_size = 50

        self.x = 0.0
        self.y = 0.0

        self.timer = self.create_timer(1.0 / 100.0, self.timer_callback)

    def timer_callback(self):
        if self.is_status == True:
            local_path_msg = Path()
            local_path_msg.header.frame_id = '/map'
            local_path_msg.header.stamp = self.get_clock().now().to_msg()
            
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
                if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                    for num in range(current_waypoint, current_waypoint + self.local_path_size):
                        tmp_pose = PoseStamped()
                        tmp_pose.pose.position.x = self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y = self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w = 1.0
                        local_path_msg.poses.append(tmp_pose)
                
                else:
                    for num in range(current_waypoint, len(self.global_path_msg.poses)):
                        tmp_pose = PoseStamped()
                        tmp_pose.pose.position.x = self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y = self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w = 1.0
                        local_path_msg.poses.append(tmp_pose)

            self.local_path_pub.publish(local_path_msg)


    def odom_callback(self, msg):
        self.is_status = True

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y


    def global_Path_callback(self, msg):
        self.global_path_msg = msg       


def main(args=None):
    rclpy.init(args=args)
    test_track = path_pub()
    
    try:
        rclpy.spin(test_track)
    except KeyboardInterrupt:
        pass
    finally:
        test_track.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()