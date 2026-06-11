#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from turtlesim.msg import Pose
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from path_reader import PathReader

class LocalPathFinder(Node):
    def __init__(self):
        super().__init__('local_path_finder')
        
        self.path_pub = self.create_publisher(Path, '/global_path', 1)
        self.local_path_pub = self.create_publisher(Path, '/local_path', 1)
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.statusCB, qos_profile)
        self.status_msg = Pose()
        self.br = TransformBroadcaster(self)
        
        # 전역 경로 로드
        p_r = PathReader("beginner_tutorials", "turtle_path.txt")
        self.global_path = p_r.global_path
        p_r.destroy_node() # 경로 데이터만 가져온 뒤 리더 노드는 메모리에서 해제
        
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def statusCB(self, data):
        self.status_msg = data
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'turtle'
        
        t.transform.translation.x = float(self.status_msg.x)
        t.transform.translation.y = float(self.status_msg.y)
        t.transform.translation.z = 0.0
        
        q = quaternion_from_euler(0.0, 0.0, self.status_msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.br.sendTransform(t)

    def find_local_path(self, ref_path, status_msg):
        out_path = Path()
        current_x = status_msg.x
        current_y = status_msg.y
        current_waypoint = 0
        min_dis = float('inf')

        for i in range(len(ref_path.poses)):
            dx = current_x - ref_path.poses[i].pose.position.x
            dy = current_y - ref_path.poses[i].pose.position.y
            dis = math.sqrt(dx*dx + dy*dy)
            if dis < min_dis:
                min_dis = dis
                current_waypoint = i

        if current_waypoint + 10 > len(ref_path.poses):
            last_local_waypoint = len(ref_path.poses)
        else:
            last_local_waypoint = current_waypoint + 10

        out_path.header.frame_id = 'map'
        out_path.header.stamp = self.get_clock().now().to_msg()
        
        for i in range(current_waypoint, last_local_waypoint):
            tmp_pose = PoseStamped()
            tmp_pose.pose.position.x = ref_path.poses[i].pose.position.x
            tmp_pose.pose.position.y = ref_path.poses[i].pose.position.y
            tmp_pose.pose.position.z = ref_path.poses[i].pose.position.z
            tmp_pose.pose.orientation.x = 0.0
            tmp_pose.pose.orientation.y = 0.0
            tmp_pose.pose.orientation.z = 0.0
            tmp_pose.pose.orientation.w = 1.0
            out_path.poses.append(tmp_pose)

        return out_path, current_waypoint

    def timer_callback(self):
        if len(self.global_path.poses) > 0:
            local_path, current_waypoint = self.find_local_path(self.global_path, self.status_msg)
            
            self.global_path.header.stamp = self.get_clock().now().to_msg()
            
            self.local_path_pub.publish(local_path)
            self.path_pub.publish(self.global_path)


def main(args=None):
    rclpy.init(args=args)
    local_path_finder = LocalPathFinder()
    
    try:
        rclpy.spin(local_path_finder)
    except KeyboardInterrupt:
        pass
    finally:
        local_path_finder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()