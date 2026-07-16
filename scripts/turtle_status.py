#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf_transformations import quaternion_from_euler

class TurtleListener(Node):
    def __init__(self):
        super().__init__('status_listener')
        
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.statusCB, qos_profile)
        self.br = tf2_ros.TransformBroadcaster(self)

    def statusCB(self, data):
        print("tf broad cast")
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'turtle'
        
        t.transform.translation.x = float(data.x)
        t.transform.translation.y = float(data.y)
        t.transform.translation.z = 0.0
        
        q = quaternion_from_euler(0.0, 0.0, data.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    tl = TurtleListener()
    
    try:
        rclpy.spin(tl)
    except KeyboardInterrupt:
        pass
    finally:
        tl.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()