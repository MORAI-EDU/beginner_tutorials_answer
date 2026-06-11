#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import MGeoPlannerMap

class GetMGeo(Node):
    def __init__(self):
        super().__init__('mgeo_pub')
        
        self.link_pub = self.create_publisher(PointCloud, 'link', 1)
        self.node_pub = self.create_publisher(PointCloud, 'node', 1)

        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/kcity'))
        mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set
        self.nodes = node_set.nodes
        self.links = link_set.lines
        
        self.link_msg = self.get_all_links()
        self.node_msg = self.get_all_nodes()

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.link_msg.header.stamp = self.get_clock().now().to_msg()
        self.node_msg.header.stamp = self.get_clock().now().to_msg()
        
        self.link_pub.publish(self.link_msg)
        self.node_pub.publish(self.node_msg)
        print("MGeo node , link data >>>> publish success !!")

    def get_all_links(self):
        all_link = PointCloud()
        all_link.header.frame_id = 'map'
        
        for link_idx in self.links:
            for link_point in self.links[link_idx].points:
                tmp_point = Point32()
                tmp_point.x = float(link_point[0])
                tmp_point.y = float(link_point[1])
                tmp_point.z = float(link_point[2])
                all_link.points.append(tmp_point)

        return all_link
    
    def get_all_nodes(self):
        all_node = PointCloud()
        all_node.header.frame_id = 'map'
        
        for node_idx in self.nodes:
            tmp_point = Point32()
            tmp_point.x = float(self.nodes[node_idx].point[0])
            tmp_point.y = float(self.nodes[node_idx].point[1])
            tmp_point.z = float(self.nodes[node_idx].point[2])
            all_node.points.append(tmp_point)

        return all_node

def main(args=None):
    rclpy.init(args=args)
    test_track = GetMGeo()
    
    try:
        rclpy.spin(test_track)
    except KeyboardInterrupt:
        pass
    finally:
        test_track.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()