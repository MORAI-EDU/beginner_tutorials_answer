#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Float32

class SCANParser(Node):

    def __init__(self):
        super().__init__('velodyne_parser')

        self.scan_sub = self.create_subscription(PointCloud2, "/velodyne_points", self.callback, 10)

        self.dist_pub = self.create_publisher(Float32, "dist_forward", 10)

        self.pc_np = None


    def callback(self, msg):
        self.pc_np = self.pointcloud2_to_xyz(msg)

        d_min = self.calc_dist_forward()

        dist_msg = Float32()
        dist_msg.data = float(d_min)

        self.dist_pub.publish(dist_msg)


    def pointcloud2_to_xyz(self, cloud_msg):
        point_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            dist = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
            angle = np.arctan2(point[1], point[0])
            if point[0] > 0 and point[2] > -1.3 and dist < 50:
                point_list.append((point[0], point[1], point[2], point[3], dist, angle))

        point_np = np.array(point_list, np.float32)
        return point_np


    def calc_dist_forward(self):
        r1_bool = self.pc_np[:, 5] > -30/180*np.pi
        r2_bool = self.pc_np[:, 5] < 30/180*np.pi

        d1 = self.pc_np[r1_bool, 4]
        d2 = self.pc_np[r2_bool, 4]
        d_list = np.concatenate([d1, d2])

        return np.min(d_list)


def main(args=None):
    rclpy.init(args=args)
    scan_parser = SCANParser()
    try:
        rclpy.spin(scan_parser)
    except KeyboardInterrupt:
        pass
    finally:
        scan_parser.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()