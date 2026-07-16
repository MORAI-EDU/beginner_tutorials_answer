#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from sklearn.cluster import DBSCAN

class SCANCluster(Node):
    def __init__(self):
        super().__init__('velodyne_clustering')
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        self.scan_sub = self.create_subscription(PointCloud2, "/velodyne_points", self.callback, qos_profile)
        self.clusterpoints_pub = self.create_publisher(PointCloud2, "/cluster_points", 10)
        self.pc_np = None
        self.dbscan = DBSCAN(eps=0.5, min_samples=10)

    def callback(self, msg):
        self.pc_np = self.pointcloud2_to_xyz(msg)
        if len(self.pc_np) == 0:
            return

        pc_xy = self.pc_np[:, :2]
        db = self.dbscan.fit_predict(pc_xy)
        n_cluster = np.max(db) + 1

        cluster_points = []
        for c in range(n_cluster):
            c_tmp = np.mean(pc_xy[db==c, :], axis=0)
            cluster_points.append([float(c_tmp[0]), float(c_tmp[1]), 0.0])

        self.publish_point_cloud(cluster_points)

    def publish_point_cloud(self, points):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "velodyne"

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        pc2_msg = pc2.create_cloud(header, fields, points)

        self.clusterpoints_pub.publish(pc2_msg)

    def pointcloud2_to_xyz(self, cloud_msg):
        point_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            dist = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
            angle = np.arctan2(point[1], point[0])
            if point[0] > 0 and 1.50 > point[2] > -1.25 and dist < 50:
                point_list.append((point[0], point[1], point[2], point[3], dist, angle))

        point_np = np.array(point_list, np.float32)
        return point_np

def main(args=None):
    rclpy.init(args=args)
    scan_cluster = SCANCluster()
    
    try:
        rclpy.spin(scan_cluster)
    except KeyboardInterrupt:
        pass
    finally:
        scan_cluster.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()