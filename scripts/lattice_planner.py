#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from math import cos, sin, sqrt, pow, atan2
from morai_ros2_msgs.msg import EgoVehicleStatus, ObjectStatusList
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np


class latticePlanner(Node):
    def __init__(self):
        super().__init__('lattice_planner')

        # (1) subscriber, publisher 선언
        self.path_sub = self.create_subscription(Path, "/local_path", self.path_callback, 5)
        self.status_sub = self.create_subscription(EgoVehicleStatus, "/ego_vehicle_status", self.status_callback, 5)
        self.object_sub = self.create_subscription(ObjectStatusList, "/object_status", self.object_callback, 5)

        self.lattice_path_pub = self.create_publisher(Path, '/lattice_path', 1)

        self.lattice_pubs = []
        for i in range(6):
            self.lattice_pubs.append(self.create_publisher(Path, f'/lattice_path_{i+1}', 1))

        self.is_path = False
        self.is_status = False
        self.is_obj = False

        self.local_path = None
        self.status_msg = None
        self.object_data = None

        self.timer = self.create_timer(1.0 / 100.0, self.timer_callback)

    def timer_callback(self):
        if self.is_path and self.is_status and self.is_obj:
            if self.checkObject(self.local_path, self.object_data):
                lattice_path = self.latticePlanner(self.local_path, self.status_msg)
                if len(lattice_path) < 6:
                    self.lattice_path_pub.publish(self.local_path)
                else:
                    lattice_path_index = self.collision_check(self.object_data, lattice_path)

                    # (7)  lattice 경로 메세지 Publish
                    self.lattice_path_pub.publish(lattice_path[lattice_path_index])
            else:
                self.lattice_path_pub.publish(self.local_path)

    def checkObject(self, ref_path, object_data):

        is_crash = False
        for obstacle in object_data.obstacle_list:
            for path in ref_path.poses:  
                dis = sqrt(pow(path.pose.position.x - obstacle.position.x, 2) + pow(path.pose.position.y - obstacle.position.y, 2))                
                if dis < 2.85:
                    is_crash = True
                    break

        return is_crash

    def collision_check(self, object_data, out_path):
        #TODO: (6) 생성된 충돌회피 경로 중 낮은 비용의 경로 선택
        
        selected_lane = -1        
        lane_weight = [5, 3, 1, 1, 3, 5] 
        
        for obstacle in object_data.obstacle_list:                        
            for path_num in range(len(out_path)) :                    
                for path_pos in out_path[path_num].poses :                                
                    dis = sqrt(pow(obstacle.position.x - path_pos.pose.position.x, 2) + pow(obstacle.position.y - path_pos.pose.position.y, 2))
                    if dis < 1.5:
                        lane_weight[path_num] = lane_weight[path_num] + 100

        selected_lane = lane_weight.index(min(lane_weight))    

        return selected_lane

    def path_callback(self,msg):
        self.is_path = True
        self.local_path = msg  
        
    def status_callback(self,msg):
        self.is_status = True
        self.status_msg = msg

    def object_callback(self,msg):
        self.is_obj = True
        self.object_data = msg

    def latticePlanner(self,ref_path, vehicle_status):
        out_path = []
        vehicle_pose_x = vehicle_status.position.x
        vehicle_pose_y = vehicle_status.position.y
        vehicle_velocity = vehicle_status.velocity.x * 3.6

        look_distance = int(vehicle_velocity * 0.2 * 2)

        if look_distance < 20 :
            look_distance = 20                    

        if len(ref_path.poses) > look_distance * 2 :  
            #TODO: (3) 좌표 변환 행렬 생성
            """
            # 좌표 변환 행렬을 만듭니다.
            # Lattice 경로를 만들기 위해서 경로 생성을 시작하는 Point 좌표에서 
            # 경로 생성이 끝나는 Point 좌표의 상대 위치를 계산해야 합니다.
            """          

            global_ref_start_point      = (ref_path.poses[0].pose.position.x, ref_path.poses[0].pose.position.y)
            global_ref_start_next_point = (ref_path.poses[1].pose.position.x, ref_path.poses[1].pose.position.y)

            global_ref_end_point = (ref_path.poses[look_distance * 2].pose.position.x, ref_path.poses[look_distance * 2].pose.position.y)
            
            theta = atan2(global_ref_start_next_point[1] - global_ref_start_point[1], global_ref_start_next_point[0] - global_ref_start_point[0])
            translation = [global_ref_start_point[0], global_ref_start_point[1]]

            trans_matrix    = np.array([    [cos(theta),                -sin(theta),                                                                      translation[0]], 
                                            [sin(theta),                 cos(theta),                                                                      translation[1]], 
                                            [         0,                          0,                                                                                   1 ]      ])

            det_trans_matrix = np.array([   [trans_matrix[0][0], trans_matrix[1][0],        -(trans_matrix[0][0] * translation[0] + trans_matrix[1][0] * translation[1])], 
                                            [trans_matrix[0][1], trans_matrix[1][1],        -(trans_matrix[0][1] * translation[0] + trans_matrix[1][1] * translation[1])],
                                            [                 0,                  0,                                                                                   1]      ])

            world_end_point = np.array([[global_ref_end_point[0]], [global_ref_end_point[1]], [1]])
            local_end_point = det_trans_matrix.dot(world_end_point)
            world_ego_vehicle_position = np.array([[vehicle_pose_x], [vehicle_pose_y], [1]])
            local_ego_vehicle_position = det_trans_matrix.dot(world_ego_vehicle_position)
            lane_off_set = [-3.5, -2.75, -2, 2, 2.75, 3.5]
            local_lattice_points = []
            
            for i in range(len(lane_off_set)):
                local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + lane_off_set[i], 1])
            
            #TODO: (4) Lattice 충돌 회피 경로 생성
            '''
            # Local 좌표계로 변경 후 3차곡선계획법에 의해 경로를 생성한 후 다시 Map 좌표계로 가져옵니다.
            # 생성된 Lattice 경로는 out_path 변수에 List 형식으로 넣습니다.
            # 충돌 회피 경로는 기존 경로를 제외하고 좌 우로 3개씩 총 6개의 경로를 가지도록 합니다.
            '''
                
            for end_point in local_lattice_points :
                lattice_path = Path()
                lattice_path.header.frame_id = 'map'
                x = []
                y = []
                x_interval = 0.5
                xs = 0
                xf = end_point[0]
                ps = local_ego_vehicle_position[1][0]

                pf = end_point[1]
                x_num = xf / x_interval

                for i in range(xs,int(x_num)) : 
                    x.append(i*x_interval)
                
                a = [0.0, 0.0, 0.0, 0.0]
                a[0] = ps
                a[1] = 0
                a[2] = 3.0 * (pf - ps) / (xf * xf)
                a[3] = -2.0 * (pf - ps) / (xf * xf * xf)
                
                for i in x:
                    result = a[3] * i * i * i + a[2] * i * i + a[1] * i + a[0]
                    y.append(result)

                for i in range(0,len(y)) :
                    local_result = np.array([[x[i]], [y[i]], [1]])
                    global_result = trans_matrix.dot(local_result)

                    read_pose = PoseStamped()
                    read_pose.pose.position.x = float(global_result[0][0])
                    read_pose.pose.position.y = float(global_result[1][0])
                    read_pose.pose.position.z = 0.0
                    read_pose.pose.orientation.x = 0.0
                    read_pose.pose.orientation.y = 0.0
                    read_pose.pose.orientation.z = 0.0
                    read_pose.pose.orientation.w = 1.0
                    lattice_path.poses.append(read_pose)

                out_path.append(lattice_path)

            add_point_size = min(int(vehicle_velocity * 2), len(ref_path.poses) )            
            
            for i in range(look_distance*2,add_point_size):
                if i+1 < len(ref_path.poses):
                    tmp_theta = atan2(ref_path.poses[i + 1].pose.position.y - ref_path.poses[i].pose.position.y,ref_path.poses[i + 1].pose.position.x - ref_path.poses[i].pose.position.x)                    
                    tmp_translation = [ref_path.poses[i].pose.position.x,ref_path.poses[i].pose.position.y]
                    tmp_t = np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]], [sin(tmp_theta), cos(tmp_theta), tmp_translation[1]], [0, 0, 1]])

                    for lane_num in range(len(lane_off_set)) :
                        local_result = np.array([[0], [lane_off_set[lane_num]], [1]])
                        global_result = tmp_t.dot(local_result)

                        read_pose = PoseStamped()
                        read_pose.pose.position.x = float(global_result[0][0])
                        read_pose.pose.position.y = float(global_result[1][0])
                        read_pose.pose.position.z = 0.0
                        read_pose.pose.orientation.x = 0.0
                        read_pose.pose.orientation.y = 0.0
                        read_pose.pose.orientation.z = 0.0
                        read_pose.pose.orientation.w = 1.0
                        out_path[lane_num].poses.append(read_pose)
                        
            #TODO: (5) 생성된 모든 Lattice 충돌 회피 경로 메시지 Publish
            '''
            # 생성된 모든 Lattice 충돌회피 경로는 ros 메세지로 송신하여
            # Rviz 창에서 시각화 하도록 합니다.

            '''
            for i in range(len(out_path)):          
                self.lattice_pubs[i].publish(out_path[i])
        
        return out_path

def main(args=None):
    rclpy.init(args=args)
    planner = latticePlanner()
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()