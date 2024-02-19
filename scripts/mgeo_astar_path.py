#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import os
import time

from math import cos,sin,sqrt,pow,atan2,pi
from geometry_msgs.msg import Pose,PoseStamped, PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import Odometry,Path
from queue import PriorityQueue

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *

class a_star_path_pub:
    def __init__(self):
        rospy.init_node('a_star_path_pub', anonymous=True)

        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        link_list_path_pub = rospy.Publisher('/link_list_path', PoseArray, queue_size=1)

        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.init_callback)

        #TODO: 1) Mgeo data 읽어온 후 데이터 확인
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/kcity'))
        mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set

        self.nodes = node_set.nodes
        self.links = link_set.lines

        self.global_planner = AStar(self.nodes, self.links)

        self.is_goal_pose = None
        self.is_init_pose = None

        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            self.global_path_msg = Path()
            self.global_path_msg.header.frame_id = '/map'
            self.global_path_pub.publish(self.global_path_msg)

            # wait for the first input
            while True:
                if self.is_goal_pose == True and self.is_init_pose == True:
                    break
                else:
                    rospy.loginfo('Waiting goal pose data')
                    rospy.loginfo('Waiting init pose data')

            # make global path
            self.global_path_msg, link_list = self.calc_a_star_path_node(self.start_node, self.end_node)
            
            link_path = PoseArray()
            for link in link_list :
                link_path.header.frame_id = 'map'        
                for point in link.points:
                    path_x = point[0]
                    path_y = point[1]
                    read_pose = Pose()
                    read_pose.position.x = path_x
                    read_pose.position.y = path_y
                    read_pose.orientation.w = 1
                    link_path.poses.append(read_pose)
                link_list_path_pub.publish(link_path)
                time.sleep(0.1)

            #TODO: (11) A* 이용해 만든 Global Path 정보 Publish
            self.global_path_pub.publish(self.global_path_msg)
            time.sleep(2)
            rate.sleep()

    def init_callback(self, msg):
        if self.is_init_pose is not False:
            self.is_goal_pose == False

        #TODO: (2) 시작 Node 와 종료 Node 정의
        # 시작 Node 는 Rviz 기능을 이용해 지정한 위치에서 가장 가까이 있는 Node 로 한다.
        start_min_dis = float('inf')
        self.init_msg = msg
        self.init_x = self.init_msg.pose.pose.position.x
        self.init_y = self.init_msg.pose.pose.position.y

        for node_idx in self.nodes:
            node_pose_x = self.nodes[node_idx].point[0]
            node_pose_y = self.nodes[node_idx].point[1]
            start_dis = sqrt(pow(self.init_x - node_pose_x, 2) + pow(self.init_y - node_pose_y, 2))
            if start_dis < start_min_dis:
                start_min_dis = start_dis
                self.start_node = node_idx

        self.is_init_pose = True

    def goal_callback(self, msg):
        #TODO: (2) 시작 Node 와 종료 Node 정의
        # 종료 Node 는 Rviz 기능을 이용해 지정한 위치에서 가장 가까이 있는 Node 로 한다.
        goal_min_dis = float('inf')
        self.goal_msg = msg
        self.goal_x = self.goal_msg.pose.position.x
        self.goal_y = self.goal_msg.pose.position.y

        for node_idx in self.nodes:
            node_pose_x = self.nodes[node_idx].point[0]
            node_pose_y = self.nodes[node_idx].point[1]
            goal_dis = sqrt(pow(self.goal_x - node_pose_x, 2) + pow(self.goal_y - node_pose_y, 2))

            if goal_dis < goal_min_dis:
                goal_min_dis = goal_dis
                self.end_node = node_idx

        self.is_goal_pose = True

    def calc_a_star_path_node(self, start_node, end_node):
        result, link_list, path = self.global_planner.find_shortest_path(start_node, end_node)

        #TODO: (10) A* 경로 데이터를 ROS Path 메세지 형식에 맞춰 정의
        out_path = Path()
        out_path.header.frame_id = '/map'

        for waypoint in path["point_path"]:
            path_x = waypoint[0]
            path_y = waypoint[1]
            read_pose = PoseStamped()
            read_pose.pose.position.x = path_x
            read_pose.pose.position.y = path_y
            read_pose.pose.orientation.w = 1
            out_path.poses.append(read_pose)

        
        return out_path, link_list

class AStar:
    def __init__(self, nodes, links):
        self.nodes = nodes
        self.links = links
        self.lane_change_link_idx = []

    def find_shortest_link_leading_to_node(self, from_node, to_node):
        #현재 노드에서 to_node로 연결되어 있는 링크를 찾고, 그 중에서 가장 빠른 링크를 찾아준다

        to_links = []
        for link in from_node.get_to_links():
            if link.to_node is to_node:
                to_links.append(link)

        if len(to_links) == 0:
            raise BaseException('[ERROR] Error @ AStar.find_shortest_path : Internal data error. There is no link from node to node')

        shortest_link = None
        min_cost = float('inf')
        for link in to_links:
            if len(link.points) < min_cost:
                min_cost = link.cost
                shortest_link = link

        return shortest_link, min_cost

    def heuristic(self, node1, node2):
        # TODO: (3) 휴리스틱 함수 정의
        # 두 노드 사이의 거리를 휴리스틱으로 사용
        # you can update heuristic function
        return sqrt(pow(node1.point[0] - node2.point[0], 2) + pow(node1.point[1] - node2.point[1], 2))

    def find_shortest_path(self, start_node_idx, end_node_idx):
        #TODO: (4) A* Path 초기화 로직
        # s 초기화         >> s = [False] * len(self.nodes)
        start_node = self.nodes[start_node_idx]
        end_node = self.nodes[end_node_idx]
        link_list = []
        que = PriorityQueue()
        dist = dict()
        from_node = dict()
        s = dict()
        for node_id in self.nodes :
            s[node_id] = False
        dist[start_node_idx] = 0

        #TODO: (5) A* 핵심 부분 구현
        que.put((dist[start_node_idx] + self.heuristic(start_node, end_node), start_node_idx))
        while(que.empty() is False):
            node_id = que.get()[1]
            if node_id == end_node_idx:
                break
            for link in self.nodes[node_id].get_to_links():
                next_node = link.get_to_node()
                if s[next_node.idx] is False :
                    from_node[next_node.idx] = node_id
                    s[next_node.idx] = True
                    dist[next_node.idx] = dist[node_id] + link.cost                    
                    que.put((dist[next_node.idx] + self.heuristic(next_node, end_node), next_node.idx))
                    link_list.append(link)


        #TODO: (6) node path 생성
        tracking_idx = end_node_idx
        node_path = [end_node_idx]

        while start_node_idx != tracking_idx:
            tracking_idx = from_node[tracking_idx]
            node_path.append(tracking_idx)

        node_path.reverse()

        #TODO: (7) link path 생성
        link_path = []
        for i in range(len(node_path) - 1):
            from_node_idx = node_path[i]
            to_node_idx = node_path[i + 1]

            from_node = self.nodes[from_node_idx]
            to_node = self.nodes[to_node_idx]

            shortest_link, min_cost = self.find_shortest_link_leading_to_node(from_node, to_node)
            link_path.append(shortest_link.idx)

        #TODO: (8) Result 판별
        if len(link_path) == 0:
            return False, link_list, {'node_path': node_path, 'link_path': link_path, 'point_path': []}

        #TODO: (9) point path 생성
        point_path = []
        for link_id in link_path:
            link = self.links[link_id]
            for point in link.points:
                point_path.append([point[0], point[1], 0])

        return True, link_list, {'node_path': node_path, 'link_path': link_path, 'point_path': point_path}



if __name__ == '__main__':
    a_star_path_pub = a_star_path_pub()