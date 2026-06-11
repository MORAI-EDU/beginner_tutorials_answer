#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

def warp_image(img, source_prop):
    image_size = (img.shape[1], img.shape[0])

    x = img.shape[1]
    y = img.shape[0]
    
    destination_points = np.float32([
        [0, y],
        [0, 0],
        [x, 0],
        [x, y]
    ])

    source_points = source_prop * np.float32([[x, y]] * 4)
    
    perspective_transform = cv2.getPerspectiveTransform(source_points, destination_points)
    warped_img = cv2.warpPerspective(img, perspective_transform, image_size, flags=cv2.INTER_LINEAR)
    
    return warped_img

class LaneBirdview(Node):
    def __init__(self):
        super().__init__('lane_birdview')
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        self.image_sub = self.create_subscription(
            CompressedImage, 
            "/camera/image/compressed", 
            self.callback, 
            qos_profile
        )        
        self.is_image = False
        
        self.img_bgr = None
        self.source_prop = np.float32([[0.01, 0.80],
                                       [0.5 - 0.14, 0.52],
                                       [0.5 + 0.14, 0.52],
                                       [1 - 0.01, 0.80]])
                                       
        self.timer = self.create_timer(1.0 / 10.0, self.timer_callback)

    def timer_callback(self):
        os.system('clear')
        if not self.is_image:
            print("[1] can't subscribe '/camera/image/compressed' topic... \n    please check your Camera sensor connection")
        else:
            print(f"""
                   Please change the camera parameters as follows:\n
                   FOV    : 90         roll  : 0.0        x : 1.8        
                   WIDTH  : 640        pitch : 7.0        y :  0        
                   HEIGHT : 480        yaw   : 0.0        z :  2
                   """)

        self.is_image = False

    def callback(self, msg):
        self.is_image = True

        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        img_warp = warp_image(self.img_bgr, self.source_prop)

        img_concat = np.concatenate([self.img_bgr, img_warp], axis=1)

        cv2.imshow("Image window", img_concat)
        cv2.waitKey(1) 


def main(args=None):
    rclpy.init(args=args)
    lane_birdview = LaneBirdview()
    
    try:
        rclpy.spin(lane_birdview)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        lane_birdview.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()