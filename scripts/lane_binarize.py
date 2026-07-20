#!/usr/bin/env python3

import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

class LaneBinarize(Node):
    def __init__(self):
        super().__init__('lane_binarize')
        
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

        self.timer = self.create_timer(1.0 / 10.0, self.timer_callback)

    def timer_callback(self):
        os.system('clear')
        if not self.is_image:
            print("[1] can't subscribe '/camera/image/compressed' topic... \n    please check your Camera sensor connection")
        else:
            print("Camera sensor was connected !")

        self.is_image = False

    def callback(self, msg):
        self.is_image = True
        
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        img_hsv = cv2.cvtColor(self.img_bgr, cv2.COLOR_BGR2HSV)

        lower_wlane = np.array([0, 0, 215])
        upper_wlane = np.array([30, 60, 255])

        img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)

        img_wlane = cv2.cvtColor(img_wlane, cv2.COLOR_GRAY2BGR)

        img_concat = np.concatenate([self.img_bgr, img_hsv, img_wlane], axis=1)

        cv2.imshow("Image window", img_concat)
        cv2.waitKey(1) 


def main(args=None):
    rclpy.init(args=args)
    lane_binarize = LaneBinarize()
    
    try:
        rclpy.spin(lane_binarize)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        lane_binarize.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()