#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

class LaneRoi(Node):
    def __init__(self):
        super().__init__('lane_roi')
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        self.image_sub = self.create_subscription(CompressedImage, "/camera/image/compressed", self.callback, qos_profile)
        self.is_image = False
        self.crop_pts = np.array(
            [[
                [0, 480],
                [280, 210],
                [360, 210],
                [640, 480]
            ]]
        )

        self.timer = self.create_timer(1.0 / 10.0, self.timer_callback)

    def timer_callback(self):
        os.system('clear')
        if not self.is_image:
            cv2.destroyAllWindows()
            print("[1] can't subscribe '/camera/image/compressed' topic... \n    please check your Camera sensor connection")
        else:
            print("Camera sensor was connected!")

        self.is_image = False

    def callback(self, msg):
        self.is_image = True
        
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        self.mask = self.mask_roi(img_bgr)

        if len(self.mask.shape) == 3:
            img_concat = np.concatenate([img_bgr, self.mask], axis=1)
        else:
            img_concat = np.concatenate([img_bgr, cv2.cvtColor(self.mask, cv2.COLOR_GRAY2BGR)], axis=1)

        cv2.imshow("lane_roi", img_concat)
        cv2.waitKey(1)

    def mask_roi(self, img):
        h = img.shape[0]
        w = img.shape[1]
        
        if len(img.shape) == 3:
            # image shape : [h, w, 3]
            c = img.shape[2]
            mask = np.zeros((h, w, c), dtype=np.uint8)
            mask_value = (255, 255, 255)
        else:
            # binarized image or grayscale image : [h, w]
            mask = np.zeros((h, w), dtype=np.uint8)
            mask_value = (255)

        cv2.fillPoly(mask, self.crop_pts, mask_value)
        mask = cv2.bitwise_and(mask, img)

        return mask


def main(args=None):
    rclpy.init(args=args)
    lane_roi = LaneRoi()
    
    try:
        rclpy.spin(lane_roi)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        lane_roi.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()