#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

class IMGParser(Node):
    def __init__(self):
        super().__init__('image_parser')
        
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
        
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        os.system('clear')
        if not self.is_image:
            print("[1] can't subscribe '/camera/image/compressed' topic... \n    please check your Camera sensor connection")
        else:
            print("Camera sensor was connected !")

    def callback(self, msg):
        self.is_image = True
        
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        cv2.imshow("Image window", img_bgr)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    img_parser = IMGParser()
    
    try:
        rclpy.spin(img_parser)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        img_parser.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()