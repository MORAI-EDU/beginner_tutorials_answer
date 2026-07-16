#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from morai_ros2_msgs.msg import CtrlCmd

class SDrive(Node):
    def __init__(self):
        super().__init__('s_drive')
        self.cmd_pub = self.create_publisher(CtrlCmd, '/ctrl_cmd', 1)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.cmd = CtrlCmd()
        self.cmd.longl_cmd_type = 2
        self.cmd.velocity = 20.0 
        self.steering_cmd = [0.2, -0.2]
        self.cmd_cnts = 50
        self.current_idx = 0
        self.counter = 0

    def timer_callback(self):
        if self.counter == 0:
            self.cmd.front_steer = self.steering_cmd[self.current_idx]
            self.get_logger().info(f'CtrlCmd: steering={self.cmd.front_steer}')

        self.cmd_pub.publish(self.cmd)
        self.counter += 1
        if self.counter >= self.cmd_cnts:
            self.counter = 0
            self.current_idx = (self.current_idx + 1) % 2

def main(args=None):
    rclpy.init(args=args)
    s_d = SDrive()
    try:
        rclpy.spin(s_d)
    except KeyboardInterrupt:
        pass
    finally:
        s_d.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()