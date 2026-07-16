#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from beginner_tutorials.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info("Ready to add two ints.")

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info("Returning [%s + %s = %s]" % (request.a, request.b, response.sum))
        return response

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_server = AddTwoIntsServer()
    
    try:
        rclpy.spin(add_two_ints_server)
    except KeyboardInterrupt:
        pass
    finally:
        add_two_ints_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()