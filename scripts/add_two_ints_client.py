#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from beginner_tutorials.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print("%s [x y]" % sys.argv[0])
        sys.exit(1)

    add_two_ints_client = AddTwoIntsClient()
    print("Requesting %s+%s" % (x, y))
    
    response = add_two_ints_client.send_request(x, y)
    if response is not None:
        print("%s + %s = %s" % (x, y, response.sum))
    else:
        print("Service call failed")

    add_two_ints_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()