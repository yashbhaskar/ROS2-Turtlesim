import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute
import time
from turtlesim.srv import Kill


class TurtleMover(Node):
    def __init__(self):
        super().__init__('turtle_mover')

        self.cli = self.create_client(TeleportAbsolute, '/turtle2/teleport_absolute')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        # Move to first point
        # self.move_to_point(5.544, 5.544, 0.0)
        # time.sleep(1)  # wait 1 second

        # Move to second point
        self.move_to_point(9.0, 2.0, 0.0)
        time.sleep(1)

        self.move_to_point(9.0, 9.0, 0.0)
        time.sleep(1)

        self.move_to_point(2.0, 9.0, 0.0)
        time.sleep(1)

        self.move_to_point(2.0, 2.0, 0.0)

        self.client = self.create_client(Kill, 'kill')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /kill service...')

        self.req = Kill.Request()
        self.req.name = 'turtle2'

        self.future = self.client.call_async(self.req)
        self.get_logger().info('Kill request sent for turtle1.')

        self.future.add_done_callback(self.callback)

    def move_to_point(self, x, y, theta):
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta

        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'Moved to ({x}, {y})')

    def callback(self, future):
        try:
            future.result()
            self.get_logger().info('Turtle1 has been killed.')
        except Exception as e:
            self.get_logger().error(f'Failed to kill turtle1: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleMover()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
