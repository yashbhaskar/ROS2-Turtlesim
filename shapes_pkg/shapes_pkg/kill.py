import rclpy
from rclpy.node import Node
from turtlesim.srv import Kill

class TurtleKiller(Node):
    def __init__(self):
        super().__init__('turtle_killer')
        self.client = self.create_client(Kill, 'kill')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /kill service...')

        self.req = Kill.Request()
        self.req.name = 'turtle1'

        self.future = self.client.call_async(self.req)
        self.get_logger().info('Kill request sent for turtle1.')

        self.future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            future.result()
            self.get_logger().info('Turtle1 has been killed.')
        except Exception as e:
            self.get_logger().error(f'Failed to kill turtle1: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleKiller()
    rclpy.spin_once(node, timeout_sec=5)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
