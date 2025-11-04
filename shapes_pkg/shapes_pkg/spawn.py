#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('turtle_spawner')
        self.client = self.create_client(Spawn, 'spawn')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        self.req = Spawn.Request()
        self.req.x = 2.0
        self.req.y = 2.0
        self.req.theta = 0.0
        self.req.name = 'turtle2'

        self.future = self.client.call_async(self.req)
        self.get_logger().info('Spawn request sent.')

        self.future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Turtle spawned: {response.name}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin_once(node, timeout_sec=5)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
