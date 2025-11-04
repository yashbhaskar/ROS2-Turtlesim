import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import time

class CircleNode(Node):
    def __init__(self):
        super().__init__('circle_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.draw_circle)
        self.get_logger().info("Starting circle...")

    def draw_circle(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9  # seconds

        if elapsed_time < 6.3:
            twist = Twist()
            twist.linear.x = 2.0     # Forward
            twist.angular.z = 1.0    # Rotate
            self.publisher_.publish(twist)
        else:
            twist = Twist()  # Zero velocities = stop
            self.publisher_.publish(twist)
            self.get_logger().info("Finished 1 full circle.")
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CircleNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
