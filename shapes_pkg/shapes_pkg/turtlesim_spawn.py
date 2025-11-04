#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute, SetPen
from geometry_msgs.msg import Twist
import time
 
class SquareBorder(Node):
    def __init__(self):
        super().__init__('square_border')
 
        self.teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.set_pen = self.create_client(SetPen, '/turtle1/set_pen')
        self.cmd_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
 
        while not self.teleport.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for teleport service...")
        while not self.set_pen.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for set_pen service...")
 
        self.draw_square()
 
    def call_set_pen(self, r, g, b, width, off):
        req = SetPen.Request()
        req.r, req.g, req.b = r, g, b
        req.width = width
        req.off = off
        self.set_pen.call_async(req)
 
    def call_teleport(self, x, y, theta=0.0):
        req = TeleportAbsolute.Request()
        req.x, req.y, req.theta = x, y, theta
        self.teleport.call_async(req)
 
    def move_to(self, x, y):
        self.call_set_pen(200, 0, 0, 2, 1)  
        self.call_teleport(x, y)
        time.sleep(0.3)
        self.call_set_pen(200, 205, 255, 3, 0)
        time.sleep(0.2)
 
    def draw_line(self, x, y):
    
        self.call_teleport(x, y)
        time.sleep(0.5)
 
    def draw_square(self):
    
        self.move_to(2.0, 2.0)
        self.draw_line(9.0, 2.0)
        self.draw_line(9.0, 9.0)
        self.draw_line(2.0, 9.0)
        self.draw_line(2.0, 2.0)
        self.move_to(5.0, 5.0)
 
        self.get_logger().info("Square border drawn!")
 
def main(args=None):
    rclpy.init(args=args)
    node = SquareBorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
 