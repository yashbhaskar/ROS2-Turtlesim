import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
 
class LineDrawer(Node):
    def __init__(self):
        super().__init__('line_drawer')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.target_x = 7.0
        self.target_y = 7.0
 
        self.pose = None
        self.reached = False
 
    def pose_callback(self, msg):
        self.pose = msg
        if not self.reached:
            self.move_to_point()
 
    def move_to_point(self):
        if self.pose is None:
            return
        dx = self.target_x - self.pose.x
        dy = self.target_y - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_target = math.atan2(dy, dx)
 
        msg = Twist()
 
        # If not facing target → rotate
        if abs(angle_to_target - self.pose.theta) > 0.05:
            msg.angular.z = 1.5 * (angle_to_target - self.pose.theta)
 
        # If not at target → move forward
        elif distance > 0.1:
            msg.linear.x = 1.0
 
        # If reached target → stop
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.reached = True
            self.get_logger().info("✅ Reached target point!")
 
        self.publisher.publish(msg)
 
def main(args=None):
    rclpy.init(args=args)
    node = LineDrawer()
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
 