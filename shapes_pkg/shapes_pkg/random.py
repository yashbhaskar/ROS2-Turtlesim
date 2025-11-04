import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
import random
import time
 
class SafeZoneTurtle(Node):
    def __init__(self, turtle_name="turtle1"):
        super().__init__("safe_zone_turtle")
        self.turtle_name = turtle_name
        self.cmd_pub = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, f'/{turtle_name}/pose', self.pose_callback, 10)
        self.timer = self.create_timer(1.0, self.random_action)
 
        self.safe_min, self.safe_max = 2.0, 9.0  
        self.rescue_count =0
        self.srv = self.create_service(Trigger , 'rescue_counting', self.get_rescue_count)
 
    def random_action(self):
        twist = Twist()
        choice = random.choice(["forward", "left", "right"])
 
        if choice == "forward":
            twist.linear.x = 2.0
        elif choice == "left":
            twist.angular.z = 1.57
        elif choice == "right":
            twist.angular.z = -1.57
 
        self.get_logger().info(f"Action: {choice}")
        self.cmd_pub.publish(twist)
 
    def pose_callback(self, msg: Pose):
        x, y = msg.x, msg.y
        if not (self.safe_min <= x <= self.safe_max and self.safe_min <= y <= self.safe_max):
            # time.sleep(2.0)
            self.rescue_count +=1
            b=f"turtle needs rescue {self.turtle_name}"
            self.get_logger().warning(b)
        else:
            self.get_logger().info(f"{self.turtle_name} is safe inside zone ({x:.2f}, {y:.2f})")
    def get_rescue_count(self , req, resp):
        resp.success = True
        resp.message = f"turtle asked for {self.rescue_count} times"
        return resp
 
 
def main():
    rclpy.init()
    node = SafeZoneTurtle("turtle1")  
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()