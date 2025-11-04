from std_srvs.srv import Empty
import rclpy
from rclpy.node import Node

class clear(Node):
    def __init__(self):
        super().__init__('clear_node')
        self.clear = self.create_client(Empty,'/clear')

        while not self.clear.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("dff")
        self.clear_req=Empty.Request()

        self.future=self.clear.call_async(self.clear_req)
        self.future.add_done_callback(self.callback_fun)
    def callback_fun(self,future):
        response =future.result()
        self.get_logger().info("cleared")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    clear_obj = clear()
    rclpy.spin(clear_obj)

if __name__=='__main__':
    main()
