#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_pub')
        self.pub = self.create_publisher(Twist, '/bcr_bot/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.2   # 앞으로 0.2 m/s
        msg.angular.z = 0.0  # 회전 없음
        self.pub.publish(msg)
        self.get_logger().info("Publishing test command: forward 0.2 m/s")

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

