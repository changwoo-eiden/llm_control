import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class AutoStartPublisher(Node):
    def __init__(self):
        super().__init__('auto_start_publisher')
        self.declare_parameter('odom_topic', '/bcr_bot/odom')
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value

        self.sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.pub = self.create_publisher(PoseStamped, '/start', 10)
        self.published_once = False
        self.get_logger().info(f"Subscribed to: {odom_topic}")

    def odom_callback(self, msg):
         #self.get_logger().info("Received /odom message ✅")
         if self.published_once:
             return
         try:
            pose = msg.pose.pose
            msg_out = PoseStamped()
            msg_out.header.frame_id = 'map'
            msg_out.header.stamp = self.get_clock().now().to_msg()
            msg_out.pose = pose
            self.pub.publish(msg_out)

        
            #self.get_logger().info(f"/start published at ({pose.position.x}, {pose.position.y}) ✅")
            self.published_once = True

         except Exception as e:
            self.get_logger().error(f"Failed to publish /start: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = AutoStartPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
