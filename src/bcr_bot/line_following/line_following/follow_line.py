import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class LineFollower(Node):

    def __init__(self) -> None:
        super().__init__('line_follower')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/kinect_camera/image_raw',  # Update this topic as needed
            self.camera_callback,
            10
        )
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 10)

    def camera_callback(self, msg: Image) -> None:
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().info(f"Error converting image: {e}")
            return

        height, width, _ = cv_image.shape
        rows_to_watch = 20
        crop_img = cv_image[height*3//4:height*3//4 + rows_to_watch, 1:width]

        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # ✅ Green color range in HSV
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])

        mask = cv2.inRange(hsv, lower_green, upper_green)
        moment = cv2.moments(mask, False)

        try:
            cx = moment['m10'] / moment['m00']
            cy = moment['m01'] / moment['m00']
        except ZeroDivisionError:
            cx, cy = width / 2, rows_to_watch / 2

        res = cv2.bitwise_and(crop_img, crop_img, mask=mask)
        cv2.circle(res, (int(cx), int(cy)), 10, (0, 0, 255), -1)
        cv2.imshow("Green Line Tracking", res)
        cv2.waitKey(1)

        error_x = cx - width / 2
        self.pub_velocities(error_x)

    def pub_velocities(self, error: float) -> None:
        twist_object = Twist()
        twist_object.linear.x = 0.2
        twist_object.angular.z = -error / 100
        self.get_logger().info(f"Green line angular adjustment: {twist_object.angular.z:.2f}")
        self.pub_vel.publish(twist_object)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
