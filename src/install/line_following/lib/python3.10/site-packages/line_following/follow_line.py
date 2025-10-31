#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.sub = self.create_subscription(Image, '/kinect_camera/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_green = np.array([45, 100, 100])
        upper_green = np.array([75, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        h, w, _ = img.shape
        mask[:, 0:int(w*0.4)] = 0
        mask[:, int(w*0.6):] = 0

        M = cv2.moments(mask)
        cmd = Twist()

        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            err = cx - w / 2
            cmd.linear.x = 0.2
            cmd.angular.z = -float(err) / 100
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.3  # rotate to search

        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

