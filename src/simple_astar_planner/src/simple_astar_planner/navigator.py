#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math, time


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.current_pose = None
        self.last_twist = None

        self.goal_pub = self.create_publisher(PoseStamped, '/goal', 10)

        

        # ✅ 센서 QoS 권장
        self.odom_sub = self.create_subscription(
            Odometry, '/bcr_bot/odom', self.odom_cb, qos_profile_sensor_data
        )

        # ❌ 루프백 구독은 제거 권장 (혼란의 원인)
        # self.goal_check_sub = self.create_subscription(
        #     PoseStamped, '/goal',
        #     lambda msg: self.get_logger().info(f"Loopback got goal {msg.pose.position}"), 10
        # )

    def odom_cb(self, msg: Odometry):
        self.current_pose = msg.pose.pose


    def goto(self, x, y, yaw=0.0, tol=0.2):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()  # ✅ 타임스탬프
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
    
        self.get_logger().info(f"Publishing goal: {x:.3f}, {y:.3f}, yaw={yaw:.4f}")
        for _ in range(5):
            self.goal_pub.publish(goal)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.05)
        self.get_logger().info(f"Goal sent: ({x:.2f}, {y:.2f})")

        # 위치 도착 판정
        while rclpy.ok():
            if self.current_pose is None:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue
            cx, cy = self.current_pose.position.x, self.current_pose.position.y
            dist = math.hypot(x - cx, y - cy)
            if dist < tol:
                self.get_logger().info(f"✅ Arrived at ({x:.2f}, {y:.2f})")
                break
            rclpy.spin_once(self, timeout_sec=0.1)


    # --- 회전 유틸 ---
    def _yaw_from_quat(self, q):
        s = 2.0 * (q.w*q.z + q.x*q.y)
        c = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        return math.atan2(s, c)


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    rclpy.spin(node)   
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
