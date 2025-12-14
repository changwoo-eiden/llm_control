#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.current_pose = None

        # goal publisher (예: planner → path_follower로 가는 상위 목표)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal', 10)

        # odom 구독 (현재 위치 확인용, 디버그/타임아웃 로그 등에 사용)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/bcr_bot/odom',
            self.odom_cb,
            qos_profile_sensor_data,
        )

        # ✅ PathFollower ALIGN 완료 신호 구독
        self.pf_done = False
        self.pf_done_sub = self.create_subscription(
            Bool,
            '/path_follower/done',
            self.pf_done_cb,
            10,
        )

        self.get_logger().info("Navigator ready (waiting for PathFollower /path_follower/done).")

    # ─────────────────────────────
    #  Callbacks
    # ─────────────────────────────
    def odom_cb(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def pf_done_cb(self, msg: Bool):
        # PathFollower에서 ALIGN까지 끝난 뒤 True 발행
        if msg.data:
            self.get_logger().info("🎯 PathFollower reported: path completed (pos & yaw).")
            self.pf_done = True
        else:
            # 새 path 시작 시 False가 올 수 있음 → 상태 초기화용
            self.pf_done = False

    # ─────────────────────────────
    #  Public API: goto
    # ─────────────────────────────
    def goto(self, x: float, y: float, yaw: float = 0.0, align_timeout: float = 20.0) -> bool:
        """
        - /goal 토픽으로 목표 pose를 publish
        - PathFollower가 FOLLOW → ALIGN → yaw까지 맞춘 뒤 /path_follower/done=True 보낼 때까지 대기
        - align_timeout 안에 완료 신호가 안 오면 False 반환
        """
        # PathFollower 완료 플래그 초기화
        self.pf_done = False

        # 1) 목표 pose 구성
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        # 2) goal 여러 번 publish (latched topic이 아닐 수 있으니)
        self.get_logger().info(f"📌 Publishing goal: x={x:.3f}, y={y:.3f}, yaw={yaw:.4f}")
        for _ in range(5):
            self.goal_pub.publish(goal)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.05)
        self.get_logger().info(f"🚀 Goal sent: ({x:.2f}, {y:.2f}) — waiting for PathFollower ALIGN...")

        # 3) PathFollower의 ALIGN 완료(/path_follower/done=True)까지 대기
        start = time.time()
        while rclpy.ok():
            # 콜백 처리 (odom, pf_done 등)
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.pf_done:
                self.get_logger().info(
                    f"✅ FULL arrival at ({x:.2f}, {y:.2f}) with yaw aligned (reported by PathFollower)."
                )
                return True

            if time.time() - start > align_timeout:
                # 디버그용으로 현재 위치와 거리를 한 번 찍어주자
                if self.current_pose is not None:
                    cx = self.current_pose.position.x
                    cy = self.current_pose.position.y
                    dist = math.hypot(x - cx, y - cy)
                    self.get_logger().warn(
                        f"⚠️ PathFollower ALIGN timeout ({align_timeout:.1f}s). "
                        f"Current dist to goal ≈ {dist:.2f} m"
                    )
                else:
                    self.get_logger().warn(
                        f"⚠️ PathFollower ALIGN timeout ({align_timeout:.1f}s). No odom yet."
                    )
                return False

        # rclpy.ok()가 False가 된 경우 (노드 종료 등)
        self.get_logger().warn("⚠️ goto aborted: ROS shutdown.")
        return False

    # --- 회전 유틸 (현재는 사용 안 하지만 남겨둠) ---
    def _yaw_from_quat(self, q):
        s = 2.0 * (q.w * q.z + q.x * q.y)
        c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(s, c)


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
