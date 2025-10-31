#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
import math

# ★ 추가
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

def clamp(v, lo, hi): return max(lo, min(hi, v))

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/bcr_bot/odom', self.odom_callback, 10)
        self.cmd_pub  = self.create_publisher(Twist, '/bcr_bot/cmd_vel', 10)

        self.current_pose = None
        self.path = []                 # 원본 path (map 프레임일 수 있음)
        self.path_frame = None         # path header frame_id 저장
        self.odom_frame = 'odom'       # odom 프레임명(필요시 변경)
        self.goal_index = 0

        # 튜닝 파라미터
        self.goal_tolerance = 0.15     # [m]
        self.lookahead_dist = 0.5      # [m] 이만큼 앞의 점을 목표로
        self.max_ang = 1.5             # [rad/s] 각속도 제한

        # ★ TF 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.control_loop)

    def path_callback(self, msg: Path):
        self.path = msg.poses
        self.path_frame = msg.header.frame_id or 'map'
        self.goal_index = 0
        self.get_logger().info(f"✅ Path: {len(self.path)} poses, frame={self.path_frame}")

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose  # odom frame 기준 포즈(보통)

    def transform_to_odom(self, pose_stamped: PoseStamped) -> PoseStamped:
        """pose_stamped(ANY frame) -> odom frame 으로 변환"""
        if (self.path_frame is None) or (self.path_frame == self.odom_frame):
            # 같은 프레임이면 그대로 사용
            return pose_stamped
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, pose_stamped.header.frame_id,
                rclpy.time.Time())  # 최신 TF
            return do_transform_pose(pose_stamped, tf)
        except Exception as e:
            # 변환 실패시, 일단 원본 반환(안전)
            self.get_logger().warn(f"TF transform failed: {e}")
            return pose_stamped

    def control_loop(self):
        if not self.path or self.current_pose is None:
            return

        # 현재 로봇 위치 (odom)
        cx, cy = self.current_pose.position.x, self.current_pose.position.y
        yaw = self.get_yaw_from_quat(self.current_pose.orientation)

        # === 1) 가장 가까운 웨이포인트 찾기 (odom 기준으로 변환해서 비교) ===
        nearest_idx = None
        nearest_dist = float('inf')

        for i, p in enumerate(self.path):
            ps = PoseStamped()
            ps.header.frame_id = self.path_frame or 'map'
            ps.pose = p.pose
            ps_odom = self.transform_to_odom(ps)

            px, py = ps_odom.pose.position.x, ps_odom.pose.position.y
            d = math.hypot(px - cx, py - cy)
            if d < nearest_dist:
                nearest_dist = d
                nearest_idx = i

        if nearest_idx is None:
            self.cmd_pub.publish(Twist())
            return

        # === 2) 룩어헤드로 목표 인덱스 선정 ===
        target_idx = nearest_idx
        acc_dist = 0.0
        last_x, last_y = cx, cy

        # 가까운 점부터 앞으로 진행하며 lookahead_dist까지 누적
        for j in range(nearest_idx, len(self.path)):
            ps = PoseStamped()
            ps.header.frame_id = self.path_frame or 'map'
            ps.pose = self.path[j].pose
            ps_odom = self.transform_to_odom(ps)

            px, py = ps_odom.pose.position.x, ps_odom.pose.position.y
            step = math.hypot(px - last_x, py - last_y)
            acc_dist += step
            last_x, last_y = px, py
            target_idx = j
            if acc_dist >= self.lookahead_dist:
                break

        # === 3) 목표점(odom)으로 제어 ===
        target_ps = PoseStamped()
        target_ps.header.frame_id = self.path_frame or 'map'
        target_ps.pose = self.path[target_idx].pose
        target_odom = self.transform_to_odom(target_ps)

        gx, gy = target_odom.pose.position.x, target_odom.pose.position.y
        dx, dy = gx - cx, gy - cy
        dist = math.hypot(dx, dy)

        # 도착 판정: 최종 점 근처면 멈춤
        if (target_idx >= len(self.path) - 1) and (dist < self.goal_tolerance):
            self.cmd_pub.publish(Twist())
            self.get_logger().info("🎯 Path completed")
            return

        angle_to_goal = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_goal - yaw)

        cmd = Twist()
        forward_gain = max(0.0, 1.0 - abs(angle_error))
        cmd.linear.x = 0.12 + 0.28 * forward_gain      # 0.12~0.40 m/s
        cmd.angular.z = clamp(0.9 * angle_error, -self.max_ang, self.max_ang)

        self.cmd_pub.publish(cmd)

        # 디버깅 로그(간헐적으로 보고 싶으면 주석 처리)
        self.get_logger().info(
            f"near_idx={nearest_idx} -> tgt_idx={target_idx} "
            f"dist={dist:.2f} yawErr={angle_error:.2f} "
            f"cmd v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}"
        )

    def get_yaw_from_quat(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, a):
        while a > math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
