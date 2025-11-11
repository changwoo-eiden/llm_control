#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry

# TF
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose


def clamp(v, lo, hi): 
    return max(lo, min(hi, v))


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # ── Params (필요시 launch에서 override) ──
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('goal_tolerance', 0.15)   # [m] 위치 허용오차
        self.declare_parameter('goal_yaw_tol', 0.07)     # [rad] 각도 허용오차(≈4°)
        self.declare_parameter('lookahead_dist', 0.5)    # [m] 룩어헤드 거리
        self.declare_parameter('max_ang', 1.5)           # [rad/s] 각속 제한
        self.declare_parameter('min_lin', 0.12)          # [m/s] 최소 선속
        self.declare_parameter('max_lin', 0.40)          # [m/s] 최대 선속
        self.declare_parameter('use_final_orientation', True)  # 최종 포즈의 orientation 사용 여부

        self.odom_frame = self.get_parameter('odom_frame').value
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.goal_yaw_tol   = float(self.get_parameter('goal_yaw_tol').value)
        self.lookahead_dist = float(self.get_parameter('lookahead_dist').value)
        self.max_ang        = float(self.get_parameter('max_ang').value)
        self.min_lin        = float(self.get_parameter('min_lin').value)
        self.max_lin        = float(self.get_parameter('max_lin').value)
        self.use_final_orientation = bool(self.get_parameter('use_final_orientation').value)

        # ── ROS I/O ──
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/bcr_bot/odom', self.odom_callback, 10)
        self.cmd_pub  = self.create_publisher(Twist, '/bcr_bot/cmd_vel', 10)

        # ── State ──
        self.current_pose = None                  # geometry_msgs/Pose
        self.path = []                            # list[PoseStamped]
        self.path_frame = None
        self.goal_index = 0
        self.state = 'FOLLOW'                     # 'FOLLOW' | 'ALIGN'
        self.cached_desired_yaw = None            # ALIGN 단계에서 사용할 최종 yaw 캐시

        # ── TF ──
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Control timer ──
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz

        self.get_logger().info("PathFollower ready. (FOLLOW→ALIGN with final yaw)")

    # ─────────────────────────────────────────────────────────────────────
    # Subscriptions
    # ─────────────────────────────────────────────────────────────────────
    def path_callback(self, msg: Path):
        self.path = list(msg.poses)
        self.path_frame = msg.header.frame_id or 'map'
        self.goal_index = 0
        self.state = 'FOLLOW'
        self.cached_desired_yaw = None
        self.get_logger().info(f"✅ Path received: {len(self.path)} poses, frame={self.path_frame}")

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose  # odom frame 기준 포즈(보통)

    # ─────────────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────────────
    def transform_to_odom(self, pose_stamped: PoseStamped) -> PoseStamped:
        """pose_stamped(any frame) → odom frame 변환. 실패 시 원본 반환."""
        src = pose_stamped.header.frame_id or self.path_frame or 'map'
        if src == self.odom_frame:
            return pose_stamped
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, src, rclpy.time.Time())
            return do_transform_pose(pose_stamped, tf)
        except Exception as e:
            self.get_logger().warn(f"[TF] {src}→{self.odom_frame} transform failed: {e}")
            return pose_stamped

    @staticmethod
    def get_yaw_from_quat(q):
        # atan2(2(wz+xy), 1-2(y^2+z^2))
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    @staticmethod
    def quat_is_unit(q, tol=1e-3):
        n2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        return abs(n2 - 1.0) < tol

    # ─────────────────────────────────────────────────────────────────────
    # Main control
    # ─────────────────────────────────────────────────────────────────────
    def control_loop(self):
        if not self.path or self.current_pose is None:
            return

        # 현재 로봇 상태(odom)
        cx = self.current_pose.position.x
        cy = self.current_pose.position.y
        yaw = self.get_yaw_from_quat(self.current_pose.orientation)

        # ── 최근접 웨이포인트 탐색 (odom 기준으로 변환해서 비교) ──
        nearest_idx, nearest_dist = None, float('inf')
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

        # ── 룩어헤드 타깃 선택 ──
        target_idx = nearest_idx
        acc_dist = 0.0
        last_x, last_y = cx, cy
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

        # ── 목표점(odom) 좌표 ──
        target_ps = PoseStamped()
        target_ps.header.frame_id = self.path_frame or 'map'
        target_ps.pose = self.path[target_idx].pose
        target_odom = self.transform_to_odom(target_ps)

        gx, gy = target_odom.pose.position.x, target_odom.pose.position.y
        dx, dy = gx - cx, gy - cy
        dist = math.hypot(dx, dy)

        last_index = len(self.path) - 1
        is_last_segment = (target_idx >= last_index)

        # ── 최종 yaw 계산 (ALIGN 단계에서 사용) ──
        desired_yaw = None
        if self.use_final_orientation:
            # 최종 포즈를 odom으로 가져와 orientation 사용(유효할 때)
            final_ps = PoseStamped()
            final_ps.header.frame_id = self.path_frame or 'map'
            final_ps.pose = self.path[last_index].pose
            final_odom = self.transform_to_odom(final_ps)

            if self.quat_is_unit(final_odom.pose.orientation):
                desired_yaw = self.get_yaw_from_quat(final_odom.pose.orientation)
            else:
                desired_yaw = None  # 아래에서 접선으로 대체
        if desired_yaw is None:
            # 마지막 두 점의 접선 (fallback)
            prev_idx = max(last_index - 1, 0)
            prev_ps = PoseStamped()
            prev_ps.header.frame_id = self.path_frame or 'map'
            prev_ps.pose = self.path[prev_idx].pose
            prev_odom = self.transform_to_odom(prev_ps)
            vx = final_odom.pose.position.x - prev_odom.pose.position.x if 'final_odom' in locals() else (gx - cx)
            vy = final_odom.pose.position.y - prev_odom.pose.position.y if 'final_odom' in locals() else (gy - cy)
            if abs(vx) < 1e-6 and abs(vy) < 1e-6:
                desired_yaw = yaw
            else:
                desired_yaw = math.atan2(vy, vx)

        desired_yaw = self.normalize_angle(desired_yaw)

        # ── 상태 전환: 위치 OK면 ALIGN ──
        if is_last_segment and dist < self.goal_tolerance:
            self.state = 'ALIGN'
            # 한 번 계산한 최종 yaw를 캐시(미세 TF 흔들림 방지용)
            if self.cached_desired_yaw is None:
                self.cached_desired_yaw = desired_yaw

        cmd = Twist()

        # ── ALIGN: 각도만 맞추기 ──
        if self.state == 'ALIGN':
            yaw_ref = self.cached_desired_yaw if self.cached_desired_yaw is not None else desired_yaw
            yaw_err = self.normalize_angle(yaw_ref - yaw)

            if abs(yaw_err) > self.goal_yaw_tol:
                cmd.linear.x = 0.0
                cmd.angular.z = clamp(1.2 * yaw_err, -self.max_ang, self.max_ang)
                self.cmd_pub.publish(cmd)
                # 간헐 로그
                self.get_logger().info(f"🧭 ALIGN: yaw_err={yaw_err:.3f} → w={cmd.angular.z:.2f}")
                return
            else:
                # 위치+각도 완료 → 정지
                self.cmd_pub.publish(Twist())
                self.get_logger().info("🎯 Path completed (pos & yaw)")
                # 다음 경로 대비 초기화
                self.state = 'FOLLOW'
                self.cached_desired_yaw = None
                return

        # ── FOLLOW: 기존 추종 ──
        angle_to_goal = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_goal - yaw)

        # 종반부 감속(부드러운 정지)
        slow_scale = 1.0
        if is_last_segment:
            # lookahead보다 작아질수록 속도 줄임(0.2~1.0)
            slow_scale = max(0.2, min(1.0, dist / (self.lookahead_dist + 1e-6)))

        forward_gain = max(0.0, 1.0 - abs(angle_error))  # heading 잘 맞을수록 더 빠르게
        v_cmd = slow_scale * (self.min_lin + (self.max_lin - self.min_lin) * forward_gain)

        cmd.linear.x = v_cmd
        cmd.angular.z = clamp(0.9 * angle_error, -self.max_ang, self.max_ang)

        self.cmd_pub.publish(cmd)

        # 간헐 디버그 로그
        if is_last_segment or target_idx % 10 == 0:
            self.get_logger().info(
                f"[{self.state}] near_idx={nearest_idx}→tgt_idx={target_idx} "
                f"dist={dist:.2f} yawErr={angle_error:.2f} v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
