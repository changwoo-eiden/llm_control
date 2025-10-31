#!/usr/bin/env python3
import rclpy, math, numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import atan2, cos, sin

def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return atan2(siny_cosp, cosy_cosp)

class DWALocalPlanner(Node):
    def __init__(self):
        super().__init__('dwa_local_planner')

        # Topics (당신 환경에 맞춤)
        self.create_subscription(Odometry, '/bcr_bot/odom', self.odom_cb, 10)
        self.create_subscription(Path, '/planned_path', self.path_cb, 10)
        self.create_subscription(LaserScan, '/bcr_bot/scan', self.scan_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/bcr_bot/cmd_vel', 10)

        # State
        self.pose = None                 # (x,y,yaw) in odom
        self.current_vel = (0.0, 0.0)    # (v,w)
        self.path = None                 # nav_msgs/Path
        self.scan = None

        # Params (회피 강화 세팅)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('predict_time', 1.5)
        self.declare_parameter('max_vel', 0.35)
        self.declare_parameter('max_w', 1.4)
        self.declare_parameter('acc_lim_v', 0.5)
        self.declare_parameter('acc_lim_w', 2.0)
        self.declare_parameter('robot_radius', 0.22)
        self.declare_parameter('safety_margin', 0.03)

        # Scoring weights
        self.declare_parameter('weight_clearance', 8.0)  # 크게!
        self.declare_parameter('weight_path', 1.0)
        self.declare_parameter('weight_vel', 0.5)
        self.declare_parameter('weight_heading', 1.0)    # lookahead 타깃 방향

        # Lookahead
        self.declare_parameter('lookahead_distance', 0.8)
        self.declare_parameter('lookahead_step', 8)      # 가까운 점에서 N칸 앞

        # Timer
        self.create_timer(self.get_parameter('dt').value, self.loop)

    # ---------- Callbacks ----------
    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_from_quat(q))
        self.current_vel = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def path_cb(self, msg: Path):
        self.path = msg

    def scan_cb(self, msg: LaserScan):
        self.scan = msg

    # ---------- Main Loop ----------
    def loop(self):
        if self.pose is None or self.path is None or self.scan is None or len(self.path.poses) == 0:
            return

        dt = self.get_parameter('dt').value
        max_v = self.get_parameter('max_vel').value
        max_w = self.get_parameter('max_w').value
        acc_v = self.get_parameter('acc_lim_v').value
        acc_w = self.get_parameter('acc_lim_w').value

        v0, w0 = self.current_vel

        # Dynamic window (가속도 제약 반영)
        v_min = max(-0.10, v0 - acc_v*dt)    # 살짝 후진 허용
        v_max = min(max_v, v0 + acc_v*dt)
        w_min = max(-max_w, w0 - acc_w*dt)
        w_max = min(max_w,  w0 + acc_w*dt)

        # 샘플 확대
        v_samples = np.linspace(v_min, v_max, 7)
        w_samples = np.linspace(w_min, w_max, 17)

        best = Twist(); best_score = -1e9

        # 룩어헤드 목표점 (경로 복귀 유도)
        target = self.get_lookahead_target()

        for v in v_samples:
            for w in w_samples:
                traj = self.simulate(v, w)  # [(x,y,th), ...] in odom
                collides, min_margin = self.check_clearance(traj, self.scan)
                score = self.evaluate(traj, collides, min_margin, v, target)

                if score > best_score:
                    best_score = score
                    best.linear.x = float(v)
                    best.angular.z = float(w)

        # 모든 후보가 충돌/저득점이면 회전으로 탈출 시도
        if best_score < -1e8:
            best.linear.x = 0.0
            best.angular.z = 0.5

        self.cmd_pub.publish(best)

    # ---------- Helpers ----------
    def simulate(self, v, w):
        dt = self.get_parameter('dt').value
        T  = self.get_parameter('predict_time').value
        x, y, th = self.pose
        traj = []
        steps = max(1, int(T/dt))
        for _ in range(steps):
            x += v*math.cos(th)*dt
            y += v*math.sin(th)*dt
            th += w*dt
            traj.append((x, y, th))
        return traj

    def check_clearance(self, traj, scan: LaserScan):
        """레이저 스캔과 예측 궤적의 최근접 여유거리. base_link로 변환 후 레이 인덱싱."""
        if not traj: return True, 0.0
        robot_r = self.get_parameter('robot_radius').value + self.get_parameter('safety_margin').value

        a0, inc = scan.angle_min, scan.angle_increment
        rmax    = scan.range_max
        ranges  = np.array(scan.ranges, dtype=float)

        x0, y0, th0 = self.pose
        c0, s0 = math.cos(th0), math.sin(th0)

        min_margin = float('inf')

        for (xw, yw, _) in traj:
            # odom -> base_link (현재 포즈 기준 근사)
            dx = xw - x0; dy = yw - y0
            xb =  dx*c0 + dy*s0
            yb = -dx*s0 + dy*c0

            ang  = math.atan2(yb, xb)
            dist = math.hypot(xb, yb)

            idx = int(round((ang - a0)/inc))
            if 0 <= idx < len(ranges):
                r = ranges[idx]
            else:
                r = rmax  # FOV 밖 → 멀다고 가정

            r_eff = r if np.isfinite(r) else rmax
            margin = r_eff - dist
            min_margin = min(min_margin, margin)

            # ★ 충돌 판정 완화: 실제 기체 크기 기준
            if margin < robot_r:
                return True, min_margin

        return False, min_margin

    def get_lookahead_target(self):
        """현재 위치에서 경로상의 최근접 인덱스를 찾고 몇 칸 앞을 목표로."""
        lx = self.get_parameter('lookahead_distance').value
        step = int(self.get_parameter('lookahead_step').value)
        x, y, _ = self.pose
        # 최근접 점
        dmin = 1e9; idx = 0
        for i, ps in enumerate(self.path.poses):
            dx = ps.pose.position.x - x
            dy = ps.pose.position.y - y
            d = dx*dx + dy*dy
            if d < dmin:
                dmin = d; idx = i
        idx = min(idx + step, len(self.path.poses)-1)
        p = self.path.poses[idx].pose.position
        return (p.x, p.y)

    def evaluate(self, traj, collides, min_margin, v_end, target_xy):
        if not traj: return -1e9
        if collides: return -1e9

        w_clear = self.get_parameter('weight_clearance').value
        w_path  = self.get_parameter('weight_path').value
        w_vel   = self.get_parameter('weight_vel').value
        w_head  = self.get_parameter('weight_heading').value

        # 1) Clearance (여유거리 클수록 좋음)
        # 최소여유를 0~1 범위로 약식 정규화
        norm_clear = max(0.0, min(1.0, min_margin))
        score_clear = norm_clear

        # 2) Path / Lookahead 정렬 (종단점이 타깃에 가까울수록)
        gx, gy, gth = traj[-1]
        tx, ty = target_xy
        d = math.hypot(tx - gx, ty - gy)
        score_path = -d

        # 3) Heading: 종단 헤딩이 타깃 방향을 바라보면 가산
        desired_yaw = atan2(ty - gx, tx - gy)
        # 주의: 위 식에 실수가 있어 below 수정
        desired_yaw = atan2(ty - gy, tx - gx)
        dyaw = self.wrap_angle(desired_yaw - gth)
        score_head = -abs(dyaw)

        # 4) 진행 보상
        score_vel = v_end

        return (w_clear*score_clear
                + w_path*score_path
                + w_head*score_head
                + w_vel*score_vel)

    @staticmethod
    def wrap_angle(a):
        while a > math.pi:  a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DWALocalPlanner())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
