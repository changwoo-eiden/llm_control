#!/usr/bin/env python3
import heapq
import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


# ========== Orientation Utilities ==========
def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q

def quat_to_yaw(q: Quaternion) -> float:
    s = 2.0 * (q.w * q.z + q.x * q.y)
    c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(s, c)


class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')

        # QoS for /map (latched)
        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)

        self.start_sub = self.create_subscription(
            PoseStamped, '/start', self.start_callback, 10)

        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal', self.goal_callback, 10)

        # Publisher
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        # Map & poses
        self.grid = None
        self.map_info = None
        self.map_frame = 'map'
        self.start_world: Optional[Tuple[float, float]] = None
        self.goal_world: Optional[Tuple[float, float]] = None
        self.goal_yaw: Optional[float] = None  # ✅ goal orientation 저장

        # Parameters
        self.declare_parameter('occupied_threshold', 50)   # 0~100
        self.declare_parameter('unknown_as_obstacle', True)
        self.declare_parameter('use_8_connected', True)
        self.declare_parameter('inflate_radius', 2)        # 셀 단위 확장 반경

        self.get_logger().info('A* planner ready. Publish /start and /goal PoseStamped.')

    # ========== ROS Callbacks ==========

    def map_callback(self, msg: OccupancyGrid):
        self.map_info = msg.info
        self.map_frame = msg.header.frame_id or 'map'

        w, h = self.map_info.width, self.map_info.height
        data = msg.data  # flat list length = w*h

        occ_th = self.get_parameter('occupied_threshold').value
        unknown_block = self.get_parameter('unknown_as_obstacle').value
        inflate_radius = self.get_parameter('inflate_radius').value

        # Build 2D grid: 0 free, 1 blocked
        grid = [[0]*w for _ in range(h)]
        for y in range(h):
            base = y * w
            for x in range(w):
                v = data[base + x]
                if v < 0:
                    grid[y][x] = 1 if unknown_block else 0
                elif v >= occ_th:
                    grid[y][x] = 1
                else:
                    grid[y][x] = 0

        # 🔽 장애물 확장 (Inflation: 정사각 마스크)
        inflated = [[0]*w for _ in range(h)]
        for y in range(h):
            for x in range(w):
                if grid[y][x] == 1:  # 장애물인 셀
                    for dy in range(-inflate_radius, inflate_radius+1):
                        for dx in range(-inflate_radius, inflate_radius+1):
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < w and 0 <= ny < h:
                                inflated[ny][nx] = 1

        # grid에 반영
        for y in range(h):
            for x in range(w):
                if inflated[y][x] == 1:
                    grid[y][x] = 1

        self.grid = grid
        self.get_logger().info(
            f"Map received: {w}x{h}, res={self.map_info.resolution:.3f} m/cell, "
            f"origin=({self.map_info.origin.position.x:.2f}, {self.map_info.origin.position.y:.2f}), "
            f"inflated radius={inflate_radius} cells"
        )

        # If we already have start & goal, try plan
        self.try_plan()

    def start_callback(self, msg: PoseStamped):
        self.start_world = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"Start set: {self.start_world}")
        self.try_plan()
    
    def goal_callback(self, msg: PoseStamped):
        self.goal_world = (msg.pose.position.x, msg.pose.position.y)
        self.goal_yaw = quat_to_yaw(msg.pose.orientation)  # ✅ 목표 yaw 저장
        self.get_logger().info(
            f"[CALLBACK] Goal set: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}), yaw={self.goal_yaw:.3f} rad"
        )
        self.try_plan()

    # ========== Core Planning Flow ==========

    def try_plan(self):
        if self.grid is None or self.map_info is None:
            return
        if self.start_world is None or self.goal_world is None:
            return

        sx, sy = self.world_to_map(*self.start_world)
        gx, gy = self.world_to_map(*self.goal_world)
        self.get_logger().info(f"indices start=({sx},{sy}) goal=({gx},{gy})")

        if not self.in_bounds(sx, sy) or not self.in_bounds(gx, gy):
           self.get_logger().warn("Start or Goal out of map bounds.")
           return
    
        if self.grid[sy][sx] == 1 or self.grid[gy][gx] == 1:
            self.get_logger().warn("Start or Goal is in obstacle.")
            return


        use8 = self.get_parameter('use_8_connected').value
        path_cells = self.astar((sx, sy), (gx, gy), use8)

        if not path_cells:
            self.get_logger().warn("No path found.")
            return

        # Convert to nav_msgs/Path in world coordinates
        path_msg = Path()
        path_msg.header = Header(frame_id=self.map_frame, stamp=self.get_clock().now().to_msg())

        # 🔽 샘플링 간격 (m 단위)
        sample_dist = 0.25
        last_wx, last_wy = None, None

        for i, (cx, cy) in enumerate(path_cells):
            wx, wy = self.map_to_world(cx, cy)

            if last_wx is None or math.hypot(wx - last_wx, wy - last_wy) >= sample_dist:
                ps = PoseStamped()
                ps.header = path_msg.header
                ps.pose.position.x = wx
                ps.pose.position.y = wy

                # 다음 점 방향으로 orientation 계산
                if i < len(path_cells) - 1:
                    nx, ny = path_cells[i+1]
                    wx2, wy2 = self.map_to_world(nx, ny)
                    yaw = math.atan2(wy2 - wy, wx2 - wx)
                else:
                    yaw = 0.0  # 마지막 샘플은 임시로 0 (아래에서 최종 goal로 덮음)

                ps.pose.orientation = yaw_to_quaternion(yaw)

                path_msg.poses.append(ps)
                last_wx, last_wy = wx, wy

        # === Pre-Goal 삽입 + 최종 Goal 자세 세팅 ===
        gx_w, gy_w = self.map_to_world(gx, gy)

        # 목표 yaw: /goal에 실린 yaw가 있으면 사용, 아니면 마지막 세그먼트 방향
        if self.goal_yaw is not None:
            yaw_goal = self.goal_yaw
        elif len(path_msg.poses) >= 2:
            p_prev = path_msg.poses[-1].pose.position
            yaw_goal = math.atan2(gy_w - p_prev.y, gx_w - p_prev.x)
        else:
            yaw_goal = 0.0

        # pre-goal: goal에서 yaw 반대방향으로 eps 만큼 뒤
        eps = 0.35  # goal_tolerance보다 살짝 크게(예: 0.15~0.2 -> 0.30~0.40)
        px = gx_w - eps * math.cos(yaw_goal)
        py = gy_w - eps * math.sin(yaw_goal)

        pre_ps = PoseStamped()
        pre_ps.header = path_msg.header
        pre_ps.pose.position.x = px
        pre_ps.pose.position.y = py
        pre_ps.pose.orientation = yaw_to_quaternion(yaw_goal)

        def dist_xy(a: PoseStamped, b: PoseStamped):
            return math.hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y)

        if len(path_msg.poses) == 0 or dist_xy(path_msg.poses[-1], pre_ps) > 0.05:
            path_msg.poses.append(pre_ps)

        goal_ps = PoseStamped()
        goal_ps.header = path_msg.header
        goal_ps.pose.position.x = gx_w
        goal_ps.pose.position.y = gy_w
        goal_ps.pose.orientation = yaw_to_quaternion(yaw_goal)
        path_msg.poses.append(goal_ps)

        self.path_pub.publish(path_msg)
        self.get_logger().info(
            f"Path published: {len(path_msg.poses)} poses (sampled every {sample_dist} m, with pre-goal, yaw={yaw_goal:.3f})."
        )

    # ========== A* Implementation ==========

    def astar(self, start: Tuple[int, int], goal: Tuple[int, int], use8=True) -> List[Tuple[int, int]]:
        (sx, sy) = start
        (gx, gy) = goal

        def h(x, y):
            dx, dy = abs(x - gx), abs(y - gy)
            if use8:
                # Octile heuristic
                return (dx + dy) + (1.41421356237 - 2) * min(dx, dy)
            else:
                return dx + dy

        neigh4 = [(1,0),(-1,0),(0,1),(0,-1)]
        neigh8 = neigh4 + [(1,1),(1,-1),(-1,1),(-1,-1)]
        neigh = neigh8 if use8 else neigh4

        open_heap = []
        heapq.heappush(open_heap, (h(sx, sy), 0.0, (sx, sy)))
        came_from = {}
        g_score = {(sx, sy): 0.0}
        closed = set()

        while open_heap:
            _, gc, (x, y) = heapq.heappop(open_heap)
            if (x, y) in closed:
                continue
            if (x, y) == (gx, gy):
                return self.reconstruct(came_from, (gx, gy))

            closed.add((x, y))
            for dx, dy in neigh:
                nx, ny = x + dx, y + dy
                if not self.in_bounds(nx, ny):
                    continue
                if self.grid[ny][nx] == 1:
                    continue
                step = 1.41421356237 if dx != 0 and dy != 0 else 1.0
                tentative = gc + step
                if tentative < g_score.get((nx, ny), float('inf')):
                    g_score[(nx, ny)] = tentative
                    came_from[(nx, ny)] = (x, y)
                    f = tentative + h(nx, ny)
                    heapq.heappush(open_heap, (f, tentative, (nx, ny)))
        return []

    def reconstruct(self, came_from, cur):
        path = [cur]
        while cur in came_from:
            cur = came_from[cur]
            path.append(cur)
        path.reverse()
        return path

    # ========== Helpers ==========

    def world_to_map(self, wx: float, wy: float) -> Tuple[int, int]:
        ox, oy = self.map_info.origin.position.x, self.map_info.origin.position.y
        res = self.map_info.resolution
        mx = math.floor((wx - ox) / res)
        my = math.floor((wy - oy) / res)
        return int(mx), int(my)

    def map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        ox, oy = self.map_info.origin.position.x, self.map_info.origin.position.y
        res = self.map_info.resolution
        wx = ox + (mx + 0.5) * res
        wy = oy + (my + 0.5) * res
        return wx, wy

    def in_bounds(self, x: int, y: int) -> bool:
        return (0 <= x < self.map_info.width) and (0 <= y < self.map_info.height)


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
