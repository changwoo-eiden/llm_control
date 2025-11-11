#!/usr/bin/env python3
import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

def obstacles_to_bboxes(path: str):
    with open(path, "r") as f:
        data = json.load(f)

    bboxes = []
    for r in data.get("regions", []):
        if r.get("type") == "obstacle":
            x, y = r["position"]
            w, h = r["size"]
            x1, x2 = x - w/2, x + w/2
            y1, y2 = y - h/2, y + h/2
            name = r.get("name", "obstacle")
            bboxes.append({
                "name": name,
                "bbox": [round(x1, 2), round(y1, 2), round(x2, 2), round(y2, 2)],
                "orientation": r.get("orientation", 0.0),
                "size": r.get("size", [0.0, 0.0]),
            })
    return bboxes

def compute_anchors(bboxes, standoff: float = 2.0):
    anchors = {}
    for item in bboxes:
        name = item["name"]
        x1, y1, x2, y2 = item["bbox"]
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        orientation = float(item.get("orientation", 0.0))

        # front
        fx = cx + standoff * math.cos(orientation)
        fy = cy + standoff * math.sin(orientation)
        fyaw = (orientation + math.pi) % (2 * math.pi)

        # back
        bx = cx - standoff * math.cos(orientation)
        by = cy - standoff * math.sin(orientation)
        byaw = orientation % (2 * math.pi)

        anchors[f"{name}.front"] = {"x": round(fx, 3), "y": round(fy, 3), "yaw": round(fyaw, 6)}
        anchors[f"{name}.back"]  = {"x": round(bx, 3), "y": round(by, 3), "yaw": round(byaw, 6)}
    return anchors

class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__("obstacle_publisher")

        self.declare_parameter("obstacle_path", "/home/changwoo/ros2_ws/src/bcr_bot/config/obstacle.json")
        path = self.get_parameter("obstacle_path").get_parameter_value().string_value

        # QoS: Transient Local + Reliable (래치)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability  = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(String, "/map_anchors", qos)

        # 페이로드 생성 후 즉시 1회 발행 + 타이머로 2회 추가 발행
        self.payload = self._build_payload(path)
        self._publish_once(1)
        self.count = 1
        self.timer = self.create_timer(0.7, self._tick)

    def _build_payload(self, path: str):
        bboxes = obstacles_to_bboxes(path)
        anchors = compute_anchors(bboxes, standoff=2.0)

        for item in bboxes:
            name = item["name"]
            f = anchors.get(f"{name}.front")
            b = anchors.get(f"{name}.back")
            if f:
                self.get_logger().info(
                    f"📍 {name}.front → x={f['x']:.3f}, y={f['y']:.3f}, yaw={f['yaw']:.6f} rad"
                )
            if b:
                self.get_logger().info(
                    f"📍 {name}.back  → x={b['x']:.3f}, y={b['y']:.3f}, yaw={b['yaw']:.6f} rad"
                )

        return {"bboxes": bboxes, "anchors": anchors}

    def _publish_once(self, idx: int):
        msg = String()
        msg.data = json.dumps(self.payload, ensure_ascii=False, separators=(",", ":"))
        self.pub.publish(msg)
        self.get_logger().info(f"📤 Published /map_anchors ({idx})")

    def _tick(self):
        if self.count >= 3:   
            self.timer.cancel()
            self.get_logger().info("✅ Finished publishing /map_anchors")
            return
        self.count += 1
        self._publish_once(self.count)

def main():
    rclpy.init()
    node = ObstaclePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
