#!/usr/bin/env python3
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from openai import OpenAI

from simple_astar_planner.navigator import Navigator
from simple_astar_planner.takephoto import PhotoTaker
from simple_astar_planner.detection_client import Detector
import os

api_key = os.getenv("OPENAI_API_KEY")
client = OpenAI(api_key=api_key)


def _validate_and_normalize_sequence(seq: list) -> list:
    cleaned = []
    for i, step in enumerate(seq, start=1):
        if not isinstance(step, dict) or "action" not in step:
            raise ValueError(f"{i}번째 스텝이 잘못됨: {step}")
        act = step["action"]

        if act == "goto":
            for k in ("x", "y", "yaw"):
                if k not in step:
                    raise ValueError(f"{i}번째 goto에 {k}가 필요합니다: {step}")
            x = float(step["x"]); y = float(step["y"]); yaw = float(step["yaw"])
            cleaned.append({"action": "goto", "x": x, "y": y, "yaw": yaw})

        elif act == "wait":
            if "duration" not in step:
                raise ValueError(f"{i}번째 wait에 duration이 필요합니다: {step}")
            duration = float(step["duration"])
            cleaned.append({"action": "wait", "duration": duration})

        elif act == "photo":
            cleaned.append({"action": "photo"})

        elif act == "detection":
            # class 또는 (arg/target) 허용
            cls = step.get("class") or step.get("arg") or step.get("target")
            if not cls or not isinstance(cls, str):
                raise ValueError(f"{i}번째 detection에 class/arg/target 중 하나가 필요합니다: {step}")
            timeout = float(step.get("timeout", 5.0))
            cleaned.append({"action": "detection", "class": cls.strip(), "timeout": timeout})

        else:
            raise ValueError(f"{i}번째 스텝 action 알 수 없음: {act}")
    return cleaned



class GPTAPIController(Node):
    def __init__(self):
        super().__init__("gpt_api_controller")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability  = DurabilityPolicy.TRANSIENT_LOCAL

        self.map_info = None
        self.sub_map = self.create_subscription(String, "/map_anchors", self.map_callback, qos)
        self.get_logger().info("🛰️ Waiting for /map_anchors data...")

        self.navigator = Navigator()
        self.photo = PhotoTaker(self.navigator)
        self.detector = Detector(self)
        
    def map_callback(self, msg):
        try:
            self.map_info = json.loads(msg.data)
            self.get_logger().info("✅ Received /map_anchors update")
        except Exception as e:
            self.get_logger().error(f"Failed to parse /map_anchors: {e}")

    def ask_gpt_for_plan(self, user_command: str) -> list:
        if self.map_info is None:
            self.get_logger().warn("❌ Map info not received yet.")
            return []

        bboxes  = self.map_info["bboxes"]
        anchors = self.map_info["anchors"]

        map_json     = json.dumps({"forbidden_bboxes": bboxes}, ensure_ascii=False, separators=(",", ":"))
        anchors_json = json.dumps({"anchors": anchors},       ensure_ascii=False, separators=(",", ":"))

        SEQUENCE_SCHEMA = {
    "type": "object",
    "properties": {
        "sequence": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "action":   {"type": "string", "enum": ["goto", "photo", "wait", "detection"]},
                    "x":        {"type": "number"},
                    "y":        {"type": "number"},
                    "yaw":      {"type": "number"},
                    "duration": {"type": "number"},
                    "class":    {"type": "string"},
                    "timeout":  {"type": "number"},
                    # (필요시 간단 alias 허용)
                    "arg":      {"type": "string"},
                    "target":    {"type": "string"}
                },
                "required": ["action"],
                "additionalProperties": False
            }
        }
    },
    "required": ["sequence"],
    "additionalProperties": False
}


        system_prompt = (
            "너는 이동 로봇 플래너다. 오직 JSON으로만 답한다. "
            "좌표는 map 프레임, 단위는 미터, yaw는 라디안이다. "
            "가능한 액션은 goto(x,y,yaw), photo(), wait(duration) 뿐이다. "
            "불필요한 텍스트 없이 JSON만 출력한다.\n"
            "규칙(아주 중요):\n"
            "A. 아래 제공된 앵커 JSON에 들어있는 좌표만 사용한다. 임의의 x,y,yaw를 발명하지 않는다.\n"
            "B. 사용자가 '책상 앞/뒤'처럼 객체 기준을 지시하면 해당 키(예: '책상.front','책상.back')의 좌표(x,y,yaw)를 그대로 goto에 넣는다.\n"
            "C. 앵커 JSON에 없는 객체 이름/면을 요구하면 계획을 중단한다(좌표를 추정하지 말 것).\n"
            "D. 모든 GOTO (x,y)는 아래 금지영역 AABB 안으로는 들어가면 안 된다.\n"
            "E. photo 전에 wait 2초를 넣는다. '돌아와'는 (0,0,0)으로 복귀.\n"
            "F. 출력은 JSON Schema를 반드시 만족시킨다.\n"
            "BEGIN_FORBIDDEN\n"
            f"{map_json}\n"
            "END_FORBIDDEN\n"
            "BEGIN_ANCHORS\n"
            f"{anchors_json}\n"
            "END_ANCHORS\n"
            "G. detection(class)를 사용하면 YOLO가 해당 class를 timeout 내 찾는지 확인한다. 찾으면 다음 스텝으로 진행한다.\n"

        )

        try:
            resp = client.chat.completions.create(
                model="gpt-4o-mini",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_command}
                ],
                response_format={
                    "type": "json_schema",
                    "json_schema": {
                        "name": "sequence_schema",
                        "schema": SEQUENCE_SCHEMA,
                        "strict": True
                    }
                },
                temperature=0
            )
            content = resp.choices[0].message.content
            self.get_logger().info(f"📥 GPT 응답(Structured): {content}")

            data = json.loads(content)
            seq = data.get("sequence", [])
            seq = _validate_and_normalize_sequence(seq)

            return seq

        except Exception as e:
            self.get_logger().error(f"❌ GPT 요청 실패: {e}")
            return []

    def run_sequence(self, sequence: list):
        for idx, step in enumerate(sequence, start=1):
            act = step["action"]

            if act == "goto":
                x, y, yaw = step["x"], step["y"], step.get("yaw", 0.0)
                self.get_logger().info(f"[{idx}] 🎯 GOTO: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
                self.navigator.goto(x, y, yaw=yaw)

            elif act == "wait":
                duration = step["duration"]
                self.get_logger().info(f"[{idx}] ⏸ WAIT: {duration:.2f}s")
                end = time.monotonic() + duration
                while rclpy.ok() and time.monotonic() < end:
                    rclpy.spin_once(self.navigator, timeout_sec=0.1)

            elif act == "photo":
                self.get_logger().info(f"[{idx}] 📸 PHOTO")
                self.photo.take_photo()
            
            elif act == "detection":
                cls = step["class"]
                timeout = float(step.get("timeout", 5.0))
                self.get_logger().info(f"[{idx}] 🔎 DETECTION: class='{cls}', timeout={timeout:.2f}s")
                found, info = self.detector.detect(cls, timeout_sec=timeout)
                if found:
                    self.get_logger().info(f"[{idx}] ✅ DETECTED: {cls} | info={info}")
                else:
                    self.get_logger().info(f"[{idx}] 🙅 NOT FOUND: {cls} (timeout)")



def main():
    rclpy.init()
    node = GPTAPIController()

    deadline = time.time() + 5.0
    while rclpy.ok() and node.map_info is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    try:
        if node.map_info is None:
            print("⚠️ 아직 /map_anchors 를 못 받았습니다. 그래도 진행합니다.")

        user_cmd = input("👉 명령을 입력하세요: ").strip()
        if not user_cmd:
            print("❌ 명령이 비어 있습니다. 종료합니다.")
            return

        seq = node.ask_gpt_for_plan(user_cmd)
        if not seq:
            print("❌ 유효한 시퀀스를 받지 못했습니다. 종료합니다.")
            return

        node.run_sequence(seq)

    finally:
        node.navigator.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
