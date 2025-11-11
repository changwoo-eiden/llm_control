#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

from openai import OpenAI

from simple_astar_planner.navigator import Navigator
from simple_astar_planner.takephoto import PhotoTaker
from simple_astar_planner.detection import DetectionAction  # detection 액션

api_key = os.getenv("OPENAI_API_KEY")
client = OpenAI(api_key=api_key)


def _validate_and_normalize_sequence(seq: list) -> list:
    """
    모델이 모든 키를 항상 내보내고, 불필요한 건 null 로 보낼 수 있으므로
    여기서 실제로 필요한 필드만 검증/정규화한다.
    """
    cleaned = []
    for i, step in enumerate(seq, start=1):
        if not isinstance(step, dict) or "action" not in step:
            raise ValueError(f"{i}번째 스텝이 잘못됨: {step}")
        act = step["action"]

        # 평면(step) 우선
        def pick(key, default=None):
            val = step.get(key, default)
            # JSON null → Python None
            return val

        if act == "goto":
            x = pick("x"); y = pick("y"); yaw = pick("yaw")
            if x is None or y is None or yaw is None:
                raise ValueError(f"{i}번째 goto에 x,y,yaw가 필요합니다: {step}")
            cleaned.append({"action": "goto", "x": float(x), "y": float(y), "yaw": float(yaw)})

        elif act == "wait":
            dur = pick("duration")
            if dur is None:
                raise ValueError(f"{i}번째 wait에 duration이 필요합니다: {step}")
            cleaned.append({"action": "wait", "duration": float(dur)})

        elif act == "photo":
            cleaned.append({"action": "photo"})

        elif act == "detection":
            cls = pick("class") or pick("arg") or pick("target")
            if cls is None or not isinstance(cls, str) or not cls.strip():
                raise ValueError(f"{i}번째 detection에 class/arg/target 중 하나가 필요합니다: {step}")
            timeout = pick("timeout", 5.0)
            timeout = 5.0 if timeout is None else float(timeout)
            cleaned.append({"action": "detection", "class": cls.strip(), "timeout": timeout})

        else:
            raise ValueError(f"{i}번째 스텝 action 알 수 없음: {act}")

    return cleaned


class GPTAPIController(Node):
    def __init__(self):
        super().__init__("gpt_api_controller")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.map_info = None
        self.sub_map = self.create_subscription(String, "/map_anchors", self.map_callback, qos)
        self.get_logger().info("🛰️ Waiting for /map_anchors data...")

        # 액션 헬퍼들
        self.navigator = Navigator()
        self.photo = PhotoTaker(self.navigator)
        self.detector = DetectionAction(
            node=self,
            detections_topic="/detections",
            image_topic="/bcr_bot/kinect_camera/image_raw",
            yolo_node_name="yolo_v5_ros2_node",
            yolo_pkg="my_perception",
            yolo_exec="yolo_v5_ros2_node",
            model_path="/home/changwoo/yolov5/yolov5s.pt",
            device="cuda",             # "cpu" | "cuda" | "cuda:0"
            conf_thres=0.35,
            iou_thres=0.45,
            backend="auto",
            keep_yolo_running=False,
            max_wait_yolo_ready=5.0,
        )

    # ─────────────────────────────────────────────────────────────

    def map_callback(self, msg: String):
        try:
            self.map_info = json.loads(msg.data)
            self.get_logger().info("✅ Received /map_anchors update")
        except Exception as e:
            self.get_logger().error(f"Failed to parse /map_anchors: {e}")

    def ask_gpt_for_plan(self, user_command: str) -> list:
        if self.map_info is None:
            self.get_logger().warn("❌ Map info not received yet.")
            return []

        bboxes = self.map_info["bboxes"]
        anchors = self.map_info["anchors"]

        map_json = json.dumps({"forbidden_bboxes": bboxes}, ensure_ascii=False, separators=(",", ":"))
        anchors_json = json.dumps({"anchors": anchors}, ensure_ascii=False, separators=(",", ":"))

        # ── 평면(flat) 스키마: 모든 키를 required 에 포함, 단 타입은 null 허용 ──
        SEQUENCE_SCHEMA = {
            "type": "object",
            "properties": {
                "sequence": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "action":   {"type": "string", "enum": ["goto", "photo", "wait", "detection"]},
                            "x":        {"type": ["number", "null"]},
                            "y":        {"type": ["number", "null"]},
                            "yaw":      {"type": ["number", "null"]},
                            "duration": {"type": ["number", "null"]},
                            "class":    {"type": ["string", "null"]},
                            "timeout":  {"type": ["number", "null"]},
                            "arg":      {"type": ["string", "null"]},
                            "target":   {"type": ["string", "null"]}
                        },
                        # ★ strict 모드의 요구를 만족시키기 위해 모든 키를 required에 포함
                        "required": ["action", "x", "y", "yaw", "duration", "class", "timeout", "arg", "target"],
                        "additionalProperties": False
                    }
                }
            },
            "required": ["sequence"],
            "additionalProperties": False
        }

        # ── 시스템 프롬프트(항상 모든 키를 포함, 불필요한 건 null 로) ──
        system_prompt = (
            "너는 이동 로봇 플래너다. 오직 JSON으로만 답한다. "
            "좌표는 map 프레임, 단위는 미터, yaw는 라디안이다. "
            "가능한 액션은 goto(x,y,yaw), photo(), wait(duration), detection(class[, timeout]) 이다. "
            "불필요한 텍스트 없이 JSON만 출력한다.\n"
            "규칙(아주 중요):\n"
            "A. 아래 제공된 앵커 JSON에 들어있는 좌표만 사용한다. 임의의 x,y,yaw를 발명하지 않는다.\n"
            "B. 사용자가 '책상 앞/뒤'처럼 객체 기준을 지시하면 해당 키(예: '책상.front','책상.back')의 좌표(x,y,yaw)를 그대로 goto에 넣는다.\n"
            "C. 앵커 JSON에 없는 객체 이름/면을 요구하면 계획을 중단한다(좌표를 추정하지 말 것).\n"
            "D. 모든 GOTO (x,y)는 아래 금지영역 AABB 안으로는 들어가면 안 된다.\n"
            "E. photo 전에 wait 2초를 넣는다. '돌아와'는 (0,0,0)으로 복귀.\n"
            "F. 출력은 JSON Schema를 반드시 만족시킨다.\n"
            "G. 각 스텝 객체에는 항상 다음 모든 키를 포함한다: "
            "action, x, y, yaw, duration, class, timeout, arg, target. "
            "사용하지 않는 키는 null 로 설정한다.\n"
            "BEGIN_FORBIDDEN\n"
            f"{map_json}\n"
            "END_FORBIDDEN\n"
            "BEGIN_ANCHORS\n"
            f"{anchors_json}\n"
            "END_ANCHORS\n"
            "예시: "
            "{\"sequence\":["
            "{\"action\":\"detection\",\"x\":null,\"y\":null,\"yaw\":null,"
            "\"duration\":null,\"class\":\"apple\",\"timeout\":5.0,\"arg\":null,\"target\":null}"
            "]}"
        )

        # ── OpenAI 호출 ──
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

    # ─────────────────────────────────────────────────────────────

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
                    rclpy.spin_once(self, timeout_sec=0.1)

            elif act == "photo":
                self.get_logger().info(f"[{idx}] 📸 PHOTO")
                self.photo.take_photo()

            elif act == "detection":
                cls = step["class"]
                timeout = float(step.get("timeout", 5.0)) if step.get("timeout") is not None else 5.0
                self.get_logger().info(f"[{idx}] 🔎 DETECTION: class='{cls}', timeout={timeout:.2f}s")
                # DetectionAction.detect()가 (bool, dict) 반환한다고 가정
                found, info = self.detector.detect(
                    target_class=cls,
                    timeout_sec=timeout,
                    prefer_param="filter",
                    auto_spawn_yolo=True
                )
                if found:
                    self.get_logger().info(f"[{idx}] ✅ DETECTED: {cls} | info={info}")
                else:
                    self.get_logger().info(f"[{idx}] 🙅 NOT FOUND: {cls} (timeout)")

            else:
                self.get_logger().warn(f"[{idx}] ❓ Unknown action: {act}")

    # ─────────────────────────────────────────────────────────────

    def ask_and_run_once(self):
        # /map_anchors를 잠깐까지 기다림(없어도 진행)
        deadline = time.time() + 5.0
        while rclpy.ok() and self.map_info is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.map_info is None:
            self.get_logger().warn("⚠️ 아직 /map_anchors 를 못 받았습니다. 그래도 진행합니다.")

        user_cmd = input("👉 명령을 입력하세요: ").strip()
        if not user_cmd:
            print("❌ 명령이 비어 있습니다. 종료합니다.")
            return

        seq = self.ask_gpt_for_plan(user_cmd)
        if not seq:
            print("❌ 유효한 시퀀스를 받지 못했습니다. 종료합니다.")
            return

        self.run_sequence(seq)


def main():
    rclpy.init()
    node = GPTAPIController()
    try:
        node.ask_and_run_once()
    finally:
        try:
            node.navigator.destroy_node()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
