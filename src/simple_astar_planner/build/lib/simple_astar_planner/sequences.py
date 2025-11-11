import rclpy
from rclpy.node import Node
import json
import time
import math
import os

from simple_astar_planner.navigator import Navigator
from simple_astar_planner.takephoto import PhotoTaker


def load_test_sequence(json_file_path: str) -> list:
    """테스트용 JSON 파일에서 시퀀스를 로드합니다."""
    try:
        with open(json_file_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        print(f"📥 JSON 파일에서 시퀀스 로드: {json_file_path}")
        print(f"📋 시퀀스: {data['sequence']}")
        return data["sequence"]
    except Exception as e:
        print(f"❌ JSON 파일 로드 실패: {e}")
        return []


def run_sequence(navigator, photo, sequence):
    """시퀀스에 따라 로봇 행동을 실행합니다."""
    for step in sequence:
        if step["action"] == "goto":
            x = float(step["x"])
            y = float(step["y"]) 
            yaw = step.get("yaw", 0.0)
            print(f"🎯 목표 위치로 이동: ({x}, {y})")
            navigator.goto(x, y, yaw=yaw)
            
        elif step["action"] == "photo":
            print("📸 사진 촬영 중...")
            photo.take_photo()
            
        elif step["action"] == "wait":
            duration = float(step.get("duration", 2.0))
            end = time.monotonic() + duration
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(navigator, timeout_sec=0.1)
        else:
            print(f"⚠️ 알 수 없는 액션: {step['action']}")


def main():
    rclpy.init()
    nav = Navigator()
    photo = PhotoTaker(nav)

    # 테스트용 JSON 파일 경로
    json_file_path = "/home/changwoo/ros2_ws/test_sequence.json"
 
    # JSON 파일에서 시퀀스 로
    sequence = load_test_sequence(json_file_path)
    
    if sequence:
        run_sequence(nav, photo, sequence)
    else:
        print("❌ 시퀀스를 로드할 수 없습니다.")

    nav.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
