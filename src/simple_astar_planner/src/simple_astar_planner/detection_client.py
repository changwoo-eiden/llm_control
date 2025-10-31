#!/usr/bin/env python3
from __future__ import annotations
import json
from typing import Tuple, Dict

import rclpy
from rclpy.node import Node
from rclpy.task import Future

from detection_manager.srv import RunDetection  # ← 앞서 만든 srv 패키지 기준

class Detector:
    """
    YOLO 클래스 필터 원샷 탐지용 클라이언트.
    - /detection/run 서비스 호출
    """
    def __init__(self, node: Node, service_name: str = '/detection/run', wait: bool = True):
        self._node = node
        self._cli = node.create_client(RunDetection, service_name)
        if wait:
            while not self._cli.wait_for_service(timeout_sec=0.5):
                node.get_logger().info(f'⏳ waiting for {service_name} ...')

    def detect(self, target_class: str, timeout_sec: float = 5.0) -> Tuple[bool, Dict]:
        """target_class를 timeout 내 찾으면 (True, info), 아니면 (False, {} or reason)"""
        req = RunDetection.Request()
        req.class_filter = target_class
        req.timeout_sec = float(timeout_sec)

        fut: Future = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self._node, fut)

        if fut.result() is None:
            self._node.get_logger().error('detection service call failed (no result).')
            return False, {}

        resp = fut.result()
        try:
            payload = json.loads(resp.result_json) if resp.result_json else {}
        except Exception:
            payload = {}
        return bool(resp.found), payload
