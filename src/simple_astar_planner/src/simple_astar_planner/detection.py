#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import json
import shlex
import signal
import subprocess
import time
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Bool, String

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


def _norm(s: str) -> str:
    return str(s).strip().lower()


class DetectionAction:
    """
    - /detections를 잠깐 구독해서 target_class 존재 여부를 반환
    - 필요 시 YOLO 노드를 내부에서 ros2 run으로 스폰
    - 이미 YOLO 노드가 떠 있으면 그대로 사용
    """

    def __init__(
        self,
        node: Node,
        *,
        detections_topic="/detections_vision",
        image_topic: str = "/bcr_bot/kinect_camera/image_raw",
        yolo_node_name: str = "yolo_v5_ros2_node",
        yolo_pkg: str = "my_perception",
        yolo_exec: str = "yolo_v5_ros2_node",
        model_path: str = "yolov5s.pt",
        device: str = "cpu",
        conf_thres: float = 0.25,
        iou_thres: float = 0.45,
        backend: str = "auto",
        keep_yolo_running: bool = False,
        max_wait_yolo_ready: float = 10.0,
    ):
        self._node = node
        self._topic = detections_topic
        self._image_topic = image_topic

        self._yolo_node_name = yolo_node_name
        self._yolo_pkg = yolo_pkg
        self._yolo_exec = yolo_exec

        self._model_path = model_path
        self._device = device
        self._conf_thres = conf_thres
        self._iou_thres = iou_thres
        self._backend = backend

        self._keep_yolo_running = keep_yolo_running
        self._max_wait_yolo_ready = max_wait_yolo_ready

        # 발행자 QoS와 맞추기(대부분 RELIABLE + KEEP_LAST)
        self._qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            # durability는 기본 VOLATILE이므로 생략해도 발행자와 일치
        )

        self._sub = None
        self._last_msg: Optional[Detection2DArray] = None

        self._param_service = f"/{self._yolo_node_name}/set_parameters"
        self._param_client = self._node.create_client(SetParameters, self._param_service)

        self._spawn_proc: Optional[subprocess.Popen] = None  # 자동 스폰한 프로세스 핸들
        self._pub_found = self._node.create_publisher(Bool, "/detection/found", 10)
        self._pub_info  = self._node.create_publisher(String, "/detection/info", 10)

        # 외부에서 읽는 경우가 있어 기본값 제공
        self.last_detected: bool = False

    # ─────────────────────────────────────────────────────────────
    # Public API

    def detect(
        self,
        target_class: str,
        *,
        timeout_sec: float = 10.0,
        prefer_param: str = "class_filter",
        auto_spawn_yolo: bool = True
    ) -> tuple[bool, dict]:
        target_norm = _norm(target_class)

        has_pub = self._has_publishers(self._topic)
        spawned_here = False
        if not has_pub and auto_spawn_yolo:
            spawned_here = self._spawn_yolo_with_filter(target_class, prefer_param)
            if spawned_here and not self._wait_yolo_ready():
                self._log_warn("YOLO node failed to get ready in time.")
        elif has_pub:
            self._apply_remote_filter(target_class, prefer_param)

        self._start_sub()

        start = time.time()
        found = False
        while (time.time() - start) < timeout_sec and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.05)
            if self._last_msg is None:
                continue
            if self._has_target(self._last_msg, target_norm):
                found = True
                break

        self._stop_sub()

        # 마지막 탐지 결과 저장
        self.last_detected = bool(found)

        # ---- YOLO 프로세스 정리 ----
        if not self._keep_yolo_running:
            self._terminate_spawned()

        # ---- info 반환 ----
        info = {
            "class": target_class,
            "timeout_sec": float(timeout_sec),
            "found": bool(found),
            "timestamp": time.time(),
            # 필요하면 최근 메시지에서 bbox/score 추출해 넣기
            # "detections": self._extract_detections(self._last_msg, target_norm),
        }

        self._pub_found.publish(Bool(data=found))
        self._pub_info.publish(String(data=json.dumps(info, ensure_ascii=False)))

        return found, info

    # ─────────────────────────────────────────────────────────────
    # Internals — publishers, subscription, parsing

    def _has_publishers(self, topic: str) -> bool:
        try:
            infos = self._node.get_publishers_info_by_topic(topic)
            return len(infos) > 0
        except Exception:
            return False

    def _start_sub(self):
        if self._sub is None:
            self._last_msg = None
            self._sub = self._node.create_subscription(
                Detection2DArray, self._topic, self._cb, self._qos
            )

    def _stop_sub(self):
        try:
            if self._sub is not None:
                self._node.destroy_subscription(self._sub)
        except Exception:
            pass
        self._sub = None

    def _cb(self, msg: Detection2DArray):
        self._last_msg = msg

    def _has_target(self, det_arr: Detection2DArray, target_norm: str) -> bool:
        try:
            for det in det_arr.detections:
                for res in det.results:
                    cid = getattr(res.hypothesis, "class_id", "")
                    if _norm(cid) == target_norm:
                        return True
        except Exception:
            return False
        return False

    # ─────────────────────────────────────────────────────────────
    # YOLO spawn & manage

    def _spawn_yolo_with_filter(self, target_class: str, prefer_param: str) -> bool:
        """
        ros2 run으로 YOLO 노드를 파라미터와 함께 실행.
        prefer_param=="filter"면 -p filter:="name"
        == "class_filter"면 -p class_filter:="['name']"
        == "auto"면 filter 적용해도 되고, 여기서는 filter로 스폰하는 게 합리적(간단)
        """
        cmd = [
            "ros2", "run", self._yolo_pkg, self._yolo_exec,
            "--ros-args",
            "-p", f"image_topic:={self._image_topic}",
            "-p", f"detections_topic:={self._topic}",
            "-p", f"model_path:={self._model_path}",
            "-p", f"device:={self._device}",
            "-p", f"conf_thres:={self._conf_thres}",
            "-p", f"iou_thres:={self._iou_thres}",
            "-p", f"backend:={self._backend}",
        ]

        if prefer_param == "class_filter":
            cmd += ["-p", f"class_filter:=['{target_class}']"]
        else:
            # 기본: filter(string) 사용
            cmd += ["-p", f"filter:={target_class}"]

        self._log_info(f"Spawning YOLO node: {' '.join(shlex.quote(c) for c in cmd)}")
        try:
            # stdout/stderr inherit하면 화면에 바로 로그가 나와 디버깅에 유리
            self._spawn_proc = subprocess.Popen(cmd)
            return True
        except Exception as e:
            self._log_warn(f"Failed to spawn YOLO node: {e}")
            self._spawn_proc = None
            return False

    def _wait_yolo_ready(self) -> bool:
        """
        퍼블리셔 또는 파라미터 서비스가 준비될 때까지 대기.
        """
        deadline = time.time() + self._max_wait_yolo_ready

        # 1) 파라미터 서비스 준비?
        while rclpy.ok() and time.time() < deadline:
            if self._param_client.wait_for_service(timeout_sec=0.1):
                return True
            time.sleep(0.05)

        # 2) /detections 퍼블리셔 등장?
        while rclpy.ok() and time.time() < deadline:
            if self._has_publishers(self._topic):
                return True
            time.sleep(0.05)

        return False

    def _terminate_spawned(self):
        """
        자동 스폰했던 YOLO 프로세스를 종료.
        """
        if self._spawn_proc is None:
            return
        try:
            # 우아한 종료 시도
            self._spawn_proc.send_signal(signal.SIGINT)  # Ctrl-C
            try:
                self._spawn_proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._spawn_proc.terminate()
                try:
                    self._spawn_proc.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    self._spawn_proc.kill()
        except Exception:
            pass
        finally:
            self._spawn_proc = None

    # ─────────────────────────────────────────────────────────────
    # Remote parameter set (if YOLO already running)

    def _apply_remote_filter(self, target_class: str, prefer_param: str):
        """
        이미 떠 있는 YOLO 노드에 filter 또는 class_filter 적용 시도.
        prefer_param: "filter" | "class_filter" | "auto"
        """
        if not self._param_client:
            return

        # 서비스 준비 잠깐 대기
        t0 = time.time()
        while not self._param_client.wait_for_service(timeout_sec=0.1):
            if (time.time() - t0) > 1.0 or not rclpy.ok():
                self._log_warn("param service not ready; skip remote filter")
                return

        # 시도 순서
        order: List[str]
        if prefer_param == "filter":
            order = ["filter"]
        elif prefer_param == "class_filter":
            order = ["class_filter"]
        else:
            order = ["filter", "class_filter"]

        for pname in order:
            if self._try_set_param(pname, target_class):
                return

        self._log_warn("remote filter not applied (both strategies failed)")

    def _try_set_param(self, pname: str, value: str) -> bool:
        if pname == "filter":
            pval = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
        elif pname == "class_filter":
            pval = ParameterValue(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                string_array_value=[value],
            )
        else:
            return False

        req = SetParameters.Request()
        req.parameters = [Parameter(name=pname, value=pval)]

        try:
            fut = self._param_client.call_async(req)
            t0 = time.time()
            while not fut.done() and (time.time() - t0) < 1.0 and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.05)

            if not fut.done():
                return False

            res = fut.result()
            return bool(res and all(r.successful for r in res.results))
        except Exception:
            return False

    # ─────────────────────────────────────────────────────────────
    # Logs

    def _log_info(self, msg: str):
        try:
            self._node.get_logger().info(msg)
        except Exception:
            print("[INFO]", msg)

    def _log_warn(self, msg: str):
        try:
            self._node.get_logger().warn(msg)
        except Exception:
            print("[WARN]", msg)
