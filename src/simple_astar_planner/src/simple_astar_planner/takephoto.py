import os
import time
import datetime
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class PhotoTaker:
    def __init__(self, node, topic="/bcr_bot/kinect_camera/image_raw"):
        self.node = node
        self.topic = topic
        self.bridge = CvBridge()
        self.latest_frame = None

        # Publisher와 QoS 맞추기 (RELIABLE / VOLATILE)
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.sub = node.create_subscription(
            Image,
            self.topic,
            self.image_callback,
            qos
        )
        self.node.get_logger().info(f"📸 PhotoTaker subscribed to: {self.topic}")

    def image_callback(self, msg: Image):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.node.get_logger().error(f"Image conversion failed: {e}")

    # =====================================================================
    # 🔥 핵심: 단일 스레드에서 안전하게 사진 찍기
    # - photo() 안에서는 spin을 돌리지 않는다.
    # - 호출 직전에 외부에서 spin_once()를 1번 돌려 최신 프레임을 확보해야 한다.
    # =====================================================================
    def take_photo(self, save_dir="/home/changwoo/ros2_ws/photos", wait_for_first_frame=1.0):

        # 1) 아직 프레임을 받아본 적이 없는 경우 → 처음 1초 동안만 기다리기
        if self.latest_frame is None:
            self.node.get_logger().info("⏳ Waiting for first image frame...")
            start = time.time()

            # 첫 프레임을 받기 위해서만 잠시 spin_once() 사용 가능 (blocking X)
            import rclpy
            while self.latest_frame is None and time.time() - start < wait_for_first_frame:
                rclpy.spin_once(self.node, timeout_sec=0.1)

        if self.latest_frame is None:
            self.node.get_logger().warn("⚠️ No image received. Cannot take photo.")
            return None

        # 2) 최신 프레임 저장
        os.makedirs(save_dir, exist_ok=True)
        now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(save_dir, f"photo_{now}.png")

        ok = cv2.imwrite(filename, self.latest_frame)

        if ok:
            self.node.get_logger().info(f"📸 Saved photo: {filename}")
            return filename
        else:
            self.node.get_logger().error(f"❌ Failed to save photo: {filename}")
            return None
