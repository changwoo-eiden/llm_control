import os, datetime, cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class PhotoTaker:
    def __init__(self, node):
        self.node = node
        self.bridge = CvBridge()
        self.latest_frame = None
        self.sub = node.create_subscription(
            Image,
            '/bcr_bot/kinect_camera/image_raw',
            self.image_callback,
            10
        )
        self.node.get_logger().info("📸 PhotoTaker initialized")

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.node.get_logger().error(f"Image conversion failed: {e}")

    def take_photo(self, save_dir="/home/changwoo/ros2_ws/photos"):
        if self.latest_frame is None:
            self.node.get_logger().warn("⚠️ No image received yet.")
            return None
        os.makedirs(save_dir, exist_ok=True)
        now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(save_dir, f"photo_{now}.png")
        cv2.imwrite(filename, self.latest_frame)
        self.node.get_logger().info(f"✅ Saved photo: {filename}")
        return filename
