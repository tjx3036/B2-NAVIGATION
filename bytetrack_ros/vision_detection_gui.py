#!/usr/bin/env python3
"""
检测结果独立 GUI：只订阅 /vision_detections_image，在单独窗口显示带框图像，不用 RViz。
"""
import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class VisionDetectionGuiNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_detection_gui")

        self.bridge = CvBridge()
        self._latest = None  # numpy array (BGR) or None
        self._lock = threading.Lock()

        self.sub = self.create_subscription(
            Image,
            "/vision_detections_image",
            self.on_image,
            10,
        )
        self.timer = self.create_timer(0.03, self.on_timer)  # ~33Hz 刷新
        self.get_logger().info(
            "检测图像 GUI 已启动，订阅 /vision_detections_image，按 q 退出。"
        )

    def on_image(self, msg: Image) -> None:
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._lock:
                self._latest = cv_img
        except Exception as e:
            self.get_logger().warn(f"cv_bridge: {e}")

    def on_timer(self) -> None:
        with self._lock:
            img = self._latest
        if img is None:
            return
        cv2.imshow("Vision Detection (ByteTrack)", img)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("按 q 退出。")
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionDetectionGuiNode()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
