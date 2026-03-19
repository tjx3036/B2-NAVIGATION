#!/usr/bin/env python3
"""
ByteTrack ROS2 桥接：订阅相机，YOLOX+ByteTrack 检测跟踪，发布 /vision_dynamic_obstacles。

需在 AA/ByteTrack-main 下下载预训练权重，例如 README 中的 bytetrack_tiny_mot17，
保存为 pretrained/bytetrack_tiny_mot17.pth（或 .pth.tar，视下载文件而定）。
"""
from __future__ import annotations

import os
import sys
import threading
import time
from types import SimpleNamespace

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

# ByteTrack 源码根目录（与仓库布局一致）
_DEFAULT_BYTETRACK_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "AA", "ByteTrack-main")
)


def _ensure_yolox_path(byteroot: str) -> None:
    if byteroot not in sys.path:
        sys.path.insert(0, byteroot)


class ByteTrackBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("bytetrack_bridge")

        self.declare_parameter("bytetrack_root", _DEFAULT_BYTETRACK_ROOT)
        self.declare_parameter(
            "exp_file",
            "exps/example/mot/yolox_tiny_mix_det.py",
        )
        self.declare_parameter(
            "ckpt",
            # ByteTrack 官方 model zoo 的文件通常是 .pth.tar（如 bytetrack_x_mot17.pth.tar）
            os.path.join(
                _DEFAULT_BYTETRACK_ROOT, "pretrained", "bytetrack_tiny_mot17.pth.tar"
            ),
        )
        self.declare_parameter("device", "gpu")  # gpu 或 cpu
        self.declare_parameter("process_hz", 15.0)  # 推理频率；15~20 较流畅，过高加重 GPU/CPU
        self.declare_parameter("camera_fov_h", 1.57)  # 与 Gazebo 前视相机一致
        self.declare_parameter("ref_object_height_m", 1.0)  # 用于由框高估计距离

        byteroot = str(self.get_parameter("bytetrack_root").value)
        exp_rel = str(self.get_parameter("exp_file").value)
        ckpt = str(self.get_parameter("ckpt").value)
        device_str = str(self.get_parameter("device").value).lower()
        # process_hz<=0 表示每帧都推理；否则为最大推理频率
        phz = float(self.get_parameter("process_hz").value)
        self._process_period = 0.0 if phz <= 0 else (1.0 / max(1.0, phz))
        self._fov_h = float(self.get_parameter("camera_fov_h").value)
        self._ref_h = float(self.get_parameter("ref_object_height_m").value)

        self.bridge = CvBridge()
        self._last_image = None
        self._lock = threading.Lock()
        self._last_process_t = 0.0
        self._frame_id = 0

        self.image_sub = self.create_subscription(
            Image,
            "/front_camera/image_raw",
            self.on_image,
            10,
        )
        self.tracks_pub = self.create_publisher(
            Float32MultiArray,
            "/vision_dynamic_obstacles",
            10,
        )
        # 阶段一：RViz 可视化 + 带框图像，便于对齐激光
        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/vision_dynamic_markers",
            10,
        )
        self.vis_image_pub = self.create_publisher(
            Image,
            "/vision_detections_image",
            10,
        )

        self._predictor = None
        self._tracker = None
        self._exp = None
        self._device = None
        self._track_args = None
        self._fx = 320.0  # 占位，在首帧按图像尺寸更新

        exp_path = os.path.join(byteroot, exp_rel) if not os.path.isabs(exp_rel) else exp_rel

        if not os.path.isdir(byteroot):
            self.get_logger().error(f"ByteTrack root 不存在: {byteroot}")
        elif not os.path.isfile(ckpt):
            self.get_logger().error(
                f"未找到权重文件: {ckpt}\n"
                "请从 ByteTrack README「Model zoo」下载 bytetrack_tiny_mot17，并将文件放入 pretrained 目录。\n"
                "若你下载下来的文件后缀不是 .pth.tar，请运行时通过参数指定："
                " -p ckpt:=/你的/实际/权重文件（例如 .pth.tar）。"
            )
        else:
            try:
                _ensure_yolox_path(byteroot)
                from yolox.exp import get_exp
                from yolox.utils import get_model_info
                from yolox.utils import postprocess as yolo_postprocess

                self._exp = get_exp(exp_path, None)
                use_cuda = device_str == "gpu" and torch.cuda.is_available()
                self._device = torch.device("cuda" if use_cuda else "cpu")
                if device_str == "gpu" and not use_cuda:
                    self.get_logger().warn("未检测到 CUDA，改用 CPU 推理。")

                model = self._exp.get_model().to(self._device)
                model.eval()
                self.get_logger().info(f"Model: {get_model_info(model, self._exp.test_size)}")

                ckpt_data = torch.load(ckpt, map_location="cpu")
                state = ckpt_data["model"] if isinstance(ckpt_data, dict) and "model" in ckpt_data else ckpt_data
                model.load_state_dict(state, strict=False)
                self.get_logger().info(f"已加载权重: {ckpt}")

                # 内联 Predictor 逻辑，避免依赖 demo_track 的 loguru 全局
                class _Predictor:
                    def __init__(self, m, exp, dev):
                        self.model = m
                        self.num_classes = exp.num_classes
                        self.confthre = exp.test_conf
                        self.nmsthre = exp.nmsthre
                        self.test_size = exp.test_size
                        self.device = dev
                        from yolox.data.data_augment import preproc

                        self._preproc = preproc
                        self.rgb_means = (0.485, 0.456, 0.406)
                        self.std = (0.229, 0.224, 0.225)

                    def inference(self, img_bgr, timer=None):
                        from yolox.tracking_utils.timer import Timer

                        if timer is None:
                            timer = Timer()
                        h, w = img_bgr.shape[:2]
                        img_info = {"id": 0, "height": h, "width": w, "raw_img": img_bgr}
                        img, ratio = self._preproc(
                            img_bgr, self.test_size, self.rgb_means, self.std
                        )
                        img_info["ratio"] = ratio
                        t = torch.from_numpy(img).unsqueeze(0).float().to(self.device)
                        with torch.no_grad():
                            timer.tic()
                            out = self.model(t)
                            out = yolo_postprocess(
                                out, self.num_classes, self.confthre, self.nmsthre
                            )
                        return out, img_info

                self._predictor = _Predictor(model, self._exp, self._device)

                self._track_args = SimpleNamespace(
                    track_thresh=0.5,
                    track_buffer=30,
                    match_thresh=0.8,
                    aspect_ratio_thresh=1.6,
                    min_box_area=10,
                    mot20=False,
                )
                from yolox.tracker.byte_tracker import BYTETracker

                self._tracker = BYTETracker(self._track_args, frame_rate=15)

                self.get_logger().info("ByteTrack 模型与跟踪器已就绪。")
            except Exception as e:
                self.get_logger().error(f"加载 ByteTrack 失败: {e}")
                import traceback

                self.get_logger().error(traceback.format_exc())
                self._predictor = None
                self._tracker = None

        self.get_logger().info("bytetrack_bridge 节点已启动。")

    def _bbox_to_base_xy(self, tlwh, img_w: int, img_h: int) -> tuple[float, float]:
        """框底边中心 -> base_link 平面近似 (x前, y左)."""
        x1, y1, bw, bh = tlwh
        cx = x1 + 0.5 * bw
        cy = y1 + bh
        half_w = img_w * 0.5
        tan_half = np.tan(self._fov_h * 0.5)
        fx = half_w / max(tan_half, 1e-6)
        # 相似三角形：距离 Z ≈ fx * H_ref / h_px
        h_px = max(float(bh), 8.0)
        z = fx * self._ref_h / h_px
        z = float(np.clip(z, 0.3, 40.0))
        x_cam = (cx - half_w) * z / fx
        # 相机前向 Z -> base x；图像右为正 x_cam -> base -y（左为正）
        x_forward = z
        y_left = -x_cam
        return x_forward, y_left

    def on_image(self, msg: Image) -> None:
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge: {e}")
            return

        now = time.time()
        if self._process_period > 0 and (now - self._last_process_t) < self._process_period:
            return
        self._last_process_t = now

        if self._predictor is None or self._tracker is None or self._exp is None:
            empty = Float32MultiArray()
            empty.data = []
            self.tracks_pub.publish(empty)
            return

        self._frame_id += 1
        try:
            from yolox.tracking_utils.timer import Timer

            timer = Timer()
            outputs, img_info = self._predictor.inference(cv_img, timer)
            H, W = img_info["height"], img_info["width"]
            self._fx = (W * 0.5) / max(np.tan(self._fov_h * 0.5), 1e-6)

            data: list[float] = []
            online_targets = []
            if outputs[0] is not None:
                online_targets = self._tracker.update(
                    outputs[0],
                    [H, W],
                    self._exp.test_size,
                )
                args = self._track_args
                for t in online_targets:
                    tlwh = t.tlwh
                    tid = int(t.track_id)
                    vertical = tlwh[2] / max(tlwh[3], 1e-6) > args.aspect_ratio_thresh
                    if tlwh[2] * tlwh[3] <= args.min_box_area or vertical:
                        continue
                    x_b, y_b = self._bbox_to_base_xy(tlwh, W, H)
                    data.extend([float(tid), x_b, y_b, 0.0, 0.0])

            out_msg = Float32MultiArray()
            out_msg.data = data
            self.tracks_pub.publish(out_msg)

            # 发布 RViz Marker（base_link 下球体），便于与激光点云对齐
            marker_array = MarkerArray()
            for idx, t in enumerate(online_targets):
                tlwh = t.tlwh
                tid = int(t.track_id)
                vertical = tlwh[2] / max(tlwh[3], 1e-6) > self._track_args.aspect_ratio_thresh
                if tlwh[2] * tlwh[3] <= self._track_args.min_box_area or vertical:
                    continue
                x_b, y_b = self._bbox_to_base_xy(tlwh, W, H)
                m = Marker()
                m.header.frame_id = "base_link"
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = "vision_tracks"
                m.id = tid
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = float(x_b)
                m.pose.position.y = float(y_b)
                m.pose.position.z = 0.3
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = 0.25
                m.color.r = 0.0
                m.color.g = 1.0
                m.color.b = 0.0
                m.color.a = 1.0
                marker_array.markers.append(m)
            if marker_array.markers:
                self.marker_pub.publish(marker_array)

            # 发布带框+ID 的图像，便于在 RViz Image 或 rqt 中看「视觉框」
            vis_img = cv_img.copy()
            for t in online_targets:
                x1, y1, w, h = [int(v) for v in t.tlwh]
                tid = int(t.track_id)
                cv2.rectangle(vis_img, (x1, y1), (x1 + w, y1 + h), (0, 255, 0), 2)
                cv2.putText(
                    vis_img, f"#{tid}", (x1, max(20, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
                )
            try:
                vis_msg = self.bridge.cv2_to_imgmsg(vis_img, encoding="bgr8")
                vis_msg.header.stamp = msg.header.stamp
                vis_msg.header.frame_id = msg.header.frame_id
                self.vis_image_pub.publish(vis_msg)
            except Exception:
                pass
        except Exception as e:
            self.get_logger().warn(f"推理异常: {e}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ByteTrackBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
