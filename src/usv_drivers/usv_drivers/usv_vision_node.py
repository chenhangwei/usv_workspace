#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 Chen Hangwei
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
无人船双目视觉检测节点

该节点负责:
1. 读取双目摄像头数据 (支持两个独立 USB 摄像头或单个拼接输出)
2. 使用深度学习模型 (YOLOv8 / ONNX) 进行目标检测
3. 利用双目视差计算障碍物距离和方位角
4. 发布 VisionObstacleArray 消息供避障节点融合使用

支持的检测后端:
- Ultralytics YOLOv8 (推荐，支持检测和实例分割)
- OpenCV DNN (ONNX 格式模型)
"""

import os

import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from common_interfaces.msg import VisionObstacle, VisionObstacleArray


class UsvVisionNode(Node):
    """
    无人船双目视觉检测节点

    从双目摄像头采集图像，运行目标检测，结合双目深度估计，
    发布视觉障碍物信息到 /vision_obstacles 话题。
    """

    def __init__(self):
        """初始化双目视觉检测节点"""
        super().__init__('usv_vision_node')

        # ---- 参数声明 ----
        self.declare_parameter('left_camera_device', '/dev/video0')   # 左目摄像头
        self.declare_parameter('right_camera_device', '/dev/video2')  # 右目摄像头
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('detection_rate', 5.0)   # 检测频率 (Hz)
        self.declare_parameter('confidence_threshold', 0.45)
        self.declare_parameter('model_path', '')        # 模型文件路径
        self.declare_parameter('focal_length', 700.0)   # 像素焦距 (标定后填写)
        self.declare_parameter('baseline', 0.025)       # 双目基线距离 (米) 实测25mm
        self.declare_parameter('calibration_file', '')   # 标定文件路径 (.yaml)

        self.left_device = self.get_parameter('left_camera_device').value
        self.right_device = self.get_parameter('right_camera_device').value
        self.frame_w = self.get_parameter('frame_width').value
        self.frame_h = self.get_parameter('frame_height').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.model_path = self.get_parameter('model_path').value
        self.focal_length = self.get_parameter('focal_length').value
        self.baseline = self.get_parameter('baseline').value

        # ---- 加载标定文件 (如果提供) ----
        self.rectify_maps = None  # (map1_l, map2_l, map1_r, map2_r)
        calib_file = self.get_parameter('calibration_file').value
        if calib_file:
            self._load_calibration(calib_file)

        # ---- 打开左目摄像头 (V4L2 + MJPG) ----
        self.cap_left = cv2.VideoCapture(self.left_device, cv2.CAP_V4L2)
        if not self.cap_left.isOpened():
            self.get_logger().error(f'无法打开左目摄像头: {self.left_device}')
            raise RuntimeError(f'Left camera open failed: {self.left_device}')
        self._configure_camera(self.cap_left, '左目')

        # ---- 打开右目摄像头 (V4L2 + MJPG) ----
        self.cap_right = cv2.VideoCapture(self.right_device, cv2.CAP_V4L2)
        if not self.cap_right.isOpened():
            self.cap_left.release()
            self.get_logger().error(f'无法打开右目摄像头: {self.right_device}')
            raise RuntimeError(f'Right camera open failed: {self.right_device}')
        self._configure_camera(self.cap_right, '右目')

        self.get_logger().info(
            f'双目摄像头已打开: 左={self.left_device} 右={self.right_device} '
            f'({self.frame_w}x{self.frame_h})')

        # ---- 初始化检测模型 ----
        self.detector = self._load_detector()

        # ---- 双目立体匹配器 (用于深度估计) ----
        self.stereo_matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=64,
            blockSize=9,
            P1=8 * 3 * 9 ** 2,
            P2=32 * 3 * 9 ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
        )

        # ---- ROS 2 发布者 ----
        self.obstacle_pub = self.create_publisher(
            VisionObstacleArray, 'vision_obstacles', 10)
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'vision_debug_image', 1)

        # ---- 定时检测回调 ----
        rate = self.get_parameter('detection_rate').value
        self.timer = self.create_timer(1.0 / rate, self._detect_callback)

        self.get_logger().info(
            f'视觉检测节点已启动 (检测频率: {rate} Hz, '
            f'置信度阈值: {self.conf_thresh})')

    # ------------------------------------------------------------------
    # 标定文件加载
    # ------------------------------------------------------------------
    def _load_calibration(self, calib_file):
        """加载双目标定结果，覆盖焦距/基线参数并启用畸变校正"""
        if not os.path.exists(calib_file):
            self.get_logger().error(f'标定文件不存在: {calib_file}')
            return

        with open(calib_file, 'r') as f:
            calib = yaml.safe_load(f)

        # 用标定结果覆盖参数
        self.focal_length = calib['focal_length_px']
        self.baseline = calib['baseline_m']
        self.get_logger().info(
            f'已加载标定参数: focal={self.focal_length:.1f}px '
            f'baseline={self.baseline*1000:.1f}mm '
            f'RMS={calib["stereo_rms_error"]:.4f}px')

        # 加载预计算的 remap 映射表
        npz_path = calib_file.replace('.yaml', '_maps.npz')
        if os.path.exists(npz_path):
            maps = np.load(npz_path)
            self.rectify_maps = (
                maps['map1_l'], maps['map2_l'],
                maps['map1_r'], maps['map2_r'])
            self.get_logger().info(f'已加载校正映射: {npz_path}')
        else:
            # 从 YAML 重新计算映射
            img_size = tuple(calib['image_size'])
            K_l = np.array(calib['K_left'])
            dist_l = np.array(calib['dist_left'])
            K_r = np.array(calib['K_right'])
            dist_r = np.array(calib['dist_right'])
            R = np.array(calib['R'])
            T = np.array(calib['T'])
            R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(
                K_l, dist_l, K_r, dist_r, img_size, R, T, alpha=0)
            map1_l, map2_l = cv2.initUndistortRectifyMap(
                K_l, dist_l, R1, P1, img_size, cv2.CV_32FC1)
            map1_r, map2_r = cv2.initUndistortRectifyMap(
                K_r, dist_r, R2, P2, img_size, cv2.CV_32FC1)
            self.rectify_maps = (map1_l, map2_l, map1_r, map2_r)
            self.get_logger().info('已从标定参数重新计算校正映射')

    # ------------------------------------------------------------------
    # 摄像头配置
    # ------------------------------------------------------------------
    def _configure_camera(self, cap, name):
        """配置摄像头参数 (MJPG编码 + 分辨率 + FPS)"""
        fourcc_mjpg = cv2.VideoWriter_fourcc(*'MJPG')
        cap.set(cv2.CAP_PROP_FOURCC, fourcc_mjpg)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_h)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'{name}实际参数: {actual_w}x{actual_h} @ {actual_fps:.0f}fps')

    # ------------------------------------------------------------------
    # 模型加载
    # ------------------------------------------------------------------
    def _load_detector(self):
        """
        加载目标检测模型

        支持后端优先级:
        1. Ultralytics YOLO (支持 YOLOv8 检测/分割，类 SOLO 效果)
        2. OpenCV DNN (ONNX 格式)

        Returns:
            tuple: (backend_name, model_object)
        """
        model_path = self.model_path
        if not model_path:
            self.get_logger().warn('未指定 model_path，将尝试加载 yolov8n.onnx')
            model_path = 'yolov8n.onnx'

        # 优先尝试 Ultralytics YOLO
        try:
            from ultralytics import YOLO
            model = YOLO(model_path)
            self.get_logger().info(f'已加载 Ultralytics 模型: {model_path}')
            return ('ultralytics', model)
        except ImportError:
            self.get_logger().info('未安装 ultralytics，尝试 OpenCV DNN 后端')
        except Exception as e:
            self.get_logger().warn(f'Ultralytics 加载失败 ({e})，尝试 OpenCV DNN')

        # 回退到 OpenCV DNN
        try:
            net = cv2.dnn.readNet(model_path)
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_DEFAULT)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            self.get_logger().info(f'已加载 OpenCV DNN 模型: {model_path}')
            return ('opencv_dnn', net)
        except Exception as e:
            self.get_logger().error(f'所有检测后端均加载失败: {e}')
            raise

    # ------------------------------------------------------------------
    # 主检测回调
    # ------------------------------------------------------------------
    def _detect_callback(self):
        """定时回调: 采集图像 → 检测 → 测距 → 发布"""
        ret_l, left_img = self.cap_left.read()
        ret_r, right_img = self.cap_right.read()
        if not ret_l or not ret_r:
            self.get_logger().warn('摄像头读取失败', throttle_duration_sec=5.0)
            return

        # 确保左右图像尺寸一致
        if left_img.shape != right_img.shape:
            right_img = cv2.resize(right_img,
                                   (left_img.shape[1], left_img.shape[0]))

        # 校正畸变 (如果有标定参数)
        if self.rectify_maps is not None:
            map1_l, map2_l, map1_r, map2_r = self.rectify_maps
            left_img = cv2.remap(left_img, map1_l, map2_l, cv2.INTER_LINEAR)
            right_img = cv2.remap(right_img, map1_r, map2_r, cv2.INTER_LINEAR)

        # 1) 在左图上运行目标检测 (校正后)
        detections = self._run_detection(left_img)

        # 2) 双目视差 → 深度图 (已校正，不再重复)
        depth_map = self._compute_depth(left_img, right_img)

        # 3) 为每个检测结果计算距离和方位
        obstacle_array = VisionObstacleArray()
        obstacle_array.header.stamp = self.get_clock().now().to_msg()
        obstacle_array.header.frame_id = 'camera_link'

        img_center_x = left_img.shape[1] / 2.0

        for det in detections:
            cls_name, conf, x1, y1, x2, y2 = det
            if conf < self.conf_thresh:
                continue

            # 取 bbox 中心区域的深度中值 (抗噪声)
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            roi_y1 = max(0, cy - 10)
            roi_y2 = min(depth_map.shape[0], cy + 10)
            roi_x1 = max(0, cx - 10)
            roi_x2 = min(depth_map.shape[1], cx + 10)
            roi = depth_map[roi_y1:roi_y2, roi_x1:roi_x2]
            valid = roi[roi > 0]
            if len(valid) == 0:
                continue
            distance = float(np.median(valid))

            # 方位角: 像素偏移 → 角度
            bearing = float(np.arctan2(cx - img_center_x, self.focal_length))

            # 估计宽度
            width = distance * (x2 - x1) / self.focal_length

            obs = VisionObstacle()
            obs.header = obstacle_array.header
            obs.class_name = cls_name
            obs.confidence = float(conf)
            obs.distance = distance
            obs.bearing = bearing
            obs.width = float(width)
            obs.bbox = [int(x1), int(y1), int(x2), int(y2)]
            obstacle_array.obstacles.append(obs)

        self.obstacle_pub.publish(obstacle_array)

        # 发布调试图像 (仅在有订阅者时)
        if self.image_pub.get_subscription_count() > 0:
            debug_img = left_img.copy()
            for obs in obstacle_array.obstacles:
                bx = obs.bbox
                cv2.rectangle(debug_img, (bx[0], bx[1]), (bx[2], bx[3]),
                              (0, 255, 0), 2)
                label = f'{obs.class_name} {obs.distance:.1f}m'
                cv2.putText(debug_img, label, (bx[0], bx[1] - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            self.image_pub.publish(
                self.bridge.cv2_to_imgmsg(debug_img, 'bgr8'))

    # ------------------------------------------------------------------
    # 目标检测
    # ------------------------------------------------------------------
    def _run_detection(self, image):
        """
        执行目标检测

        Args:
            image: BGR 格式的输入图像

        Returns:
            list[tuple]: [(class_name, confidence, x1, y1, x2, y2), ...]
        """
        backend, model = self.detector

        if backend == 'ultralytics':
            results = model(image, verbose=False)
            dets = []
            for r in results:
                for box in r.boxes:
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    x1, y1, x2, y2 = xyxy
                    conf = float(box.conf[0])
                    cls_id = int(box.cls[0])
                    cls_name = model.names.get(cls_id, str(cls_id))
                    dets.append((cls_name, conf, x1, y1, x2, y2))
            return dets

        elif backend == 'opencv_dnn':
            blob = cv2.dnn.blobFromImage(
                image, 1 / 255.0, (640, 640), swapRB=True)
            model.setInput(blob)
            out_names = model.getUnconnectedOutLayersNames()
            outputs = model.forward(out_names)
            return self._parse_opencv_dnn_output(outputs, image.shape)

        return []

    def _parse_opencv_dnn_output(self, outputs, img_shape):
        """
        解析 OpenCV DNN YOLO 输出

        Args:
            outputs: DNN forward 输出
            img_shape: 原图尺寸 (h, w, c)

        Returns:
            list[tuple]: 检测结果列表
        """
        h, w = img_shape[:2]
        dets = []
        for output in outputs:
            for row in output[0]:
                scores = row[4:]
                class_id = int(np.argmax(scores))
                conf = float(scores[class_id])
                if conf < self.conf_thresh:
                    continue
                cx, cy, bw, bh = row[:4]
                x1 = int((cx - bw / 2) * w / 640)
                y1 = int((cy - bh / 2) * h / 640)
                x2 = int((cx + bw / 2) * w / 640)
                y2 = int((cy + bh / 2) * h / 640)
                dets.append((str(class_id), conf, x1, y1, x2, y2))
        return dets

    # ------------------------------------------------------------------
    # 双目深度计算
    # ------------------------------------------------------------------
    def _compute_depth(self, left, right):
        """
        双目视差 → 深度图

        利用 StereoSGBM 计算左右图像视差，再由
            depth = focal_length * baseline / disparity
        转换为深度 (米)。

        Args:
            left: 左目 BGR 图像
            right: 右目 BGR 图像

        Returns:
            np.ndarray: 深度图 (float32，单位: 米)
        """
        gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        disparity = self.stereo_matcher.compute(
            gray_l, gray_r).astype(np.float32) / 16.0
        with np.errstate(divide='ignore', invalid='ignore'):
            depth = np.where(
                disparity > 0,
                self.focal_length * self.baseline / disparity,
                0.0)
        return depth.astype(np.float32)

    # ------------------------------------------------------------------
    # 节点销毁
    # ------------------------------------------------------------------
    def destroy_node(self):
        """释放摄像头资源"""
        if hasattr(self, 'cap_left') and self.cap_left.isOpened():
            self.cap_left.release()
        if hasattr(self, 'cap_right') and self.cap_right.isOpened():
            self.cap_right.release()
        if hasattr(self, 'timer'):
            self.timer.cancel()
        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = UsvVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
