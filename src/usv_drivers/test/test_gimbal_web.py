#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双目视觉 + 云台控制 Web 测试工具

在浏览器中实时显示:
- 左目检测画面 (标定校正 + YOLO 检测框 + 距离)
- 深度图 (热力图)
- 云台状态仪表盘 (Pan/Tilt 角度, 工作模式)
- 云台手动控制面板 (方向键/按钮/模式切换)

云台功能:
- PATROL (巡视): 自动水平扫描 + 垂直阶梯, 发现目标自动锁定
- TRACK  (跟踪): PID 闭环跟踪最近/最大/最高置信度目标
- MANUAL (手动): 网页按钮/键盘方向键控制云台角度
- IDLE   (停止): 保持当前位置不动

========== 启动服务 ==========

  cd /home/chenhangwei/usv_workspace/src/usv_drivers
  CUDA_VISIBLE_DEVICES="" python3 test/test_gimbal_web.py \\
      --calib ./stereo_calibration.yaml --port 8767

========== 关闭服务 ==========

  Ctrl+C  或  kill $(lsof -t -i:8767)

========== 可选参数 ==========

  --left /dev/video0        左摄像头设备
  --right /dev/video2       右摄像头设备
  --width 640               图像宽度
  --height 480              图像高度
  --calib PATH              标定文件路径
    --model PATH              SCRFD/YuNet 人脸模型路径 (.onnx)
  --confidence 0.45         检测置信度阈值
  --port 8767               Web 服务端口
  --no-hardware             无硬件模式 (模拟舵机, 不操作 PCA9685)
  --pan-channel 1           PCA9685 Pan 通道
  --tilt-channel 2          PCA9685 Tilt 通道

========== 注意事项 ==========

  - 必须设置 CUDA_VISIBLE_DEVICES="" 防止 GPU 崩溃
  - 键盘控制: ← → ↑ ↓ 方向键移动云台, P=巡视, T=跟踪, M=手动, S=停止
  - 云台与 usv_head_action_node 共用 PCA9685 (CH0=扭脖子)
"""

import argparse
import json
import math
import os
import threading
import time
import urllib.request
import zipfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import numpy as np

class _NumpyEncoder(json.JSONEncoder):
    def default(self, o):
        if isinstance(o, (np.integer,)):
            return int(o)
        if isinstance(o, (np.floating,)):
            return float(o)
        if isinstance(o, np.ndarray):
            return o.tolist()
        return super().default(o)

import cv2
import numpy as np
import yaml

ORT_AVAILABLE = True
try:
    import onnxruntime as ort
    ort.set_default_logger_severity(3)
except Exception:
    ORT_AVAILABLE = False
    ort = None

# ---- 可选 PCA9685 硬件依赖 ----
HARDWARE_ADAFRUIT = True
_busio = None
_SCL = None
_SDA = None
_PCA9685 = None
try:
    from board import SCL as _SCL, SDA as _SDA
    import busio as _busio
    from adafruit_pca9685 import PCA9685 as _PCA9685
except Exception:
    HARDWARE_ADAFRUIT = False

# smbus2 后端 (USB-to-I2C, 如 CH341A)
HARDWARE_SMBUS = True
try:
    import smbus2 as _smbus2
except ImportError:
    HARDWARE_SMBUS = False

# ---- 模式常量 ----
MODE_IDLE = 0
MODE_PATROL = 1
MODE_TRACK = 2
MODE_MANUAL = 3
MODE_NAMES = {MODE_IDLE: 'IDLE', MODE_PATROL: 'PATROL',
              MODE_TRACK: 'TRACK', MODE_MANUAL: 'MANUAL'}

FACE_COLORS = [
    (255, 0, 255),
    (0, 200, 255),
    (0, 220, 0),
    (255, 128, 0),
    (255, 80, 80),
    (180, 80, 255),
]


# ======================================================================
# PCA9685 smbus2 后端 (USB-to-I2C, 如 CH341A)
# ======================================================================
class _SMBusChannelProxy:
    def __init__(self, pca, index):
        self._pca = pca
        self._index = index
        self._duty = 0

    @property
    def duty_cycle(self):
        return self._duty

    @duty_cycle.setter
    def duty_cycle(self, value):
        value = max(0, min(0xFFFF, int(value)))
        self._duty = value
        off_val = value >> 4
        if off_val >= 4096:
            self._pca._write_ch(self._index, 4096, 0)
        elif off_val == 0:
            self._pca._write_ch(self._index, 0, 4096)
        else:
            self._pca._write_ch(self._index, 0, off_val)


class PCA9685SMBus:
    _MODE1 = 0x00
    _PRESCALE = 0xFE
    _LED0_ON_L = 0x06

    def __init__(self, bus=0, address=0x40):
        self._bus = _smbus2.SMBus(bus)
        self._addr = address
        self.channels = [_SMBusChannelProxy(self, i) for i in range(16)]
        self._freq = 50
        self._bus.write_byte_data(self._addr, self._MODE1, 0x00)
        time.sleep(0.005)

    @property
    def frequency(self):
        return self._freq

    @frequency.setter
    def frequency(self, freq):
        prescale = int(round(25000000.0 / (4096.0 * freq)) - 1)
        prescale = max(3, min(255, prescale))
        old = self._bus.read_byte_data(self._addr, self._MODE1)
        self._bus.write_byte_data(self._addr, self._MODE1, (old & 0x7F) | 0x10)
        self._bus.write_byte_data(self._addr, self._PRESCALE, prescale)
        self._bus.write_byte_data(self._addr, self._MODE1, old)
        time.sleep(0.005)
        self._bus.write_byte_data(self._addr, self._MODE1, old | 0xA0)
        self._freq = freq

    def _write_ch(self, ch, on, off):
        reg = self._LED0_ON_L + 4 * ch
        self._bus.write_byte_data(self._addr, reg, on & 0xFF)
        self._bus.write_byte_data(self._addr, reg + 1, (on >> 8) & 0x0F)
        self._bus.write_byte_data(self._addr, reg + 2, off & 0xFF)
        self._bus.write_byte_data(self._addr, reg + 3, (off >> 8) & 0x0F)

    def deinit(self):
        try:
            for ch in range(16):
                self._write_ch(ch, 0, 0)
            self._bus.close()
        except Exception:
            pass


# ======================================================================
# PID 控制器
# ======================================================================
class PIDController:
    def __init__(self, kp, ki, kd, output_limit, i_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        # 限制积分项的最大贡献量（默认最大占总输出限制的15%，或不超过 30 度/秒）
        self.i_limit = i_limit if i_limit is not None else min(output_limit * 0.15, 30.0)
        self._integral = 0.0
        self._prev_error = 0.0

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, error, dt):
        if dt <= 0:
            return 0.0
        self._integral += error * dt
        # 修复积分抗饱和 (Integral Windup)
        # 将积分值的绝对上限，限制在合理的范围内，避免舵机在物理死角处长时间累积庞大误差
        max_integral = self.i_limit / max(self.ki, 1e-6)
        self._integral = max(-max_integral, min(max_integral, self._integral))
        
        derivative = (error - self._prev_error) / dt
        self._prev_error = error
        out = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(-self.output_limit, min(self.output_limit, out))


# ======================================================================
# 全局状态
# ======================================================================
class AppState:
    def __init__(self):
        # 摄像头
        self.cap_left = None
        self.cap_right = None
        self.frame_w = 640
        self.frame_h = 480
        self.swap_stereo = False
        self.stereo_probe_metrics = None

        # 标定
        self.calib_file = ''
        self.rectify_maps = None
        self.focal_length = 328.5
        self.baseline = 0.024
        self.cx = 320.0
        self.cy = 240.0

        # 检测
        self.detector = None
        self.face_detector_name = 'HAAR'
        self.conf_thresh = 0.45
        self.detect_min_conf_scrfd = 0.64
        self.detect_min_conf_yunet = 0.84
        self.detect_min_conf_haar = 0.88
        self.detect_top_ignore_ratio = 0.18
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        self.face_cascade_alt = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_alt2.xml')
        self.eye_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_eye.xml')
        self.face_profile_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_profileface.xml')

        # 立体匹配
        self.stereo_matcher = None

        # 云台硬件
        self.pca = None
        self.pan_channel = 1
        self.tilt_channel = 2
        self.pan_inverted = True
        self.tilt_inverted = True
        self.pulse_min_us = 500
        self.pulse_max_us = 2500
        self.pwm_freq = 50

        # 云台角度范围
        self.pan_min = -90.0
        self.pan_max = 90.0
        self.tilt_min = -45.0
        self.tilt_max = 30.0
        self.max_angular_speed = 60.0

        # 云台状态
        self.current_pan = 0.0
        self.current_tilt = 0.0
        self.target_pan = 0.0
        self.target_tilt = 0.0
        self.current_pan_vel = 0.0
        self.current_tilt_vel = 0.0
        self.smooth_speed = 260.0   # 平滑最大角速度 deg/s
        self.smooth_accel = 520.0   # 平滑最大角加速度 deg/s^2
        self.track_pan_smooth_speed = 620.0   # 左右跟踪更高的最大角速度
        self.track_pan_smooth_accel = 2200.0  # 左右跟踪更高的最大角加速度
        self.track_pan_response_gain = 8.4    # 左右跟踪更积极地追目标角
        self.track_smooth_speed = 460.0   # 跟踪时更高的最大角速度
        self.track_smooth_accel = 1400.0  # 跟踪时更高的最大角加速度
        self.track_response_gain = 6.8    # 跟踪时更快靠近目标角
        self.switch_pan_smooth_speed = 760.0  # 左右切换时的短时最大角速度
        self.switch_pan_smooth_accel = 3200.0 # 左右切换时的短时最大角加速度
        self.switch_pan_response_gain = 9.0   # 左右切换时更积极地追目标角
        self.switch_smooth_speed = 520.0  # 切换人脸时的短时最大角速度
        self.switch_smooth_accel = 1800.0 # 切换人脸时的短时最大角加速度
        self.switch_response_gain = 7.5   # 切换人脸时更积极地追目标角
        self.smooth_alpha = 0.25   # 指数平滑因子 (0~1, 越小越平滑)
        self.mode = MODE_PATROL
        self.tracking = False
        self.track_target_class = ''
        self.track_target_distance = 0.0
        self.track_target_face_id = None

        # 巡视参数
        self.patrol_pan_speed = 20.0
        self.patrol_pan_range = 80.0
        self.patrol_tilt_center = 0.0
        self.patrol_edge_hold = 0.5
        self._patrol_direction = 1
        self._patrol_tilt_idx = 0
        self._patrol_holding = False
        self._patrol_hold_until = 0.0

        # 跟踪参数
        self.track_lost_timeout = 10.0
        self.track_iou_thresh = 0.3
        self.track_bbox_alpha = 0.28
        self.track_pan_deadzone_px = 4.0
        self.track_tilt_deadzone_px = 10.0
        self.track_pan_soft_zone_px = 44.0
        self.track_tilt_soft_zone_px = 60.0
        self.track_confirm_frames = 4
        self.track_min_face_area = 1800.0
        self.track_top_ignore_ratio = 0.22
        self.track_min_conf_scrfd = 0.58
        self.track_min_conf_yunet = 0.78
        self.track_min_conf_haar = 0.84
        self._track_target_bbox = None
        self._track_last_seen = 0.0
        self._pending_track_bbox = None
        self._pending_track_hits = 0
        self._pending_track_distance = 0.0
        self.pan_pid = PIDController(0.14, 0.001, 0.040, 320.0)
        self.tilt_pid = PIDController(0.12, 0.001, 0.035, 260.0)

        # 多人脸轮换参数
        self.face_gaze_duration = 12.0   # 每张脸注视时长 (秒)
        self.face_switch_boost_duration = 0.9
        self.face_target_grace_duration = 0.6
        self._face_roster = []          # 当前帧所有人脸列表
        self._face_current_idx = 0      # 当前注视的人脸索引
        self._face_switch_time = 0.0    # 上次切换人脸的时刻
        self._face_current_track_id = None
        self._face_next_id = 1
        self._face_tracks = []
        self.face_track_timeout = 1.5
        self.face_match_iou_thresh = 0.25
        self._face_switch_boost_until = 0.0

        # 手动控制
        self._manual_vel_pan = 0.0
        self._manual_vel_tilt = 0.0

        # 运行时
        self.lock = threading.Lock()
        self.latest_frame = None
        self.latest_depth = None
        self.running = True
        self.fps = 0.0
        self.detections = []
        self.frame_count = 0
        self._last_ctrl_time = time.time()


STATE = AppState()


# ======================================================================
# 初始化
# ======================================================================
def open_camera(device, width, height):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f'[ERROR] 无法打开摄像头: {device}')
        return None
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f'[INFO] {device}: {actual_w}x{actual_h}')
    return cap


def load_calibration(calib_file):
    s = STATE
    if not calib_file or not os.path.exists(calib_file):
        print('[WARN] 无标定文件，使用默认参数')
        return
    with open(calib_file, 'r') as f:
        calib = yaml.safe_load(f)
    s.focal_length = calib['focal_length_px']
    s.baseline = calib['baseline_m']
    s.cx = calib.get('cx', 320.0)
    s.cy = calib.get('cy', 240.0)
    npz_path = calib_file.replace('.yaml', '_maps.npz')
    if os.path.exists(npz_path):
        maps = np.load(npz_path)
        s.rectify_maps = (maps['map1_l'], maps['map2_l'],
                          maps['map1_r'], maps['map2_r'])
        print(f'[INFO] 已加载校正映射: {npz_path}')
    else:
        img_size = tuple(calib['image_size'])
        K_l, dist_l = np.array(calib['K_left']), np.array(calib['dist_left'])
        K_r, dist_r = np.array(calib['K_right']), np.array(calib['dist_right'])
        R, T = np.array(calib['R']), np.array(calib['T'])
        R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(
            K_l, dist_l, K_r, dist_r, img_size, R, T, alpha=0)
        m1l, m2l = cv2.initUndistortRectifyMap(
            K_l, dist_l, R1, P1, img_size, cv2.CV_32FC1)
        m1r, m2r = cv2.initUndistortRectifyMap(
            K_r, dist_r, R2, P2, img_size, cv2.CV_32FC1)
        s.rectify_maps = (m1l, m2l, m1r, m2r)
    print(f'[INFO] 标定参数: focal={s.focal_length:.1f}px '
          f'baseline={s.baseline*1000:.1f}mm')


def _compute_rectified_pair_metrics(rect_left, rect_right, stereo_matcher):
    gray_l = cv2.cvtColor(rect_left, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(rect_right, cv2.COLOR_BGR2GRAY)

    disparity = stereo_matcher.compute(gray_l, gray_r).astype(np.float32) / 16.0
    valid = disparity[disparity > 0.5]
    valid_ratio = float(valid.size) / float(disparity.size) if disparity.size else 0.0

    orb = cv2.ORB_create(1000)
    keypoints_l, desc_l = orb.detectAndCompute(gray_l, None)
    keypoints_r, desc_r = orb.detectAndCompute(gray_r, None)
    median_abs_dy = 999.0
    p90_abs_dy = 999.0
    median_dx = 0.0
    matches_used = 0

    if desc_l is not None and desc_r is not None:
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = matcher.match(desc_l, desc_r)
        matches = sorted(matches, key=lambda m: m.distance)[:200]
        dys = []
        dxs = []
        for match in matches:
            point_l = keypoints_l[match.queryIdx].pt
            point_r = keypoints_r[match.trainIdx].pt
            dx = point_r[0] - point_l[0]
            dy = point_r[1] - point_l[1]
            if abs(dx) < 200.0 and abs(dy) < 80.0:
                dys.append(abs(dy))
                dxs.append(dx)
        matches_used = len(dys)
        if dys:
            median_abs_dy = float(np.median(dys))
            p90_abs_dy = float(np.percentile(dys, 90))
            median_dx = float(np.median(dxs))

    return {
        'valid_ratio': valid_ratio,
        'median_disp': float(np.median(valid)) if valid.size else -1.0,
        'p90_disp': float(np.percentile(valid, 90)) if valid.size else -1.0,
        'matches': matches_used,
        'median_abs_dy': median_abs_dy,
        'p90_abs_dy': p90_abs_dy,
        'median_dx': median_dx,
    }


def _score_rectified_pair(metrics):
    score = metrics['valid_ratio']
    score -= 0.03 * min(metrics['median_abs_dy'], 10.0)
    if metrics['median_dx'] < -1.0:
        score += 0.15
    elif metrics['median_dx'] > 1.0:
        score -= 0.15
    return score


def auto_configure_stereo_order():
    s = STATE
    if s.rectify_maps is None or s.cap_left is None or s.cap_right is None:
        return

    probe_left = None
    probe_right = None
    for _ in range(12):
        ret_l, frame_l = s.cap_left.read()
        ret_r, frame_r = s.cap_right.read()
        if ret_l and ret_r and frame_l is not None and frame_r is not None:
            probe_left = frame_l
            probe_right = frame_r

    if probe_left is None or probe_right is None:
        print('[WARN] 立体顺序自检失败: 启动时未取到有效双目帧')
        return

    m1l, m2l, m1r, m2r = s.rectify_maps
    rect_left = cv2.remap(probe_left, m1l, m2l, cv2.INTER_LINEAR)
    rect_right = cv2.remap(probe_right, m1r, m2r, cv2.INTER_LINEAR)
    rect_left_swapped = cv2.remap(probe_right, m1l, m2l, cv2.INTER_LINEAR)
    rect_right_swapped = cv2.remap(probe_left, m1r, m2r, cv2.INTER_LINEAR)

    normal_metrics = _compute_rectified_pair_metrics(
        rect_left, rect_right, s.stereo_matcher)
    swapped_metrics = _compute_rectified_pair_metrics(
        rect_left_swapped, rect_right_swapped, s.stereo_matcher)
    normal_score = _score_rectified_pair(normal_metrics)
    swapped_score = _score_rectified_pair(swapped_metrics)

    s.stereo_probe_metrics = {
        'normal': normal_metrics,
        'swapped': swapped_metrics,
        'normal_score': normal_score,
        'swapped_score': swapped_score,
    }

    if swapped_score > normal_score + 0.12:
        s.swap_stereo = True
        print('[WARN] 检测到当前左右目顺序与标定不一致，已自动交换左右输入')
    else:
        s.swap_stereo = False

    order = 'SWAPPED' if s.swap_stereo else 'NORMAL'
    print(
        '[INFO] 立体顺序自检: '
        f'normal(valid={normal_metrics["valid_ratio"]:.3f}, '
        f'dy={normal_metrics["median_abs_dy"]:.3f}px, '
        f'dx={normal_metrics["median_dx"]:.3f}, '
        f'score={normal_score:.3f}) | '
        f'swapped(valid={swapped_metrics["valid_ratio"]:.3f}, '
        f'dy={swapped_metrics["median_abs_dy"]:.3f}px, '
        f'dx={swapped_metrics["median_dx"]:.3f}, '
        f'score={swapped_score:.3f}) | '
        f'use={order}'
    )


def _distance_to_bbox(points, distance, max_shape=None):
    x1 = points[:, 0] - distance[:, 0]
    y1 = points[:, 1] - distance[:, 1]
    x2 = points[:, 0] + distance[:, 2]
    y2 = points[:, 1] + distance[:, 3]
    if max_shape is not None:
        x1 = np.clip(x1, 0, max_shape[1])
        y1 = np.clip(y1, 0, max_shape[0])
        x2 = np.clip(x2, 0, max_shape[1])
        y2 = np.clip(y2, 0, max_shape[0])
    return np.stack((x1, y1, x2, y2), axis=-1)


def _distance_to_kps(points, distance, max_shape=None):
    x = points[:, 0]
    y = points[:, 1]
    preds = []
    for idx in range(0, distance.shape[1], 2):
        px = x + distance[:, idx]
        py = y + distance[:, idx + 1]
        if max_shape is not None:
            px = np.clip(px, 0, max_shape[1])
            py = np.clip(py, 0, max_shape[0])
        preds.extend([px, py])
    return np.stack(preds, axis=-1)


class SCRFDFaceDetector:
    def __init__(self, model_file):
        if not ORT_AVAILABLE:
            raise RuntimeError('onnxruntime 不可用')
        if not model_file or not os.path.exists(model_file):
            raise FileNotFoundError(model_file)
        self.model_file = model_file
        self.session = ort.InferenceSession(
            model_file, providers=['CPUExecutionProvider'])
        self.center_cache = {}
        self.nms_thresh = 0.40
        self.det_thresh = 0.50
        self.input_size = None
        self.input_mean = 127.5
        self.input_std = 128.0
        self.batched = False
        self.use_kps = False
        self._num_anchors = 1
        self._init_session()

    def _init_session(self):
        input_cfg = self.session.get_inputs()[0]
        input_shape = input_cfg.shape
        if not isinstance(input_shape[2], str) and not isinstance(input_shape[3], str):
            self.input_size = tuple(input_shape[2:4][::-1])
        self.input_name = input_cfg.name
        outputs = self.session.get_outputs()
        if outputs and len(outputs[0].shape) == 3:
            self.batched = True
        self.output_names = [output.name for output in outputs]
        out_count = len(outputs)
        if out_count == 6:
            self.fmc = 3
            self._feat_stride_fpn = [8, 16, 32]
            self._num_anchors = 2
        elif out_count == 9:
            self.fmc = 3
            self._feat_stride_fpn = [8, 16, 32]
            self._num_anchors = 2
            self.use_kps = True
        elif out_count == 10:
            self.fmc = 5
            self._feat_stride_fpn = [8, 16, 32, 64, 128]
            self._num_anchors = 1
        elif out_count == 15:
            self.fmc = 5
            self._feat_stride_fpn = [8, 16, 32, 64, 128]
            self._num_anchors = 1
            self.use_kps = True
        else:
            raise RuntimeError(f'不支持的 SCRFD 输出数量: {out_count}')

    def prepare(self, det_thresh=0.50, nms_thresh=0.40, input_size=None):
        self.det_thresh = det_thresh
        self.nms_thresh = nms_thresh
        if input_size is not None and self.input_size is None:
            self.input_size = input_size

    def _get_anchor_centers(self, width, height, stride):
        key = (width, height, stride, self._num_anchors)
        if key in self.center_cache:
            return self.center_cache[key]
        grid_x, grid_y = np.meshgrid(np.arange(width), np.arange(height))
        centers = np.stack((grid_x, grid_y), axis=-1).astype(np.float32)
        centers = (centers.reshape((-1, 2)) * stride)
        if self._num_anchors > 1:
            centers = np.repeat(centers, self._num_anchors, axis=0)
        self.center_cache[key] = centers
        return centers

    def forward(self, image, threshold):
        blob = cv2.dnn.blobFromImage(
            image, 1.0 / self.input_std,
            (image.shape[1], image.shape[0]),
            (self.input_mean, self.input_mean, self.input_mean),
            swapRB=True)
        outputs = self.session.run(self.output_names, {self.input_name: blob})
        input_height = blob.shape[2]
        input_width = blob.shape[3]
        scores_list = []
        boxes_list = []
        kps_list = []

        for idx, stride in enumerate(self._feat_stride_fpn):
            scores = outputs[idx]
            bbox_preds = outputs[idx + self.fmc]
            if self.batched:
                scores = scores[0]
                bbox_preds = bbox_preds[0]
            bbox_preds = bbox_preds * stride
            if self.use_kps:
                kps_preds = outputs[idx + self.fmc * 2]
                if self.batched:
                    kps_preds = kps_preds[0]
                kps_preds = kps_preds * stride
            else:
                kps_preds = None

            scores = scores.reshape(-1)
            bbox_preds = bbox_preds.reshape((-1, 4))
            if kps_preds is not None:
                kps_preds = kps_preds.reshape((-1, kps_preds.shape[-1]))

            feat_height = input_height // stride
            feat_width = input_width // stride
            centers = self._get_anchor_centers(feat_width, feat_height, stride)
            keep = np.where(scores >= threshold)[0]
            if keep.size == 0:
                continue

            selected_centers = centers[keep]
            scores_list.append(scores[keep].reshape((-1, 1)))
            boxes_list.append(_distance_to_bbox(selected_centers, bbox_preds[keep]))
            if kps_preds is not None:
                kps_list.append(_distance_to_kps(selected_centers, kps_preds[keep]))

        return scores_list, boxes_list, kps_list

    def _nms(self, dets):
        if dets.shape[0] == 0:
            return []
        x1 = dets[:, 0]
        y1 = dets[:, 1]
        x2 = dets[:, 2]
        y2 = dets[:, 3]
        scores = dets[:, 4]
        areas = (x2 - x1 + 1.0) * (y2 - y1 + 1.0)
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            w = np.maximum(0.0, xx2 - xx1 + 1.0)
            h = np.maximum(0.0, yy2 - yy1 + 1.0)
            inter = w * h
            union = areas[i] + areas[order[1:]] - inter
            iou = np.divide(inter, union, out=np.zeros_like(inter), where=union > 0)
            inds = np.where(iou <= self.nms_thresh)[0]
            order = order[inds + 1]
        return keep

    def detect(self, image, thresh=None, input_size=None, max_num=0, metric='default'):
        det_thresh = self.det_thresh if thresh is None else thresh
        det_size = input_size or self.input_size
        if det_size is None:
            raise RuntimeError('SCRFD 需要固定输入尺寸')

        image_h, image_w = image.shape[:2]
        image_ratio = float(image_h) / max(1.0, float(image_w))
        model_ratio = float(det_size[1]) / max(1.0, float(det_size[0]))
        if image_ratio > model_ratio:
            new_h = det_size[1]
            new_w = max(1, int(round(new_h / image_ratio)))
        else:
            new_w = det_size[0]
            new_h = max(1, int(round(new_w * image_ratio)))
        det_scale = float(new_h) / max(1.0, float(image_h))

        resized = cv2.resize(image, (new_w, new_h))
        det_img = np.zeros((det_size[1], det_size[0], 3), dtype=np.uint8)
        det_img[:new_h, :new_w, :] = resized

        scores_list, boxes_list, kps_list = self.forward(det_img, det_thresh)
        if not scores_list:
            return np.empty((0, 5), dtype=np.float32), None

        scores = np.vstack(scores_list)
        boxes = np.vstack(boxes_list) / det_scale
        order = scores.ravel().argsort()[::-1]
        det = np.hstack((boxes, scores)).astype(np.float32, copy=False)
        det = det[order, :]
        keep = self._nms(det)
        det = det[keep, :]

        if self.use_kps and kps_list:
            kpss = np.vstack(kps_list) / det_scale
            kpss = kpss[order, :]
            kpss = kpss[keep, :]
        else:
            kpss = None

        if max_num > 0 and det.shape[0] > max_num:
            area = (det[:, 2] - det[:, 0]) * (det[:, 3] - det[:, 1])
            img_center = np.array([image_w * 0.5, image_h * 0.5], dtype=np.float32)
            offsets = np.stack(
                (((det[:, 0] + det[:, 2]) * 0.5) - img_center[0],
                 ((det[:, 1] + det[:, 3]) * 0.5) - img_center[1]),
                axis=0)
            offset_dist2 = np.sum(np.power(offsets, 2.0), axis=0)
            values = area if metric == 'max' else area - offset_dist2 * 2.0
            selected = np.argsort(values)[::-1][:max_num]
            det = det[selected, :]
            if kpss is not None:
                kpss = kpss[selected, :]

        return det, kpss


def _download_url(url, target_path):
    os.makedirs(os.path.dirname(target_path), exist_ok=True)
    urllib.request.urlretrieve(url, target_path)


def _ensure_scrfd_model(model_path=''):
    if model_path and os.path.exists(model_path):
        return model_path

    default_paths = [
        os.path.expanduser('~/.cache/usv_models/det_500m.onnx'),
        os.path.join(os.path.dirname(__file__), 'models', 'det_500m.onnx'),
    ]
    for path in default_paths:
        if os.path.exists(path):
            return path

    target_path = default_paths[0]
    zip_path = os.path.join(os.path.dirname(target_path), 'buffalo_sc.zip')
    model_url = ('https://github.com/deepinsight/insightface/releases/'
                 'download/v0.7/buffalo_sc.zip')
    try:
        print('[INFO] 尝试下载 SCRFD 模型包 buffalo_sc...')
        _download_url(model_url, zip_path)
        with zipfile.ZipFile(zip_path, 'r') as zf:
            if 'det_500m.onnx' not in zf.namelist():
                raise RuntimeError('buffalo_sc.zip 中未找到 det_500m.onnx')
            zf.extract('det_500m.onnx', os.path.dirname(target_path))
        print(f'[INFO] 已下载 SCRFD 模型: {target_path}')
        return target_path
    except Exception as e:
        print(f'[WARN] SCRFD 模型下载失败: {e}')
        return None
    finally:
        try:
            if os.path.exists(zip_path):
                os.remove(zip_path)
        except OSError:
            pass


def _ensure_yunet_model(model_path=''):
    if model_path and os.path.exists(model_path):
        return model_path

    default_paths = [
        os.path.expanduser('~/.cache/usv_models/face_detection_yunet_2023mar.onnx'),
        os.path.join(os.path.dirname(__file__), 'models',
                     'face_detection_yunet_2023mar.onnx'),
    ]
    for path in default_paths:
        if os.path.exists(path):
            return path

    target_path = default_paths[0]
    os.makedirs(os.path.dirname(target_path), exist_ok=True)
    url = ('https://github.com/opencv/opencv_zoo/raw/main/models/'
           'face_detection_yunet/face_detection_yunet_2023mar.onnx')
    try:
        print('[INFO] 尝试下载 YuNet 人脸模型...')
        urllib.request.urlretrieve(url, target_path)
        print(f'[INFO] 已下载 YuNet 模型: {target_path}')
        return target_path
    except Exception as e:
        print(f'[WARN] YuNet 模型下载失败: {e}')
        return None


def load_face_detector(model_path, conf_thresh, input_size):
    if ORT_AVAILABLE:
        scrfd_path = _ensure_scrfd_model(model_path)
        if scrfd_path:
            try:
                detector = SCRFDFaceDetector(scrfd_path)
                detector.prepare(det_thresh=conf_thresh, input_size=(640, 640))
                print(f'[INFO] 已启用 SCRFD 人脸检测: {scrfd_path}')
                return {'kind': 'scrfd', 'model': detector}, 'SCRFD'
            except Exception as e:
                print(f'[WARN] SCRFD 初始化失败，回退到 YuNet/Haar: {e}')
    else:
        print('[WARN] onnxruntime 不可用，跳过 SCRFD，回退到 YuNet/Haar')

    if not hasattr(cv2, 'FaceDetectorYN_create'):
        print('[WARN] 当前 OpenCV 不支持 FaceDetectorYN，回退到 Haar')
        return None, 'HAAR'

    yunet_path = _ensure_yunet_model(model_path)
    if yunet_path:
        try:
            detector = cv2.FaceDetectorYN_create(
                yunet_path, '', input_size, conf_thresh, 0.3, 5000)
            print(f'[INFO] 已启用 YuNet 人脸检测: {yunet_path}')
            return {'kind': 'yunet', 'model': detector}, 'YUNET'
        except Exception as e:
            print(f'[WARN] YuNet 初始化失败，回退到 Haar: {e}')

    return None, 'HAAR'


def init_pca9685(pan_ch, tilt_ch, no_hardware, i2c_backend='auto', i2c_bus=0):
    s = STATE
    s.pan_channel = pan_ch
    s.tilt_channel = tilt_ch
    if no_hardware:
        print('[INFO] --no-hardware 指定，模拟模式运行')
        return

    backends = []
    if i2c_backend == 'auto':
        backends = ['adafruit', 'smbus']
    elif i2c_backend in ('adafruit', 'smbus'):
        backends = [i2c_backend]
    else:
        print(f'[INFO] i2c_backend={i2c_backend}，模拟模式运行')
        return

    for be in backends:
        if be == 'adafruit' and HARDWARE_ADAFRUIT:
            try:
                from typing import Any, cast
                i2c = cast(Any, _busio).I2C(_SCL, _SDA)
                s.pca = cast(Any, _PCA9685)(i2c)
                s.pca.frequency = s.pwm_freq
                print(f'[INFO] PCA9685 已通过 adafruit (板载 I2C) 初始化 '
                      f'(Pan=CH{pan_ch}, Tilt=CH{tilt_ch})')
                return
            except Exception as e:
                print(f'[WARN] adafruit 后端失败: {e}')

        elif be == 'smbus' and HARDWARE_SMBUS:
            try:
                s.pca = PCA9685SMBus(bus=i2c_bus, address=0x40)
                s.pca.frequency = s.pwm_freq
                print(f'[INFO] PCA9685 已通过 smbus2 (/dev/i2c-{i2c_bus}) 初始化'
                      f' — CH341 USB-I2C (Pan=CH{pan_ch}, Tilt=CH{tilt_ch})')
                return
            except Exception as e:
                print(f'[WARN] smbus2 后端失败 (bus={i2c_bus}): {e}')

    print(f'[WARN] 所有 I2C 后端均失败，模拟模式运行')


def run_detection(detector, image, conf_thresh):
    if detector is None:
        return []
    backend, model = detector
    if backend == 'ultralytics':
        results = model(image, verbose=False, conf=conf_thresh)
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
        blob = cv2.dnn.blobFromImage(image, 1/255.0, (640, 640), swapRB=True)
        model.setInput(blob)
        outputs = model.forward(model.getUnconnectedOutLayersNames())
        h, w = image.shape[:2]
        dets = []
        for output in outputs:
            for row in output[0]:
                scores = row[4:]
                cls_id = int(np.argmax(scores))
                conf = float(scores[cls_id])
                if conf < conf_thresh:
                    continue
                cx, cy, bw, bh = row[:4]
                x1 = int((cx - bw/2) * w / 640)
                y1 = int((cy - bh/2) * h / 640)
                x2 = int((cx + bw/2) * w / 640)
                y2 = int((cy + bh/2) * h / 640)
                dets.append((str(cls_id), conf, x1, y1, x2, y2))
        return dets
    return []


# ======================================================================
# 舵机控制
# ======================================================================
def apply_angles(pan, tilt):
    """设置目标角度，实际运动由 servo_smooth_thread 平滑驱动"""
    s = STATE
    s.target_pan = max(s.pan_min, min(s.pan_max, pan))
    s.target_tilt = max(s.tilt_min, min(s.tilt_max, tilt))


def apply_angles_immediate(pan, tilt):
    """立即跳转到指定角度（无平滑）"""
    s = STATE
    s.target_pan = max(s.pan_min, min(s.pan_max, pan))
    s.target_tilt = max(s.tilt_min, min(s.tilt_max, tilt))
    s.current_pan = s.target_pan
    s.current_tilt = s.target_tilt
    s.current_pan_vel = 0.0
    s.current_tilt_vel = 0.0
    _set_servo(s.pan_channel, s.current_pan, s.pan_min, s.pan_max)
    _set_servo(s.tilt_channel, s.current_tilt, s.tilt_min, s.tilt_max)


def _set_servo(channel, angle_deg, angle_min, angle_max):
    s = STATE
    if s.pca is None:
        return
    ratio = (angle_deg - angle_min) / (angle_max - angle_min)
    ratio = max(0.0, min(1.0, ratio))
    if channel == s.pan_channel and s.pan_inverted:
        ratio = 1.0 - ratio
    if channel == s.tilt_channel and s.tilt_inverted:
        ratio = 1.0 - ratio
    pulse_us = s.pulse_min_us + ratio * (s.pulse_max_us - s.pulse_min_us)
    period_us = 1e6 / s.pwm_freq
    duty = int(pulse_us / period_us * 0xFFFF)
    duty = max(0, min(0xFFFF, duty))
    try:
        s.pca.channels[channel].duty_cycle = duty
    except Exception:
        pass


# ======================================================================
# 舵机平滑线程 (50Hz)
# ======================================================================
def servo_smooth_thread():
    """独立高频线程: 50Hz 平滑驱动舵机 (含手动速度控制)"""
    s = STATE
    interval = 0.02  # 50Hz
    last_t = time.time()
    while s.running:
        now = time.time()
        dt = now - last_t
        last_t = now
        if dt <= 0 or dt > 0.2:
            dt = interval

        # 手动模式: 直接在此线程按速度更新 target
        if s.mode == MODE_MANUAL:
            if s._manual_vel_pan != 0.0 or s._manual_vel_tilt != 0.0:
                s.target_pan = max(s.pan_min, min(s.pan_max,
                    s.target_pan + s._manual_vel_pan * dt))
                s.target_tilt = max(s.tilt_min, min(s.tilt_max,
                    s.target_tilt + s._manual_vel_tilt * dt))

        pan_smooth_speed = s.smooth_speed
        pan_smooth_accel = s.smooth_accel
        pan_response_gain = 4.0
        tilt_smooth_speed = s.smooth_speed
        tilt_smooth_accel = s.smooth_accel
        tilt_response_gain = 4.0
        if s.mode == MODE_TRACK:
            pan_smooth_speed = s.track_pan_smooth_speed
            pan_smooth_accel = s.track_pan_smooth_accel
            pan_response_gain = s.track_pan_response_gain
            tilt_smooth_speed = s.track_smooth_speed
            tilt_smooth_accel = s.track_smooth_accel
            tilt_response_gain = s.track_response_gain
            if now < s._face_switch_boost_until:
                pan_smooth_speed = s.switch_pan_smooth_speed
                pan_smooth_accel = s.switch_pan_smooth_accel
                pan_response_gain = s.switch_pan_response_gain
                tilt_smooth_speed = s.switch_smooth_speed
                tilt_smooth_accel = s.switch_smooth_accel
                tilt_response_gain = s.switch_response_gain

        max_pan_vel_step = pan_smooth_accel * dt
        max_tilt_vel_step = tilt_smooth_accel * dt

        # Pan: 速度/加速度受限，形成缓起缓停
        diff_pan = s.target_pan - s.current_pan
        if abs(diff_pan) > 0.01:
            target_pan_vel = max(-pan_smooth_speed,
                                 min(pan_smooth_speed, diff_pan * pan_response_gain))
            vel_delta = target_pan_vel - s.current_pan_vel
            vel_step = min(abs(vel_delta), max_pan_vel_step)
            s.current_pan_vel += vel_step if vel_delta > 0 else -vel_step
            s.current_pan += s.current_pan_vel * dt
            s.current_pan = max(s.pan_min, min(s.pan_max, s.current_pan))
            if ((diff_pan > 0 and s.current_pan > s.target_pan) or
                    (diff_pan < 0 and s.current_pan < s.target_pan)):
                s.current_pan = s.target_pan
                s.current_pan_vel = 0.0
        else:
            s.current_pan = s.target_pan
            s.current_pan_vel = 0.0

        # Tilt: 速度/加速度受限，形成缓起缓停
        diff_tilt = s.target_tilt - s.current_tilt
        if abs(diff_tilt) > 0.01:
            target_tilt_vel = max(-tilt_smooth_speed,
                                  min(tilt_smooth_speed, diff_tilt * tilt_response_gain))
            vel_delta = target_tilt_vel - s.current_tilt_vel
            vel_step = min(abs(vel_delta), max_tilt_vel_step)
            s.current_tilt_vel += vel_step if vel_delta > 0 else -vel_step
            s.current_tilt += s.current_tilt_vel * dt
            s.current_tilt = max(s.tilt_min, min(s.tilt_max, s.current_tilt))
            if ((diff_tilt > 0 and s.current_tilt > s.target_tilt) or
                    (diff_tilt < 0 and s.current_tilt < s.target_tilt)):
                s.current_tilt = s.target_tilt
                s.current_tilt_vel = 0.0
        else:
            s.current_tilt = s.target_tilt
            s.current_tilt_vel = 0.0

        # 写入硬件
        _set_servo(s.pan_channel, s.current_pan, s.pan_min, s.pan_max)
        _set_servo(s.tilt_channel, s.current_tilt, s.tilt_min, s.tilt_max)

        elapsed = time.time() - now
        sleep_t = interval - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)


# ======================================================================
# IOU 计算
# ======================================================================
def compute_iou(a, b):
    x1 = max(a[0], b[0])
    y1 = max(a[1], b[1])
    x2 = min(a[2], b[2])
    y2 = min(a[3], b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def _face_color(face_id):
    if not isinstance(face_id, int) or face_id <= 0:
        return (255, 0, 255)
    return FACE_COLORS[(face_id - 1) % len(FACE_COLORS)]


def _face_center(face):
    return ((face['x1'] + face['x2']) * 0.5,
            (face['y1'] + face['y2']) * 0.5)


def _trigger_track_speed_boost(now_ts):
    s = STATE
    s._face_switch_boost_until = max(
        s._face_switch_boost_until,
        now_ts + s.face_switch_boost_duration,
    )
    s.current_pan_vel = 0.0
    s.current_tilt_vel = 0.0


def _assign_face_ids(face_detections, now_ts):
    s = STATE
    tracks = [
        track for track in s._face_tracks
        if now_ts - track['last_seen'] <= s.face_track_timeout
    ]
    matched_track_indices = set()
    labeled_faces = []

    for face in sorted(face_detections, key=lambda item: item['conf'], reverse=True):
        bbox = [face['x1'], face['y1'], face['x2'], face['y2']]
        bw = max(1.0, float(face['x2'] - face['x1']))
        bh = max(1.0, float(face['y2'] - face['y1']))
        cx, cy = _face_center(face)

        best_track_idx = None
        best_score = -1.0
        for track_idx, track in enumerate(tracks):
            if track_idx in matched_track_indices:
                continue
            track_bbox = track['bbox']
            iou = compute_iou(track_bbox, bbox)
            tcx = (track_bbox[0] + track_bbox[2]) * 0.5
            tcy = (track_bbox[1] + track_bbox[3]) * 0.5
            center_dist = math.hypot(cx - tcx, cy - tcy)
            tbw = max(1.0, float(track_bbox[2] - track_bbox[0]))
            tbh = max(1.0, float(track_bbox[3] - track_bbox[1]))
            max_center_dist = 0.6 * max(bw, bh, tbw, tbh) + 25.0
            if iou < s.face_match_iou_thresh and center_dist > max_center_dist:
                continue
            center_score = max(0.0, 1.0 - center_dist / max(max_center_dist, 1.0))
            score = iou * 2.0 + center_score
            if score > best_score:
                best_score = score
                best_track_idx = track_idx

        if best_track_idx is None:
            track = {
                'face_id': s._face_next_id,
                'bbox': list(bbox),
                'last_seen': now_ts,
            }
            s._face_next_id += 1
            tracks.append(track)
            best_track_idx = len(tracks) - 1
        else:
            track = tracks[best_track_idx]
            alpha = 0.30
            prev_bbox = track['bbox']
            track['bbox'] = [
                (1.0 - alpha) * prev_bbox[0] + alpha * bbox[0],
                (1.0 - alpha) * prev_bbox[1] + alpha * bbox[1],
                (1.0 - alpha) * prev_bbox[2] + alpha * bbox[2],
                (1.0 - alpha) * prev_bbox[3] + alpha * bbox[3],
            ]
            track['last_seen'] = now_ts

        matched_track_indices.add(best_track_idx)
        labeled_face = dict(face)
        labeled_face['face_id'] = tracks[best_track_idx]['face_id']
        labeled_faces.append(labeled_face)

    s._face_tracks = tracks
    return labeled_faces


def _dedupe_face_boxes(face_boxes, iou_thresh=0.35):
    if not face_boxes:
        return []
    kept = []
    for face in sorted(face_boxes, key=lambda item: item['conf'], reverse=True):
        bbox = [face['x1'], face['y1'], face['x2'], face['y2']]
        duplicated = False
        for existing in kept:
            existing_bbox = [existing['x1'], existing['y1'],
                             existing['x2'], existing['y2']]
            if compute_iou(bbox, existing_bbox) >= iou_thresh:
                duplicated = True
                break
        if not duplicated:
            kept.append(face)
    return kept


def _is_face_like_region(gray, x, y, bw, bh):
    if bw < 40 or bh < 40:
        return False
    aspect = bw / max(bh, 1)
    if aspect < 0.55 or aspect > 1.6:
        return False
    roi = gray[y:y + bh, x:x + bw]
    if roi.size == 0:
        return False
    mean_val = float(np.mean(roi))
    std_val = float(np.std(roi))
    if mean_val > 235.0 and std_val < 28.0:
        return False
    if std_val < 12.0:
        return False
    return True


def _has_enough_face_structure(gray, x, y, bw, bh, require_eyes=False,
                               relaxed=False):
    s = STATE
    roi = gray[y:y + bh, x:x + bw]
    if roi.size == 0:
        return False

    edges = cv2.Canny(roi, 60, 140)
    edge_density = float(np.count_nonzero(edges)) / float(roi.size)
    min_edge_density = 0.02 if relaxed else 0.03
    if edge_density < min_edge_density:
        return False

    center_roi = roi[bh // 4: (bh * 3) // 4, bw // 6: (bw * 5) // 6]
    if center_roi.size == 0:
        return False
    center_std = float(np.std(center_roi))
    min_center_std = 12.0 if relaxed else 14.0
    if center_std < min_center_std:
        return False

    if require_eyes:
        if s.eye_cascade.empty():
            return center_std >= 18.0 and edge_density >= 0.04
        eyes = s.eye_cascade.detectMultiScale(
            roi, scaleFactor=1.1, minNeighbors=4, minSize=(8, 8))
        return len(eyes) >= 1 or (center_std >= 20.0 and edge_density >= 0.045)

    if relaxed:
        return edge_density >= 0.03 or center_std >= 16.0
    return edge_density >= 0.045 or center_std >= 20.0


def _passes_face_detection_gate(face, frame_w, frame_h):
    s = STATE
    x1, y1, x2, y2 = face['x1'], face['y1'], face['x2'], face['y2']
    bw = max(1, x2 - x1)
    bh = max(1, y2 - y1)
    area = bw * bh
    center_y = (y1 + y2) * 0.5
    if s.face_detector_name == 'SCRFD':
        conf_thresh = s.detect_min_conf_scrfd
    elif s.face_detector_name == 'YUNET':
        conf_thresh = s.detect_min_conf_yunet
    else:
        conf_thresh = s.detect_min_conf_haar
    if face['conf'] < conf_thresh:
        return False
    if area < 1200:
        return False
    if center_y < frame_h * s.detect_top_ignore_ratio and area < frame_w * frame_h * 0.06:
        return False
    return True


def _detect_faces(image):
    s = STATE
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    image_width = gray.shape[1]
    
    # 辅助函数: ONNX 推理表情
    def _get_emotion(gray_frame, bx1, by1, bx2, by2):
        if not getattr(s, 'emotion_model', None):
            return None, None
        face_roi = gray_frame[by1:by2, bx1:bx2]
        if face_roi.shape[0] == 0 or face_roi.shape[1] == 0:
            return None, None
        try:
            inp = cv2.resize(face_roi, (64, 64)).astype(np.float32)
            inp = np.expand_dims(inp, axis=(0, 1))
            out = s.emotion_model.run(None, {s.emotion_input_name: inp})[0]
            # FER+ labels: 0:neutral 1:happiness 2:surprise 3:sadness 4:anger 5:disgust 6:fear 7:contempt
            emotions = [
                ("[ Neutral ._. ]", (200, 200, 200)),
                ("[ Happy ^_^ ]", (0, 255, 0)),
                ("[ Surprise O_O ]", (0, 255, 255)),
                ("[ Sad T_T ]", (255, 0, 0)),
                ("[ Angry >_< ]", (0, 0, 255)),
                ("[ Disgust +_+ ]", (128, 0, 128)),
                ("[ Fear =_= ]", (0, 165, 255)),
                ("[ Contempt -_- ]", (128, 128, 128)),
            ]
            idx = np.argmax(out[0])
            return emotions[idx][0], emotions[idx][1]
        except Exception:
            return None, None

    if s.detector is not None and s.detector.get('kind') == 'scrfd':
        try:
            boxes, kpss = s.detector['model'].detect(
                image, thresh=s.conf_thresh, input_size=(640, 640))
            detected = []
            for i, box in enumerate(boxes):
                x1, y1, x2, y2, score = box
                x1 = max(0, int(round(x1)))
                y1 = max(0, int(round(y1)))
                x2 = min(image.shape[1], int(round(x2)))
                y2 = min(image.shape[0], int(round(y2)))
                bw_i = x2 - x1
                bh_i = y2 - y1
                if bw_i <= 0 or bh_i <= 0:
                    continue
                if not _is_face_like_region(gray, x1, y1, bw_i, bh_i):
                    continue
                # SCRFD 高置信度框优先信任，避免侧脸/遮挡脸被手工纹理规则误杀。
                if float(score) < 0.72:
                    if not _has_enough_face_structure(
                            gray, x1, y1, bw_i, bh_i, relaxed=True):
                        continue
                
                # 表情推断
                expression = "[ Neutral ._. ]"
                color = (200, 200, 200)
                
                # 预处理：利用人脸关键点识别“歪头”动作及“露齿”动作
                is_head_tilt = False
                is_teeth_showing = False
                eye_dist = 0
                mouth_width = 0
                if kpss is not None and i < len(kpss):
                    pts = kpss[i]
                    if pts.ndim == 1 and pts.shape[0] == 10:
                        eye_dx = pts[2] - pts[0]
                        eye_dy = pts[3] - pts[1]
                    elif pts.ndim == 2 and pts.shape[0] == 5:
                        eye_dx = pts[1][0] - pts[0][0]
                        eye_dy = pts[1][1] - pts[0][1]
                    else:
                        eye_dx = eye_dy = 0
                    
                    eye_dist = math.hypot(eye_dx, eye_dy)
                    
                    # 只要眼睛水平线歪斜超过 15 度，即判断为歪头
                    if eye_dist > 5:
                        if eye_dx == 0:  # 完全垂直
                            is_head_tilt = True
                        else:
                            angle = abs(math.degrees(math.atan2(eye_dy, eye_dx)))
                            if angle > 15.0 and angle < 165.0:
                                is_head_tilt = True
                                
                    # 获取嘴巴坐标用于提取嘴部 ROI 进行露齿检测
                    if pts.ndim == 1 and pts.shape[0] == 10:
                        kx3, ky3 = pts[6], pts[7]   # 左嘴角
                        kx4, ky4 = pts[8], pts[9]   # 右嘴角
                    elif pts.ndim == 2 and pts.shape[0] == 5:
                        kx3, ky3 = pts[3]
                        kx4, ky4 = pts[4]
                    else:
                        kx3=ky3=kx4=ky4=0
                        
                    if eye_dist > 15 and abs(kx3) > 1:
                        mouth_width = math.hypot(kx4 - kx3, ky4 - ky3)
                        # 放宽咧嘴的要求（嘴角宽度达到眼距的70%）
                        if mouth_width > 10 and mouth_width / eye_dist > 0.70:
                            mx = int((kx3 + kx4) / 2)
                            my = int((ky3 + ky4) / 2)
                            roi_w = int(mouth_width * 0.6)
                            roi_h = int(mouth_width * 0.4) # 获取嘴唇中间稍微高一点的区域
                            tx1 = max(0, mx - roi_w // 2)
                            tx2 = min(image.shape[1], mx + roi_w // 2)
                            ty1 = max(0, my - roi_h // 2)
                            ty2 = min(image.shape[0], my + roi_h // 2)
                            
                            if tx2 > tx1 and ty2 > ty1:
                                roi_bgr = image[ty1:ty2, tx1:tx2]
                                hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
                                # 适度放宽亮度和饱和度限制，兼顾室内环境下的牙齿(可能偏暗偏黄)
                                lower_white = np.array([0, 0, 110])
                                upper_white = np.array([180, 70, 255])
                                teeth_mask = cv2.inRange(hsv, lower_white, upper_white)
                                
                                ratio = cv2.countNonZero(teeth_mask) / ((tx2 - tx1) * (ty2 - ty1) + 1e-5)
                                if ratio > 0.05: # 放宽占比要求到 5%
                                    is_teeth_showing = True

                if is_teeth_showing:
                    expression = "[ Teeth :D ]"
                    color = (255, 255, 255) # 白色
                elif is_head_tilt:
                    expression = "[ HeadTilt /_\\ ]"
                    color = (0, 255, 255) # 黄色
                else:
                    # 使用 ONNX FER+ 模型 (如果存在)
                    onnx_expr, onnx_color = _get_emotion(gray, x1, y1, x2, y2)
                    if onnx_expr:
                        expression = onnx_expr
                        color = onnx_color
                    elif kpss is not None and i < len(kpss):
                        # 降级到几何关键点分析
                        if eye_dist > 5 and mouth_width > 0:
                            ratio = mouth_width / eye_dist
                            if ratio > 1.25:
                                expression = "[ Laughing =D ]"
                                color = (0, 165, 255)
                            elif ratio > 1.05:
                                expression = "[ Smiling ^_^ ]"
                                color = (0, 255, 0)

                face = {
                    'cls': 'face', 'conf': float(score),
                    'x1': x1, 'y1': y1,
                    'x2': x2, 'y2': y2,
                    'expr': expression,
                    'expr_color': color
                }
                if _passes_face_detection_gate(face, image.shape[1], image.shape[0]):
                    detected.append(face)
            return _dedupe_face_boxes(detected)
        except Exception as e:
            print(f'[WARN] SCRFD 检测失败，回退到 YuNet/Haar: {e}')
            s.detector, s.face_detector_name = load_face_detector(
                '', s.conf_thresh, (image.shape[1], image.shape[0]))

    if s.detector is not None and s.detector.get('kind') == 'yunet':
        try:
            yunet = s.detector['model']
            yunet.setInputSize((image.shape[1], image.shape[0]))
            _, faces = yunet.detect(image)
            if faces is not None:
                detected = []
                for face in faces:
                    x, y, bw, bh = face[:4]
                    score = float(face[-1])
                    x1 = max(0, int(round(x)))
                    y1 = max(0, int(round(y)))
                    x2 = min(image.shape[1], int(round(x + bw)))
                    y2 = min(image.shape[0], int(round(y + bh)))
                    bw_i = x2 - x1
                    bh_i = y2 - y1
                    if bw_i <= 0 or bh_i <= 0:
                        continue
                    if not _is_face_like_region(gray, x1, y1, bw_i, bh_i):
                        continue
                    if not _has_enough_face_structure(gray, x1, y1, bw_i, bh_i):
                        continue

                    # 表情推断
                    expression = "[ Neutral ._. ]"
                    color = (200, 200, 200)
                    if len(face) >= 15:
                        eye_dx = face[6] - face[4]
                        eye_dy = face[7] - face[5]
                        eye_dist = math.hypot(eye_dx, eye_dy)

                        mouth_dx = face[12] - face[10]
                        mouth_dy = face[13] - face[11]
                        mouth_width = math.hypot(mouth_dx, mouth_dy)

                        if eye_dist > 5:
                            ratio = mouth_width / eye_dist
                            if ratio > 1.25:
                                expression = "[ Laughing =D ]"
                                color = (0, 165, 255)
                            elif ratio > 1.05:
                                expression = "[ Smiling ^_^ ]"
                                color = (0, 255, 0)
                                
                    out_face = {
                        'cls': 'face', 'conf': score,
                        'x1': x1, 'y1': y1,
                        'x2': x2, 'y2': y2,
                        'expr': expression,
                        'expr_color': color
                    }
                    if _passes_face_detection_gate(out_face, image.shape[1], image.shape[0]):
                        detected.append(out_face)
                return _dedupe_face_boxes(detected)
        except Exception as e:
            print(f'[WARN] YuNet 检测失败，回退到 Haar: {e}')
            s.detector = None
            s.face_detector_name = 'HAAR'

    detected = []

    frontal_cascades = [
        (s.face_cascade, 0.92),
        (s.face_cascade_alt, 0.88),
    ]
    for cascade, conf in frontal_cascades:
        if cascade.empty():
            continue
        boxes = cascade.detectMultiScale(
            gray, scaleFactor=1.08, minNeighbors=4, minSize=(36, 36))
        for (x, y, bw, bh) in boxes:
            if not _is_face_like_region(gray, x, y, bw, bh):
                continue
            if not _has_enough_face_structure(gray, x, y, bw, bh,
                                              require_eyes=True):
                continue
            face = {
                'cls': 'face', 'conf': conf,
                'x1': int(x), 'y1': int(y),
                'x2': int(x + bw), 'y2': int(y + bh),
                'expr': '[ Neutral ._. ]',
                'expr_color': (200, 200, 200)
            }
            if _passes_face_detection_gate(face, image.shape[1], image.shape[0]):
                detected.append(face)

    if not s.face_profile_cascade.empty():
        boxes = s.face_profile_cascade.detectMultiScale(
            gray, scaleFactor=1.08, minNeighbors=4, minSize=(36, 36))
        for (x, y, bw, bh) in boxes:
            if not _is_face_like_region(gray, x, y, bw, bh):
                continue
            if not _has_enough_face_structure(gray, x, y, bw, bh):
                continue
            face = {
                'cls': 'face', 'conf': 0.86,
                'x1': int(x), 'y1': int(y),
                'x2': int(x + bw), 'y2': int(y + bh),
                'expr': '[ Neutral ._. ]',
                'expr_color': (200, 200, 200)
            }
            if _passes_face_detection_gate(face, image.shape[1], image.shape[0]):
                detected.append(face)

        flipped = cv2.flip(gray, 1)
        boxes_flipped = s.face_profile_cascade.detectMultiScale(
            flipped, scaleFactor=1.08, minNeighbors=4, minSize=(36, 36))
        for (x, y, bw, bh) in boxes_flipped:
            x1 = image_width - (x + bw)
            x2 = image_width - x
            if not _is_face_like_region(gray, x1, y, bw, bh):
                continue
            if not _has_enough_face_structure(gray, x1, y, bw, bh):
                continue
            face = {
                'cls': 'face', 'conf': 0.84,
                'x1': int(x1), 'y1': int(y),
                'x2': int(x2), 'y2': int(y + bh),
                'expr': '[ Neutral ._. ]',
                'expr_color': (200, 200, 200)
            }
            if _passes_face_detection_gate(face, image.shape[1], image.shape[0]):
                detected.append(face)

    return _dedupe_face_boxes(detected)


def _is_trackable_face(face):
    s = STATE
    x1, y1, x2, y2 = face['x1'], face['y1'], face['x2'], face['y2']
    bw = max(1, x2 - x1)
    bh = max(1, y2 - y1)
    area = float(bw * bh)
    aspect = bw / float(bh)
    center_y = (y1 + y2) * 0.5
    frame_area = float(max(1, s.frame_w * s.frame_h))

    if s.face_detector_name == 'SCRFD':
        conf_thresh = s.track_min_conf_scrfd
    elif s.face_detector_name == 'YUNET':
        conf_thresh = s.track_min_conf_yunet
    else:
        conf_thresh = s.track_min_conf_haar
    if face['conf'] < conf_thresh:
        return False
    if area < s.track_min_face_area:
        return False
    if aspect < 0.6 or aspect > 1.45:
        return False
    if (center_y < s.frame_h * s.track_top_ignore_ratio and
            area < frame_area * 0.05):
        return False
    return True


def _estimate_bbox_distance(depth_map, x1, y1, x2, y2):
    h, w = depth_map.shape[:2]

    def _clip_roi(ax1, ay1, ax2, ay2):
        ax1 = max(0, min(w, int(round(ax1))))
        ay1 = max(0, min(h, int(round(ay1))))
        ax2 = max(0, min(w, int(round(ax2))))
        ay2 = max(0, min(h, int(round(ay2))))
        if ax2 <= ax1 or ay2 <= ay1:
            return None
        return depth_map[ay1:ay2, ax1:ax2]

    bw = max(1, x2 - x1)
    bh = max(1, y2 - y1)
    cx = (x1 + x2) * 0.5
    cy = (y1 + y2) * 0.5

    rois = [
        _clip_roi(cx - 15, cy - 15, cx + 15, cy + 15),
        _clip_roi(x1 + bw * 0.15, y1 + bh * 0.10, x2 - bw * 0.15, y2 - bh * 0.10),
        _clip_roi(x1 - bw * 0.20, y1 - bh * 0.10, x2 + bw * 0.20, y2 + bh * 0.25),
    ]

    for roi in rois:
        if roi is None or roi.size == 0:
            continue
        valid = roi[(roi > 0.05) & (roi < 20.0)]
        if valid.size < 12:
            continue
        valid = np.sort(valid)
        front_cluster = valid[:max(12, int(valid.size * 0.45))]
        return float(np.median(front_cluster))

    return -1.0


# ======================================================================
# 云台控制循环
# ======================================================================
def gimbal_control_step(dets_enriched):
    """一次云台控制更新 (与视觉检测同步调用)"""
    s = STATE
    now = time.time()
    dt = now - s._last_ctrl_time
    s._last_ctrl_time = now
    if dt <= 0 or dt > 1.0:
        dt = 0.05

    # ---- 处理检测结果，选择跟踪目标 ----
    if dets_enriched and s.mode in (MODE_PATROL, MODE_TRACK):
        best = _select_target(dets_enriched)
        if best is not None:
            new_bbox = [best['x1'], best['y1'], best['x2'], best['y2']]
            new_face_id = best.get('face_id')
            prev_face_id = s.track_target_face_id
            face_switched = (
                s.tracking and
                prev_face_id is not None and
                new_face_id is not None and
                new_face_id != prev_face_id
            )
            if s.mode == MODE_PATROL and not s.tracking:
                if (s._pending_track_bbox is not None and
                        compute_iou(s._pending_track_bbox, new_bbox) >= 0.35):
                    s._pending_track_hits += 1
                    alpha = 0.35
                    prev_bbox = s._pending_track_bbox
                    s._pending_track_bbox = [
                        (1.0 - alpha) * prev_bbox[0] + alpha * new_bbox[0],
                        (1.0 - alpha) * prev_bbox[1] + alpha * new_bbox[1],
                        (1.0 - alpha) * prev_bbox[2] + alpha * new_bbox[2],
                        (1.0 - alpha) * prev_bbox[3] + alpha * new_bbox[3],
                    ]
                else:
                    s._pending_track_bbox = new_bbox
                    s._pending_track_hits = 1
                s._pending_track_distance = best['dist']

                if s._pending_track_hits < s.track_confirm_frames:
                    return

                s._track_target_bbox = list(s._pending_track_bbox)
                s._pending_track_bbox = None
                s._pending_track_hits = 0
            elif s._track_target_bbox is None or not s.tracking:
                s._track_target_bbox = new_bbox
            elif face_switched:
                s._track_target_bbox = new_bbox
                _trigger_track_speed_boost(now)
            else:
                alpha = s.track_bbox_alpha
                prev_bbox = s._track_target_bbox
                s._track_target_bbox = [
                    (1.0 - alpha) * prev_bbox[0] + alpha * new_bbox[0],
                    (1.0 - alpha) * prev_bbox[1] + alpha * new_bbox[1],
                    (1.0 - alpha) * prev_bbox[2] + alpha * new_bbox[2],
                    (1.0 - alpha) * prev_bbox[3] + alpha * new_bbox[3],
                ]
            s.track_target_class = best['cls']
            s.track_target_distance = best['dist']
            s.track_target_face_id = best.get('face_id')
            s._track_last_seen = now
            s.tracking = True
            
            # --- 表情联动行为策略 ---
            if prev_face_id != s.track_target_face_id:
                s._kiss_triggered = False
                s._is_shaking = False
                s._is_nodding = False
                s._kiss_detect_frames = 0 # 重置连续帧计数

            expr = best.get('expr', '')

            # 当检测到人脸持续靠近一段时间时，触发低头回避动作
            if getattr(s, "track_target_distance", 2.0) < 0.25:
                s._proximity_frames = getattr(s, "_proximity_frames", 0) + 1
                if s._proximity_frames >= 5: # 连续5帧(<0.25m)视为切实贴近
                    if getattr(s, "_proximity_triggered", False):
                        pass
                    else:
                        s._proximity_triggered = True
                        if not getattr(s, "_is_hiding", False):
                            print(f"[GIMBAL] 探测到用户持续贴近 (连续5帧 < 0.25m) -> 执行低头回避动作 3 秒")
                            s._is_hiding = True
                            s._hide_start_time = now
                        if len(s._face_roster) > 1:
                            print(f"[GIMBAL] 抛弃过度贴近的 Face#{s.track_target_face_id}")
                            s._face_switch_time = 0.0
                            _trigger_track_speed_boost(now)
            else:
                s._proximity_frames = 0
                s._proximity_triggered = False

            if "Happy" in expr or "Laughing" in expr or "Smiling" in expr or "Surprise" in expr:
                s.face_gaze_duration = 15.0
                if not getattr(s, "_is_nodding", False):
                    if now - getattr(s, "_nod_start_time", 0) > 3.0:
                        print(f"[GIMBAL] 探测到喜悦/惊讶 {expr} -> 触发 2 次点头")
                        s._is_nodding = True
                        s._nod_start_time = now
            elif "Teeth" in expr:
                s.face_gaze_duration = 15.0
                if not getattr(s, "_is_shaking", False):
                    if now - getattr(s, "_kiss_start_time", 0) > 3.0:
                        print(f"[GIMBAL] 探测到露出牙齿 {expr} -> 触发 2 次摇头反馈")
                        s._is_shaking = True
                        s._kiss_start_time = now
            elif "HeadTilt" in expr:
                s.face_gaze_duration = 15.0
            else:
                s.face_gaze_duration = 8.0


            
            if getattr(s, '_is_nodding', False) and now - getattr(s, '_nod_start_time', 0) > 1.0:
                s._is_nodding = False
                
            if getattr(s, '_is_shaking', False) and now - getattr(s, '_kiss_start_time', 0) > 1.0:
                s._is_shaking = False # 摇头持续1秒

            if s.mode == MODE_PATROL:
                s.mode = MODE_TRACK
                s._face_switch_time = now  # 首次进入跟踪, 重置轮换计时
                s._face_current_track_id = best.get('face_id')
                s._face_current_idx = next(
                    (idx for idx, face in enumerate(s._face_roster)
                     if face.get('face_id') == s._face_current_track_id),
                    0,
                )
                s.pan_pid.reset()
                s.tilt_pid.reset()
                _trigger_track_speed_boost(now)
                n_faces = len(s._face_roster)
                print(f'[GIMBAL] 发现 {n_faces} 张人脸, '
                      f'优先跟踪 Face#{best.get("face_id", "?")} '
                      f'{best["dist"]:.1f}m → TRACK')
    elif s.mode == MODE_PATROL:
        s._pending_track_bbox = None
        s._pending_track_hits = 0

    # ---- 模式控制 ----
    if s.mode == MODE_PATROL:
        _update_patrol(dt, now)
    elif s.mode == MODE_TRACK:
        _update_track(dt, now)
    # MODE_MANUAL 的速度更新已移至 servo_smooth_thread (50Hz)


def _select_target(dets):
    """多人脸轮换选择: 以稳定 Face#ID 为单位，每张脸持续注视 12 秒后轮换。"""
    s = STATE
    now = time.time()
    valid = [d for d in dets if d['conf'] > 0.3]
    if not valid:
        s._face_roster = []
        s._face_current_track_id = None
        return None

    roster = sorted(valid, key=lambda d: d.get('face_id', 0))
    s._face_roster = roster

    if len(roster) == 1:
        only_face = roster[0]
        if (s.tracking and s._face_current_track_id is not None and
                only_face.get('face_id') != s._face_current_track_id and
                now - s._track_last_seen <= s.face_target_grace_duration):
            return None
        if s._face_current_track_id != only_face.get('face_id'):
            s._face_current_track_id = only_face.get('face_id')
            s._face_current_idx = 0
            s._face_switch_time = now
        return only_face

    current_idx = next(
        (idx for idx, face in enumerate(roster)
         if face.get('face_id') == s._face_current_track_id),
        None,
    )

    if current_idx is None:
        if (s.tracking and s._face_current_track_id is not None and
                now - s._track_last_seen <= s.face_target_grace_duration):
            return None
        initial_face = max(
            roster,
            key=lambda face: face['dist'] if face['dist'] > 0 else -1.0,
        )
        s._face_current_track_id = initial_face.get('face_id')
        current_idx = next(
            idx for idx, face in enumerate(roster)
            if face.get('face_id') == s._face_current_track_id
        )
        s._face_current_idx = current_idx
        s._face_switch_time = now
        return initial_face

    if now - s._face_switch_time >= s.face_gaze_duration:
        current_idx = (current_idx + 1) % len(roster)
        s._face_current_track_id = roster[current_idx].get('face_id')
        s._face_current_idx = current_idx
        s._face_switch_time = now
        s.pan_pid.reset()
        s.tilt_pid.reset()
        _trigger_track_speed_boost(now)
        print(
            f'[GIMBAL] 轮换注视 Face#{s._face_current_track_id} '
            f'({current_idx + 1}/{len(roster)})'
        )
    else:
        s._face_current_idx = current_idx

    return roster[s._face_current_idx]


def _update_patrol(dt, now):
    s = STATE
    if s._patrol_holding:
        if now >= s._patrol_hold_until:
            s._patrol_holding = False
            s._patrol_direction *= -1
        else:
            return
    pan_delta = s.patrol_pan_speed * dt * s._patrol_direction
    new_pan = s.current_pan + pan_delta
    limit = s.patrol_pan_range
    if new_pan >= limit:
        new_pan = limit
        s._patrol_holding = True
        s._patrol_hold_until = now + s.patrol_edge_hold
    elif new_pan <= -limit:
        new_pan = -limit
        s._patrol_holding = True
        s._patrol_hold_until = now + s.patrol_edge_hold
    target_tilt = s.patrol_tilt_center
    tilt_diff = target_tilt - s.current_tilt
    max_step = 30.0 * dt
    if abs(tilt_diff) <= max_step:
        new_tilt = target_tilt
    else:
        new_tilt = s.current_tilt + max_step * (1 if tilt_diff > 0 else -1)
    apply_angles(new_pan, new_tilt)


def _update_track(dt, now):
    s = STATE
    
    # --- 优先处理单人情绪回避（低头掩面）逻辑 ---
    if getattr(s, '_is_hiding', False):
        if now - getattr(s, '_hide_start_time', now) < 3.0:
            # 维持 TRACK 模式且不超时，暂停 PID 计算
            s._track_last_seen = now 
            s.pan_pid.reset()
            s.tilt_pid.reset()
            # -20.0 修正低头方向
            apply_angles(s.current_pan, -20.0)
            return
        else:
            s._is_hiding = False
            s._is_recovering = True
            s._recover_start_time = now
            print("[GIMBAL] 结束低头回避 3 秒，快速重新抬头恢复捕捉")
            
    if getattr(s, '_is_recovering', False):
        if now - getattr(s, '_recover_start_time', now) < 0.5:
            # 强行快速抬头到水平 (0.0度)，跳过巡视模式的缓速上限
            s._track_last_seen = now 
            s.pan_pid.reset()
            s.tilt_pid.reset()
            apply_angles(s.current_pan, 0.0)
            return
        else:
            s._is_recovering = False
            # 抬头动作完成后，清空滞后的目标框，迫使云台利用新视线重新寻找并锁定
            s._track_target_bbox = None
            s._track_last_seen = 0
            return

    if now - s._track_last_seen > s.track_lost_timeout:
        print(f'[GIMBAL] 目标丢失 {s.track_lost_timeout:.1f}s → PATROL')
        s.tracking = False
        s.track_target_face_id = None
        s._track_target_bbox = None
        s._pending_track_bbox = None
        s._pending_track_hits = 0
        s._face_current_track_id = None
        s.mode = MODE_PATROL
        s._patrol_direction = 1
        s._patrol_tilt_idx = 0
        s._patrol_holding = False
        s.pan_pid.reset()
        s.tilt_pid.reset()
        return
    if s._track_target_bbox is None:
        return
    bbox = s._track_target_bbox
    tcx = (bbox[0] + bbox[2]) / 2.0
    tcy = (bbox[1] + bbox[3]) / 2.0
    err_x = tcx - s.frame_w / 2.0
    err_y = tcy - s.frame_h / 2.0

    axis_ref = max(s.frame_w, s.frame_h) / 2.0

    def shape_error(err, axis_half, deadzone_px, soft_zone_px):
        abs_err = abs(err)
        deadzone_ratio = deadzone_px / max(axis_half, 1.0)
        soft_zone_ratio = soft_zone_px / max(axis_half, 1.0)
        err_ratio = abs_err / max(axis_half, 1.0)
        if err_ratio <= deadzone_ratio:
            return 0.0
        active_ratio = err_ratio - deadzone_ratio
        soft_ratio = min(1.0, active_ratio / max(soft_zone_ratio, 1e-6))
        shaped_ratio = active_ratio * (0.35 + 0.65 * soft_ratio * soft_ratio)
        shaped = shaped_ratio * axis_ref
        return shaped if err > 0 else -shaped

    err_x = shape_error(
        err_x,
        s.frame_w / 2.0,
        s.track_pan_deadzone_px,
        s.track_pan_soft_zone_px,
    )
    err_y = shape_error(
        err_y,
        s.frame_h / 2.0,
        s.track_tilt_deadzone_px,
        s.track_tilt_soft_zone_px,
    )
    pan_rate = s.pan_pid.compute(err_x, dt)
    tilt_rate = s.tilt_pid.compute(err_y, dt)

    # 惊讶/高兴情绪 -> 加入硬件点头动作
    if getattr(s, '_is_nodding', False):
        t_elapsed = now - getattr(s, '_nod_start_time', now)
        if t_elapsed < 1.0:
            # 2Hz 频率的点头，上下摆动幅度约 30 度/秒
            nod_rate = math.sin(t_elapsed * math.pi * 4.0) * 30.0
            tilt_rate += nod_rate
            
    # 露齿情绪 -> 加入硬件摇头动作
    if getattr(s, '_is_shaking', False):
        t_elapsed = now - getattr(s, '_kiss_start_time', now)
        if t_elapsed < 1.0:
            # 2Hz 频率的摇头，左右摆动幅度约 30 度/秒
            shake_rate = math.sin(t_elapsed * math.pi * 4.0) * 30.0
            pan_rate += shake_rate

    apply_angles(s.current_pan + pan_rate * dt,
                 s.current_tilt - tilt_rate * dt)


# _update_manual 已移入 servo_smooth_thread，不再需要


# ======================================================================
# 视觉+云台处理线程
# ======================================================================
def vision_gimbal_thread():
    s = STATE
    fps_counter = 0
    fps_time = time.time()
    ctrl_interval = 0.05  # 20Hz 云台控制

    while s.running:
        if s.cap_left is None or s.cap_right is None:
            time.sleep(0.1)
            continue

        ret_l, left = s.cap_left.read()
        ret_r, right = s.cap_right.read()
        if not ret_l or not ret_r:
            time.sleep(0.05)
            continue

        if s.swap_stereo:
            left, right = right, left

        # 校正
        if s.rectify_maps is not None:
            m1l, m2l, m1r, m2r = s.rectify_maps
            left = cv2.remap(left, m1l, m2l, cv2.INTER_LINEAR)
            right = cv2.remap(right, m1r, m2r, cv2.INTER_LINEAR)

        # 深度图
        gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        disparity = s.stereo_matcher.compute(
            gray_l, gray_r).astype(np.float32) / 16.0
        disparity[disparity <= 0] = 0.0
        disparity = cv2.medianBlur(disparity, 5)
        with np.errstate(divide='ignore', invalid='ignore'):
            depth_map = np.where(
                disparity > 0,
                s.focal_length * s.baseline / disparity,
                0.0).astype(np.float32)

        # 人脸检测 (每帧)
        s.frame_count += 1
        now = time.time()
        face_dets_enriched = []
        faces = _detect_faces(left)
        for face in faces:
            fx = face['x1']
            fy = face['y1']
            fx2 = face['x2']
            fy2 = face['y2']
            cx_px = (fx + fx2) // 2
            cy_px = (fy + fy2) // 2
            dist = _estimate_bbox_distance(depth_map, fx, fy, fx2, fy2)
            bearing_deg = float(np.degrees(
                np.arctan2(cx_px - s.cx, s.focal_length)))
            face_dets_enriched.append({
                'cls': 'face', 'conf': face['conf'],
                'x1': int(fx), 'y1': int(fy),
                'x2': int(fx2), 'y2': int(fy2),
                'dist': dist, 'bearing': bearing_deg,
                'expr': face.get('expr', '[ Neutral ._. ]'),
                'expr_color': face.get('expr_color', (200, 200, 200))
            })

        face_dets_enriched = _assign_face_ids(face_dets_enriched, now)

        with s.lock:
            s.detections = face_dets_enriched

        # ---- 云台控制 ----
        trackable_faces = [face for face in face_dets_enriched
                   if _is_trackable_face(face)]
        control_faces = trackable_faces
        if s.mode == MODE_TRACK or s.tracking:
            control_faces = face_dets_enriched
        gimbal_control_step(control_faces)

        # ---- 渲染主画面 ----
        disp = left.copy()
        with s.lock:
            dets_draw = list(s.detections)

        for d in dets_draw:
            x1, y1, x2, y2 = d['x1'], d['y1'], d['x2'], d['y2']
            dist = d['dist']
            face_id = d.get('face_id')
            is_locked_face = s.tracking and face_id == s.track_target_face_id
            color = _face_color(face_id)
            thickness = 3 if is_locked_face else 2
            cv2.rectangle(disp, (x1, y1), (x2, y2), color, thickness)
            label = f'Face#{face_id if face_id is not None else "?"} {d["conf"]:.0%}'
            if is_locked_face:
                label = '[LOOK] ' + label
            if dist > 0:
                label += f' {dist:.2f}m'
                
            # 绘制表情符号
            expr = d.get('expr', '')
            expr_color = d.get('expr_color', (200, 200, 200))
            if expr:
                cv2.putText(disp, expr, (x1, max(y1 - 36, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, expr_color, 2, cv2.LINE_AA)

            (tw, th), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(disp, (x1, y1 - th - 8), (x1 + tw + 4, y1),
                          color, -1)
            cv2.putText(disp, label, (x1 + 2, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            if dist > 0:
                info = f'{dist:.2f}m / {d["bearing"]:+.1f}deg'
                cv2.putText(disp, info, (x1, y2 + 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
            if is_locked_face and len(s._face_roster) > 1:
                remain = max(0.0, s.face_gaze_duration - (now - s._face_switch_time))
                cv2.putText(disp, f'NEXT IN {remain:.1f}s', (x1, y2 + 32),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        # 跟踪指示器
        if s.tracking and s._track_target_bbox:
            bbox = [int(round(v)) for v in s._track_target_bbox]
            cv2.rectangle(disp, (bbox[0] - 3, bbox[1] - 3),
                          (bbox[2] + 3, bbox[3] + 3), (0, 255, 255), 2)
            track_label = 'TRACKING'
            if s.track_target_face_id is not None:
                track_label += f' Face#{s.track_target_face_id}'
            cv2.putText(disp, track_label, (bbox[0], bbox[1] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # HUD
        fps_counter += 1
        now = time.time()
        if now - fps_time >= 1.0:
            s.fps = fps_counter / (now - fps_time)
            fps_counter = 0
            fps_time = now

        hud_h = 30
        cv2.rectangle(disp, (0, 0), (disp.shape[1], hud_h), (0, 0, 0), -1)
        mode_name = MODE_NAMES.get(s.mode, '?')
        track_info = f'LOCK:Face#{s.track_target_face_id}' if s.tracking and s.track_target_face_id is not None else ''
        hud = (f'FPS:{s.fps:.1f} | Pan:{s.current_pan:+.1f} '
               f'Tilt:{s.current_tilt:+.1f} | {mode_name} {track_info}')
        cv2.putText(disp, hud, (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        _, buf = cv2.imencode('.jpg', disp, [cv2.IMWRITE_JPEG_QUALITY, 80])
        with s.lock:
            s.latest_frame = buf.tobytes()

        # ---- 渲染深度图 ----
        depth_vis = np.clip(depth_map, 0.2, 5.0)
        depth_vis = ((depth_vis - 0.2) / 4.8 * 255).astype(np.uint8)
        depth_vis = cv2.GaussianBlur(depth_vis, (5, 5), 0)
        depth_color = cv2.applyColorMap(255 - depth_vis, cv2.COLORMAP_JET)
        depth_color[depth_map <= 0] = 0
        for d in dets_draw:
            color = _face_color(d.get('face_id'))
            cv2.rectangle(depth_color,
                          (d['x1'], d['y1']), (d['x2'], d['y2']),
                          color, 1)
            face_tag = f'F#{d.get("face_id", "?")}'
            if d['dist'] > 0:
                cv2.putText(depth_color, f'{face_tag} {d["dist"]:.2f}m',
                            (d['x1'], d['y1'] - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                            color, 1)
            else:
                cv2.putText(depth_color, face_tag,
                            (d['x1'], d['y1'] - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                            color, 1)

        _, dbuf = cv2.imencode('.jpg', depth_color,
                               [cv2.IMWRITE_JPEG_QUALITY, 75])
        with s.lock:
            s.latest_depth = dbuf.tobytes()

        time.sleep(0.01)


# ======================================================================
# HTML 页面
# ======================================================================
HTML_PAGE = '''<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<title>视觉云台测试</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:'Segoe UI',Arial,sans-serif;background:#111;color:#eee}
.header{background:#1a1a2e;padding:8px 16px;display:flex;align-items:center;
  gap:12px;border-bottom:2px solid #e94560}
.header h1{font-size:16px;color:#e94560}
.tag{font-size:11px;padding:2px 8px;border-radius:10px;background:#0f3460;color:#aaa}
.tag.ok{background:#00c853;color:#111}
.tag.warn{background:#ff6d00;color:#111}
.tag.track{background:#e94560;color:#fff;animation:pulse 1s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:0.6}}

.content{display:flex;height:calc(100vh - 44px)}
.left-panel{flex:1;display:flex;flex-direction:column;overflow:hidden;padding:4px;gap:4px}
.video-row{display:flex;gap:4px;flex:1}
.video-box{flex:1;position:relative;background:#222;border-radius:6px;
  overflow:hidden;border:1px solid #333}
.video-box img{width:100%;height:100%;object-fit:contain}
.video-box .label{position:absolute;top:4px;left:6px;font-size:11px;
  background:rgba(0,0,0,0.7);padding:2px 8px;border-radius:4px;color:#0f0}

.sidebar{width:320px;background:#1a1a2e;border-left:2px solid #0f3460;
  overflow-y:auto;padding:12px;display:flex;flex-direction:column;gap:12px}
.sidebar h3{font-size:13px;color:#e94560;margin-bottom:6px}

/* 云台仪表 */
.gimbal-panel{background:#0a0a1a;border-radius:8px;padding:12px;text-align:center}
.gimbal-canvas{width:200px;height:200px;margin:0 auto 8px;display:block}
.angle-info{display:flex;justify-content:center;gap:20px;font-size:14px;margin-top:6px}
.angle-val{font-size:22px;font-weight:700;color:#00e5ff}

/* 模式按钮 */
.mode-btns{display:flex;gap:6px;flex-wrap:wrap}
.mode-btn{flex:1;min-width:60px;padding:8px 4px;border:none;border-radius:6px;
  cursor:pointer;font-size:12px;font-weight:600;background:#0f3460;color:#aaa;
  transition:all 0.2s}
.mode-btn:hover{background:#16213e;color:#fff}
.mode-btn.active{background:#e94560;color:#fff}

/* 方向盘 */
.dpad{display:grid;grid-template-columns:50px 50px 50px;gap:4px;
  justify-content:center;margin:8px 0}
.dpad button{width:50px;height:50px;border:none;border-radius:8px;
  cursor:pointer;font-size:18px;background:#0f3460;color:#ddd;
  transition:all 0.15s;user-select:none}
.dpad button:active{background:#e94560;transform:scale(0.92)}
.dpad button.center{background:#16213e;font-size:12px}
.dpad-hint{text-align:center;font-size:10px;color:#555;margin-top:4px}

/* 检测列表 */
.det-list{list-style:none}
.det-item{background:#0f3460;border-radius:6px;padding:8px;margin-bottom:6px;
  font-size:12px}
.det-item .name{font-weight:600;font-size:14px}
.det-item .dist{font-size:18px;font-weight:700;color:#00e676}
.det-item .dist.near{color:#ff5252}
.det-item .dist.mid{color:#ffab40}
.no-det{color:#666;font-style:italic;padding:12px 0}

.stats{font-size:11px;color:#666;padding-top:8px;border-top:1px solid #333}
.stats div{margin-bottom:3px}

/* 跟踪配置 */
.config-row{display:flex;align-items:center;gap:8px;margin-bottom:6px;font-size:12px}
.config-row label{flex:1;color:#aaa}
.config-row select,.config-row input{background:#0a0a1a;color:#eee;border:1px solid #333;
  border-radius:4px;padding:3px 6px;font-size:12px}

@media(max-width:900px){
  .content{flex-direction:column}
  .sidebar{width:100%;height:auto;flex-direction:row;flex-wrap:wrap}
}
</style>
</head>
<body>

<div class="header">
  <h1>&#128302; 视觉云台测试</h1>
  <span class="tag" id="tag-calib">标定: --</span>
  <span class="tag" id="tag-model">模型: --</span>
  <span class="tag" id="tag-mode">模式: --</span>
  <span class="tag" id="tag-fps">FPS: --</span>
  <span class="tag" id="tag-hw">硬件: --</span>
</div>

<div class="content">
  <div class="left-panel">
    <div class="video-row">
      <div class="video-box">
        <img id="stream-main" src="/stream/main">
        <div class="label">检测画面</div>
      </div>
      <div class="video-box">
        <img id="stream-depth" src="/stream/depth">
        <div class="label" style="color:#ff0">深度图</div>
      </div>
    </div>
  </div>

  <div class="sidebar">
    <!-- 云台仪表 -->
    <div class="gimbal-panel">
      <h3>&#127912; 云台状态</h3>
      <canvas class="gimbal-canvas" id="gimbal-canvas" width="200" height="200"></canvas>
      <div class="angle-info">
        <div>Pan <span class="angle-val" id="val-pan">0.0</span>°</div>
        <div>Tilt <span class="angle-val" id="val-tilt">0.0</span>°</div>
      </div>
    </div>

    <!-- 模式切换 -->
    <div>
      <h3>&#9881; 模式切换</h3>
      <div class="mode-btns">
        <button class="mode-btn" data-mode="0" id="btn-idle">IDLE</button>
        <button class="mode-btn" data-mode="1" id="btn-patrol">PATROL</button>
        <button class="mode-btn" data-mode="2" id="btn-track">TRACK</button>
        <button class="mode-btn" data-mode="3" id="btn-manual">MANUAL</button>
      </div>
    </div>

    <!-- 方向控制 -->
    <div>
      <h3>&#127918; 手动控制</h3>
      <div class="dpad">
        <div></div>
        <button id="d-up" onmousedown="jogStart('up')" onmouseup="jogStop()"
                ontouchstart="jogStart('up')" ontouchend="jogStop()">&#9650;</button>
        <div></div>
        <button id="d-left" onmousedown="jogStart('left')" onmouseup="jogStop()"
                ontouchstart="jogStart('left')" ontouchend="jogStop()">&#9664;</button>
        <button class="center" onclick="sendCmd('center')">HOME</button>
        <button id="d-right" onmousedown="jogStart('right')" onmouseup="jogStop()"
                ontouchstart="jogStart('right')" ontouchend="jogStop()">&#9654;</button>
        <div></div>
        <button id="d-down" onmousedown="jogStart('down')" onmouseup="jogStop()"
                ontouchstart="jogStart('down')" ontouchend="jogStop()">&#9660;</button>
        <div></div>
      </div>
      <div class="dpad-hint">键盘: ← → ↑ ↓ 方向键 / P T M S 切换模式</div>
    </div>

    <!-- 检测列表 -->
    <div>
      <h3>&#127919; 检测目标</h3>
      <ul class="det-list" id="det-list">
        <li class="no-det">等待检测...</li>
      </ul>
    </div>

    <div class="stats" id="stats">
      <div>焦距: --</div>
    </div>
  </div>
</div>

<script>
// ---- 云台仪表绘制 ----
const canvas = document.getElementById('gimbal-canvas');
const ctx = canvas.getContext('2d');
function drawGimbal(pan, tilt, mode, tracking) {
  const w = canvas.width, h = canvas.height;
  const cx = w/2, cy = h/2, r = 80;
  ctx.clearRect(0, 0, w, h);

  // 外圈
  ctx.beginPath();
  ctx.arc(cx, cy, r, 0, Math.PI*2);
  ctx.strokeStyle = '#0f3460';
  ctx.lineWidth = 2;
  ctx.stroke();

  // 刻度线
  for (let a = -90; a <= 90; a += 30) {
    const rad = (a - 90) * Math.PI / 180;
    const x1 = cx + Math.cos(rad) * (r - 6);
    const y1 = cy + Math.sin(rad) * (r - 6);
    const x2 = cx + Math.cos(rad) * r;
    const y2 = cy + Math.sin(rad) * r;
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2);
    ctx.strokeStyle = '#333'; ctx.lineWidth = 1; ctx.stroke();
  }

  // Pan 指针 (水平)
  const panRad = pan * Math.PI / 180;
  const px = cx + Math.sin(panRad) * (r * 0.7);
  const py = cy;
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(px, py);
  ctx.strokeStyle = '#00e5ff'; ctx.lineWidth = 3; ctx.stroke();
  ctx.beginPath(); ctx.arc(px, py, 5, 0, Math.PI*2);
  ctx.fillStyle = '#00e5ff'; ctx.fill();

  // Tilt 指针 (垂直)
  const tiltRad = -tilt * Math.PI / 180;
  const tx = cx;
  const ty = cy + Math.sin(tiltRad) * (r * 0.7);
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(tx, ty);
  ctx.strokeStyle = '#ff9800'; ctx.lineWidth = 3; ctx.stroke();
  ctx.beginPath(); ctx.arc(tx, ty, 5, 0, Math.PI*2);
  ctx.fillStyle = '#ff9800'; ctx.fill();

  // 中心点
  ctx.beginPath(); ctx.arc(cx, cy, 4, 0, Math.PI*2);
  ctx.fillStyle = tracking ? '#e94560' : '#555'; ctx.fill();

  // 标签
  ctx.fillStyle = '#00e5ff'; ctx.font = '10px sans-serif';
  ctx.textAlign = 'center';
  ctx.fillText('Pan', cx, h - 4);
  ctx.fillStyle = '#ff9800';
  ctx.fillText('Tilt', cx + 50, cy - r - 4);

  // 角度范围标识
  ctx.fillStyle = '#444'; ctx.font = '9px sans-serif';
  ctx.textAlign = 'left'; ctx.fillText('-90°', 2, cy + 4);
  ctx.textAlign = 'right'; ctx.fillText('+90°', w - 2, cy + 4);
}

// ---- 状态轮询 ----
function pollStatus() {
  fetch('/api/status').then(r => r.json()).then(d => {
    // 顶栏
    const tc = document.getElementById('tag-calib');
    tc.textContent = '标定: ' + (d.calibrated ? '✓' : '✗');
    tc.className = 'tag' + (d.calibrated ? ' ok' : '');

    const tm = document.getElementById('tag-model');
    tm.textContent = '模型: ' + d.model;
    tm.className = 'tag' + (d.model !== 'NONE' ? ' ok' : '');

    const tmode = document.getElementById('tag-mode');
    tmode.textContent = '模式: ' + d.mode_name;
    if (d.tracking) {
      tmode.className = 'tag track';
            tmode.textContent += ' [LOCK:Face#' + (d.track_face_id ?? '?') + ']';
    } else {
      tmode.className = 'tag' + (d.mode > 0 ? ' warn' : '');
    }

    document.getElementById('tag-fps').textContent = 'FPS: ' + d.fps.toFixed(1);
    document.getElementById('tag-hw').textContent =
      '硬件: ' + (d.hardware ? '✓' : '模拟');
    document.getElementById('tag-hw').className =
      'tag' + (d.hardware ? ' ok' : '');

    // 云台仪表
    document.getElementById('val-pan').textContent = d.pan.toFixed(1);
    document.getElementById('val-tilt').textContent = d.tilt.toFixed(1);
    drawGimbal(d.pan, d.tilt, d.mode, d.tracking);

    // 模式按钮高亮
    ['idle','patrol','track','manual'].forEach((n, i) => {
      document.getElementById('btn-'+n).className =
        'mode-btn' + (d.mode === i ? ' active' : '');
    });

    // 检测列表
    const list = document.getElementById('det-list');
    if (d.detections.length === 0) {
      list.innerHTML = '<li class="no-det">未检测到目标</li>';
    } else {
      list.innerHTML = '';
      d.detections.forEach(det => {
        const li = document.createElement('li');
        li.className = 'det-item';
                const isCurrent = d.tracking && det.face_id === d.track_face_id;
        const dc = det.dist < 1 ? 'near' : det.dist < 3 ? 'mid' : '';
        const dt = det.dist > 0 ? det.dist.toFixed(2) + ' m' : '--';
        li.innerHTML =
                    '<span class="name">' + (isCurrent ? '[LOOK] ' : '') + 'Face#' + (det.face_id ?? '?') + '</span> '
          + '<span style="color:#aaa">' + (det.conf*100).toFixed(0) + '%</span><br>'
          + '<span class="dist ' + dc + '">' + dt + '</span><br>'
          + '<div style="color:#888;margin-top:3px;font-size:11px">'
                    + '方位: ' + det.bearing.toFixed(1) + '°'
                    + (isCurrent && d.face_cycle_remaining > 0 ? ' | 下次轮换 ' + d.face_cycle_remaining.toFixed(1) + 's' : '')
                    + '</div>';
        list.appendChild(li);
      });
    }

    // 统计
    document.getElementById('stats').innerHTML =
      '<div>焦距: ' + d.focal.toFixed(1) + ' px</div>'
      + '<div>基线: ' + (d.baseline*1000).toFixed(1) + ' mm</div>'
      + '<div>帧数: ' + d.frame_count + '</div>'
      + '<div>分辨率: ' + d.width + 'x' + d.height + '</div>';
  }).catch(() => {});
}
setInterval(pollStatus, 300);

// ---- 控制 API ----
function sendCmd(action, value) {
  const body = JSON.stringify({action: action, value: value || 0});
  fetch('/api/control', {method:'POST',
    headers:{'Content-Type':'application/json'}, body: body});
}

function jogStart(dir) {
  sendCmd('jog_start', dir);
}
function jogStop() {
  sendCmd('jog_stop');
}

// 模式按钮
document.querySelectorAll('.mode-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    sendCmd('set_mode', parseInt(btn.dataset.mode));
  });
});

// 键盘控制
const keysDown = {};
document.addEventListener('keydown', e => {
  if (keysDown[e.key]) return;
  keysDown[e.key] = true;
  switch(e.key) {
    case 'ArrowUp': jogStart('up'); e.preventDefault(); break;
    case 'ArrowDown': jogStart('down'); e.preventDefault(); break;
    case 'ArrowLeft': jogStart('left'); e.preventDefault(); break;
    case 'ArrowRight': jogStart('right'); e.preventDefault(); break;
    case 'p': case 'P': sendCmd('set_mode', 1); break;
    case 't': case 'T': sendCmd('set_mode', 2); break;
    case 'm': case 'M': sendCmd('set_mode', 3); break;
    case 's': case 'S': sendCmd('set_mode', 0); break;
  }
});
document.addEventListener('keyup', e => {
  delete keysDown[e.key];
  if (['ArrowUp','ArrowDown','ArrowLeft','ArrowRight'].includes(e.key)) {
    jogStop();
  }
});
</script>
</body></html>
'''


# ======================================================================
# HTTP 服务器
# ======================================================================
class GimbalVisionHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        path = self.path.split('?')[0]

        if path in ('/', '/index.html'):
            data = HTML_PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        elif path == '/stream/main':
            self._stream_mjpeg('latest_frame')

        elif path == '/stream/depth':
            self._stream_mjpeg('latest_depth')

        elif path == '/api/status':
            s = STATE
            with s.lock:
                dets = list(s.detections)
            data = {
                'fps': s.fps,
                'frame_count': s.frame_count,
                'calibrated': s.rectify_maps is not None,
                'stereo_swapped': s.swap_stereo,
                'model': s.face_detector_name,
                'hardware': s.pca is not None,
                'focal': s.focal_length,
                'baseline': s.baseline,
                'width': s.frame_w,
                'height': s.frame_h,
                'pan': s.current_pan,
                'tilt': s.current_tilt,
                'mode': s.mode,
                'mode_name': MODE_NAMES.get(s.mode, '?'),
                'tracking': s.tracking,
                'track_class': s.track_target_class,
                'track_face_id': s.track_target_face_id,
                'track_distance': s.track_target_distance,
                'face_roster_count': len(s._face_roster),
                'face_cycle_remaining': max(0.0, s.face_gaze_duration - (time.time() - s._face_switch_time)) if s.tracking and len(s._face_roster) > 1 else 0.0,
                'detections': dets,
            }
            body = json.dumps(data, ensure_ascii=False, cls=_NumpyEncoder).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        path = self.path.split('?')[0]
        if path == '/api/control':
            length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(length)
            try:
                cmd = json.loads(body)
            except (json.JSONDecodeError, ValueError):
                self.send_response(400)
                self.end_headers()
                return
            self._handle_control(cmd)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            resp = b'{"ok":true}'
            self.send_header('Content-Length', str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)
        else:
            self.send_response(404)
            self.end_headers()

    def _handle_control(self, cmd):
        s = STATE
        action = cmd.get('action', '')
        value = cmd.get('value', 0)
        jog_speed = 40.0  # 度/秒

        if action == 'set_mode':
            mode = int(value)
            if mode in MODE_NAMES:
                old = s.mode
                s.mode = mode
                s._manual_vel_pan = 0.0
                s._manual_vel_tilt = 0.0
                if mode == MODE_TRACK:
                    s.pan_pid.reset()
                    s.tilt_pid.reset()
                if mode == MODE_PATROL:
                    s._patrol_direction = 1
                    s._patrol_tilt_idx = 0
                    s._patrol_holding = False
                print(f'[WEB] 模式: {MODE_NAMES[old]} → {MODE_NAMES[mode]}')

        elif action == 'jog_start':
            s.mode = MODE_MANUAL
            d = value if isinstance(value, str) else str(value)
            if d == 'left':
                s._manual_vel_pan = -jog_speed
            elif d == 'right':
                s._manual_vel_pan = jog_speed
            elif d == 'up':
                s._manual_vel_tilt = jog_speed
            elif d == 'down':
                s._manual_vel_tilt = -jog_speed

        elif action == 'jog_stop':
            s._manual_vel_pan = 0.0
            s._manual_vel_tilt = 0.0

        elif action == 'center':
            s.mode = MODE_MANUAL
            s._manual_vel_pan = 0.0
            s._manual_vel_tilt = 0.0
            apply_angles_immediate(0.0, 0.0)

    def _stream_mjpeg(self, attr):
        self.send_response(200)
        boundary = 'frame'
        self.send_header('Content-Type',
                         f'multipart/x-mixed-replace; boundary={boundary}')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        s = STATE
        try:
            while s.running:
                with s.lock:
                    frame = getattr(s, attr, None)
                if frame is None:
                    time.sleep(0.05)
                    continue
                self.wfile.write(f'--{boundary}\r\n'.encode())
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(
                    f'Content-Length: {len(frame)}\r\n\r\n'.encode())
                self.wfile.write(frame)
                self.wfile.write(b'\r\n')
                self.wfile.flush()
                time.sleep(0.05)
        except (BrokenPipeError, ConnectionResetError):
            pass


# ======================================================================
# 主入口
# ======================================================================
def main():
    p = argparse.ArgumentParser(description='双目视觉+云台控制 Web 测试')
    p.add_argument('--left', default='/dev/video0', help='左摄像头设备')
    p.add_argument('--right', default='/dev/video2', help='右摄像头设备')
    p.add_argument('--width', type=int, default=640)
    p.add_argument('--height', type=int, default=480)
    p.add_argument('--calib', default='./stereo_calibration.yaml',
                   help='标定文件路径')
    p.add_argument('--model', default='', help='SCRFD/YuNet 人脸模型路径 (.onnx)')
    p.add_argument('--emotion-model', default='/tmp/emotion.onnx', help='FER+ 情绪模型路径 (.onnx)')
    p.add_argument('--confidence', type=float, default=0.45)
    p.add_argument('--port', type=int, default=8767)
    p.add_argument('--no-hardware', action='store_true',
                   help='不连接 PCA9685，模拟模式')
    p.add_argument('--i2c-backend', default='auto',
                   choices=['auto', 'adafruit', 'smbus', 'none'],
                   help='I2C 后端: auto(自动), adafruit(板载), smbus(USB-I2C), none(模拟)')
    p.add_argument('--i2c-bus', type=int, default=0,
                   help='smbus2 总线编号 (/dev/i2c-N), CH341 通常为 0')
    p.add_argument('--pan-channel', type=int, default=1,
                   help='PCA9685 Pan 通道')
    p.add_argument('--tilt-channel', type=int, default=2,
                   help='PCA9685 Tilt 通道')
    args = p.parse_args()

    s = STATE
    s.frame_w = args.width
    s.frame_h = args.height
    s.conf_thresh = args.confidence

    # 下载情绪模型
    if args.emotion_model and not os.path.exists(args.emotion_model):
        try:
            print(f"[INFO] 自动下载情绪识别模型 {args.emotion_model}...")
            urllib.request.urlretrieve("https://github.com/onnx/models/raw/main/validated/vision/body_analysis/emotion_ferplus/model/emotion-ferplus-8.onnx", args.emotion_model)
        except Exception as e:
            print(f"[WARN] 情绪模型下载失败: {e}")

    if args.emotion_model and os.path.exists(args.emotion_model) and ORT_AVAILABLE:
        try:
            s.emotion_model = ort.InferenceSession(args.emotion_model)
            s.emotion_input_name = s.emotion_model.get_inputs()[0].name
            print(f"[INFO] 已加载情绪识别模型 {args.emotion_model}")
        except Exception as e:
            print(f"[WARN] 情绪模型加载失败: {e}")
            s.emotion_model = None
    else:
        s.emotion_model = None

    # 标定
    load_calibration(args.calib)

    # PCA9685
    backend = 'none' if args.no_hardware else args.i2c_backend
    init_pca9685(args.pan_channel, args.tilt_channel, args.no_hardware,
                 i2c_backend=backend, i2c_bus=args.i2c_bus)

    # 摄像头
    s.cap_left = open_camera(args.left, args.width, args.height)
    s.cap_right = open_camera(args.right, args.width, args.height)
    if s.cap_left is None or s.cap_right is None:
        print('[ERROR] 摄像头打开失败')
        return

    # 立体匹配
    s.stereo_matcher = cv2.StereoSGBM_create(
        minDisparity=0, numDisparities=64, blockSize=9,
        P1=8*3*9**2, P2=32*3*9**2, disp12MaxDiff=1,
        uniquenessRatio=10, speckleWindowSize=100, speckleRange=32)

    auto_configure_stereo_order()

    # 人脸检测
    s.detector, s.face_detector_name = load_face_detector(
        args.model, args.confidence, (args.width, args.height))
    print(f'[INFO] 已启用纯人脸检测模式 ({s.face_detector_name})')

    # 启动线程
    t = threading.Thread(target=vision_gimbal_thread, daemon=True)
    t.start()
    t_servo = threading.Thread(target=servo_smooth_thread, daemon=True)
    t_servo.start()

    # Web 服务
    server = ThreadingHTTPServer(('0.0.0.0', args.port), GimbalVisionHandler)
    print(f'\n{"="*55}')
    print(f'  双目视觉 + 云台控制 已启动')
    print(f'{"="*55}')
    print(f'  标定: {"✓ " + args.calib if s.rectify_maps else "✗ 未标定"}')
    print(f'  左右目: {"自动交换后匹配标定" if s.swap_stereo else "当前顺序匹配标定"}')
    print(f'  检测: 纯人脸模式 ({s.face_detector_name})')
    print(f'  云台: {"PCA9685 CH" + str(args.pan_channel) + "/" + str(args.tilt_channel) if s.pca else "模拟模式"}')
    print(f'  参数: f={s.focal_length:.1f}px  b={s.baseline*1000:.1f}mm')
    print(f'\n  浏览器打开: http://localhost:{args.port}')
    print(f'  键盘: ← → ↑ ↓ 移动 | P=巡视 T=跟踪 M=手动 S=停止')
    print(f'  按 Ctrl+C 退出\n')

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\n[INFO] 关闭中...')
    finally:
        s.running = False
        apply_angles(0.0, 0.0)
        time.sleep(0.3)
        if s.pca:
            try:
                s.pca.channels[s.pan_channel].duty_cycle = 0
                s.pca.channels[s.tilt_channel].duty_cycle = 0
                if hasattr(s.pca, 'deinit'):
                    s.pca.deinit()
            except Exception:
                pass
        server.server_close()
        if s.cap_left:
            s.cap_left.release()
        if s.cap_right:
            s.cap_right.release()
        print('[INFO] 已退出')


if __name__ == '__main__':
    main()
