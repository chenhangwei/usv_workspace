#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双目视觉检测 Web 测试工具

使用标定参数校正图像，运行 YOLOv8 目标检测 + 双目测距，
在浏览器中实时显示检测框、距离、深度图。

========== 启动服务 ==========

  # 基本启动（进入 usv_drivers 目录）
  cd /home/chenhangwei/usv_workspace/src/usv_drivers
  CUDA_VISIBLE_DEVICES="" python3 test/test_vision_web.py --calib ./stereo_calibration.yaml --port 8766

  # 后台启动
  CUDA_VISIBLE_DEVICES="" python3 test/test_vision_web.py --calib ./stereo_calibration.yaml --port 8766 &

  # 启动后浏览器打开
  http://localhost:8766

========== 关闭服务 ==========

  # 方式1: 前台运行时直接按
  Ctrl+C

  # 方式2: 后台运行时通过端口号关闭
  kill $(lsof -t -i:8766)

  # 方式3: 通过进程名关闭
  pkill -f test_vision_web.py

========== 可选参数 ==========

  --left /dev/video0        左摄像头设备 (默认 /dev/video0)
  --right /dev/video2       右摄像头设备 (默认 /dev/video2)
  --width 640               图像宽度 (默认 640)
  --height 480              图像高度 (默认 480)
  --calib PATH              标定文件路径 (默认 ./stereo_calibration.yaml)
  --model PATH              YOLO 模型路径 (默认自动下载 yolov8n.pt)
  --confidence 0.45         检测置信度阈值 (默认 0.45)
  --port 8766               Web 服务端口 (默认 8766)

========== 注意事项 ==========

  - 必须设置 CUDA_VISIBLE_DEVICES="" 防止 GPU 不兼容导致崩溃
  - 启动前确保没有其他程序占用摄像头 (如 calibrate_stereo_web.py)
  - 查看摄像头占用: fuser /dev/video0 /dev/video2
  - 释放摄像头: kill $(fuser /dev/video0 2>/dev/null)
"""

import argparse
import glob
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import cv2
import numpy as np
import yaml


# ======================================================================
# 全局状态
# ======================================================================
class AppState:
    def __init__(self):
        self.cap_left = None
        self.cap_right = None
        self.frame_w = 640
        self.frame_h = 480

        # 标定参数
        self.calib_file = ''
        self.rectify_maps = None
        self.focal_length = 328.5
        self.baseline = 0.024
        self.cx = 320.0
        self.cy = 240.0

        # 检测
        self.detector = None
        self.conf_thresh = 0.45

        # 立体匹配
        self.stereo_matcher = None

        # 运行时状态
        self.lock = threading.Lock()
        self.latest_frame = None       # 主视图 JPEG
        self.latest_depth = None       # 深度图 JPEG
        self.running = True
        self.fps = 0.0
        self.detections = []           # 当前检测结果
        self.frame_count = 0
        self.show_depth = True
        self.show_rectified = False

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
    """加载标定文件"""
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
        K_l = np.array(calib['K_left'])
        dist_l = np.array(calib['dist_left'])
        K_r = np.array(calib['K_right'])
        dist_r = np.array(calib['dist_right'])
        R = np.array(calib['R'])
        T = np.array(calib['T'])
        R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(
            K_l, dist_l, K_r, dist_r, img_size, R, T, alpha=0)
        m1l, m2l = cv2.initUndistortRectifyMap(
            K_l, dist_l, R1, P1, img_size, cv2.CV_32FC1)
        m1r, m2r = cv2.initUndistortRectifyMap(
            K_r, dist_r, R2, P2, img_size, cv2.CV_32FC1)
        s.rectify_maps = (m1l, m2l, m1r, m2r)

    print(f'[INFO] 标定参数: focal={s.focal_length:.1f}px '
          f'baseline={s.baseline*1000:.1f}mm '
          f'RMS={calib["stereo_rms_error"]:.4f}px')


def load_detector(model_path, conf_thresh):
    """加载检测模型"""
    if not model_path:
        # 尝试默认路径
        for default in ['yolov8n.pt', 'yolov8n.onnx',
                         os.path.expanduser('~/yolov8n.pt')]:
            if os.path.exists(default):
                model_path = default
                break

    if not model_path:
        print('[INFO] 未找到模型文件，尝试自动下载 yolov8n.pt ...')
        try:
            import torch
            torch.set_default_device('cpu')
            os.environ['CUDA_VISIBLE_DEVICES'] = ''
            from ultralytics import YOLO
            model = YOLO('yolov8n.pt')
            model.to('cpu')
            print('[INFO] 已下载并加载 yolov8n.pt (CPU)')
            return ('ultralytics', model)
        except Exception as e:
            print(f'[WARN] 模型下载失败: {e}，将运行无检测模式')
            return None

    try:
        import torch
        torch.set_default_device('cpu')
        os.environ['CUDA_VISIBLE_DEVICES'] = ''
        from ultralytics import YOLO
        model = YOLO(model_path)
        model.to('cpu')
        print(f'[INFO] 已加载模型: {model_path} (CPU)')
        return ('ultralytics', model)
    except Exception as e:
        print(f'[WARN] 模型加载失败: {e}')

    try:
        net = cv2.dnn.readNet(model_path)
        print(f'[INFO] 已加载 OpenCV DNN: {model_path}')
        return ('opencv_dnn', net)
    except Exception:
        pass

    print('[WARN] 所有检测后端失败，无检测模式')
    return None


def run_detection(detector, image, conf_thresh):
    """目标检测"""
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
# 主处理线程
# ======================================================================
def vision_thread():
    """后台线程：采集 → 校正 → 检测 → 测距 → 渲染"""
    s = STATE
    fps_counter = 0
    fps_time = time.time()

    while s.running:
        if s.cap_left is None or s.cap_right is None:
            time.sleep(0.1)
            continue

        ret_l, left = s.cap_left.read()
        ret_r, right = s.cap_right.read()
        if not ret_l or not ret_r:
            time.sleep(0.05)
            continue

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
        with np.errstate(divide='ignore', invalid='ignore'):
            depth_map = np.where(
                disparity > 0,
                s.focal_length * s.baseline / disparity,
                0.0).astype(np.float32)

        # 目标检测（每 2 帧一次减轻负担）
        s.frame_count += 1
        if s.frame_count % 2 == 0:
            dets = run_detection(s.detector, left, s.conf_thresh)
            # 为每个检测加上距离和方位
            enriched = []
            for cls_name, conf, x1, y1, x2, y2 in dets:
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                roi_y1 = max(0, cy - 15)
                roi_y2 = min(depth_map.shape[0], cy + 15)
                roi_x1 = max(0, cx - 15)
                roi_x2 = min(depth_map.shape[1], cx + 15)
                roi = depth_map[roi_y1:roi_y2, roi_x1:roi_x2]
                valid = roi[(roi > 0.05) & (roi < 20.0)]
                dist = float(np.median(valid)) if len(valid) > 0 else -1.0
                bearing_deg = float(np.degrees(
                    np.arctan2(cx - s.cx, s.focal_length)))
                enriched.append({
                    'cls': cls_name, 'conf': conf,
                    'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                    'dist': dist, 'bearing': bearing_deg
                })
            with s.lock:
                s.detections = enriched

        # ---- 渲染主画面 ----
        disp = left.copy()

        with s.lock:
            dets_draw = list(s.detections)

        for d in dets_draw:
            x1, y1, x2, y2 = d['x1'], d['y1'], d['x2'], d['y2']
            dist = d['dist']
            # 颜色：近=红，中=黄，远=绿
            if dist < 0:
                color = (128, 128, 128)
            elif dist < 1.0:
                color = (0, 0, 255)
            elif dist < 3.0:
                color = (0, 180, 255)
            else:
                color = (0, 220, 0)

            cv2.rectangle(disp, (x1, y1), (x2, y2), color, 2)

            # 标签背景
            label = f'{d["cls"]} {d["conf"]:.0%}'
            if dist > 0:
                label += f' {dist:.2f}m'
            (tw, th), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(disp, (x1, y1 - th - 8), (x1 + tw + 4, y1),
                          color, -1)
            cv2.putText(disp, label, (x1 + 2, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # 距离/方位信息
            if dist > 0:
                info = f'{dist:.2f}m / {d["bearing"]:+.1f}deg'
                cv2.putText(disp, info, (x1, y2 + 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        # HUD 信息栏
        fps_counter += 1
        now = time.time()
        if now - fps_time >= 1.0:
            s.fps = fps_counter / (now - fps_time)
            fps_counter = 0
            fps_time = now

        hud_h = 30
        overlay = disp[:hud_h, :].copy()
        cv2.rectangle(disp, (0, 0), (disp.shape[1], hud_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.3, disp[:hud_h, :], 0.7, 0, disp[:hud_h, :])
        calib_tag = 'CALIBRATED' if s.rectify_maps else 'UNCALIBRATED'
        det_tag = f'{len(dets_draw)} objects' if dets_draw else 'no detection'
        model_tag = 'YOLO' if s.detector else 'NO MODEL'
        hud_text = (f'FPS: {s.fps:.1f} | {calib_tag} | '
                    f'f={s.focal_length:.0f} b={s.baseline*1000:.0f}mm | '
                    f'{model_tag} | {det_tag}')
        cv2.putText(disp, hud_text, (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        # 编码主画面
        _, buf = cv2.imencode('.jpg', disp, [cv2.IMWRITE_JPEG_QUALITY, 80])
        with s.lock:
            s.latest_frame = buf.tobytes()

        # ---- 渲染深度图 ----
        # 归一化到可视范围 (0.1m ~ 5m)
        depth_vis = np.clip(depth_map, 0.1, 5.0)
        depth_vis = ((depth_vis - 0.1) / 4.9 * 255).astype(np.uint8)
        depth_color = cv2.applyColorMap(255 - depth_vis, cv2.COLORMAP_JET)
        # 无效区域设为黑色
        depth_color[depth_map <= 0] = 0

        # 在深度图上也画检测框
        for d in dets_draw:
            x1, y1, x2, y2 = d['x1'], d['y1'], d['x2'], d['y2']
            cv2.rectangle(depth_color, (x1, y1), (x2, y2), (255, 255, 255), 1)
            if d['dist'] > 0:
                cv2.putText(depth_color, f'{d["dist"]:.2f}m',
                            (x1, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX,
                            0.4, (255, 255, 255), 1)

        # 深度色条图例
        bar_x = depth_color.shape[1] - 30
        for y in range(50, depth_color.shape[0] - 50):
            ratio = (y - 50) / (depth_color.shape[0] - 100)
            val = int(255 * (1 - ratio))
            c = cv2.applyColorMap(np.array([[val]], dtype=np.uint8),
                                  cv2.COLORMAP_JET)[0][0]
            cv2.line(depth_color, (bar_x, y), (bar_x + 20, y),
                     c.tolist(), 1)
        cv2.putText(depth_color, '0.1m', (bar_x - 10, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
        cv2.putText(depth_color, '5.0m',
                    (bar_x - 10, depth_color.shape[0] - 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)

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
<title>双目视觉测试</title>
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body {
    font-family: 'Segoe UI', Arial, sans-serif;
    background: #111; color: #eee;
  }
  .header {
    background: #1a1a2e; padding: 8px 16px;
    display: flex; align-items: center; gap: 15px;
    border-bottom: 2px solid #e94560;
  }
  .header h1 { font-size: 16px; color: #e94560; }
  .header .tag {
    font-size: 12px; padding: 2px 8px; border-radius: 10px;
    background: #0f3460; color: #aaa;
  }
  .header .tag.ok { background: #00c853; color: #111; }
  .main {
    display: flex; flex-wrap: wrap; gap: 4px; padding: 4px;
    justify-content: center;
  }
  .panel {
    position: relative; background: #222; border-radius: 6px;
    overflow: hidden; border: 1px solid #333;
  }
  .panel img {
    display: block; width: 100%; height: auto;
  }
  .panel .label {
    position: absolute; top: 4px; left: 6px;
    font-size: 11px; background: rgba(0,0,0,0.7);
    padding: 2px 8px; border-radius: 4px; color: #0f0;
  }
  .sidebar {
    position: fixed; right: 0; top: 48px; width: 290px;
    background: #1a1a2e; height: calc(100vh - 48px);
    padding: 12px; overflow-y: auto;
    border-left: 2px solid #0f3460;
  }
  .sidebar h3 {
    font-size: 13px; color: #e94560; margin-bottom: 8px;
  }
  .det-list { list-style: none; }
  .det-item {
    background: #0f3460; border-radius: 6px; padding: 8px;
    margin-bottom: 6px; font-size: 12px;
  }
  .det-item .name { font-weight: 600; font-size: 14px; }
  .det-item .dist {
    font-size: 20px; font-weight: 700; color: #00e676;
  }
  .det-item .dist.near { color: #ff5252; }
  .det-item .dist.mid { color: #ffab40; }
  .det-item .info { color: #aaa; margin-top: 4px; }
  .no-det { color: #666; font-style: italic; padding: 20px 0; }
  .stats {
    margin-top: 12px; padding-top: 10px;
    border-top: 1px solid #333; font-size: 12px; color: #888;
  }
  .stats div { margin-bottom: 4px; }
  .content-area {
    margin-right: 294px;
  }
  @media (max-width: 900px) {
    .sidebar { position: static; width: 100%; height: auto; }
    .content-area { margin-right: 0; }
  }
</style>
</head>
<body>
<div class="header">
  <h1>&#128065; 双目视觉检测</h1>
  <span class="tag" id="tag-calib">标定: --</span>
  <span class="tag" id="tag-model">模型: --</span>
  <span class="tag" id="tag-fps">FPS: --</span>
  <span class="tag" id="tag-objects">目标: 0</span>
</div>

<div class="content-area">
  <div class="main">
    <div class="panel" style="flex:1; min-width:400px; max-width:700px;">
      <img id="stream-main" src="/stream/main">
      <div class="label">检测画面 (校正后)</div>
    </div>
    <div class="panel" style="flex:1; min-width:400px; max-width:700px;">
      <img id="stream-depth" src="/stream/depth">
      <div class="label" style="color:#ff0;">深度图 (近=红 远=蓝)</div>
    </div>
  </div>
</div>

<div class="sidebar">
  <h3>&#127919; 检测目标</h3>
  <ul class="det-list" id="det-list">
    <li class="no-det">等待检测结果...</li>
  </ul>
  <div class="stats" id="stats">
    <div>焦距: --</div>
    <div>基线: --</div>
    <div>帧数: --</div>
  </div>
</div>

<script>
function pollStatus() {
  fetch('/api/status').then(r => r.json()).then(d => {
    // 顶栏标签
    const tc = document.getElementById('tag-calib');
    tc.textContent = '标定: ' + (d.calibrated ? '✓' : '✗');
    tc.className = 'tag' + (d.calibrated ? ' ok' : '');

    const tm = document.getElementById('tag-model');
    tm.textContent = '模型: ' + d.model;
    tm.className = 'tag' + (d.model !== 'NONE' ? ' ok' : '');

    document.getElementById('tag-fps').textContent =
      'FPS: ' + d.fps.toFixed(1);
    document.getElementById('tag-objects').textContent =
      '目标: ' + d.detections.length;

    // 检测列表
    const list = document.getElementById('det-list');
    if (d.detections.length === 0) {
      list.innerHTML = '<li class="no-det">未检测到目标</li>';
    } else {
      list.innerHTML = '';
      d.detections.forEach(det => {
        const li = document.createElement('li');
        li.className = 'det-item';
        const distClass = det.dist < 1 ? 'near' : det.dist < 3 ? 'mid' : '';
        const distText = det.dist > 0
          ? det.dist.toFixed(2) + ' m'
          : '--';
        li.innerHTML =
          '<span class="name">' + det.cls + '</span> '
          + '<span style="color:#aaa">' + (det.conf * 100).toFixed(0) + '%</span><br>'
          + '<span class="dist ' + distClass + '">' + distText + '</span><br>'
          + '<div class="info">'
          + '方位: ' + det.bearing.toFixed(1) + '°  '
          + 'bbox: ' + det.x1 + ',' + det.y1 + ' ~ ' + det.x2 + ',' + det.y2
          + '</div>';
        list.appendChild(li);
      });
    }

    // 统计信息
    document.getElementById('stats').innerHTML =
      '<div>焦距: ' + d.focal.toFixed(1) + ' px</div>'
      + '<div>基线: ' + (d.baseline * 1000).toFixed(1) + ' mm</div>'
      + '<div>帧数: ' + d.frame_count + '</div>'
      + '<div>分辨率: ' + d.width + 'x' + d.height + '</div>';
  }).catch(() => {});
}

setInterval(pollStatus, 500);
</script>
</body></html>
'''


# ======================================================================
# HTTP 服务器
# ======================================================================
class VisionHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == '/' or path == '/index.html':
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
                'model': 'YOLO' if s.detector else 'NONE',
                'focal': s.focal_length,
                'baseline': s.baseline,
                'width': s.frame_w,
                'height': s.frame_h,
                'detections': dets,
            }
            body = json.dumps(data, ensure_ascii=False).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        else:
            self.send_response(404)
            self.end_headers()

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
    p = argparse.ArgumentParser(description='双目视觉检测 Web 测试')
    p.add_argument('--left', default='/dev/video0')
    p.add_argument('--right', default='/dev/video2')
    p.add_argument('--width', type=int, default=640)
    p.add_argument('--height', type=int, default=480)
    p.add_argument('--calib', default='./stereo_calibration.yaml',
                   help='标定文件路径')
    p.add_argument('--model', default='', help='检测模型路径')
    p.add_argument('--confidence', type=float, default=0.45)
    p.add_argument('--port', type=int, default=8766)
    args = p.parse_args()

    s = STATE
    s.frame_w = args.width
    s.frame_h = args.height
    s.conf_thresh = args.confidence

    # 标定
    load_calibration(args.calib)

    # 摄像头
    s.cap_left = open_camera(args.left, args.width, args.height)
    s.cap_right = open_camera(args.right, args.width, args.height)
    if s.cap_left is None or s.cap_right is None:
        print('[ERROR] 摄像头打开失败')
        return

    # 立体匹配
    s.stereo_matcher = cv2.StereoSGBM_create(
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

    # 检测模型
    print('[INFO] 加载检测模型...')
    s.detector = load_detector(args.model, args.confidence)

    # 启动处理线程
    t = threading.Thread(target=vision_thread, daemon=True)
    t.start()

    # Web 服务
    server = ThreadingHTTPServer(('0.0.0.0', args.port), VisionHandler)
    print(f'\n{"="*50}')
    print(f'  双目视觉检测已启动')
    print(f'{"="*50}')
    print(f'  标定: {"✓ " + args.calib if s.rectify_maps else "✗ 未标定"}')
    print(f'  模型: {"✓" if s.detector else "✗ 无检测模型"}')
    print(f'  参数: f={s.focal_length:.1f}px  '
          f'b={s.baseline*1000:.1f}mm')
    print(f'\n  浏览器打开: http://localhost:{args.port}')
    print(f'  按 Ctrl+C 退出\n')

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\n[INFO] 关闭中...')
    finally:
        s.running = False
        server.server_close()
        if s.cap_left:
            s.cap_left.release()
        if s.cap_right:
            s.cap_right.release()


if __name__ == '__main__':
    main()
