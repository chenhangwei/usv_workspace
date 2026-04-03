#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双目视觉检测测试程序

独立运行，不依赖 ROS 2。打开左右摄像头，计算双目深度，
运行目标检测（如有模型），在窗口中实时显示检测结果和深度信息。

用法:
    python3 test_vision.py [--left /dev/video0] [--right /dev/video2] [--model yolov8n.pt]
    python3 test_vision.py --demo    # 无摄像头时用合成图像测试

按 'q' 退出。
"""

import argparse
import time
import cv2
import numpy as np


def parse_args():
    p = argparse.ArgumentParser(description='双目视觉检测测试')
    p.add_argument('--left', default='/dev/video0', help='左目摄像头设备')
    p.add_argument('--right', default='/dev/video2', help='右目摄像头设备')
    p.add_argument('--width', type=int, default=640, help='单目图像宽度')
    p.add_argument('--height', type=int, default=480, help='单目图像高度')
    p.add_argument('--fps', type=int, default=30, help='摄像头帧率')
    p.add_argument('--model', default='', help='检测模型路径 (YOLOv8 .pt 或 .onnx)')
    p.add_argument('--confidence', type=float, default=0.45, help='检测置信度阈值')
    p.add_argument('--focal-length', type=float, default=700.0, help='像素焦距')
    p.add_argument('--baseline', type=float, default=0.025, help='双目基线 (米)')
    p.add_argument('--demo', action='store_true',
                   help='演示模式: 用合成图像代替摄像头 (无需硬件)')
    p.add_argument('--headless', action='store_true',
                   help='无头模式: 保存截图而不弹窗 (SSH远程时使用)')
    p.add_argument('--max-frames', type=int, default=0,
                   help='最大帧数 (0=无限, 配合 --headless 使用)')
    return p.parse_args()


def generate_demo_frames(width, height, frame_count):
    """生成合成的双目测试图像 (带模拟视差)"""
    left = np.zeros((height, width, 3), dtype=np.uint8)
    right = np.zeros((height, width, 3), dtype=np.uint8)

    # 背景 — 模拟水面
    left[:] = (180, 140, 100)   # 淡蓝色水面
    right[:] = (180, 140, 100)

    # 模拟障碍物 (船只)—— 在左右图中有视差偏移
    t = frame_count * 0.02
    # 障碍物1: 近处 (大视差)
    obj1_x_left = int(width * 0.3 + 40 * np.sin(t))
    obj1_x_right = obj1_x_left - 20  # 20px 视差 → 近处
    obj1_y = int(height * 0.4)
    cv2.rectangle(left, (obj1_x_left - 40, obj1_y - 25),
                  (obj1_x_left + 40, obj1_y + 25), (0, 0, 200), -1)
    cv2.rectangle(right, (obj1_x_right - 40, obj1_y - 25),
                  (obj1_x_right + 40, obj1_y + 25), (0, 0, 200), -1)
    cv2.putText(left, 'Boat', (obj1_x_left - 20, obj1_y + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    cv2.putText(right, 'Boat', (obj1_x_right - 20, obj1_y + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    # 障碍物2: 远处 (小视差)
    obj2_x_left = int(width * 0.7 + 30 * np.cos(t * 0.7))
    obj2_x_right = obj2_x_left - 5  # 5px 视差 → 远处
    obj2_y = int(height * 0.35)
    cv2.circle(left, (obj2_x_left, obj2_y), 15, (0, 180, 0), -1)
    cv2.circle(right, (obj2_x_right, obj2_y), 15, (0, 180, 0), -1)

    # 添加网格线方便观察视差
    for x in range(0, width, 80):
        cv2.line(left, (x, 0), (x, height), (160, 130, 90), 1)
        cv2.line(right, (x, 0), (x, height), (160, 130, 90), 1)

    return left, right


def load_detector(model_path, conf_thresh):
    """尝试加载检测模型，失败则返回 None"""
    if not model_path:
        print('[INFO] 未指定模型，仅显示深度图 (无目标检测)')
        return None

    # 尝试 ultralytics
    try:
        from ultralytics import YOLO
        model = YOLO(model_path)
        print(f'[INFO] 已加载 Ultralytics 模型: {model_path}')
        return ('ultralytics', model)
    except ImportError:
        pass
    except Exception as e:
        print(f'[WARN] Ultralytics 加载失败: {e}')

    # 尝试 OpenCV DNN
    try:
        net = cv2.dnn.readNet(model_path)
        print(f'[INFO] 已加载 OpenCV DNN 模型: {model_path}')
        return ('opencv_dnn', net)
    except Exception as e:
        print(f'[WARN] OpenCV DNN 加载失败: {e}')

    print('[WARN] 模型加载失败，仅显示深度图')
    return None


def run_detection(detector, image, conf_thresh):
    """运行目标检测，返回 [(class_name, conf, x1, y1, x2, y2), ...]"""
    if detector is None:
        return []

    backend, model = detector

    if backend == 'ultralytics':
        results = model(image, verbose=False)
        dets = []
        for r in results:
            for box in r.boxes:
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                x1, y1, x2, y2 = xyxy
                conf = float(box.conf[0])
                if conf < conf_thresh:
                    continue
                cls_id = int(box.cls[0])
                cls_name = model.names.get(cls_id, str(cls_id))
                dets.append((cls_name, conf, x1, y1, x2, y2))
        return dets

    elif backend == 'opencv_dnn':
        h, w = image.shape[:2]
        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (640, 640), swapRB=True)
        model.setInput(blob)
        outputs = model.forward(model.getUnconnectedOutLayersNames())
        dets = []
        for output in outputs:
            for row in output[0]:
                scores = row[4:]
                class_id = int(np.argmax(scores))
                conf = float(scores[class_id])
                if conf < conf_thresh:
                    continue
                cx, cy, bw, bh = row[:4]
                x1 = int((cx - bw / 2) * w / 640)
                y1 = int((cy - bh / 2) * h / 640)
                x2 = int((cx + bw / 2) * w / 640)
                y2 = int((cy + bh / 2) * h / 640)
                dets.append((str(class_id), conf, x1, y1, x2, y2))
        return dets

    return []


def main():
    args = parse_args()

    use_demo = args.demo

    if use_demo:
        cap_left = None
        cap_right = None
        print(f'[INFO] 演示模式: 使用合成图像 ({args.width}x{args.height})')
    else:
        # 打开摄像头 (使用 V4L2 后端 + MJPG 编码，匹配 stereo_camera_bringup 配置)
        cap_left = cv2.VideoCapture(args.left, cv2.CAP_V4L2)
        cap_right = cv2.VideoCapture(args.right, cv2.CAP_V4L2)

        if not cap_left.isOpened():
            print(f'[ERROR] 无法打开左目摄像头: {args.left}')
            print('[HINT] 可用 --demo 模式先测试流程')
            return
        if not cap_right.isOpened():
            cap_left.release()
            print(f'[ERROR] 无法打开右目摄像头: {args.right}')
            print('[HINT] 可用 --demo 模式先测试流程')
            return

        # 设置 MJPG 编码 (减少 USB 带宽占用，避免超时)
        fourcc_mjpg = cv2.VideoWriter_fourcc(*'MJPG')
        for cap, name in [(cap_left, '左目'), (cap_right, '右目')]:
            cap.set(cv2.CAP_PROP_FOURCC, fourcc_mjpg)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
            cap.set(cv2.CAP_PROP_FPS, args.fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 最小缓冲，降低延迟
            actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = cap.get(cv2.CAP_PROP_FPS)
            print(f'[INFO] {name}实际参数: {actual_w}x{actual_h} @ {actual_fps:.0f}fps')

    print(f'[INFO] 左目: {args.left}  右目: {args.right}  '
          f'分辨率: {args.width}x{args.height}'
          f'{" (演示模式)" if use_demo else ""}')
    print(f'[INFO] 焦距: {args.focal_length}px  基线: {args.baseline}m')
    print('[INFO] 按 q 退出')

    # 加载检测模型
    detector = load_detector(args.model, args.confidence)

    # 双目匹配器
    stereo = cv2.StereoSGBM_create(
        minDisparity=0, numDisparities=64, blockSize=9,
        P1=8 * 3 * 9 ** 2, P2=32 * 3 * 9 ** 2,
        disp12MaxDiff=1, uniquenessRatio=10,
        speckleWindowSize=100, speckleRange=32,
    )

    fps_history = []
    frame_count = 0

    while True:
        t_start = time.time()

        if use_demo:
            left_img, right_img = generate_demo_frames(
                args.width, args.height, frame_count)
        else:
            ret_l, left_img = cap_left.read()
            ret_r, right_img = cap_right.read()
            if not ret_l or not ret_r:
                print('[WARN] 摄像头读取失败，重试...')
                time.sleep(0.1)
                continue

        # 确保尺寸一致
        if left_img.shape != right_img.shape:
            right_img = cv2.resize(right_img,
                                   (left_img.shape[1], left_img.shape[0]))

        # 双目深度
        gray_l = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
        disparity = stereo.compute(gray_l, gray_r).astype(np.float32) / 16.0
        with np.errstate(divide='ignore', invalid='ignore'):
            depth_map = np.where(
                disparity > 0,
                args.focal_length * args.baseline / disparity,
                0.0).astype(np.float32)

        # 深度图可视化 (伪彩色)
        depth_vis = depth_map.copy()
        depth_vis = np.clip(depth_vis, 0, 10.0)  # 截断到 10m
        depth_norm = (depth_vis / 10.0 * 255).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

        # 目标检测
        detections = run_detection(detector, left_img, args.confidence)

        # 在左图和深度图上绘制检测结果
        display_img = left_img.copy()
        img_center_x = display_img.shape[1] / 2.0
        det_info_lines = []

        for cls_name, conf, x1, y1, x2, y2 in detections:
            # 取 bbox 中心深度
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            roi_y1 = max(0, cy - 10)
            roi_y2 = min(depth_map.shape[0], cy + 10)
            roi_x1 = max(0, cx - 10)
            roi_x2 = min(depth_map.shape[1], cx + 10)
            roi = depth_map[roi_y1:roi_y2, roi_x1:roi_x2]
            valid = roi[roi > 0]
            dist = float(np.median(valid)) if len(valid) > 0 else -1.0

            bearing_deg = np.degrees(np.arctan2(cx - img_center_x,
                                                args.focal_length))

            # 绘制边界框
            color = (0, 255, 0)
            cv2.rectangle(display_img, (x1, y1), (x2, y2), color, 2)
            cv2.rectangle(depth_color, (x1, y1), (x2, y2), (255, 255, 255), 2)

            if dist > 0:
                label = f'{cls_name} {conf:.0%} {dist:.1f}m {bearing_deg:+.0f}deg'
            else:
                label = f'{cls_name} {conf:.0%}'
            cv2.putText(display_img, label, (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            cv2.putText(depth_color, label, (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            det_info_lines.append(
                f'  {cls_name}: {dist:.2f}m, {bearing_deg:+.1f}deg, '
                f'conf={conf:.2f}')

        # FPS 计算
        elapsed = time.time() - t_start
        fps = 1.0 / max(elapsed, 1e-6)
        fps_history.append(fps)
        if len(fps_history) > 30:
            fps_history.pop(0)
        avg_fps = sum(fps_history) / len(fps_history)
        frame_count += 1

        # 绘制 HUD 信息到左图
        info_lines = [
            f'FPS: {avg_fps:.1f}  Frame: {frame_count}',
            f'Left: {args.left}  Right: {args.right}',
            f'Focal: {args.focal_length}  Baseline: {args.baseline*1000:.0f}mm',
            f'Detections: {len(detections)}',
        ]
        y_offset = 20
        for line in info_lines:
            cv2.putText(display_img, line, (10, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            y_offset += 18

        # 在深度图上标注中心点距离
        center_y, center_x = depth_map.shape[0] // 2, depth_map.shape[1] // 2
        center_depth = depth_map[center_y, center_x]
        cv2.drawMarker(depth_color, (center_x, center_y),
                       (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
        cv2.putText(depth_color, f'Center: {center_depth:.2f}m',
                    (center_x + 15, center_y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        # 缩小右目图像用于参考
        right_small = cv2.resize(right_img, (args.width // 3, args.height // 3))
        # 在右目小图上加标签
        cv2.putText(right_small, 'Right', (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        # 拼接显示: 上排 = 左目检测 + 深度图, 下方叠加右目小图
        top_row = np.hstack([display_img, depth_color])
        # 把右目小图贴到左下角
        rh, rw = right_small.shape[:2]
        top_row[top_row.shape[0] - rh:, :rw] = right_small

        # 缩放到合适窗口大小
        scale = min(1280 / top_row.shape[1], 720 / top_row.shape[0], 1.0)
        if scale < 1.0:
            show = cv2.resize(top_row, None, fx=scale, fy=scale)
        else:
            show = top_row

        cv2.imshow('Stereo Vision Test', show)

        # 终端输出
        if frame_count % 30 == 0:
            print(f'[Frame {frame_count}] FPS={avg_fps:.1f} '
                  f'Detections={len(detections)} '
                  f'CenterDepth={center_depth:.2f}m')
            for line in det_info_lines:
                print(line)

        # --headless 模式: 保存截图
        if args.headless and frame_count in (1, 5, 30):
            out_path = f'/tmp/vision_test_frame_{frame_count}.png'
            cv2.imwrite(out_path, show)
            print(f'[SAVE] 截图已保存: {out_path}')

        # 退出条件
        if args.max_frames > 0 and frame_count >= args.max_frames:
            print(f'[INFO] 已达最大帧数 {args.max_frames}，退出')
            break

        if not args.headless:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
        else:
            # headless 模式下不等待按键，控制帧率
            time.sleep(0.03)

    if cap_left is not None:
        cap_left.release()
    if cap_right is not None:
        cap_right.release()
    cv2.destroyAllWindows()
    print('[INFO] 测试结束')


if __name__ == '__main__':
    main()
