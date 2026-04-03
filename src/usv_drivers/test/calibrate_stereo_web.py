#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双目摄像头标定工具 — Web 可视化版

在浏览器中实时预览摄像头画面和棋盘格检测状态，
点击按钮采集图像，完成后一键标定。

用法:
  python3 calibrate_stereo_web.py --square-size 0.030
  然后在浏览器打开 http://<IP>:8765

功能:
  - 实时 MJPEG 视频流 (左右目 + 角点检测叠加)
  - 绿色/红色状态指示 (是否检测到棋盘格)
  - 点击 [采集] 按钮拍照
  - 点击 [开始标定] 执行标定计算
  - 点击 [验证] 查看校正效果
  - 已采集图像缩略图回顾
  - 删除单张不满意的图像
"""

import argparse
import glob
import io
import json
import os
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler, ThreadingHTTPServer
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
        self.board_size = (8, 5)
        self.square_size = 0.030
        self.image_dir = './calib_images'
        self.output = './stereo_calibration.yaml'
        self.frame_w = 640
        self.frame_h = 480

        # 运行时状态
        self.lock = threading.Lock()
        self.latest_frame = None      # JPEG bytes (合并画面)
        self.found_left = False
        self.found_right = False
        self.count = 0
        self.running = True
        self.mode = 'collect'         # collect / verify
        self.calib_result = None      # 标定结果文本
        self.last_capture_time = 0
        self.message = ''             # 临时提示
        self.message_time = 0

        # 验证模式的映射表
        self.verify_maps = None
        self.verify_info = {}

STATE = AppState()


# ======================================================================
# 摄像头 + 检测线程
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


def camera_thread():
    """后台线程：持续采集并检测角点，生成预览帧"""
    s = STATE
    while s.running:
        if s.cap_left is None or s.cap_right is None:
            time.sleep(0.1)
            continue

        ret_l, left = s.cap_left.read()
        ret_r, right = s.cap_right.read()
        if not ret_l or not ret_r:
            time.sleep(0.05)
            continue

        if s.mode == 'verify' and s.verify_maps is not None:
            # 验证模式：显示校正后画面 + 水平线
            m1l, m2l, m1r, m2r = s.verify_maps
            rect_l = cv2.remap(left, m1l, m2l, cv2.INTER_LINEAR)
            rect_r = cv2.remap(right, m1r, m2r, cv2.INTER_LINEAR)
            combined = np.hstack([rect_l, rect_r])
            h = combined.shape[0]
            for y in range(0, h, 32):
                cv2.line(combined, (0, y), (combined.shape[1], y),
                         (0, 255, 0), 1)
            info = s.verify_info
            cv2.putText(combined,
                        f'[VERIFY] Focal={info.get("f", 0):.0f}px  '
                        f'Baseline={info.get("b", 0)*1000:.1f}mm  '
                        f'RMS={info.get("rms", 0):.4f}px',
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 255, 255), 2)
            cv2.putText(combined,
                        'Green lines should pass through same object on both sides',
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (200, 200, 200), 1)
        else:
            # 采集模式：检测角点
            gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
            gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
            # CLAHE 均衡化，对抗反光和光照不均
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            gray_l_eq = clahe.apply(gray_l)
            gray_r_eq = clahe.apply(gray_r)

            cb_flags = (cv2.CALIB_CB_ADAPTIVE_THRESH
                        | cv2.CALIB_CB_NORMALIZE_IMAGE
                        | cv2.CALIB_CB_FILTER_QUADS)
            found_l, corners_l = cv2.findChessboardCorners(
                gray_l_eq, s.board_size, cb_flags)
            found_r, corners_r = cv2.findChessboardCorners(
                gray_r_eq, s.board_size, cb_flags)

            with s.lock:
                s.found_left = found_l
                s.found_right = found_r

            disp_l = left.copy()
            disp_r = right.copy()
            if found_l:
                cv2.drawChessboardCorners(disp_l, s.board_size,
                                          corners_l, found_l)
            if found_r:
                cv2.drawChessboardCorners(disp_r, s.board_size,
                                          corners_r, found_r)

            # 左目状态栏
            both = found_l and found_r
            color_l = (0, 200, 0) if found_l else (0, 0, 220)
            color_r = (0, 200, 0) if found_r else (0, 0, 220)
            # 半透明状态条
            overlay_l = disp_l.copy()
            cv2.rectangle(overlay_l, (0, 0), (disp_l.shape[1], 65),
                          (40, 40, 40), -1)
            cv2.addWeighted(overlay_l, 0.6, disp_l, 0.4, 0, disp_l)

            overlay_r = disp_r.copy()
            cv2.rectangle(overlay_r, (0, 0), (disp_r.shape[1], 65),
                          (40, 40, 40), -1)
            cv2.addWeighted(overlay_r, 0.6, disp_r, 0.4, 0, disp_r)

            # 状态圆点 + 文字
            cv2.circle(disp_l, (20, 22), 10, color_l, -1)
            cv2.putText(disp_l,
                        f'LEFT  {"DETECTED" if found_l else "NOT FOUND"}',
                        (38, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 2)
            cv2.putText(disp_l, f'Saved: {s.count} pairs',
                        (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (200, 200, 200), 1)

            cv2.circle(disp_r, (20, 22), 10, color_r, -1)
            cv2.putText(disp_r,
                        f'RIGHT  {"DETECTED" if found_r else "NOT FOUND"}',
                        (38, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 2)
            if both:
                cv2.putText(disp_r, 'READY TO CAPTURE',
                            (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (0, 255, 0), 2)

            # 外框颜色
            border = (0, 200, 0) if both else (0, 0, 180)
            cv2.rectangle(disp_l, (0, 0),
                          (disp_l.shape[1]-1, disp_l.shape[0]-1), border, 3)
            cv2.rectangle(disp_r, (0, 0),
                          (disp_r.shape[1]-1, disp_r.shape[0]-1), border, 3)

            # 消息提示
            now = time.time()
            if s.message and now - s.message_time < 2.0:
                cv2.putText(disp_l, s.message,
                            (10, disp_l.shape[0] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 255, 255), 2)

            combined = np.hstack([disp_l, disp_r])

        # 缩放到适合浏览器的宽度
        scale = min(1280 / combined.shape[1], 1.0)
        if scale < 1.0:
            combined = cv2.resize(combined, None, fx=scale, fy=scale)

        # 编码为 JPEG
        _, buf = cv2.imencode('.jpg', combined,
                              [cv2.IMWRITE_JPEG_QUALITY, 75])
        with s.lock:
            s.latest_frame = buf.tobytes()

        time.sleep(0.03)  # ~30fps


def do_capture():
    """采集当前帧"""
    s = STATE
    if s.cap_left is None or s.cap_right is None:
        return '摄像头未打开'

    ret_l, left = s.cap_left.read()
    ret_r, right = s.cap_right.read()
    if not ret_l or not ret_r:
        return '采集失败：读取摄像头出错'

    # 验证角点
    gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray_l_eq = clahe.apply(gray_l)
    gray_r_eq = clahe.apply(gray_r)
    cb_flags = (cv2.CALIB_CB_ADAPTIVE_THRESH
                | cv2.CALIB_CB_NORMALIZE_IMAGE
                | cv2.CALIB_CB_FILTER_QUADS)
    found_l, _ = cv2.findChessboardCorners(
        gray_l_eq, s.board_size, cb_flags)
    found_r, _ = cv2.findChessboardCorners(
        gray_r_eq, s.board_size, cb_flags)

    if not (found_l and found_r):
        return '采集失败：未同时检测到左右角点'

    os.makedirs(s.image_dir, exist_ok=True)
    path_l = os.path.join(s.image_dir, f'left_{s.count:03d}.png')
    path_r = os.path.join(s.image_dir, f'right_{s.count:03d}.png')
    cv2.imwrite(path_l, left)
    cv2.imwrite(path_r, right)
    s.count += 1
    s.message = f'Captured #{s.count}'
    s.message_time = time.time()
    print(f'[{s.count}] 已保存: {path_l}, {path_r}')
    return f'已采集第 {s.count} 对图像'


def do_delete(index):
    """删除指定编号的图像对"""
    s = STATE
    path_l = os.path.join(s.image_dir, f'left_{index:03d}.png')
    path_r = os.path.join(s.image_dir, f'right_{index:03d}.png')
    deleted = False
    if os.path.exists(path_l):
        os.remove(path_l)
        deleted = True
    if os.path.exists(path_r):
        os.remove(path_r)
        deleted = True
    # 重新计数
    s.count = len(glob.glob(os.path.join(s.image_dir, 'left_*.png')))
    if deleted:
        return f'已删除第 {index} 对图像，剩余 {s.count} 对'
    return f'第 {index} 对图像不存在'


def do_calibrate():
    """执行标定（带自动剔除坏图）"""
    s = STATE
    board_size = s.board_size
    square_size = s.square_size

    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0],
                            0:board_size[1]].T.reshape(-1, 2)
    objp *= square_size

    obj_points = []
    img_points_l = []
    img_points_r = []
    img_names = []
    img_size = None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    left_files = sorted(glob.glob(os.path.join(s.image_dir, 'left_*.png')))
    right_files = sorted(glob.glob(os.path.join(s.image_dir, 'right_*.png')))

    if len(left_files) == 0:
        return '错误：未找到标定图像'
    if len(left_files) != len(right_files):
        return f'错误：左右图像数量不匹配 L={len(left_files)} R={len(right_files)}'

    log = [f'图像对数: {len(left_files)}',
           f'棋盘格: {board_size[0]}x{board_size[1]}, 每格 {square_size*1000:.0f}mm',
           '']

    for i, (lf, rf) in enumerate(zip(left_files, right_files)):
        img_l = cv2.imread(lf)
        img_r = cv2.imread(rf)
        if img_l is None or img_r is None:
            log.append(f'  [{i}] 读取失败')
            continue

        gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)
        if img_size is None:
            img_size = (gray_l.shape[1], gray_l.shape[0])

        cb_flags = (cv2.CALIB_CB_ADAPTIVE_THRESH
                    | cv2.CALIB_CB_NORMALIZE_IMAGE
                    | cv2.CALIB_CB_FILTER_QUADS)
        found_l, corners_l = cv2.findChessboardCorners(
            gray_l, board_size, cb_flags)
        found_r, corners_r = cv2.findChessboardCorners(
            gray_r, board_size, cb_flags)

        if found_l and found_r:
            corners_l = cv2.cornerSubPix(
                gray_l, corners_l, (11, 11), (-1, -1), criteria)
            corners_r = cv2.cornerSubPix(
                gray_r, corners_r, (11, 11), (-1, -1), criteria)
            obj_points.append(objp)
            img_points_l.append(corners_l)
            img_points_r.append(corners_r)
            img_names.append(os.path.basename(lf))
            log.append(f'  [{i}] {os.path.basename(lf)} ✓')
        else:
            log.append(f'  [{i}] {os.path.basename(lf)} ✗ '
                       f'(L={found_l} R={found_r})')

    valid = len(obj_points)
    log.append(f'\n有效: {valid}/{len(left_files)}')
    if valid < 3:
        return '\n'.join(log) + '\n\n有效图像不足 3 对，无法标定'

    # ================================================================
    # 迭代标定 + 自动剔除坏图
    # ================================================================
    max_rounds = 3
    for round_num in range(max_rounds):
        log.append(f'\n[轮次 {round_num+1}] 使用 {len(obj_points)} 对图像...')

        # 单目标定
        ret_l, K_l, dist_l, rvecs_l, tvecs_l = cv2.calibrateCamera(
            obj_points, img_points_l, img_size, None, None,
            flags=cv2.CALIB_FIX_K3)
        ret_r, K_r, dist_r, rvecs_r, tvecs_r = cv2.calibrateCamera(
            obj_points, img_points_r, img_size, None, None,
            flags=cv2.CALIB_FIX_K3)
        log.append(f'  左目 RMS: {ret_l:.4f}  右目 RMS: {ret_r:.4f}')

        # 计算每张图的重投影误差
        per_image_err = []
        for j in range(len(obj_points)):
            proj_l, _ = cv2.projectPoints(
                obj_points[j], rvecs_l[j], tvecs_l[j], K_l, dist_l)
            err_l = cv2.norm(
                img_points_l[j], proj_l, cv2.NORM_L2) / len(proj_l)
            proj_r, _ = cv2.projectPoints(
                obj_points[j], rvecs_r[j], tvecs_r[j], K_r, dist_r)
            err_r = cv2.norm(
                img_points_r[j], proj_r, cv2.NORM_L2) / len(proj_r)
            per_image_err.append(max(err_l, err_r))

        # 剔除误差大于 2 倍均值的图像
        mean_err = np.mean(per_image_err)
        threshold = max(mean_err * 2.0, 1.5)
        bad_indices = [j for j, e in enumerate(per_image_err) if e > threshold]

        if not bad_indices or len(obj_points) - len(bad_indices) < 3:
            # 没有坏图或剔除后不足 3 张，停止迭代
            if bad_indices:
                log.append(f'  发现 {len(bad_indices)} 张坏图，'
                           f'但剔除后不足 3 张，保留全部')
            else:
                log.append(f'  所有图像误差合理 (均值={mean_err:.3f}px)')
            break

        # 剔除坏图
        for j in sorted(bad_indices, reverse=True):
            log.append(f'  剔除: {img_names[j]} (误差={per_image_err[j]:.3f}px)')
            del obj_points[j]
            del img_points_l[j]
            del img_points_r[j]
            del img_names[j]

        log.append(f'  剩余 {len(obj_points)} 对')

    # ================================================================
    # 最终双目标定
    # ================================================================
    log.append(f'\n[双目标定] 使用 {len(obj_points)} 对图像...')

    # 先不固定内参做一次联合优化
    ret_stereo, K_l, dist_l, K_r, dist_r, R, T, E, F = \
        cv2.stereoCalibrate(
            obj_points, img_points_l, img_points_r,
            K_l, dist_l, K_r, dist_r, img_size,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                      200, 1e-7),
            flags=cv2.CALIB_FIX_K3)
    baseline = np.linalg.norm(T)
    log.append(f'  双目 RMS: {ret_stereo:.4f} px')
    log.append(f'  基线距离: {baseline * 1000:.1f} mm')

    # 合理性校验
    warnings = []
    w, h = img_size
    fx_l, fy_l = K_l[0, 0], K_l[1, 1]
    cx_l, cy_l = K_l[0, 2], K_l[1, 2]
    if not (100 < fx_l < 2000):
        warnings.append(f'⚠ 左目焦距 fx={fx_l:.1f} 异常 (期望 200~1200)')
    if not (w * 0.2 < cx_l < w * 0.8):
        warnings.append(f'⚠ 左目光心 cx={cx_l:.1f} 偏离中心 '
                        f'(期望 {w*0.3:.0f}~{w*0.7:.0f})')
    if not (5 < baseline * 1000 < 100):
        warnings.append(f'⚠ 基线 {baseline*1000:.1f}mm 异常 (期望 15~50mm)')
    if warnings:
        log.append('\n合理性警告:')
        for w_msg in warnings:
            log.append(f'  {w_msg}')

    # 立体校正
    log.append('\n[计算校正映射]')
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        K_l, dist_l, K_r, dist_r, img_size, R, T,
        alpha=0, flags=cv2.CALIB_ZERO_DISPARITY)
    map1_l, map2_l = cv2.initUndistortRectifyMap(
        K_l, dist_l, R1, P1, img_size, cv2.CV_32FC1)
    map1_r, map2_r = cv2.initUndistortRectifyMap(
        K_r, dist_r, R2, P2, img_size, cv2.CV_32FC1)

    focal_length = P1[0, 0]
    cx, cy = P1[0, 2], P1[1, 2]

    # 保存
    calib_data = {
        'image_size': list(img_size),
        'stereo_rms_error': float(ret_stereo),
        'baseline_m': float(baseline),
        'focal_length_px': float(focal_length),
        'cx': float(cx), 'cy': float(cy),
        'K_left': K_l.tolist(),
        'dist_left': dist_l.flatten().tolist(),
        'K_right': K_r.tolist(),
        'dist_right': dist_r.flatten().tolist(),
        'R': R.tolist(), 'T': T.flatten().tolist(),
        'R1': R1.tolist(), 'R2': R2.tolist(),
        'P1': P1.tolist(), 'P2': P2.tolist(),
        'Q': Q.tolist(),
    }
    with open(s.output, 'w') as f:
        yaml.dump(calib_data, f, default_flow_style=False)

    npz_path = s.output.replace('.yaml', '_maps.npz')
    np.savez(npz_path,
             map1_l=map1_l, map2_l=map2_l,
             map1_r=map1_r, map2_r=map2_r)

    # 保存验证映射
    s.verify_maps = (map1_l, map2_l, map1_r, map2_r)
    s.verify_info = {'f': focal_length, 'b': baseline, 'rms': ret_stereo}

    quality = ('优秀 ✓' if ret_stereo < 0.5
               else '良好' if ret_stereo < 1.0
               else '一般，建议重新采集')

    log.append(f'\n{"="*40}')
    log.append(f'  标定完成！')
    log.append(f'{"="*40}')
    log.append(f'  图像尺寸:   {img_size[0]}x{img_size[1]}')
    log.append(f'  重投影误差: {ret_stereo:.4f} px ({quality})')
    log.append(f'  焦距:       {focal_length:.1f} px')
    log.append(f'  光心:       ({cx:.1f}, {cy:.1f})')
    log.append(f'  基线距离:   {baseline*1000:.1f} mm')
    log.append(f'  已保存:     {s.output}')
    log.append(f'              {npz_path}')
    log.append(f'\nROS 节点使用:')
    log.append(f'  ros2 run usv_drivers usv_vision_node --ros-args \\')
    log.append(f'    -p calibration_file:={os.path.abspath(s.output)}')

    result = '\n'.join(log)
    s.calib_result = result
    print(result)
    return result


def do_verify():
    """切换到验证模式"""
    s = STATE
    if not os.path.exists(s.output):
        return '标定文件不存在，请先标定'

    with open(s.output, 'r') as f:
        calib = yaml.safe_load(f)

    npz_path = s.output.replace('.yaml', '_maps.npz')
    if os.path.exists(npz_path):
        maps = np.load(npz_path)
        s.verify_maps = (maps['map1_l'], maps['map2_l'],
                         maps['map1_r'], maps['map2_r'])
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
        s.verify_maps = (m1l, m2l, m1r, m2r)

    s.verify_info = {
        'f': calib['focal_length_px'],
        'b': calib['baseline_m'],
        'rms': calib['stereo_rms_error'],
    }
    s.mode = 'verify'
    return (f'验证模式已开启\n'
            f'焦距: {calib["focal_length_px"]:.1f}px\n'
            f'基线: {calib["baseline_m"]*1000:.1f}mm\n'
            f'RMS: {calib["stereo_rms_error"]:.4f}px\n\n'
            f'绿色水平线应穿过左右图像中的同一物体')


def get_thumbnails():
    """获取已采集图像的缩略图列表"""
    s = STATE
    files = sorted(glob.glob(os.path.join(s.image_dir, 'left_*.png')))
    result = []
    for f in files:
        basename = os.path.basename(f)
        idx = int(basename.replace('left_', '').replace('.png', ''))
        result.append({'index': idx, 'name': basename})
    return result


def get_thumbnail_jpg(index):
    """生成指定编号左目图像的缩略图"""
    s = STATE
    path = os.path.join(s.image_dir, f'left_{index:03d}.png')
    if not os.path.exists(path):
        return None
    img = cv2.imread(path)
    if img is None:
        return None
    thumb = cv2.resize(img, (160, 120))
    _, buf = cv2.imencode('.jpg', thumb, [cv2.IMWRITE_JPEG_QUALITY, 70])
    return buf.tobytes()


# ======================================================================
# Web 界面 HTML
# ======================================================================
HTML_PAGE = '''<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<title>双目标定工具</title>
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body {
    font-family: 'Segoe UI', Arial, sans-serif;
    background: #1a1a2e; color: #eee;
    display: flex; flex-direction: column; height: 100vh;
  }
  .header {
    background: #16213e; padding: 10px 20px;
    display: flex; align-items: center; gap: 20px;
    border-bottom: 2px solid #0f3460;
  }
  .header h1 { font-size: 18px; color: #e94560; }
  .status-bar {
    display: flex; gap: 15px; align-items: center; flex: 1;
  }
  .status-dot {
    width: 14px; height: 14px; border-radius: 50%;
    display: inline-block; margin-right: 4px;
    vertical-align: middle;
  }
  .dot-green { background: #00e676; box-shadow: 0 0 8px #00e676; }
  .dot-red { background: #ff1744; box-shadow: 0 0 8px #ff1744; }
  .dot-gray { background: #666; }
  .main { display: flex; flex: 1; overflow: hidden; }
  .video-area {
    flex: 1; display: flex; justify-content: center;
    align-items: center; background: #111;
    position: relative;
  }
  .video-area img {
    max-width: 100%; max-height: 100%; object-fit: contain;
  }
  .sidebar {
    width: 280px; background: #16213e;
    display: flex; flex-direction: column;
    border-left: 2px solid #0f3460;
    overflow: hidden;
  }
  .controls { padding: 15px; }
  .btn {
    display: block; width: 100%; padding: 12px;
    border: none; border-radius: 6px; cursor: pointer;
    font-size: 15px; font-weight: 600;
    margin-bottom: 10px; transition: all 0.2s;
  }
  .btn:active { transform: scale(0.97); }
  .btn-capture {
    background: #00e676; color: #1a1a2e;
  }
  .btn-capture:hover { background: #69f0ae; }
  .btn-capture:disabled {
    background: #444; color: #888; cursor: not-allowed;
  }
  .btn-calib {
    background: #e94560; color: white;
  }
  .btn-calib:hover { background: #ff6b81; }
  .btn-calib:disabled {
    background: #444; color: #888; cursor: not-allowed;
  }
  .btn-verify {
    background: #0f3460; color: #eee; border: 1px solid #e94560;
  }
  .btn-verify:hover { background: #1a4a80; }
  .btn-collect {
    background: #0f3460; color: #eee; border: 1px solid #00e676;
  }
  .btn-collect:hover { background: #1a4a80; }
  .info-text {
    font-size: 12px; color: #aaa; padding: 5px 0;
    text-align: center;
  }
  .thumbs-area {
    flex: 1; overflow-y: auto; padding: 10px;
    border-top: 1px solid #0f3460;
  }
  .thumbs-area h3 {
    font-size: 13px; color: #aaa; margin-bottom: 8px;
  }
  .thumb-grid {
    display: grid; grid-template-columns: 1fr 1fr;
    gap: 6px;
  }
  .thumb-item {
    position: relative; border-radius: 4px; overflow: hidden;
    border: 1px solid #333;
  }
  .thumb-item img { width: 100%; display: block; }
  .thumb-item .idx {
    position: absolute; top: 2px; left: 4px;
    font-size: 10px; background: rgba(0,0,0,0.6);
    padding: 1px 4px; border-radius: 3px;
  }
  .thumb-item .del-btn {
    position: absolute; top: 2px; right: 4px;
    background: rgba(255,0,0,0.7); color: white;
    border: none; border-radius: 3px; cursor: pointer;
    font-size: 10px; padding: 1px 5px;
  }
  .thumb-item .del-btn:hover { background: #ff1744; }
  .log-area {
    max-height: 300px; overflow-y: auto;
    background: #111; padding: 10px; margin: 10px;
    border-radius: 6px; font-family: monospace;
    font-size: 12px; white-space: pre-wrap;
    line-height: 1.5; display: none;
  }
  .log-area.show { display: block; }
  .mode-badge {
    padding: 3px 10px; border-radius: 12px; font-size: 12px;
    font-weight: 600;
  }
  .mode-collect { background: #00e676; color: #1a1a2e; }
  .mode-verify { background: #e94560; color: white; }
  .hotkey { font-size: 11px; color: #666; text-align: center; padding: 5px; }
</style>
</head>
<body>
<div class="header">
  <h1>&#128247; 双目标定工具</h1>
  <div class="status-bar">
    <span>
      <span class="status-dot dot-gray" id="dot-left"></span>
      <span id="status-left">左目: --</span>
    </span>
    <span>
      <span class="status-dot dot-gray" id="dot-right"></span>
      <span id="status-right">右目: --</span>
    </span>
    <span id="mode-badge" class="mode-badge mode-collect">采集模式</span>
    <span id="count-text" style="margin-left:auto; font-size:14px;">
      已采集: 0 对
    </span>
  </div>
</div>

<div class="main">
  <div class="video-area">
    <img id="stream" src="/stream">
  </div>
  <div class="sidebar">
    <div class="controls">
      <button class="btn btn-capture" id="btn-capture"
              onclick="doCapture()" disabled>
        &#128248; 采集 (Space)
      </button>
      <button class="btn btn-calib" id="btn-calib"
              onclick="doCalibrate()" disabled>
        &#9881; 开始标定
      </button>
      <button class="btn btn-verify" id="btn-verify"
              onclick="doVerify()">
        &#128269; 验证校正效果
      </button>
      <button class="btn btn-collect" id="btn-back"
              onclick="doBackCollect()" style="display:none">
        &#8592; 返回采集
      </button>
      <div class="hotkey">快捷键: Space=采集</div>
    </div>
    <div class="log-area" id="log-area"></div>
    <div class="thumbs-area">
      <h3>已采集图像</h3>
      <div class="thumb-grid" id="thumb-grid"></div>
    </div>
  </div>
</div>

<script>
let pollTimer = null;

function pollStatus() {
  fetch('/api/status').then(r => r.json()).then(d => {
    document.getElementById('dot-left').className =
      'status-dot ' + (d.found_left ? 'dot-green' : 'dot-red');
    document.getElementById('dot-right').className =
      'status-dot ' + (d.found_right ? 'dot-green' : 'dot-red');
    document.getElementById('status-left').textContent =
      '左目: ' + (d.found_left ? '已检测' : '未检测');
    document.getElementById('status-right').textContent =
      '右目: ' + (d.found_right ? '已检测' : '未检测');
    document.getElementById('count-text').textContent =
      '已采集: ' + d.count + ' 对';

    const capBtn = document.getElementById('btn-capture');
    const calBtn = document.getElementById('btn-calib');
    capBtn.disabled = !(d.found_left && d.found_right) || d.mode !== 'collect';
    calBtn.disabled = d.count < 3;

    const badge = document.getElementById('mode-badge');
    if (d.mode === 'verify') {
      badge.className = 'mode-badge mode-verify';
      badge.textContent = '验证模式';
      document.getElementById('btn-capture').style.display = 'none';
      document.getElementById('btn-calib').style.display = 'none';
      document.getElementById('btn-verify').style.display = 'none';
      document.getElementById('btn-back').style.display = 'block';
    } else {
      badge.className = 'mode-badge mode-collect';
      badge.textContent = '采集模式';
      document.getElementById('btn-capture').style.display = 'block';
      document.getElementById('btn-calib').style.display = 'block';
      document.getElementById('btn-verify').style.display = 'block';
      document.getElementById('btn-back').style.display = 'none';
    }
  }).catch(() => {});
}

function refreshThumbs() {
  fetch('/api/thumbnails').then(r => r.json()).then(list => {
    const grid = document.getElementById('thumb-grid');
    grid.innerHTML = '';
    list.forEach(item => {
      const div = document.createElement('div');
      div.className = 'thumb-item';
      div.innerHTML = '<img src="/api/thumb?index=' + item.index + '">'
        + '<span class="idx">#' + item.index + '</span>'
        + '<button class="del-btn" onclick="doDelete(' + item.index
        + ')">X</button>';
      grid.appendChild(div);
    });
  }).catch(() => {});
}

function doCapture() {
  fetch('/api/capture', {method:'POST'}).then(r => r.json()).then(d => {
    showLog(d.message);
    refreshThumbs();
  });
}

function doDelete(idx) {
  if (!confirm('删除第 ' + idx + ' 对图像？')) return;
  fetch('/api/delete?index=' + idx, {method:'POST'}).then(r => r.json())
    .then(d => { showLog(d.message); refreshThumbs(); });
}

function doCalibrate() {
  const btn = document.getElementById('btn-calib');
  btn.disabled = true;
  btn.textContent = '标定中...';
  showLog('正在标定，请稍候...');
  fetch('/api/calibrate', {method:'POST'}).then(r => r.json()).then(d => {
    showLog(d.message);
    btn.textContent = '\\u2699 开始标定';
    btn.disabled = false;
  });
}

function doVerify() {
  fetch('/api/verify', {method:'POST'}).then(r => r.json()).then(d => {
    showLog(d.message);
  });
}

function doBackCollect() {
  fetch('/api/back_collect', {method:'POST'}).then(r => r.json()).then(d => {
    showLog(d.message);
  });
}

function showLog(text) {
  const el = document.getElementById('log-area');
  el.textContent = text;
  el.className = 'log-area show';
  el.scrollTop = el.scrollHeight;
}

// 快捷键
document.addEventListener('keydown', function(e) {
  if (e.code === 'Space') {
    e.preventDefault();
    const btn = document.getElementById('btn-capture');
    if (!btn.disabled) doCapture();
  }
});

// 定期刷新
pollTimer = setInterval(pollStatus, 300);
refreshThumbs();
setInterval(refreshThumbs, 5000);
</script>
</body></html>
'''


# ======================================================================
# HTTP 服务器
# ======================================================================
class CalibHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass  # 不打印每个HTTP请求

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == '/' or path == '/index.html':
            self._respond_html(HTML_PAGE)

        elif path == '/stream':
            self._respond_mjpeg()

        elif path == '/api/status':
            s = STATE
            with s.lock:
                data = {
                    'found_left': s.found_left,
                    'found_right': s.found_right,
                    'count': s.count,
                    'mode': s.mode,
                }
            self._respond_json(data)

        elif path == '/api/thumbnails':
            self._respond_json(get_thumbnails())

        elif path == '/api/thumb':
            qs = parse_qs(parsed.query)
            idx = int(qs.get('index', [0])[0])
            jpg = get_thumbnail_jpg(idx)
            if jpg:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(jpg)))
                self.end_headers()
                self.wfile.write(jpg)
            else:
                self.send_response(404)
                self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == '/api/capture':
            msg = do_capture()
            self._respond_json({'message': msg})

        elif path == '/api/delete':
            qs = parse_qs(parsed.query)
            idx = int(qs.get('index', [0])[0])
            msg = do_delete(idx)
            self._respond_json({'message': msg})

        elif path == '/api/calibrate':
            msg = do_calibrate()
            self._respond_json({'message': msg})

        elif path == '/api/verify':
            msg = do_verify()
            self._respond_json({'message': msg})

        elif path == '/api/back_collect':
            STATE.mode = 'collect'
            STATE.verify_maps = None
            self._respond_json({'message': '已返回采集模式'})

        else:
            self.send_response(404)
            self.end_headers()

    def _respond_html(self, html):
        data = html.encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _respond_json(self, obj):
        data = json.dumps(obj, ensure_ascii=False).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _respond_mjpeg(self):
        """MJPEG 流"""
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
                    frame = s.latest_frame
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
                time.sleep(0.033)  # ~30fps
        except (BrokenPipeError, ConnectionResetError):
            pass


# ======================================================================
# 主入口
# ======================================================================
def main():
    p = argparse.ArgumentParser(description='双目标定工具 (Web 界面)')
    p.add_argument('--left', default='/dev/video0', help='左目设备')
    p.add_argument('--right', default='/dev/video2', help='右目设备')
    p.add_argument('--width', type=int, default=640)
    p.add_argument('--height', type=int, default=480)
    p.add_argument('--board-cols', type=int, default=8,
                   help='内角点列数 (9x6方格板=8)')
    p.add_argument('--board-rows', type=int, default=5,
                   help='内角点行数 (9x6方格板=5)')
    p.add_argument('--square-size', type=float, default=0.030,
                   help='每格尺寸 (米, 默认 0.030=30mm)')
    p.add_argument('--image-dir', default='./calib_images')
    p.add_argument('--output', default='./stereo_calibration.yaml')
    p.add_argument('--port', type=int, default=8765,
                   help='Web 服务端口')
    args = p.parse_args()

    s = STATE
    s.board_size = (args.board_cols, args.board_rows)
    s.square_size = args.square_size
    s.image_dir = args.image_dir
    s.output = args.output
    s.frame_w = args.width
    s.frame_h = args.height
    s.count = len(glob.glob(os.path.join(args.image_dir, 'left_*.png')))

    # 打开摄像头
    s.cap_left = open_camera(args.left, args.width, args.height)
    s.cap_right = open_camera(args.right, args.width, args.height)
    if s.cap_left is None or s.cap_right is None:
        print('[ERROR] 摄像头打开失败')
        return

    # 启动摄像头线程
    cam_t = threading.Thread(target=camera_thread, daemon=True)
    cam_t.start()

    # 启动 Web 服务器
    server = ThreadingHTTPServer(('0.0.0.0', args.port), CalibHandler)
    print(f'\n{"="*50}')
    print(f'  双目标定工具已启动')
    print(f'  棋盘格: {args.board_cols}x{args.board_rows}, '
          f'每格 {args.square_size*1000:.0f}mm')
    print(f'{"="*50}')
    print(f'\n  请在浏览器中打开:')
    print(f'    http://localhost:{args.port}')
    print(f'\n  按 Ctrl+C 退出\n')

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\n[INFO] 正在关闭...')
    finally:
        s.running = False
        server.server_close()
        if s.cap_left:
            s.cap_left.release()
        if s.cap_right:
            s.cap_right.release()
        print('[INFO] 已退出')


if __name__ == '__main__':
    main()
