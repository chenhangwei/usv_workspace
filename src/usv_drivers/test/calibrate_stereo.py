#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双目摄像头标定工具

使用棋盘格标定板对双目摄像头进行标定，输出:
- 左右摄像头的内参矩阵 (焦距、光心)
- 畸变系数
- 外参 (旋转、平移矩阵)
- 立体校正映射表

标定步骤:
  1. 打印一张棋盘格标定板 (推荐 9x6 内角点，每格 25mm)
  2. 运行本脚本采集图像:
       python3 calibrate_stereo.py --collect
  3. 对着摄像头在不同角度、距离展示标定板，按空格采集 (建议 15-25 张)
  4. 按 'q' 开始标定计算
  5. 标定结果自动保存到 stereo_calibration.yaml

验证:
  python3 calibrate_stereo.py --verify

直接使用已采集的图像标定:
  python3 calibrate_stereo.py --calibrate --image-dir ./calib_images
"""

import argparse
import glob
import os
import time

import cv2
import numpy as np
import yaml


def parse_args():
    p = argparse.ArgumentParser(description='双目摄像头标定工具')
    p.add_argument('--left', default='/dev/video0', help='左目设备')
    p.add_argument('--right', default='/dev/video2', help='右目设备')
    p.add_argument('--width', type=int, default=640, help='图像宽度')
    p.add_argument('--height', type=int, default=480, help='图像高度')

    # 棋盘格参数
    p.add_argument('--board-cols', type=int, default=8,
                   help='棋盘格内角点列数 (9x6方格板=8)')
    p.add_argument('--board-rows', type=int, default=5,
                   help='棋盘格内角点行数 (9x6方格板=5)')
    p.add_argument('--square-size', type=float, default=0.030,
                   help='棋盘格每格实际尺寸 (米, 默认 0.030=30mm)')

    # 操作模式
    p.add_argument('--collect', action='store_true',
                   help='采集标定图像')
    p.add_argument('--calibrate', action='store_true',
                   help='使用已采集图像进行标定')
    p.add_argument('--verify', action='store_true',
                   help='验证标定结果 (显示校正后图像)')
    p.add_argument('--image-dir', default='./calib_images',
                   help='标定图像保存/读取目录')
    p.add_argument('--output', default='./stereo_calibration.yaml',
                   help='标定结果输出文件')
    p.add_argument('--headless', action='store_true',
                   help='无头模式 (保存图片而不弹窗)')
    return p.parse_args()


def open_camera(device, width, height):
    """打开摄像头 (V4L2 + MJPG)"""
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


def collect_images(args):
    """采集标定图像对"""
    cap_left = open_camera(args.left, args.width, args.height)
    cap_right = open_camera(args.right, args.width, args.height)
    if cap_left is None or cap_right is None:
        return

    os.makedirs(args.image_dir, exist_ok=True)
    board_size = (args.board_cols, args.board_rows)
    count = len(glob.glob(os.path.join(args.image_dir, 'left_*.png')))

    print(f'\n===== 双目标定图像采集 =====')
    print(f'棋盘格: {args.board_cols}x{args.board_rows} 内角点, '
          f'每格 {args.square_size * 1000:.0f}mm')
    print(f'保存目录: {args.image_dir}')
    print(f'已有图像: {count} 对')
    print()
    print('操作说明:')
    print('  - 将棋盘格放在两个摄像头都能看到的位置')
    print('  - 在不同距离 (0.3m~1.5m) 和角度展示')
    print('  - 按 [空格] 采集当前帧 (需两侧都检测到角点)')
    print('  - 建议采集 15~25 对图像')
    print('  - 按 [q] 结束采集并开始标定')
    print()

    while True:
        ret_l, left = cap_left.read()
        ret_r, right = cap_right.read()
        if not ret_l or not ret_r:
            time.sleep(0.05)
            continue

        gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

        # 检测角点
        found_l, corners_l = cv2.findChessboardCorners(
            gray_l, board_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
            | cv2.CALIB_CB_FILTER_QUADS)
        found_r, corners_r = cv2.findChessboardCorners(
            gray_r, board_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
            | cv2.CALIB_CB_FILTER_QUADS)

        # 显示检测结果
        disp_l = left.copy()
        disp_r = right.copy()
        if found_l:
            cv2.drawChessboardCorners(disp_l, board_size, corners_l, found_l)
        if found_r:
            cv2.drawChessboardCorners(disp_r, board_size, corners_r, found_r)

        # 状态指示
        status_l = 'OK' if found_l else 'X'
        status_r = 'OK' if found_r else 'X'
        both = found_l and found_r
        color = (0, 255, 0) if both else (0, 0, 255)
        cv2.putText(disp_l, f'Left: {status_l}  [{count} saved]',
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.putText(disp_r, f'Right: {status_r}',
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        if both:
            cv2.putText(disp_l, 'READY - Press SPACE',
                        (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0), 2)

        combined = np.hstack([disp_l, disp_r])
        scale = min(1280 / combined.shape[1], 1.0)
        if scale < 1.0:
            combined = cv2.resize(combined, None, fx=scale, fy=scale)

        if args.headless:
            # 无头模式: 自动采集检测到的帧
            if both:
                path_l = os.path.join(args.image_dir, f'left_{count:03d}.png')
                path_r = os.path.join(args.image_dir, f'right_{count:03d}.png')
                cv2.imwrite(path_l, left)
                cv2.imwrite(path_r, right)
                count += 1
                print(f'[{count}] 已采集第 {count} 对图像')
                time.sleep(1.5)  # 给用户时间移动标定板
                if count >= 20:
                    break
            else:
                time.sleep(0.1)
        else:
            cv2.imshow('Stereo Calibration - Collect', combined)
            key = cv2.waitKey(30) & 0xFF
            if key == ord(' ') and both:
                path_l = os.path.join(args.image_dir, f'left_{count:03d}.png')
                path_r = os.path.join(args.image_dir, f'right_{count:03d}.png')
                cv2.imwrite(path_l, left)
                cv2.imwrite(path_r, right)
                count += 1
                print(f'[{count}] 已保存: {path_l}, {path_r}')
            elif key == ord('q'):
                break

    cap_left.release()
    cap_right.release()
    cv2.destroyAllWindows()

    print(f'\n共采集 {count} 对图像')
    if count >= 10:
        print('开始标定...')
        run_calibration(args)
    else:
        print('[WARN] 图像不足 10 对，建议采集更多后运行:')
        print(f'  python3 calibrate_stereo.py --calibrate '
              f'--image-dir {args.image_dir}')


def run_calibration(args):
    """执行双目标定"""
    board_size = (args.board_cols, args.board_rows)
    square_size = args.square_size

    # 构建世界坐标系中的角点坐标
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0],
                            0:board_size[1]].T.reshape(-1, 2)
    objp *= square_size

    obj_points = []  # 世界坐标系角点
    img_points_l = []  # 左图像角点
    img_points_r = []  # 右图像角点
    img_size = None

    # 亚像素精化参数
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                30, 0.001)

    # 读取所有图像对
    left_files = sorted(glob.glob(
        os.path.join(args.image_dir, 'left_*.png')))
    right_files = sorted(glob.glob(
        os.path.join(args.image_dir, 'right_*.png')))

    if len(left_files) == 0:
        print(f'[ERROR] 在 {args.image_dir} 中未找到标定图像')
        return
    if len(left_files) != len(right_files):
        print(f'[ERROR] 左右图像数量不匹配: L={len(left_files)} R={len(right_files)}')
        return

    print(f'\n===== 双目标定 =====')
    print(f'图像对数: {len(left_files)}')
    print(f'棋盘格: {board_size[0]}x{board_size[1]}, '
          f'每格 {square_size * 1000:.0f}mm')

    for i, (lf, rf) in enumerate(zip(left_files, right_files)):
        img_l = cv2.imread(lf)
        img_r = cv2.imread(rf)
        if img_l is None or img_r is None:
            print(f'  [{i}] 读取失败，跳过')
            continue

        gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)
        if img_size is None:
            img_size = (gray_l.shape[1], gray_l.shape[0])

        found_l, corners_l = cv2.findChessboardCorners(
            gray_l, board_size, cv2.CALIB_CB_ADAPTIVE_THRESH)
        found_r, corners_r = cv2.findChessboardCorners(
            gray_r, board_size, cv2.CALIB_CB_ADAPTIVE_THRESH)

        if found_l and found_r:
            # 亚像素精化
            corners_l = cv2.cornerSubPix(
                gray_l, corners_l, (11, 11), (-1, -1), criteria)
            corners_r = cv2.cornerSubPix(
                gray_r, corners_r, (11, 11), (-1, -1), criteria)
            obj_points.append(objp)
            img_points_l.append(corners_l)
            img_points_r.append(corners_r)
            print(f'  [{i}] {os.path.basename(lf)} - OK')
        else:
            print(f'  [{i}] {os.path.basename(lf)} - '
                  f'角点检测失败 (L={found_l}, R={found_r})')

    valid = len(obj_points)
    print(f'\n有效图像对: {valid}/{len(left_files)}')
    if valid < 8:
        print('[ERROR] 有效图像不足 8 对，标定可能不准确')
        if valid < 3:
            return

    # ---- 1. 单目标定 (获取初始内参) ----
    print('\n[1/3] 单目标定...')
    flags_mono = cv2.CALIB_FIX_K3  # 固定高阶畸变

    ret_l, K_l, dist_l, _, _ = cv2.calibrateCamera(
        obj_points, img_points_l, img_size, None, None, flags=flags_mono)
    ret_r, K_r, dist_r, _, _ = cv2.calibrateCamera(
        obj_points, img_points_r, img_size, None, None, flags=flags_mono)

    print(f'  左目重投影误差: {ret_l:.4f} px')
    print(f'  右目重投影误差: {ret_r:.4f} px')

    # ---- 2. 双目标定 ----
    print('\n[2/3] 双目标定...')
    flags_stereo = (
        cv2.CALIB_FIX_INTRINSIC  # 使用单目标定的内参
    )
    criteria_stereo = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)

    ret_stereo, K_l, dist_l, K_r, dist_r, R, T, E, F = \
        cv2.stereoCalibrate(
            obj_points, img_points_l, img_points_r,
            K_l, dist_l, K_r, dist_r, img_size,
            criteria=criteria_stereo, flags=flags_stereo)

    print(f'  双目重投影误差: {ret_stereo:.4f} px')
    baseline = np.linalg.norm(T)
    print(f'  计算得到的基线距离: {baseline * 1000:.1f} mm')

    # ---- 3. 立体校正 ----
    print('\n[3/3] 计算校正映射...')
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        K_l, dist_l, K_r, dist_r, img_size, R, T,
        alpha=0, flags=cv2.CALIB_ZERO_DISPARITY)

    map1_l, map2_l = cv2.initUndistortRectifyMap(
        K_l, dist_l, R1, P1, img_size, cv2.CV_32FC1)
    map1_r, map2_r = cv2.initUndistortRectifyMap(
        K_r, dist_r, R2, P2, img_size, cv2.CV_32FC1)

    # 提取校正后的焦距 (P1 和 P2 的 fx 相同)
    focal_length = P1[0, 0]
    cx = P1[0, 2]
    cy = P1[1, 2]

    # ---- 保存结果 ----
    calib_data = {
        'image_size': list(img_size),
        'stereo_rms_error': float(ret_stereo),
        'baseline_m': float(baseline),
        'focal_length_px': float(focal_length),
        'cx': float(cx),
        'cy': float(cy),
        'K_left': K_l.tolist(),
        'dist_left': dist_l.flatten().tolist(),
        'K_right': K_r.tolist(),
        'dist_right': dist_r.flatten().tolist(),
        'R': R.tolist(),
        'T': T.flatten().tolist(),
        'R1': R1.tolist(),
        'R2': R2.tolist(),
        'P1': P1.tolist(),
        'P2': P2.tolist(),
        'Q': Q.tolist(),
    }

    # 保存 YAML
    with open(args.output, 'w') as f:
        yaml.dump(calib_data, f, default_flow_style=False)
    print(f'\n标定结果已保存: {args.output}')

    # 保存 remap 映射表 (npz 格式，运行时加载快)
    npz_path = args.output.replace('.yaml', '_maps.npz')
    np.savez(npz_path,
             map1_l=map1_l, map2_l=map2_l,
             map1_r=map1_r, map2_r=map2_r)
    print(f'校正映射已保存: {npz_path}')

    # ---- 输出摘要 ----
    print(f'\n{"="*50}')
    print(f'  标定结果摘要')
    print(f'{"="*50}')
    print(f'  图像尺寸:      {img_size[0]} x {img_size[1]}')
    print(f'  重投影误差:    {ret_stereo:.4f} px '
          f'({"优秀" if ret_stereo < 0.5 else "良好" if ret_stereo < 1.0 else "一般"})')
    print(f'  焦距 (fx):     {focal_length:.1f} px')
    print(f'  光心:          ({cx:.1f}, {cy:.1f})')
    print(f'  基线距离:      {baseline * 1000:.1f} mm')
    print(f'{"="*50}')
    print()
    print('在 usv_vision_node 中使用标定参数:')
    print(f'  --ros-args '
          f'-p focal_length:={focal_length:.1f} '
          f'-p baseline:={baseline:.4f}')
    print()
    print('在测试脚本中使用:')
    print(f'  python3 test_vision.py '
          f'--focal-length {focal_length:.1f} '
          f'--baseline {baseline:.4f}')


def verify_calibration(args):
    """验证标定结果 — 显示校正后的图像"""
    if not os.path.exists(args.output):
        print(f'[ERROR] 标定文件不存在: {args.output}')
        return

    # 加载标定结果
    with open(args.output, 'r') as f:
        calib = yaml.safe_load(f)

    npz_path = args.output.replace('.yaml', '_maps.npz')
    if os.path.exists(npz_path):
        maps = np.load(npz_path)
        map1_l, map2_l = maps['map1_l'], maps['map2_l']
        map1_r, map2_r = maps['map1_r'], maps['map2_r']
    else:
        print('[INFO] 重新计算校正映射...')
        img_size = tuple(calib['image_size'])
        K_l = np.array(calib['K_left'])
        dist_l = np.array(calib['dist_left'])
        K_r = np.array(calib['K_right'])
        dist_r = np.array(calib['dist_right'])
        R = np.array(calib['R'])
        T = np.array(calib['T'])
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            K_l, dist_l, K_r, dist_r, img_size, R, T, alpha=0)
        map1_l, map2_l = cv2.initUndistortRectifyMap(
            K_l, dist_l, R1, P1, img_size, cv2.CV_32FC1)
        map1_r, map2_r = cv2.initUndistortRectifyMap(
            K_r, dist_r, R2, P2, img_size, cv2.CV_32FC1)

    print(f'焦距: {calib["focal_length_px"]:.1f} px')
    print(f'基线: {calib["baseline_m"] * 1000:.1f} mm')
    print(f'重投影误差: {calib["stereo_rms_error"]:.4f} px')
    print()

    cap_left = open_camera(args.left, args.width, args.height)
    cap_right = open_camera(args.right, args.width, args.height)
    if cap_left is None or cap_right is None:
        return

    print('[INFO] 校正验证 — 水平线应穿过左右图像中的同一物体')
    print('按 q 退出')

    frame_count = 0
    while True:
        ret_l, left = cap_left.read()
        ret_r, right = cap_right.read()
        if not ret_l or not ret_r:
            time.sleep(0.05)
            continue

        # 应用校正映射
        rect_l = cv2.remap(left, map1_l, map2_l, cv2.INTER_LINEAR)
        rect_r = cv2.remap(right, map1_r, map2_r, cv2.INTER_LINEAR)

        # 拼接并绘制水平参考线
        combined = np.hstack([rect_l, rect_r])
        h = combined.shape[0]
        for y in range(0, h, 30):
            cv2.line(combined, (0, y), (combined.shape[1], y),
                     (0, 255, 0), 1)

        cv2.putText(combined,
                    f'Focal={calib["focal_length_px"]:.0f} '
                    f'Baseline={calib["baseline_m"]*1000:.0f}mm '
                    f'RMS={calib["stereo_rms_error"]:.3f}',
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 255), 1)

        scale = min(1280 / combined.shape[1], 1.0)
        if scale < 1.0:
            combined = cv2.resize(combined, None, fx=scale, fy=scale)

        frame_count += 1
        if args.headless:
            if frame_count in (1, 10):
                out_path = f'/tmp/stereo_verify_frame_{frame_count}.png'
                cv2.imwrite(out_path, combined)
                print(f'[SAVE] {out_path}')
            if frame_count >= 10:
                break
            time.sleep(0.1)
        else:
            cv2.imshow('Stereo Verify (green lines should align)', combined)
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break

    cap_left.release()
    cap_right.release()
    cv2.destroyAllWindows()


def main():
    args = parse_args()

    if args.verify:
        verify_calibration(args)
    elif args.calibrate:
        run_calibration(args)
    elif args.collect:
        collect_images(args)
    else:
        print('请指定操作模式:')
        print('  --collect    采集标定图像 (需要棋盘格标定板)')
        print('  --calibrate  使用已采集图像标定')
        print('  --verify     验证标定结果')
        print()
        print('完整标定流程:')
        print('  1. 打印 9x6 棋盘格 (每格 25mm)')
        print('  2. python3 calibrate_stereo.py --collect')
        print('  3. 对准摄像头展示标定板，按空格采集 15~25 张')
        print('  4. 按 q 自动开始标定')
        print('  5. python3 calibrate_stereo.py --verify')


if __name__ == '__main__':
    main()
