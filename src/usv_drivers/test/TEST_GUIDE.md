# USV 视觉 & 云台 Web 测试工具 使用指南

## 前提条件

- WSL2 Ubuntu 24.04，已编译自定义内核（含 USBIP + UVC 模块）
- Windows 已安装 usbipd-win
- 两个 USB 摄像头已连接到 Windows 主机
- Python 3, OpenCV, ultralytics 已安装

## 1. 启动 usbipd 服务

usbipd Windows 服务可能会自动崩溃（exit code 1067），此时可用前台进程替代。

### 方式一：Windows 服务（如果正常）

打开管理员 PowerShell：
```powershell
sc.exe query usbipd
# 确认 STATE: RUNNING
```

### 方式二：前台进程运行（推荐）

当服务频繁崩溃时，先禁用服务，然后以前台进程运行：

```powershell
# 停止并禁用服务
sc.exe stop usbipd
sc.exe config usbipd start= demand

# 以管理员身份前台运行（保持此窗口不关）
& "C:\Program Files\usbipd-win\usbipd.exe" server
```

> **注意**：此 PowerShell 窗口必须保持打开，关闭后 USB 设备会断开。

## 2. 绑定并透传 USB 摄像头

打开 **另一个** 管理员 PowerShell：

```powershell
# 查看设备列表
usbipd list

# 绑定摄像头（首次需要）
usbipd bind --busid 1-1
usbipd bind --busid 1-2

# 挂载到 WSL
usbipd attach --wsl --busid 1-1
usbipd attach --wsl --busid 1-2
```

## 3. 在 WSL 中验证

```bash
# 确认设备出现
ls /dev/video*
v4l2-ctl --list-devices

# 预期输出:
# RGB Camera (usb-vhci_hcd.0-1): /dev/video0, /dev/video1
# RGB Camera (usb-vhci_hcd.0-2): /dev/video2, /dev/video3
```

## 4. 运行测试工具

### 4.1 双目视觉 Web 工具（仅视觉）

```bash
cd ~/usv_workspace/src/usv_drivers
CUDA_VISIBLE_DEVICES="" python3 test/test_vision_web.py --port 8766
```

打开浏览器访问 `http://localhost:8766`

### 4.2 双目视觉 + 云台控制 Web 工具

**无云台硬件（模拟模式）：**
```bash
cd ~/usv_workspace/src/usv_drivers
CUDA_VISIBLE_DEVICES="" python3 test/test_gimbal_web.py --no-hardware --port 8767
```

**有云台硬件（CH341 USB-I2C + PCA9685）：**
```bash
# 先透传 CH341（在 Windows PowerShell）
# usbipd bind --busid 1-7
# usbipd attach --wsl --busid 1-7

# 加载 CH341 I2C 驱动（在 WSL）
# sudo insmod ~/ch341-i2c/i2c-ch341-usb.ko

cd ~/usv_workspace/src/usv_drivers
CUDA_VISIBLE_DEVICES="" python3 test/test_gimbal_web.py \
    --i2c-backend smbus --i2c-bus 0 \
    --pan-channel 1 --tilt-channel 2 \
    --port 8767
```

打开浏览器访问 `http://localhost:8767`

### 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--left` | /dev/video0 | 左摄像头设备 |
| `--right` | /dev/video2 | 右摄像头设备 |
| `--width` | 640 | 图像宽度 |
| `--height` | 480 | 图像高度 |
| `--calib` | ./stereo_calibration.yaml | 标定文件路径 |
| `--model` | (空) | YOLO 模型路径 |
| `--confidence` | 0.45 | 检测置信度阈值 |
| `--port` | 8767 | Web 服务端口 |
| `--no-hardware` | false | 不连接 PCA9685, 模拟模式 |
| `--i2c-backend` | auto | I2C 后端: auto/adafruit/smbus/none |
| `--i2c-bus` | 0 | smbus 总线编号 (/dev/i2c-N) |
| `--pan-channel` | 1 | PCA9685 Pan 通道 |
| `--tilt-channel` | 2 | PCA9685 Tilt 通道 |

## 5. 常见问题

### 摄像头打开失败 (EBUSY)
上一次进程未正常退出，设备被占用：
```bash
fuser /dev/video0
kill <PID>
```

### usbipd attach 失败: "Attach Request failed"
WSL 中存在冲突的 usbip 客户端：
```bash
sudo mv /usr/local/sbin/usbip /usr/local/sbin/usbip.bak
sudo mv /usr/local/sbin/usbipd /usr/local/sbin/usbipd.bak
```

### vhci_hcd 模块未加载
```bash
sudo modprobe vhci-hcd
# 永久加载: /etc/modules-load.d/usbip.conf 中添加 vhci-hcd
```
