#!/bin/bash
# ============================================================
# CH341A USB-to-I2C 内核模块编译脚本 (WSL2)
#
# 用途: 为自定义 WSL2 内核编译 i2c-ch341-usb 驱动模块
#       使 CH341A 适配器在 WSL2 中创建 /dev/i2c-N 设备节点
#
# 前提条件:
#   1. 已编译自定义 WSL2 内核 (~/WSL2-Linux-Kernel)
#   2. CH341A USB-to-I2C 适配器
#   3. usbipd-win 已安装
#
# 使用方法:
#   chmod +x build_ch341_i2c.sh
#   ./build_ch341_i2c.sh
#
# 然后:
#   1. PowerShell(管理员): wsl --shutdown
#   2. 重新打开 WSL2
#   3. sudo modprobe i2c-dev
#   4. sudo insmod ~/ch341-i2c/i2c-ch341-usb.ko
#   5. PowerShell: usbipd attach --wsl --busid <CH341的BUSID>
#   6. ls /dev/i2c-*   ← 应该出现 /dev/i2c-0 或类似设备
#
# ============================================================
set -e

KERNEL_DIR="$HOME/WSL2-Linux-Kernel"
CH341_DIR="$HOME/ch341-i2c"
REPO_URL="https://github.com/frank-zago/ch341-i2c-spi-gpio.git"

echo "============================================"
echo " CH341A I2C 内核模块编译"
echo "============================================"

# ---- 检查内核源码 ----
if [ ! -d "$KERNEL_DIR" ]; then
    echo "[ERROR] 未找到内核源码: $KERNEL_DIR"
    echo "        请先编译自定义 WSL2 内核"
    exit 1
fi

# ---- 克隆驱动源码 ----
if [ -d "$CH341_DIR" ]; then
    echo "[INFO] 已存在 $CH341_DIR，拉取最新代码..."
    cd "$CH341_DIR"
    git pull || true
else
    echo "[INFO] 克隆 CH341 I2C 驱动..."
    git clone "$REPO_URL" "$CH341_DIR"
    cd "$CH341_DIR"
fi

# ---- 编译模块 ----
echo "[INFO] 编译内核模块 (使用 $KERNEL_DIR)..."
make KDIR="$KERNEL_DIR" clean 2>/dev/null || true
make KDIR="$KERNEL_DIR"

if [ ! -f "i2c-ch341-usb.ko" ]; then
    echo "[ERROR] 编译失败，未找到 i2c-ch341-usb.ko"
    exit 1
fi

echo ""
echo "============================================"
echo " 编译成功: $CH341_DIR/i2c-ch341-usb.ko"
echo "============================================"
echo ""
echo " ===== 使用步骤 ====="
echo ""
echo " 1. 加载 I2C 核心模块:"
echo "    sudo modprobe i2c-dev"
echo ""
echo " 2. 加载 CH341 I2C 模块:"
echo "    sudo insmod $CH341_DIR/i2c-ch341-usb.ko"
echo ""
echo " 3. 在 Windows PowerShell(管理员) 中透传 CH341 USB 设备:"
echo "    usbipd list                         # 找到 CH341 的 BUSID"
echo "    usbipd attach --wsl --busid X-X     # 替换为实际 BUSID"
echo ""
echo " 4. 检查 I2C 设备:"
echo "    ls -la /dev/i2c-*"
echo "    i2cdetect -l"
echo "    i2cdetect -y 0                      # 扫描总线 0"
echo ""
echo " 5. 验证 PCA9685 (应在地址 0x40 出现):"
echo "    i2cdetect -y 0 | grep 40"
echo ""
echo " 6. 启动云台测试:"
echo "    cd ~/usv_workspace/src/usv_drivers"
echo "    CUDA_VISIBLE_DEVICES='' python3 test/test_gimbal_web.py \\"
echo "        --calib ./stereo_calibration.yaml --i2c-backend smbus --i2c-bus 0"
echo ""
echo " ===== 开机自动加载 ====="
echo " echo 'i2c-dev' | sudo tee -a /etc/modules-load.d/modules.conf"
echo " echo '$CH341_DIR/i2c-ch341-usb.ko' | sudo tee -a /etc/modules-load.d/ch341-i2c.conf"
echo ""
echo " ===== 安装 Python 依赖 ====="
echo " pip3 install smbus2 i2c-tools"
echo ""
