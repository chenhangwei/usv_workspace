#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PCA9685 PWM 驱动 —— smbus2 后端

通过 /dev/i2c-N (如 CH341 USB-to-I2C) 驱动 PCA9685，
API 兼容 adafruit_pca9685，可作为替代后端使用。

用法:
    pca = PCA9685SMBus(bus=0, address=0x40)
    pca.frequency = 50
    pca.channels[1].duty_cycle = 0x3FF0   # 与 adafruit 完全一致
    pca.deinit()
"""

import time

try:
    import smbus2
    SMBUS2_AVAILABLE = True
except ImportError:
    SMBUS2_AVAILABLE = False


class _ChannelProxy:
    """模拟 adafruit PCA9685 channel 接口"""

    def __init__(self, pca, index):
        self._pca = pca
        self._index = index
        self._duty = 0

    @property
    def duty_cycle(self):
        return self._duty

    @duty_cycle.setter
    def duty_cycle(self, value):
        """设置占空比, value: 0~0xFFFF (与 adafruit 一致)"""
        value = max(0, min(0xFFFF, int(value)))
        self._duty = value
        # 映射 16 位 → 12 位
        off_val = value >> 4  # 0xFFFF → 0xFFF
        if off_val >= 4096:
            # 全开
            self._pca._write_channel(self._index, 4096, 0)
        elif off_val == 0:
            # 全关
            self._pca._write_channel(self._index, 0, 4096)
        else:
            self._pca._write_channel(self._index, 0, off_val)


class PCA9685SMBus:
    """
    PCA9685 16 路 PWM 驱动 (smbus2 后端)

    参数:
        bus:     I2C 总线编号 (对应 /dev/i2c-N)
        address: PCA9685 I2C 地址 (默认 0x40)
    """

    # PCA9685 寄存器
    _MODE1 = 0x00
    _MODE2 = 0x01
    _PRESCALE = 0xFE
    _LED0_ON_L = 0x06

    def __init__(self, bus=0, address=0x40):
        if not SMBUS2_AVAILABLE:
            raise RuntimeError(
                'smbus2 未安装。请执行: pip3 install smbus2')
        self._bus = smbus2.SMBus(bus)
        self._addr = address
        self._channels = [_ChannelProxy(self, i) for i in range(16)]
        self._frequency = 50

        # 复位
        self._bus.write_byte_data(self._addr, self._MODE1, 0x00)
        time.sleep(0.005)

    @property
    def channels(self):
        return self._channels

    @property
    def frequency(self):
        return self._frequency

    @frequency.setter
    def frequency(self, freq):
        """设置 PWM 频率 (Hz)"""
        freq = max(24, min(1526, freq))
        self._frequency = freq
        # prescale = round(25MHz / (4096 * freq)) - 1
        prescale = int(round(25000000.0 / (4096.0 * freq)) - 1)
        prescale = max(3, min(255, prescale))

        old_mode = self._bus.read_byte_data(self._addr, self._MODE1)
        # 进入 sleep 模式才能设置 prescale
        self._bus.write_byte_data(
            self._addr, self._MODE1, (old_mode & 0x7F) | 0x10)
        self._bus.write_byte_data(self._addr, self._PRESCALE, prescale)
        # 恢复模式
        self._bus.write_byte_data(self._addr, self._MODE1, old_mode)
        time.sleep(0.005)
        # 启用 auto-increment
        self._bus.write_byte_data(
            self._addr, self._MODE1, old_mode | 0xA0)

    def _write_channel(self, channel, on, off):
        """写入单通道 ON/OFF 值 (12 位)"""
        reg = self._LED0_ON_L + 4 * channel
        self._bus.write_byte_data(self._addr, reg, on & 0xFF)
        self._bus.write_byte_data(self._addr, reg + 1, (on >> 8) & 0x0F)
        self._bus.write_byte_data(self._addr, reg + 2, off & 0xFF)
        self._bus.write_byte_data(self._addr, reg + 3, (off >> 8) & 0x0F)

    def deinit(self):
        """释放总线"""
        try:
            # 所有通道关闭
            for ch in range(16):
                self._write_channel(ch, 0, 0)
            self._bus.close()
        except Exception:
            pass
