# Ubuntu 彩色 Emoji 字体配置指南

## 为什么需要配置

默认情况下，Ubuntu 只有黑白轮廓 emoji，甚至会显示为"豆腐块"（□）。
配置彩色 emoji 字体后，可以在所有 GTK/Qt 应用中正常显示彩色 emoji 图标。

## 一分钟快速配置（适用于 Ubuntu 22.04 及以上）

### 1. 安装彩色 Emoji 字体

```bash
sudo apt install fonts-noto-color-emoji
```

### 2. 刷新字体缓存

```bash
sudo fc-cache -fv
```

### 3. 配置字体优先级

创建全局配置文件（所有用户生效）：

```bash
sudo nano /etc/fonts/local.conf
```

或创建用户配置文件（仅当前用户）：

```bash
mkdir -p ~/.config/fontconfig
nano ~/.config/fontconfig/fonts.conf
```

写入以下内容：

```xml
<?xml version="1.0"?>
<!DOCTYPE fontconfig SYSTEM "fonts.dtd">
<fontconfig>
  <alias>
    <family>sans-serif</family>
    <prefer><family>Noto Color Emoji</family></prefer>
  </alias>
  <alias>
    <family>serif</family>
    <prefer><family>Noto Color Emoji</family></prefer>
  </alias>
  <alias>
    <family>monospace</family>
    <prefer><family>Noto Color Emoji</family></prefer>
  </alias>
</fontconfig>
```

### 4. 重启生效

```bash
# 注销当前会话并重新登录
# 或直接重启系统
sudo reboot
```

## 验证配置

### 桌面应用验证

在以下应用中输入或粘贴 emoji：
- GNOME Text Editor
- Gedit
- VS Code
- Firefox
- Telegram

测试字符：`🎉🐧🚀📊🔧💡`

应该能看到**彩色**图标（而非黑白轮廓或豆腐块）。

### 终端验证

```bash
echo -e "\U1F680 \U1F427 \U1F389 \U1F4CA \U1F527 \U1F4A1"
```

应该显示：🚀 🐧 🎉 📊 🔧 💡（彩色）

### Python/Qt 应用验证

```bash
cd ~/usv_workspace
source install/setup.bash
ros2 launch gs_bringup gs_launch.py
```

界面中的所有 emoji 应该显示为彩色。

## 常见问题

### 问题 1: 只显示黑白符号

**原因**：程序优先调用了其他符号字体（如 Symbola、DejaVu）

**解决**：在 `local.conf` 中确保 `Noto Color Emoji` 在 `<prefer>` 列表的**最前面**

### 问题 2: 还是显示豆腐块 □

**原因**：字体未安装或缓存未刷新

**解决**：
```bash
sudo apt install fonts-noto-color-emoji
sudo fc-cache -fv
```

### 问题 3: 某些 Qt 程序显示灰框

**原因**：Qt ≤ 5.15 默认不支持彩色字体

**解决**：
- 升级到 Qt 6（推荐）
- 或使用 KDE 应用
- 或在代码中设置字体渲染选项

### 问题 4: 重启后仍不生效

**原因**：配置文件权限或路径错误

**解决**：
```bash
# 检查配置文件是否存在
ls -la /etc/fonts/local.conf
# 或
ls -la ~/.config/fontconfig/fonts.conf

# 检查权限
sudo chmod 644 /etc/fonts/local.conf

# 强制刷新
sudo fc-cache -fv
```

## ROS 2 GUI 应用特殊说明

USV 地面站 GUI 使用 PyQt5，完全支持彩色 emoji。

配置完成后，界面中的以下元素将显示为彩色：
- 🚀 启动按钮
- 📋 列表标题
- 🎯 目标点
- 📝 日志输出
- 🔄 刷新按钮
- ⏹ 停止按钮
- ✅ 成功状态
- ⚠️ 警告提示
- ❌ 错误信息
- 🔍 搜索图标

## 支持的终端模拟器

以下终端完全支持彩色 emoji：
- ✅ GNOME Terminal（Ubuntu 默认）
- ✅ Konsole（KDE）
- ✅ Alacritty
- ✅ Kitty
- ✅ Wezterm
- ⚠️ xterm（不支持彩色，仅黑白）

## 总结

1. 安装 `fonts-noto-color-emoji`
2. 配置 fontconfig 优先级
3. 注销或重启
4. 验证效果

完成后，Ubuntu 的 emoji 显示效果将与 macOS/Windows 一致 😎🎉

---

**相关文档**：
- Noto Color Emoji 官方：https://fonts.google.com/noto/specimen/Noto+Color+Emoji
- Fontconfig 配置：https://www.freedesktop.org/software/fontconfig/fontconfig-user.html

**修改日期**：2025-11-06
