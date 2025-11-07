# 低电量声音播放修复

## 🐛 问题描述

**症状**: USV 进入低电量模式后，LED 正常显示红色闪烁，但声音没有播放低电量警告音

**用户反馈**: usv_02 已经进入低电量模式，led 已经正常显示低电量状态，但 sound 没有发出低电量状态的声音

## 🔍 根本原因分析

### 问题定位

通过分析代码，发现低电量声音播放的逻辑缺陷：

1. **低电量检测正常工作**
   - ✅ `usv_status_node.py` 正确检测电量百分比 < 5%
   - ✅ 正确发布 `low_voltage_mode` 话题（Bool类型）
   - ✅ LED 节点正确订阅并响应（显示红色闪烁）

2. **Sound 节点的问题**
   - ✅ `usv_sound_node.py` 正确订阅 `low_voltage_mode` 话题
   - ✅ `low_voltage_mode_callback` 正确设置 `self.low_voltage = True`
   - ❌ **但声音播放需要先启动 `sound_loop` 循环**
   - ❌ **如果没有手动发送 `sound_start` 命令，声音循环不会运行**

### 逻辑流程对比

#### LED 节点（正常工作）
```python
# usv_led_node.py
def low_voltage_mode_callback(self, msg):
    if msg.data:
        self.low_voltage = True
        self.set_led_low_battery()  # ✅ 立即设置 LED 为红色闪烁
```

#### Sound 节点（原实现 - 有问题）
```python
# usv_sound_node.py (修复前)
def low_voltage_mode_callback(self, msg):
    if msg.data:
        self.low_voltage = True  # ❌ 仅设置标志，不启动播放
        # 声音需要 sound_loop 循环运行才能播放
        # 但循环需要手动 sound_start 命令才能启动

def sound_loop(self):
    while not self.loop_stop_event.is_set():
        if self.low_voltage:
            sound_type = self.moon_type  # 低电量音效
        else:
            sound_type = random.choice(self.sound_types)
        self.sound_play(sound_type)
        # ❌ 如果循环没有运行，永远不会执行到这里
```

### 为什么 LED 工作但 Sound 不工作？

| 组件 | 触发方式 | 是否需要预启动 | 结果 |
|------|---------|---------------|------|
| **LED** | 直接设置 GPIO | 否 | ✅ 正常工作 |
| **Sound** | 依赖循环线程 | 是（需要 `sound_start`） | ❌ 不播放 |

## ✅ 修复方案

### 修改内容

在 `low_voltage_mode_callback` 中，当检测到进入低电量模式时，**自动启动声音循环**：

```python
def low_voltage_mode_callback(self, msg):
    """低电压模式专用回调函数"""
    try:
        if not isinstance(msg, Bool):
            self.get_logger().warn('收到无效的低电压模式消息类型')
            return
        
        if msg.data and not self.low_voltage:
            # 进入低电量模式
            self.low_voltage = True
            self.get_logger().error('[!][!][!] 低电压模式触发！')
            
            # 🔥 修复：自动启动声音循环播放低电量警告
            if not (self.loop_thread and self.loop_thread.is_alive()):
                self.get_logger().error('[!] 自动启动低电量警告声音播放')
                self.start_sound_loop()
            
        elif not msg.data and self.low_voltage:
            # 退出低电量模式
            self.low_voltage = False
            self.get_logger().info('[OK] 退出低电量模式')
            
    except Exception as e:
        self.get_logger().error(f'处理低电压模式回调时发生错误: {e}')
```

### 修复逻辑

1. **检测低电量触发**: `msg.data == True` 且 `self.low_voltage == False`
2. **设置低电量标志**: `self.low_voltage = True`
3. **检查声音循环状态**: 
   - 如果循环未运行 → 自动启动 `start_sound_loop()`
   - 如果循环已运行 → 不重复启动（保持运行）
4. **循环线程自动选择低电量音效**: 
   - `if self.low_voltage: sound_type = self.moon_type`

## 🧪 测试验证

### 测试场景 1: 低电量触发时声音循环未启动

**步骤**:
1. USV 上电，声音循环未启动（未发送 `sound_start`）
2. 电池电压降至低电量阈值（< 5%）
3. `usv_status_node` 发布 `low_voltage_mode: True`

**期望结果**:
- ✅ LED 显示红色闪烁
- ✅ **Sound 自动启动循环并播放 `moon101.wav`**（修复后）
- ✅ 日志输出：`[!] 自动启动低电量警告声音播放`

### 测试场景 2: 低电量触发时声音循环已启动

**步骤**:
1. USV 上电，已手动发送 `sound_start`（正常播放 gaga 音效）
2. 电池电压降至低电量阈值（< 5%）
3. `usv_status_node` 发布 `low_voltage_mode: True`

**期望结果**:
- ✅ LED 显示红色闪烁
- ✅ Sound 立即切换到低电量音效 `moon101.wav`
- ✅ 不会重复启动循环（避免冲突）

### 测试场景 3: 退出低电量模式

**步骤**:
1. USV 处于低电量模式（播放 `moon101.wav`）
2. 电池充电，电压升至恢复阈值（> 8%）
3. `usv_status_node` 发布 `low_voltage_mode: False`

**期望结果**:
- ✅ LED 恢复正常显示
- ✅ Sound 切换回正常音效（随机播放 gaga）
- ✅ **循环继续运行**（不停止）

## 🚀 部署步骤

### 1. 更新代码

```bash
cd ~/usv_workspace
# 代码已修改，直接构建
colcon build --packages-select usv_sound
source install/setup.bash
```

### 2. 重启 USV 节点

**方法 A: 重启整个 USV 系统**
```bash
# 在 USV 机载计算机上
# Ctrl+C 停止现有节点
ros2 launch usv_bringup usv_launch.py namespace:=usv_02 ...
```

**方法 B: 仅重启 Sound 节点**
```bash
# 在 USV 机载计算机上
# 1. 找到 sound 节点进程
ps aux | grep usv_sound_node

# 2. 杀掉进程
kill <PID>

# 3. 重新启动
ros2 run usv_sound usv_sound_node --ros-args -r __ns:=/usv_02
```

### 3. 验证修复

**测试低电量触发**（使用模拟消息）:
```bash
# 在地面站或任意终端
# 模拟发送低电量模式消息
ros2 topic pub /usv_02/low_voltage_mode std_msgs/msg/Bool "data: true" --once

# 观察日志
ros2 topic echo /rosout | grep usv_sound
```

**期望输出**:
```
[ERROR] [usv_sound_node]: [!][!][!] 低电压模式触发！
[ERROR] [usv_sound_node]: [!] 自动启动低电量警告声音播放
[INFO] [usv_sound_node]: 声音循环播放已启动
[INFO] [usv_sound_node]: 循环播放: moon101.wav, 延时: Xs, 次数: Y
```

## 📊 修复效果对比

### Before（修复前）

| 条件 | LED | Sound | 问题 |
|------|-----|-------|------|
| 低电量 + 无 sound_start | ✅ 红色闪烁 | ❌ 无声音 | **主要问题** |
| 低电量 + 已 sound_start | ✅ 红色闪烁 | ✅ moon101.wav | 正常 |

### After（修复后）

| 条件 | LED | Sound | 改进 |
|------|-----|-------|------|
| 低电量 + 无 sound_start | ✅ 红色闪烁 | ✅ **自动播放 moon101.wav** | **已修复** |
| 低电量 + 已 sound_start | ✅ 红色闪烁 | ✅ moon101.wav | 保持正常 |

## 🔧 相关配置

### 低电量阈值配置

在 `usv_status_node.py` 中：
```python
LOW_BATTERY_THRESHOLD = 5.0   # 触发阈值：5%
RECOVER_THRESHOLD = 8.0       # 恢复阈值：8%（滞后设计）
```

### 声音文件配置

在 `usv_sound_node.py` 参数中：
```python
self.declare_parameter('sound_types', ['gaga101', 'gaga102', 'gaga103', 'gaga104'])
self.declare_parameter('moon_type', 'moon101')  # 低电量警告音
```

**音频文件位置**:
```
usv_sound/resource/
├── gaga101.wav  # 正常音效 1
├── gaga102.wav  # 正常音效 2
├── gaga103.wav  # 正常音效 3
├── gaga104.wav  # 正常音效 4
└── moon101.wav  # 低电量警告音 ⚠️
```

## 🐛 故障排查

### 问题 1: 修复后仍无声音

**可能原因**:
1. 声音文件缺失
2. PyAudio 初始化失败
3. 音频设备未连接

**检查方法**:
```bash
# 1. 检查声音文件
ls ~/usv_workspace/install/usv_sound/share/usv_sound/resource/*.wav

# 2. 检查日志
ros2 topic echo /rosout | grep -i "sound\|audio\|moon"

# 3. 测试音频设备
aplay -l  # 列出音频设备
```

### 问题 2: 声音循环启动但无播放

**可能原因**:
- `moon101.wav` 文件损坏或格式不支持

**检查方法**:
```bash
# 播放测试
aplay ~/usv_workspace/install/usv_sound/share/usv_sound/resource/moon101.wav

# 检查文件信息
file moon101.wav
```

### 问题 3: 低电量话题未发布

**可能原因**:
- `usv_status_node` 未运行
- 电池电压未达到触发阈值

**检查方法**:
```bash
# 查看电压和百分比
ros2 topic echo /usv_02/usv_status --field battery_voltage
ros2 topic echo /usv_02/usv_status --field battery_percentage

# 查看低电量话题
ros2 topic echo /usv_02/low_voltage_mode
```

## 📝 总结

### 修复内容

✅ 在 `usv_sound_node.py` 的 `low_voltage_mode_callback` 中添加自动启动声音循环的逻辑

### 修复效果

- ✅ 低电量模式下，即使未手动启动声音循环，也会自动播放警告音
- ✅ 与 LED 节点保持一致的响应逻辑
- ✅ 不影响正常模式下的声音播放

### 关键改进

**Before**: 依赖手动 `sound_start` → 容易遗漏 → 无声音警告  
**After**: 自动检测并启动 → 可靠触发 → 确保警告音播放

---

**修复日期**: 2025-11-07  
**影响版本**: ROS 2 Humble/Iron  
**修复文件**: `usv_sound/usv_sound/usv_sound_node.py`
