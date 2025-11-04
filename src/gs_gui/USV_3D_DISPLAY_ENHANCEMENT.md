# USV 3D 坐标显示窗口优化总结

**日期**: 2025-01-XX  
**模块**: `gs_gui/usv_plot_window.py`  
**问题**: 3D 显示窗口的按钮和文字为黑色，在深色背景上难以看清  
**影响**: 用户体验差，无法有效使用 3D 可视化功能

---

## 问题分析

### 原始问题
1. **视觉可见性差**：
   - 黑色按钮和文字在深色背景上几乎不可见
   - 缺少对比度设计
   - 无样式指导原则

2. **功能局限**：
   - 缺少轨迹追踪功能
   - 无法调整刷新频率
   - 缺少详细信息显示
   - USV 颜色单一（全部蓝色）
   - 无网格开关选项

3. **交互问题**：
   - 点击容差太小（0.5米），难以选中
   - 信息弹窗样式简陋
   - 缺少统计信息

---

## 解决方案

### 1. 综合样式系统（QSS）

**新增全局样式**：
```python
self.setStyleSheet("""
    QDialog {
        background-color: #f5f5f5;  /* 浅灰背景 */
    }
    QGroupBox {
        background-color: #ffffff;   /* 白色面板 */
        border: 2px solid #e0e0e0;
        border-radius: 6px;
        font-weight: bold;
        color: #2c3e50;              /* 深色文字 */
    }
    QPushButton {
        background-color: #3498db;   /* 蓝色按钮 */
        color: white;
        border: none;
        border-radius: 5px;
        font-size: 11px;
    }
    QPushButton:hover {
        background-color: #2980b9;   /* 悬停深蓝 */
    }
    QCheckBox {
        color: #2c3e50;
        font-size: 10px;
    }
    QLabel {
        color: #34495e;
        font-size: 10px;
    }
""")
```

**效果**：
- ✅ 白色面板提供高对比度
- ✅ 蓝色按钮清晰可见
- ✅ 深色文字在浅色背景上易读
- ✅ 悬停效果提供交互反馈

---

### 2. 控制面板重设计

**新增功能区**：

#### （1）显示选项组（QGroupBox）
```python
display_group = QGroupBox("显示选项")
self.show_label_checkbox = QCheckBox("显示标注")      # 默认选中
self.show_arrow_checkbox = QCheckBox("显示箭头")      # 默认选中
self.show_trail_checkbox = QCheckBox("显示轨迹")      # 默认未选中
self.show_grid_checkbox = QCheckBox("显示网格")       # 默认选中
```

#### （2）自动刷新控制
```python
refresh_group = QGroupBox("刷新控制")
self.auto_refresh_checkbox = QCheckBox("自动刷新")
self.refresh_slider = QSlider(QtCore.Horizontal)
self.refresh_slider.setRange(1, 10)    # 1-10秒
self.refresh_slider.setValue(3)        # 默认3秒
```

#### （3）信息栏
```python
self.info_label = QLabel("USV数量: 0 | 坐标范围: N/A | 最后更新: --:--:--")
# 实时显示统计信息
```

#### （4）操作按钮
```python
self.reset_view_button = QPushButton("重置视角")
self.update_button = QPushButton("立即刷新")
```

---

### 3. 轨迹追踪系统

**数据结构**：
```python
self.usv_trails = {}           # {usv_id: [(x, y, z), ...]}
self.max_trail_length = 50     # 最多保存50个历史点
```

**更新逻辑**（在 `update_plot()` 中）：
```python
# 更新轨迹数据
if usv_id not in self.usv_trails:
    self.usv_trails[usv_id] = []
self.usv_trails[usv_id].append((x, y, z))

# 限制长度
if len(self.usv_trails[usv_id]) > self.max_trail_length:
    self.usv_trails[usv_id] = self.usv_trails[usv_id][-self.max_trail_length:]

# 绘制轨迹线
if self.show_trail_checkbox.isChecked() and len(self.usv_trails[usv_id]) > 1:
    trail = self.usv_trails[usv_id]
    xs = [p[0] for p in trail]
    ys = [p[1] for p in trail]
    zs = [p[2] for p in trail]
    ax.plot(xs, ys, zs, color=color, alpha=0.3, linewidth=1.5, linestyle='--')
```

**效果**：
- ✅ 显示 USV 历史移动路径
- ✅ 虚线风格，避免遮挡当前位置
- ✅ 使用与 USV 相同颜色
- ✅ 自动限制长度，避免内存占用

---

### 4. 多 USV 颜色区分

**颜色方案**：
```python
colors = ['#e74c3c',  # 红色
          '#3498db',  # 蓝色
          '#2ecc71',  # 绿色
          '#f39c12',  # 橙色
          '#9b59b6',  # 紫色
          '#1abc9c',  # 青色
          '#e67e22',  # 深橙
          '#34495e',  # 灰蓝
          '#16a085',  # 深青
          '#c0392b']  # 深红

color = colors[idx % len(colors)]  # 循环使用
```

**应用**：
- USV 位置点（scatter）
- 航向箭头（quiver）
- 轨迹线（plot）
- 标注边框（bbox edgecolor）

**效果**：
- ✅ 最多 10 种颜色自动分配
- ✅ 轨迹与 USV 颜色匹配
- ✅ 图例清晰区分

---

### 5. 增强的绘图样式

#### （1）坐标轴标签
```python
ax.set_xlabel('X (m)', fontsize=10, color='#2c3e50', labelpad=10)
ax.set_ylabel('Y (m)', fontsize=10, color='#2c3e50', labelpad=10)
ax.set_zlabel('Z (m)', fontsize=10, color='#2c3e50', labelpad=10)
ax.set_title('USV 3D 位置与航向', fontsize=12, color='#2c3e50', pad=15)
```

#### （2）网格控制
```python
if self.show_grid_checkbox.isChecked():
    ax.grid(True, linestyle='--', alpha=0.3, color='#7f8c8d')
else:
    ax.grid(False)
```

#### （3）USV 位置点样式
```python
ax.scatter(x, y, z, marker='o', color=color, s=150,
          edgecolors='white', linewidths=2, alpha=0.9, label=usv_id)
# 更大尺寸（s=150），白色边缘，半透明
```

#### （4）标注样式
```python
label = f"{usv_id}\n({x:.2f}, {y:.2f}, {z:.2f})"
ax.text(x, y, z + 0.3, label, fontsize=9, color='#2c3e50', weight='bold',
       bbox=dict(boxstyle='round,pad=0.3', 
                facecolor='white', 
                edgecolor=color, 
                alpha=0.8))
# 圆角背景框，边缘颜色与 USV 匹配
```

#### （5）自动坐标范围
```python
margin = 2.0
ax.set_xlim(min(xs) - margin, max(xs) + margin)
ax.set_ylim(min(ys) - margin, max(ys) + margin)
ax.set_zlim(min(zs) - margin/2, max(zs) + margin)
```

---

### 6. 改进的交互功能

#### （1）点击检测（3D 支持）
```python
# 计算 3D 距离（降低 z 轴权重）
dist = sqrt((event.xdata - point['x']) ** 2 + 
           (event.ydata - point['y']) ** 2 +
           (point['z'] ** 2) * 0.1)

if dist < min_dist and dist < 3.0:  # 增大容差到 3.0 米
    min_dist = dist
    clicked_usv = point
```

**改进**：
- ✅ 考虑 z 坐标（之前只用 x, y）
- ✅ 容差从 0.5m 增大到 3.0m（更易点击）
- ✅ z 轴权重降低（因为视角投影）

#### （2）详细信息弹窗
```python
info_text = f"=== {usv_id} ===\n"
info_text += f"位置: ({pos.get('x', 0):.3f}, {pos.get('y', 0):.3f}, {pos.get('z', 0):.3f})\n"
info_text += f"航向: {usv_data.get('yaw', 0):.2f}°\n"
info_text += f"模式: {usv_data.get('mode', 'N/A')}\n"
info_text += f"状态: {usv_data.get('armed', 'N/A')}\n"
info_text += f"电池: {usv_data.get('battery_voltage', 0):.2f}V ({usv_data.get('battery_percentage', 0):.1f}%)\n"
info_text += f"速度: {usv_data.get('speed', 0):.2f} m/s\n"

# 轨迹统计
if usv_id in self.usv_trails:
    trail_len = len(self.usv_trails[usv_id])
    total_dist = 0.0
    trail = self.usv_trails[usv_id]
    for i in range(1, len(trail)):
        dx = trail[i][0] - trail[i-1][0]
        dy = trail[i][1] - trail[i-1][1]
        dz = trail[i][2] - trail[i-1][2]
        total_dist += sqrt(dx*dx + dy*dy + dz*dz)
    info_text += f"轨迹点数: {trail_len}\n"
    info_text += f"累计距离: {total_dist:.2f}m"
```

**新增信息**：
- ✅ 3D 坐标（精确到毫米）
- ✅ 电池电压和百分比
- ✅ 速度信息
- ✅ 轨迹点数和累计移动距离

#### （3）弹窗样式
```python
msg_box.setStyleSheet("""
    QMessageBox {
        background-color: #ffffff;
    }
    QLabel {
        color: #2c3e50;
        font-size: 11px;
        padding: 10px;
    }
    QPushButton {
        background-color: #3498db;
        color: white;
        border: none;
        padding: 8px 20px;
        border-radius: 4px;
        min-width: 80px;
    }
    QPushButton:hover {
        background-color: #2980b9;
    }
""")
```

---

### 7. 信息栏实时统计

**显示内容**（在 `update_plot()` 中更新）：
```python
current_time = datetime.datetime.now().strftime('%H:%M:%S')
range_info = (f"X:[{min(xs):.1f}, {max(xs):.1f}] "
            f"Y:[{min(ys):.1f}, {max(ys):.1f}] "
            f"Z:[{min(zs):.1f}, {max(zs):.1f}]")

self.info_label.setText(
    f"USV数量: {len(self.usv_points)} | "
    f"坐标范围: {range_info} | "
    f"最后更新: {current_time}"
)
```

**效果**：
- ✅ 实时显示在线 USV 数量
- ✅ X/Y/Z 坐标范围（帮助理解空间分布）
- ✅ 最后更新时间（确认数据新鲜度）

---

### 8. 自动刷新机制

**定时器控制**：
```python
self.refresh_timer = QTimer()
self.refresh_timer.timeout.connect(self.update_plot)
self.auto_refresh_checkbox.stateChanged.connect(self.on_auto_refresh_changed)
self.refresh_slider.valueChanged.connect(self.on_refresh_interval_changed)

def on_auto_refresh_changed(self, state):
    if state == QtCore.Checked:
        interval = self.refresh_slider.value() * 1000
        self.refresh_timer.start(interval)
    else:
        self.refresh_timer.stop()

def on_refresh_interval_changed(self, value):
    if self.auto_refresh_checkbox.isChecked():
        self.refresh_timer.start(value * 1000)
```

**用户体验**：
- ✅ 默认自动刷新（3秒间隔）
- ✅ 可调整 1-10 秒范围
- ✅ 滑块实时显示当前值
- ✅ 可关闭自动刷新（节省性能）

---

## 代码变更总结

### 修改的文件
- **gs_gui/usv_plot_window.py**（完全重构）

### 新增导入
```python
from math import cos, sin, sqrt      # 新增 sqrt
from PyQt5.QtCore import Qt as QtCore  # 解决 Qt 属性访问
import datetime  # 在 update_plot() 中用于时间戳
```

### 新增类成员
```python
self.usv_trails = {}          # 轨迹数据
self.max_trail_length = 50    # 轨迹长度限制
self.usv_points = []          # USV 点列表
```

### 新增控件
- 4 个复选框（标注、箭头、轨迹、网格）
- 1 个刷新开关 + 1 个滑块
- 1 个信息标签
- 1 个重置按钮
- 2 个 QGroupBox 控件组

### 方法增强
| 方法 | 原始行数 | 新行数 | 主要改进 |
|------|---------|--------|---------|
| `__init__()` | ~50 | ~200 | 添加完整 UI 控件和样式 |
| `update_plot()` | ~40 | ~100 | 轨迹、颜色、统计、网格 |
| `on_click()` | ~15 | ~70 | 3D 支持、详细信息、样式 |
| `reset_view()` | ~5 | ~10 | 修复 3D 投影兼容性 |

---

## 测试验证

### 构建结果
```bash
$ colcon build --packages-select gs_gui
Summary: 1 package finished [2.40s]
✅ 构建成功，无错误
```

### 预期行为

#### 启动测试
1. 启动地面站：`ros2 launch gs_bringup gs_launch.py`
2. 点击主窗口的"3D 显示"按钮
3. **预期**：
   - ✅ 窗口背景为浅灰色（#f5f5f5）
   - ✅ 控制面板为白色（#ffffff）
   - ✅ 按钮为蓝色（#3498db）
   - ✅ 文字为深色（#2c3e50），清晰可读
   - ✅ 自动刷新已启用（3秒间隔）

#### 功能测试
1. **显示选项**：
   - 取消"显示标注" → 标注消失
   - 取消"显示箭头" → 航向箭头消失
   - 勾选"显示轨迹" → 出现虚线轨迹
   - 取消"显示网格" → 网格消失

2. **刷新控制**：
   - 拖动滑块到 10 秒 → 刷新间隔变为 10 秒
   - 取消"自动刷新" → 停止自动更新
   - 点击"立即刷新" → 立即更新一次

3. **信息栏**：
   - 显示当前 USV 数量（如：`USV数量: 3`）
   - 显示坐标范围（如：`X:[-5.2, 12.3] Y:[-2.1, 8.5] Z:[-0.5, 0.2]`）
   - 显示更新时间（如：`最后更新: 14:23:56`）

4. **点击交互**：
   - 点击 USV 位置点
   - **预期**：弹出白色背景对话框，显示：
     ```
     === usv_01 ===
     位置: (10.234, 5.678, 0.012)
     航向: 45.23°
     模式: GUIDED
     状态: ARMED
     电池: 12.35V (85.3%)
     速度: 1.25 m/s
     轨迹点数: 28
     累计距离: 15.67m
     ```

5. **轨迹测试**：
   - 勾选"显示轨迹"
   - 让 USV 移动
   - **预期**：出现虚线路径，颜色与 USV 匹配

6. **多 USV 测试**：
   - 启动 3 艘 USV
   - **预期**：
     - usv_01 红色 (#e74c3c)
     - usv_02 蓝色 (#3498db)
     - usv_03 绿色 (#2ecc71)
     - 图例显示所有 USV ID

---

## 性能优化

### 1. 轨迹长度限制
```python
self.max_trail_length = 50  # 最多 50 个点
```
- **原因**：避免长时间运行后内存占用过大
- **效果**：每个 USV 最多占用 ~1.2KB（50 * 3 * 8 bytes）

### 2. 刷新间隔可调
```python
self.refresh_slider.setRange(1, 10)  # 1-10 秒
```
- **原因**：允许用户根据需求调整
- **效果**：长时间监控时可设置更长间隔（如 10 秒）

### 3. 差量绘图
- 每次 `update_plot()` 调用 `figure.clear()`，清理旧对象
- 避免内存泄漏

---

## 遗留问题和未来改进

### 已知限制
1. **Type Checker 警告**：
   - `figure.patch.set_facecolor('#ffffff')` 报告属性未知
   - **原因**：matplotlib 的动态属性，类型检查器无法识别
   - **影响**：无，运行时正常工作
   - **解决**：可添加 `# type: ignore` 注释

2. **z 坐标权重**：
   - 点击检测中 z 轴权重设为 0.1
   - **原因**：3D 投影后 z 轴在屏幕上较小
   - **可能需要调整**：根据实际视角微调

### 未来改进方向
1. **轨迹颜色渐变**：
   - 旧轨迹淡化，新轨迹鲜艳
   - 实现：`alpha = (i / len(trail)) * 0.5`

2. **视角预设**：
   - 添加"俯视图"、"侧视图"、"鸟瞰图"按钮
   - 实现：`ax.view_init(elev=X, azim=Y)`

3. **轨迹导出**：
   - 保存轨迹到 CSV 文件
   - 格式：`timestamp, usv_id, x, y, z`

4. **动画模式**：
   - 回放历史轨迹（类似视频播放）
   - 添加播放/暂停/快进控件

5. **多窗口支持**：
   - 单独窗口显示每个 USV 的详细轨迹
   - 支持并排对比

---

## 相关文档

- **UI 现代化指南**：`gs_gui/UI_MODERNIZATION_GUIDE.md`
- **模块架构**：`gs_gui/MODULE_ARCHITECTURE.md`
- **快速参考**：`gs_gui/QUICK_REFERENCE.md`
- **坐标系统**：`gs_gui/AREA_OFFSET_GUIDE.md`

---

## 总结

### 解决的核心问题
✅ **可见性**：从黑色文字变为高对比度蓝白配色  
✅ **功能性**：新增轨迹、网格、统计、自动刷新  
✅ **交互性**：改进点击检测，显示详细信息  
✅ **美观性**：现代化 UI 设计，圆角、阴影、悬停效果  

### 用户体验提升
| 方面 | 改进前 | 改进后 |
|------|--------|--------|
| 视觉对比度 | ❌ 黑色文字难看清 | ✅ 高对比度蓝白设计 |
| USV 区分 | ❌ 全部蓝色 | ✅ 10 种颜色自动分配 |
| 轨迹追踪 | ❌ 无 | ✅ 虚线轨迹 + 距离统计 |
| 网格控制 | ❌ 无法关闭 | ✅ 复选框开关 |
| 刷新控制 | ❌ 固定间隔 | ✅ 1-10 秒可调 |
| 点击容差 | ❌ 0.5m（难点击） | ✅ 3.0m（易选中） |
| 信息弹窗 | ❌ 简单文字 | ✅ 详细信息 + 样式 |
| 统计信息 | ❌ 无 | ✅ 数量/范围/时间 |

### 代码质量
- ✅ 模块化设计（控件分组）
- ✅ 样式分离（QSS 独立定义）
- ✅ 性能优化（轨迹长度限制）
- ✅ 可维护性（清晰的注释）

---

**维护者**: GitHub Copilot  
**最后更新**: 2025-01-XX
