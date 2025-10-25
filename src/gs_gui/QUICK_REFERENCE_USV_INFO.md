# USV 信息面板 - 快速参考卡

## 🚀 快速开始

### 1. 快速演示（最快）
```bash
cd /home/chenhangwei/usv_workspace/src/gs_gui
python3 scripts/demo_usv_info_panel.py
```
**效果**: 打开独立窗口，自动切换 4 种 USV 状态，展示所有功能。

### 2. 运行测试
```bash
cd /home/chenhangwei/usv_workspace/src/gs_gui
python3 -m pytest test/test_usv_info_panel.py -v
```
**效果**: 运行 8 个单元测试，验证功能正确性。

### 3. 自动集成（推荐）
```bash
cd /home/chenhangwei/usv_workspace/src/gs_gui
python3 scripts/integrate_usv_info_panel.py
cd /home/chenhangwei/usv_workspace
colcon build --packages-select gs_gui
source install/setup.bash
ros2 launch gs_bringup gs_launch.py
```
**效果**: 自动修改代码并集成到主程序。

---

## 📁 新增文件清单

```
✅ gs_gui/gs_gui/usv_info_panel.py              (670 行) - 核心组件
✅ gs_gui/test/test_usv_info_panel.py           (230 行) - 测试文件
✅ gs_gui/scripts/integrate_usv_info_panel.py   (350 行) - 自动集成
✅ gs_gui/scripts/demo_usv_info_panel.py        (140 行) - 演示脚本
✅ gs_gui/USV_INFO_PANEL_README.md              - 项目总结
✅ gs_gui/USV_INFO_PANEL_GUIDE.md               - 详细集成指南
✅ gs_gui/INTEGRATION_COMPLETE.md               - 完成总结
✅ gs_gui/QUICK_REFERENCE_USV_INFO.md           - 本文件（快速参考）
```

---

## 🎨 功能亮点

### 五大信息组
| 组名 | 颜色 | 内容 |
|------|------|------|
| 📌 基本信息 | 蓝色 | ID、模式、状态、解锁 |
| 🗺️ 位置信息 | 绿色 | X、Y、Z、Yaw |
| 🔋 电池信息 | 橙色 | 百分比、电压、电流 |
| 🛰️ GPS 信息 | 紫色 | 卫星数、精度 |
| 💨 速度信息 | 红色 | 地速、航向 |

### 智能颜色编码
- **模式**: GUIDED(绿) / MANUAL(橙) / AUTO(蓝)
- **电池**: >60%(绿) / 30-60%(橙) / <30%(红)
- **GPS**: >=10(绿) / 6-9(橙) / <6(红)
- **解锁**: 已解锁(红) / 已锁定(绿)

---

## 💡 常用命令

### 测试相关
```bash
# 可视化演示
python3 scripts/demo_usv_info_panel.py

# 单元测试
python3 -m pytest test/test_usv_info_panel.py -v

# 单个测试
python3 -m pytest test/test_usv_info_panel.py::TestUsvInfoPanel::test_update_full_state -v
```

### 集成相关
```bash
# 自动集成（会备份文件）
python3 scripts/integrate_usv_info_panel.py

# 查看集成报告
cat INTEGRATION_REPORT.txt

# 构建项目
cd /home/chenhangwei/usv_workspace
colcon build --packages-select gs_gui
source install/setup.bash

# 启动地面站
ros2 launch gs_bringup gs_launch.py
```

### 开发相关
```bash
# 检查语法错误
python3 -m py_compile gs_gui/usv_info_panel.py

# 代码格式化
black gs_gui/usv_info_panel.py --line-length=120

# Flake8 检查
flake8 gs_gui/usv_info_panel.py --max-line-length=120
```

---

## 📖 文档导航

| 文档 | 用途 | 何时阅读 |
|------|------|----------|
| `QUICK_REFERENCE_USV_INFO.md` | 快速参考 | ⭐ 现在！ |
| `USV_INFO_PANEL_README.md` | 项目总结 | 了解概况 |
| `USV_INFO_PANEL_GUIDE.md` | 详细集成指南 | 准备集成时 |
| `INTEGRATION_COMPLETE.md` | 完成总结 | 完成后回顾 |

---

## 🔧 自定义

### 修改颜色
编辑 `gs_gui/usv_info_panel.py`，在 `_create_*_group` 方法中：
```python
border: 2px solid #YOUR_COLOR;
```

### 添加新字段
1. 在 `_create_*_group` 中添加标签
2. 在 `update_state` 中添加更新逻辑
3. 在 `_clear_display` 中添加清空逻辑

### 修改更新频率
在 `state_handler.py` 中修改定时器间隔：
```python
self._refresh_timer.start(200)  # 毫秒
```

---

## ❓ 常见问题

**Q: 面板显示空白？**
```bash
# 检查布局
# 在 main_gui_app.py 中手动清除并添加
while self.ui.groupBox_3.layout().count():
    item = self.ui.groupBox_3.layout().takeAt(0)
    if item.widget():
        item.widget().deleteLater()
self.ui.groupBox_3.layout().addWidget(self.usv_info_panel)
```

**Q: 某些字段显示 "--"？**
- 正常现象，表示状态数据中缺少该字段
- 可以在 `usv_status_node.py` 中添加字段填充

**Q: 如何测试新面板？**
```bash
# 方法1: 独立演示
python3 scripts/demo_usv_info_panel.py

# 方法2: 单元测试
python3 -m pytest test/test_usv_info_panel.py -v

# 方法3: 集成测试
ros2 launch gs_bringup gs_launch.py
# 然后在 GUI 中选中 USV 查看
```

---

## 📊 改进对比

| 维度 | 原有 | 新版 | 提升 |
|------|------|------|------|
| 信息字段 | 5 | 15+ | ⬆️ 300% |
| 颜色编码 | ❌ | ✅ | ⬆️ 新功能 |
| 进度条 | ❌ | ✅ | ⬆️ 新功能 |
| 图标 | ❌ | ✅ | ⬆️ 新功能 |
| 单位标注 | ❌ | ✅ | ⬆️ 新功能 |
| 用户体验 | 单调 | 美观 | ⬆️ 显著 |

---

## 🎯 下一步

### 立即行动
1. ✅ 运行演示脚本查看效果
2. ✅ 运行测试验证功能
3. ✅ 执行自动集成
4. ✅ 构建并测试

### 可选增强
- 添加历史曲线图（matplotlib）
- 添加报警系统（低电量、GPS 差）
- 添加数据导出（JSON/CSV）
- 添加多 USV 对比视图

---

## 📞 获取帮助

1. **查看详细指南**: `USV_INFO_PANEL_GUIDE.md`
2. **查看测试用例**: `test/test_usv_info_panel.py`
3. **查看集成报告**: `INTEGRATION_REPORT.txt`（集成后生成）

---

**快速开始命令**:
```bash
cd /home/chenhangwei/usv_workspace/src/gs_gui
python3 scripts/demo_usv_info_panel.py  # 立即查看效果！
```

🎉 **享受全新的 USV 信息显示体验！** 🎉
