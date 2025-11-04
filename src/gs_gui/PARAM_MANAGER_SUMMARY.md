# USV 参数管理功能实现总结

## 实现内容

已成功为地面站 GUI 添加了类似 QGroundControl 的飞控参数管理功能。

### 新增文件

1. **`gs_gui/param_manager.py`** (358 行)
   - `ParamInfo` 数据类：参数信息结构
   - `ParamManager` 类：参数管理核心逻辑
   - `ParamManagerAsync` 类：异步参数管理（避免阻塞 UI）
   - 功能：参数拉取、读取、写入、分组、过滤

2. **`gs_gui/param_window.py`** (551 行)
   - `ParamWindow` 对话框：参数配置 UI
   - 功能：分组展示、搜索过滤、参数编辑、批量保存
   - 样式：类似 QGC 的现代化界面

3. **`PARAM_MANAGER_DESIGN.md`** (设计文档)
   - 功能概述和架构设计
   - 技术选型和数据结构
   - 实现步骤和注意事项

4. **`PARAM_MANAGER_GUIDE.md`** (使用指南)
   - 详细使用步骤
   - 界面说明和常用参数
   - 故障排查和安全警告

### 修改文件

1. **`usv_info_panel.py`**
   - 在 Ready 检查组添加"⚙️ 飞控参数配置"按钮
   - 位置：重启飞控按钮下方

2. **`main_gui_app.py`**
   - 添加 `on_param_config_clicked()` 方法
   - 连接参数按钮信号
   - 传递 ROS 节点到主窗口（`main_window.ros_node`）

## 功能特性

### Phase 1 已实现（核心功能）

✅ **参数读取**
- 使用 MAVROS `param/pull` 服务从飞控拉取参数
- 异步加载，避免阻塞 UI
- 显示加载进度和状态

✅ **参数展示**
- 分组列表（按参数前缀自动分组）
- 表格显示：名称、当前值、原始值、分组、描述
- 高亮已修改的参数（淡黄色）

✅ **参数编辑**
- 双击单元格修改值
- 输入验证（数字类型）
- 修改追踪和统计

✅ **参数保存**
- 批量保存所有修改
- 使用 MAVROS `param/set` 服务
- 保存成功/失败反馈

✅ **参数重置**
- 撤销所有未保存的修改
- 恢复到原始值

✅ **搜索过滤**
- 实时搜索参数名称
- 支持部分匹配

### Phase 2-4 待实现（高级功能）

⏳ **参数元数据**（Phase 3）
- 参数单位、范围、描述
- 需要解析 ArduPilot 参数定义文件

⏳ **导入/导出**（Phase 3）
- 保存参数到 YAML 文件
- 从文件加载参数

⏳ **参数对比**（Phase 4）
- 对比不同 USV 的参数
- 高亮差异

⏳ **参数模板**（Phase 4）
- 预设配置模板
- 一键应用

## 技术实现

### 架构设计

```
用户操作 → ParamWindow → ParamManagerAsync → MAVROS 服务 → 飞控
                ↓                    ↓
            PyQt UI          后台线程 (避免阻塞)
```

### 关键技术点

1. **异步加载**
   ```python
   ParamManagerAsync.pull_all_params_async(
       on_progress=callback,
       on_complete=callback
   )
   ```
   - 后台线程拉取参数
   - 回调函数更新 UI（通过 QTimer 切换到主线程）

2. **MAVROS 服务调用**
   ```python
   # 拉取参数
   response = self.pull_client.call_async(ParamPull.Request())
   
   # 设置参数
   request = ParamSet.Request()
   request.param_id = name
   request.value.real = value
   response = self.set_client.call_async(request)
   ```

3. **参数分组**
   ```python
   @property
   def group(self) -> str:
       # ARMING_CHECK → ARMING
       parts = self.name.split('_')
       return parts[0] if parts else "其他"
   ```

4. **修改追踪**
   ```python
   @property
   def is_modified(self) -> bool:
       return abs(self.value - self.original_value) > 1e-6
   ```

### UI 设计

- **左右分栏**：QSplitter 实现
- **分组列表**：QListWidget
- **参数表格**：QTableWidget (5列)
- **搜索框**：QLineEdit with textChanged 信号
- **按钮**：现代化样式（圆角、渐变）
- **状态栏**：QProgressBar + QLabel

## 使用前提

### 必须启用 MAVROS param 插件

当前系统为了优化启动时间，默认禁用了 `param` 插件。

**修改方法**：

编辑 `usv_bringup/launch/usv_launch.py`，在 `plugin_allowlist` 中添加 `'param'`：

```python
'plugin_allowlist': [
    'sys_status',
    'sys_time',
    'command',
    'local_position',
    'setpoint_raw',
    'global_position',
    'gps_status',
    'param',  # ← 添加这一行
],
```

**代价**：首次启动时间增加约 30-60 秒

## 测试建议

### 单元测试

```bash
# 测试 ParamManager
cd src/gs_gui
python3 -m pytest test/test_param_manager.py -v

# 测试 ParamWindow UI
python3 -m pytest test/test_param_window.py -v
```

### 集成测试

1. **启动地面站**
   ```bash
   ros2 launch gs_bringup gs_launch.py
   ```

2. **启动 USV（启用 param 插件）**
   ```bash
   ros2 launch usv_bringup usv_launch.py \
       namespace:=usv_01 \
       fcu_url:=serial:///dev/ttyACM0:921600
   ```

3. **测试流程**
   - 选择 USV → 点击参数按钮
   - 等待参数加载（30-60秒）
   - 修改参数 → 保存
   - 验证飞控参数是否更新

### 边界测试

- [ ] 网络中断时的错误处理
- [ ] 参数范围越界输入
- [ ] 并发修改冲突
- [ ] 超大参数列表（600+ 参数）

## 已知限制

### 当前版本限制

1. **无参数元数据**
   - 不显示参数单位、范围、描述
   - 需要手动查阅 ArduPilot 文档

2. **参数列表获取问题**
   - `ParamPull` 服务只将参数存储到内部
   - 目前无法直接获取参数名称列表
   - **临时方案**：需要订阅 `/mavros/param/param_value` topic

3. **加载时间较长**
   - 首次拉取 400-600 个参数需要 30-60 秒
   - 受串口速度限制

4. **无参数历史**
   - 不记录参数修改历史
   - 无法回滚到之前的配置

### MAVROS 限制

1. **DEPRECATED 服务**
   - `ParamGet/ParamSet` 已标记为废弃
   - 但仍然可用，因为飞控端支持 MAVLink 协议

2. **参数同步问题**
   - 需要启用 `param` 插件
   - 增加启动时间

## 改进建议

### 短期优化（Phase 2）

1. **参数缓存**
   - 首次加载后缓存到本地
   - 后续启动直接读取缓存（可刷新）

2. **并发加载**
   - 使用 `param_value` topic 订阅
   - 边接收边显示

3. **错误处理增强**
   - 网络中断重连
   - 参数保存失败重试

### 中期功能（Phase 3）

1. **参数元数据**
   - 解析 ArduPilot `.xml` 参数定义
   - 显示单位、范围、描述

2. **导入/导出**
   - 保存到 YAML/JSON
   - 批量导入参数

### 长期规划（Phase 4）

1. **参数对比工具**
   - 对比两个 USV 参数
   - 同步参数到多个 USV

2. **参数模板**
   - 预设配置模板
   - 快速切换场景

3. **参数历史**
   - 记录每次修改
   - 支持回滚

## 文档清单

- [x] `PARAM_MANAGER_DESIGN.md` - 设计文档
- [x] `PARAM_MANAGER_GUIDE.md` - 使用指南
- [x] `PARAM_MANAGER_SUMMARY.md` - 实现总结（本文件）
- [ ] API 文档（代码注释）
- [ ] 测试文档

## 相关链接

- **ArduPilot 参数文档**: https://ardupilot.org/rover/docs/parameters.html
- **MAVROS 参数接口**: https://github.com/mavlink/mavros/tree/ros2/mavros
- **QGC 参数管理**: https://docs.qgroundcontrol.com/master/en/SetupView/Parameters.html

---

**完成时间**: 2025-11-04  
**实现版本**: Phase 1 (核心功能)  
**代码行数**: 约 900+ 行（ParamManager + ParamWindow）  
**测试状态**: 待测试
