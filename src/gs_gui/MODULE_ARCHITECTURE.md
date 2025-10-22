# Ground Station GUI 模块关系图

## 文件对比

### 重构前
```
main_gui_app.py (70KB, ~1200行)
└── 包含所有功能
```

### 重构后
```
main_gui_app.py (19KB, ~400行)          # 主窗口，整合所有模块
├── table_manager.py (11KB)             # 表格管理
├── usv_commands.py (9.3KB)             # 命令处理
├── cluster_task_manager.py (13KB)      # 任务管理
├── usv_list_manager.py (5.8KB)         # 列表管理
├── state_handler.py (4.0KB)            # 状态处理
└── ui_utils.py (7.3KB)                 # UI工具

总计: ~69KB (与原文件大小相当，但更易维护)
```

## 模块依赖关系

```
┌─────────────────────────────────────────────────────────────┐
│                     main_gui_app.py                         │
│                      (MainWindow)                           │
│                                                             │
│  • 整合所有模块                                              │
│  • 连接ROS信号                                              │
│  • 连接UI信号                                               │
│  • 提供包装方法                                              │
└─────┬───────┬───────┬───────┬───────┬───────────────────────┘
      │       │       │       │       │
      ▼       ▼       ▼       ▼       ▼
   ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐
   │ UI  │ │Table│ │List │ │State│ │Task │
   │Utils│ │ Mgr │ │ Mgr │ │ Hdr │ │ Mgr │
   └──┬──┘ └──┬──┘ └──┬──┘ └──┬──┘ └─────┘
      │       │       │       │
      │       │       │       │
   ┌──┴───────┴───────┴───────┴──────┐
   │     USVCommandHandler            │
   │   (命令发送到ROS节点)             │
   └──────────────────────────────────┘
```

## 详细模块说明

### 1. MainWindow (main_gui_app.py)
**角色**: 协调者
- 初始化所有子模块
- 连接ROS和UI信号
- 协调模块间交互

**依赖**:
- UIUtils
- TableManager
- USVListManager
- StateHandler
- USVCommandHandler
- ClusterTaskManager

---

### 2. UIUtils (ui_utils.py)
**角色**: UI助手
- 信息显示（带缓冲）
- 绘图窗口管理
- 选中行更新

**特性**:
- 日志缓冲队列
- 500ms刷新间隔
- 最大500行限制

**依赖**: 无（独立模块）

---

### 3. TableManager (table_manager.py)
**角色**: 表格控制器
- 管理集群/离群表格
- 格式化数据显示
- 提供行选择接口

**特性**:
- 差量更新（只更新变化的单元格）
- 自动排序
- 电池状态映射

**依赖**: 无（独立模块）

---

### 4. USVListManager (usv_list_manager.py)
**角色**: 列表管理器
- 维护三个列表：集群/离群/在线
- 处理USV在列表间的移动
- 自动更新列表状态

**特性**:
- 自动同步在线状态
- 命名空间提取
- 列表一致性保证

**依赖**: 无（独立模块）

---

### 5. StateHandler (state_handler.py)
**角色**: 状态同步器
- 接收ROS状态更新
- 缓存USV状态
- 定时刷新UI

**特性**:
- 状态缓存机制
- 200ms定时刷新
- 脏标记优化

**依赖**:
- TableManager（用于更新表格）
- USVListManager（用于更新列表）

---

### 6. USVCommandHandler (usv_commands.py)
**角色**: 命令发送器
- 封装所有USV命令
- 统一命令接口
- 处理命令反馈

**命令分类**:
- 集群命令（解锁/加锁/模式切换）
- 离群命令（解锁/加锁/模式切换）
- 外设命令（声音/颈部/LED）
- 特殊命令（Boot Pose）

**依赖**: 无（独立模块）

---

### 7. ClusterTaskManager (cluster_task_manager.py)
**角色**: 任务协调器
- XML任务文件读取
- 任务生命周期管理
- 进度跟踪

**特性**:
- 错误处理完善
- 状态机管理
- 进度回调

**依赖**: 无（独立模块）

---

## 信号流向

### 状态更新流程
```
ROS Node → receive_state_callback
         ↓
    StateHandler (缓存)
         ↓ (200ms定时)
    flush_state_cache_to_ui
         ↓
    USVListManager.update_online_list()
         ↓
    TableManager.update_cluster_table()
    TableManager.update_departed_table()
```

### 命令发送流程
```
UI Button Click
         ↓
    MainWindow.xxx_command()
         ↓
    USVCommandHandler.xxx()
         ↓
    ros_signal.xxx_command.emit()
         ↓
    ROS Node
```

### 任务执行流程
```
Read XML File
         ↓
    ClusterTaskManager.read_data_from_file()
         ↓
    cluster_position_list
         ↓
    User clicks Start
         ↓
    toggle_task() / start_task()
         ↓
    ros_signal.cluster_target_point_command.emit()
         ↓
    ROS Node executes task
         ↓
    Progress updates
         ↓
    ClusterTaskManager.update_progress()
```

## 性能优化

### 1. UI刷新优化
- **问题**: 高频状态更新导致UI卡顿
- **解决**: 
  - StateHandler使用200ms定时器批量刷新
  - UIUtils使用500ms定时器批量输出日志
  - TableManager只更新变化的单元格

### 2. 内存优化
- **问题**: 日志无限增长占用内存
- **解决**: UIUtils限制最大500行日志

### 3. 数据同步优化
- **问题**: 列表状态不一致
- **解决**: StateHandler统一管理状态缓存，使用脏标记减少不必要的更新

## 扩展性

### 添加新命令
1. 在 `usv_commands.py` 中添加新方法
2. 在 `main_gui_app.py` 中添加包装方法
3. 连接UI按钮信号

### 添加新表格
1. 在 `table_manager.py` 中添加新表格管理方法
2. 在 `main_gui_app.py` 中初始化新表格
3. 在 `state_handler.py` 中添加更新逻辑

### 添加新任务类型
1. 创建新的任务管理器类（参考 `cluster_task_manager.py`）
2. 在 `main_gui_app.py` 中集成新管理器
3. 添加相应的UI控件和信号

## 测试建议

### 单元测试
每个模块都可以独立测试：
```python
# 测试 TableManager
def test_table_manager():
    table_view = MockTableView()
    manager = TableManager(table_view, table_view)
    manager.update_cluster_table([test_data], {})
    assert table_view.model().rowCount() == 1

# 测试 USVCommandHandler
def test_command_handler():
    signal = MockSignal()
    handler = USVCommandHandler(signal, print)
    handler.set_cluster_arming([{'namespace': 'usv1'}])
    assert signal.arm_command_called
```

### 集成测试
测试模块间的协作：
```python
def test_state_update_flow():
    # 模拟状态更新流程
    state_handler.receive_state_callback([test_state])
    # 等待定时器触发
    time.sleep(0.3)
    # 验证表格已更新
    assert table_manager.model.rowCount() > 0
```

## 维护建议

1. **保持模块独立**: 避免模块间直接调用，使用回调函数
2. **统一错误处理**: 所有模块都应该妥善处理异常
3. **文档更新**: 修改模块时更新相应文档
4. **代码审查**: 确保新功能符合模块职责
5. **性能监控**: 注意定时器和缓冲的性能影响
