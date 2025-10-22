# Ground Station GUI 模块化重构说明

## 概述
`main_gui_app.py` 已经被重构为模块化设计，将原来的一个大文件（约1200行）拆分成多个功能模块，便于维护和管理。

## 模块结构

### 1. `table_manager.py` - 表格管理模块
**功能**: 负责集群和离群USV表格的显示和更新

**主要类**:
- `TableManager`: 表格管理器类

**主要方法**:
- `update_cluster_table()`: 更新集群表格
- `update_departed_table()`: 更新离群表格
- `refresh_table_header()`: 刷新表格表头
- `get_selected_usv_info()`: 获取选中行的USV信息
- `map_power_supply_status()`: 电池状态映射

### 2. `usv_commands.py` - USV命令模块
**功能**: 负责处理所有USV控制命令的发送

**主要类**:
- `USVCommandHandler`: USV命令处理器

**主要方法**:
- 集群命令: `set_cluster_arming()`, `cluster_disarming()`, `set_cluster_guided()`, `set_cluster_manual()`
- 离群命令: `departed_arming()`, `departed_disarming()`, `set_departed_guided()`, `set_departed_manual()`, `set_departed_arco()`
- 声音命令: `sound_start()`, `sound_stop()`
- 颈部命令: `neck_swinging()`, `neck_stop()`
- LED命令: `led_color_switching()`, `led_random_color()`, `led_select_color()`, `led_off()`
- Boot Pose命令: `set_boot_pose()`, `set_boot_pose_all()`

### 3. `cluster_task_manager.py` - 集群任务管理模块
**功能**: 负责集群任务的创建、执行、暂停和停止

**主要类**:
- `ClusterTaskManager`: 集群任务管理器

**主要方法**:
- `read_data_from_file()`: 从XML文件读取任务数据
- `toggle_task()`: 切换任务运行状态（运行/暂停）
- `start_task()`: 开始执行任务
- `stop_task()`: 停止任务
- `update_progress()`: 更新任务进度
- `is_task_active()`: 检查任务是否活动
- `get_button_text()`: 获取按钮文本

### 4. `usv_list_manager.py` - USV列表管理模块
**功能**: 负责管理集群和离群USV列表

**主要类**:
- `USVListManager`: USV列表管理器

**主要方法**:
- `add_to_cluster()`: 添加USV到集群列表
- `remove_from_cluster()`: 从集群列表移除USV
- `update_departed_list_status()`: 更新离群列表状态
- `update_cluster_list()`: 更新集群列表
- `update_online_list()`: 更新在线列表
- `extract_namespaces()`: 提取命名空间列表

### 5. `state_handler.py` - 状态处理模块
**功能**: 负责处理USV状态的接收、缓存和更新

**主要类**:
- `StateHandler`: 状态处理器

**主要方法**:
- `receive_state_callback()`: 接收状态回调
- `update_nav_status()`: 更新导航状态
- `get_usv_state()`: 获取USV状态
- `_flush_state_cache_to_ui()`: 刷新状态缓存到UI

**特性**:
- 使用状态缓存机制，避免高频UI更新
- 使用QTimer定时批量刷新UI（200ms间隔）

### 6. `ui_utils.py` - UI工具模块
**功能**: 负责UI辅助功能，包括信息显示、绘图窗口等

**主要类**:
- `UIUtils`: UI工具类

**主要方法**:
- `append_info()`: 追加信息到信息框
- `append_warning()`: 追加警告信息
- `show_usv_plot_window()`: 显示USV绘图窗口
- `start_rviz()`: 启动RViz2
- `update_selected_table_row()`: 更新选中行数据

**特性**:
- 使用缓冲机制限制日志输出频率（500ms间隔）
- 自动限制日志最大行数（500行）

### 7. `main_gui_app.py` - 主窗口模块（重构后）
**功能**: 主窗口类，整合所有模块

**主要类**:
- `MainWindow`: 主窗口类

**职责**:
- 初始化所有子模块
- 连接ROS信号
- 连接UI信号
- 提供简单的包装方法调用子模块功能

## 架构设计

```
MainWindow (主窗口)
    ├── UIUtils (UI工具)
    │   ├── 信息显示缓冲
    │   ├── 绘图窗口管理
    │   └── RViz启动
    │
    ├── TableManager (表格管理)
    │   ├── 集群表格
    │   └── 离群表格
    │
    ├── USVListManager (列表管理)
    │   ├── 集群列表
    │   ├── 离群列表
    │   └── 在线列表
    │
    ├── StateHandler (状态处理)
    │   ├── 状态缓存
    │   ├── 导航状态
    │   └── UI刷新定时器
    │
    ├── USVCommandHandler (命令处理)
    │   ├── 集群命令
    │   ├── 离群命令
    │   ├── 声音/颈部/LED命令
    │   └── Boot Pose命令
    │
    └── ClusterTaskManager (任务管理)
        ├── 任务数据读取
        ├── 任务执行控制
        └── 任务进度跟踪
```

## 优势

1. **代码组织清晰**: 每个模块负责单一功能，职责明确
2. **易于维护**: 修改某个功能只需关注对应模块
3. **可测试性强**: 每个模块可以独立测试
4. **可扩展性好**: 添加新功能时不影响现有模块
5. **性能优化**: 使用缓冲和定时器机制，避免高频UI更新

## 文件大小对比

- 原文件: `main_gui_app.py` (~1200行)
- 重构后:
  - `main_gui_app.py`: ~400行
  - `table_manager.py`: ~250行
  - `usv_commands.py`: ~200行
  - `cluster_task_manager.py`: ~220行
  - `usv_list_manager.py`: ~150行
  - `state_handler.py`: ~120行
  - `ui_utils.py`: ~170行

## 使用说明

重构后的代码与原代码功能完全相同，无需修改启动脚本。所有的API接口保持不变。

## 备份

原始文件已备份为 `main_gui_app_backup.py`，如需回滚可以使用：
```bash
cd /home/chenhangwei/usv_workspace/src/gs_gui/gs_gui
cp main_gui_app_backup.py main_gui_app.py
```

## 注意事项

1. 确保所有新模块文件都在 `gs_gui/` 目录下
2. 模块间的依赖关系已经优化，避免循环依赖
3. 所有模块都使用回调函数传递信息，保持松耦合
