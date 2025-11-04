# USV 参数管理功能设计文档

## ⚠️ 实现状态

**当前版本 (v1.0.0)**: 功能暂不可用

由于 MAVROS param 插件的技术限制，参数管理功能暂时无法正常工作：

### 技术障碍

1. **ParamPull 阻塞问题**
   - `ParamPull` 服务会阻塞 30-60 秒等待参数同步
   - `rclpy.spin_until_future_complete()` 在后台线程中无法正常工作
   - 会导致整个 ROS 节点被阻塞

2. **参数列表获取问题**
   - MAVROS 将参数存储在节点内部
   - 无法通过 ROS 2 参数 API 直接访问
   - 需要订阅 `/mavros/param/param_value` topic 逐个接收

3. **异步实现复杂度**
   - 需要重构为完全异步的实现
   - 需要处理参数同步的各种边界情况
   - 开发时间超出预期

### 推荐替代方案

在功能完善前，请使用以下工具管理飞控参数：

1. **QGroundControl** (推荐) - 功能最完善
2. **Mission Planner** - Windows 用户
3. **命令行方式** - 高级用户

详见 `PARAM_QUICK_REF.md`

---

## 1. 功能概述 (原始设计)

为地面站 GUI 添加类似 QGroundControl (QGC) 的飞控参数管理功能，允许用户读取、修改和保存无人船飞控参数。

### 1.1 核心功能

- **参数读取**：从选中的 USV 飞控读取所有参数
- **参数写入**：修改参数值并写入飞控
- **分组展示**：按参数前缀（如 `ARMING_`, `GPS_`, `EK2_`）分组
- **搜索过滤**：快速搜索参数名称或描述
- **值验证**：输入值范围检查和类型验证
- **批量操作**：支持导出/导入参数文件（YAML）
- **实时状态**：显示参数修改状态（已修改、未保存）

### 1.2 参考设计

参考 QGroundControl 参数界面：
- 左侧：参数分组列表
- 右侧：参数详情表格（名称、值、单位、描述）
- 顶部：搜索框、刷新、保存按钮
- 底部：状态栏（显示加载进度、修改数量）

## 2. 技术架构

### 2.1 模块划分

```
gs_gui/
├── gs_gui/
│   ├── param_manager.py         # 参数管理器（ROS 服务调用）
│   ├── param_window.py          # 参数窗口 UI
│   ├── param_model.py           # 参数数据模型（QAbstractTableModel）
│   ├── usv_info_panel.py        # 修改：添加参数按钮
│   └── ground_station_node.py   # 修改：添加参数服务客户端
```

### 2.2 MAVROS 参数服务

虽然 MAVROS 的 `ParamGet/ParamSet` 服务已被标记为 **DEPRECATED**，但我们仍然可以使用，因为：
1. 飞控端仍然支持 MAVLink PARAM_REQUEST_READ/PARAM_SET 消息
2. 新的 ROS 2 参数 API 不适用于跨网络的飞控参数管理
3. QGC 也使用类似的 MAVLink 参数协议

**可用服务**：
```bash
# MAVROS 提供的参数服务（在 /usv_XX/mavros/param/ 命名空间下）
- param/pull           # 从飞控拉取所有参数到 ROS 参数服务器
- param/push           # 推送 ROS 参数服务器到飞控
- param/get            # 获取单个参数值
- param/set            # 设置单个参数值
- param/set_param      # 设置参数（新版本）
```

**注意**：由于我们在 `usv_launch.py` 中已经禁用了 `param` 插件（优化启动时间），需要重新启用才能使用参数服务。

## 3. 数据结构

### 3.1 参数数据模型

```python
@dataclass
class ParamInfo:
    """飞控参数信息"""
    name: str              # 参数名称（如 ARMING_CHECK）
    value: float           # 当前值
    original_value: float  # 原始值（用于检测修改）
    param_type: str        # 参数类型（int/float）
    description: str       # 参数描述
    unit: str              # 单位（如 m, deg, %）
    min_value: float       # 最小值
    max_value: float       # 最大值
    increment: float       # 步进值
    group: str             # 参数组（从名称提取）
    
    @property
    def is_modified(self) -> bool:
        """是否已修改"""
        return self.value != self.original_value
```

### 3.2 参数分组

从参数名称自动提取分组：
```python
# 示例参数分组
ARMING_CHECK → ARMING
GPS_TYPE → GPS
EK2_ALT_SOURCE → EK2
BATT_CAPACITY → BATT
```

## 4. UI 设计

### 4.1 参数窗口布局

```
┌─────────────────────────────────────────────────────────────────┐
│ USV-01 飞控参数配置                                    [─][□][×]│
├─────────────────────────────────────────────────────────────────┤
│  搜索: [__________________] [🔍]  [🔄刷新] [💾保存] [📥导入] [📤导出] │
├───────────┬─────────────────────────────────────────────────────┤
│           │ 参数名称       │  值    │ 单位 │ 描述                │
│  分组列表  ├─────────────────────────────────────────────────────┤
│           │ ARMING_CHECK   │ 1.000  │      │ Arming checks      │
│ ☑ ARMING  │ ARMING_VOLT_MIN│ 10.500 │  V   │ Minimum voltage    │
│ ☑ GPS     ├─────────────────────────────────────────────────────┤
│ ☑ EK2     │ GPS_TYPE       │ 1.000  │      │ GPS type           │
│ ☑ BATT    │ GPS_AUTO_SWITCH│ 1.000  │      │ Auto switch        │
│ ☐ ...     ├─────────────────────────────────────────────────────┤
│           │                                                      │
│           │                                                      │
└───────────┴─────────────────────────────────────────────────────┘
│ 状态: 已加载 456 个参数 | 已修改 3 个参数 | 未保存                │
└─────────────────────────────────────────────────────────────────┘
```

### 4.2 参数编辑交互

1. **双击单元格**：进入编辑模式
2. **值验证**：输入时检查范围和类型
3. **颜色标识**：
   - 白色：未修改
   - 黄色：已修改未保存
   - 绿色：保存成功
   - 红色：验证失败/保存失败

### 4.3 USV 信息面板集成

在 `usv_info_panel.py` 的 Ready 检查组下方添加参数按钮：

```python
# 参数配置按钮
self.param_button = QPushButton("⚙️ 飞控参数配置")
self.param_button.setEnabled(True)
self.param_button.clicked.connect(self._on_param_button_clicked)
```

## 5. 实现步骤

### 5.1 Phase 1: 基础参数读取（核心功能）

- [x] 创建 `ParamManager` 类
- [x] 实现 `pull_all_params()` 方法
- [x] 创建参数窗口 UI
- [x] 添加参数按钮到信息面板

### 5.2 Phase 2: 参数编辑和保存

- [ ] 实现参数值编辑
- [ ] 实现 `set_param()` 方法
- [ ] 添加修改状态追踪
- [ ] 实现批量保存

### 5.3 Phase 3: 高级功能

- [ ] 参数分组展示
- [ ] 搜索过滤
- [ ] 导入/导出 YAML
- [ ] 参数描述和单位（需要参数元数据文件）

### 5.4 Phase 4: 优化和测试

- [ ] 添加加载进度条
- [ ] 错误处理和重试
- [ ] 测试用例
- [ ] 用户文档

## 6. 注意事项

### 6.1 MAVROS param 插件启用

当前 `usv_launch.py` 中已禁用 `param` 插件，需要修改：

**选项 A**：为需要参数管理的 USV 单独启用
```python
'plugin_allowlist': [
    # ... 现有插件 ...
    'param',  # 添加参数插件（按需启用）
]
```

**选项 B**：使用 MAVLink 消息直接通信（不依赖 MAVROS param 插件）
- 发送 `PARAM_REQUEST_LIST` 获取所有参数
- 发送 `PARAM_SET` 设置参数值
- 更复杂但避免启动时间增加

**推荐**：选项 A，在需要时通过 launch 参数动态启用

### 6.2 参数元数据

ArduPilot 参数描述、单位、范围等元数据需要从外部获取：
- 方案 1：解析 ArduPilot 源码中的 `.xml` 参数定义文件
- 方案 2：使用预生成的参数元数据 JSON
- 方案 3：仅显示参数名称和值（最简单，后续扩展）

**推荐**：Phase 1 只显示名称和值，Phase 3 添加元数据

### 6.3 性能考虑

- 参数数量：ArduPilot Rover 约 400-600 个参数
- 加载时间：首次拉取需要 30-60 秒（串口速度受限）
- 解决方案：
  - 显示加载进度条
  - 后台线程加载，避免 UI 冻结
  - 缓存参数到本地（可选）

## 7. 测试计划

### 7.1 单元测试
- `ParamManager.pull_all_params()` 读取测试
- `ParamManager.set_param()` 写入测试
- 参数值验证测试

### 7.2 集成测试
- 与真实飞控通信测试
- UI 交互测试
- 多 USV 切换测试

### 7.3 边界测试
- 网络中断处理
- 参数范围越界
- 并发修改冲突

## 8. 文档

### 8.1 用户文档
- 参数管理功能使用指南
- 常用参数说明
- 故障排查

### 8.2 开发文档
- API 文档
- 模块架构图
- 扩展指南

---

**创建时间**: 2025-11-04  
**作者**: GitHub Copilot  
**状态**: 设计中 → 实现中
