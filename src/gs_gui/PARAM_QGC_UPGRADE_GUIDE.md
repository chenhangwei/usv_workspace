# 参数管理功能 QGC 同级体验升级指南

## 概述

本指南说明如何将 USV 参数管理功能提升到 QGroundControl (QGC) 同级的用户体验。

## 已完成的改进

### 1. ✅ 参数元数据支持

**新增文件**：
- `gs_gui/param_metadata.py`: 参数元数据加载器
- `gs_gui/param_validator.py`: 参数值验证器

**功能特性**：
- 📋 参数描述、单位、范围、默认值
- 🔢 枚举值映射（如 GPS_TYPE: 0=None, 1=AUTO）
- 🎯 位掩码解释（如 ARMING_CHECK 各个位的含义）
- ⚠️ 重启提示（reboot_required 标记）
- 🔒 只读标记（read_only 标记）

**使用方法**：
```python
from gs_gui.param_metadata import get_param_metadata

# 获取参数元数据
metadata = get_param_metadata("GPS_TYPE")
print(f"描述: {metadata.description}")
print(f"单位: {metadata.unit}")
print(f"范围: {metadata.min_value} - {metadata.max_value}")
print(f"枚举值: {metadata.values}")
```

### 2. ✅ 参数值验证

**功能特性**：
- ✔️ 类型检查（整数参数只允许整数）
- ✔️ 范围验证（min_value ~ max_value）
- ✔️ 步进值检查（increment）
- ✔️ 智能值建议（suggest_valid_value）
- ⚠️ 警告级别（0-3级）
- 📝 警告消息生成

**使用方法**：
```python
from gs_gui.param_validator import ParamValidator

# 验证参数值
valid, error_msg = ParamValidator.validate(param, new_value)
if not valid:
    print(f"验证失败: {error_msg}")

# 获取建议值
suggested = ParamValidator.suggest_valid_value(param, input_value)

# 获取值描述
desc = ParamValidator.get_value_description(param, value)
print(f"GPS_TYPE=1 表示: {desc}")  # "AUTO（自动检测）"
```

### 3. ✅ 参数管理器集成

**修改文件**：`gs_gui/param_manager.py`

**改进点**：
- 自动加载参数元数据
- 参数值回调时合并元数据
- 缓存时保存元数据

## 后续改进任务

### Phase 3.1: 改进参数表格显示 🚧

**目标**：提升 UI 体验到 QGC 水平

**待实现功能**：

1. **参数值单元格改进**
   - [ ] 使用自定义编辑器（QSpinBox/QDoubleSpinBox/QComboBox）
   - [ ] 显示参数单位
   - [ ] 枚举参数使用下拉框
   - [ ] 位掩码参数使用复选框组

2. **工具提示增强**
   - [ ] 悬停显示完整描述
   - [ ] 显示有效范围
   - [ ] 显示默认值
   - [ ] 显示枚举值列表

3. **列布局调整**
   ```
   原来: [参数名称 | 当前值 | 原始值 | 分组 | 描述]
   改进: [参数名称 | 值（带单位）| 默认值 | 分组 | 简短描述]
   ```

4. **默认值对比**
   - [ ] 高亮非默认值的参数
   - [ ] 添加"重置为默认值"按钮

**实现示例**：
```python
# param_window.py 改进

def _setup_param_table(self):
    # 修改列定义
    headers = ["参数名称", "值", "默认值", "单位", "分组", "描述"]
    
    # 自定义编辑器
    self.param_table.setItemDelegateForColumn(1, ParamValueDelegate(self))

class ParamValueDelegate(QItemDelegate):
    """参数值自定义编辑器"""
    
    def createEditor(self, parent, option, index):
        param_name = self.get_param_name(index)
        param = self.param_manager.get_param(param_name)
        metadata = get_param_metadata(param_name)
        
        # 枚举参数使用下拉框
        if metadata and metadata.values:
            combo = QComboBox(parent)
            for value, label in metadata.values.items():
                combo.addItem(f"{value}: {label}", value)
            return combo
        
        # 整数参数使用 SpinBox
        elif param.param_type == ParamType.INTEGER:
            spinbox = QSpinBox(parent)
            if metadata:
                spinbox.setRange(
                    int(metadata.min_value or 0),
                    int(metadata.max_value or 999999)
                )
            return spinbox
        
        # 浮点数使用 DoubleSpinBox
        else:
            spinbox = QDoubleSpinBox(parent)
            if metadata:
                spinbox.setRange(
                    metadata.min_value or 0.0,
                    metadata.max_value or 999999.0
                )
                spinbox.setDecimals(6)
            return spinbox
```

### Phase 3.2: 参数导入/导出 🚧

**目标**：支持参数文件管理

**待实现功能**：

1. **导出格式**
   - [ ] QGC .param 格式（兼容 QGroundControl）
   - [ ] JSON 格式（带元数据）
   - [ ] INI 格式（ArduPilot 传统格式）

2. **导入功能**
   - [ ] 从文件加载参数
   - [ ] 预览差异
   - [ ] 选择性应用

3. **参数模板**
   - [ ] 预设配置模板（室内定位/GPS模式/调试模式）
   - [ ] 保存自定义模板
   - [ ] 快速应用

**实现示例**：
```python
# param_manager.py 添加方法

def export_to_param_file(self, file_path: str) -> bool:
    """导出为 QGC .param 格式"""
    try:
        with open(file_path, 'w') as f:
            # QGC 格式: 参数名  值  类型
            f.write("# USV Parameters\n")
            f.write(f"# Exported: {datetime.now()}\n")
            f.write(f"# Vehicle: {self.usv_namespace}\n\n")
            
            for param in sorted(self._params.values(), key=lambda p: p.name):
                type_id = 9 if param.param_type == ParamType.REAL else 6
                f.write(f"{param.name}\t{param.value}\t{type_id}\n")
        
        return True
    except Exception as e:
        self.logger.error(f"导出参数失败: {e}")
        return False

def import_from_param_file(self, file_path: str) -> Dict[str, float]:
    """从 .param 文件导入"""
    params_to_import = {}
    
    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            parts = line.split('\t')
            if len(parts) >= 2:
                param_name = parts[0]
                param_value = float(parts[1])
                params_to_import[param_name] = param_value
    
    return params_to_import
```

### Phase 3.3: 参数对比和差异显示 🚧

**待实现功能**：

1. **默认值对比**
   - [ ] 高亮显示非默认值
   - [ ] 批量重置为默认值

2. **多 USV 对比**
   - [ ] 选择两个 USV 对比参数
   - [ ] 高亮显示差异
   - [ ] 同步参数

3. **缓存对比**
   - [ ] 对比当前值与缓存值
   - [ ] 检测飞控端的修改

### Phase 3.4: 搜索和过滤增强 🚧

**待实现功能**：

1. **高级搜索**
   - [ ] 在描述中搜索
   - [ ] 模糊匹配
   - [ ] 正则表达式搜索

2. **智能过滤**
   - [ ] 仅显示已修改
   - [ ] 仅显示非默认值
   - [ ] 按警告级别过滤

3. **快速访问**
   - [ ] 常用参数收藏
   - [ ] 最近修改历史
   - [ ] 搜索历史记录

### Phase 3.5: 实时参数监控 🚧

**待实现功能**：

1. **参数变化监听**
   - [ ] 订阅 `/mavros/param/param_value` topic
   - [ ] 实时更新显示
   - [ ] 飞控端修改提示

2. **参数修改日志**
   - [ ] 记录所有参数修改
   - [ ] 时间戳和操作者
   - [ ] 回滚功能

## 完整的 QGC 同级功能清单

### ✅ 已实现
- [x] 参数读取
- [x] 参数修改
- [x] 分组展示
- [x] 基础搜索
- [x] 修改追踪
- [x] 批量保存
- [x] 参数缓存
- [x] 参数元数据
- [x] 值验证
- [x] 警告系统

### 🚧 进行中
- [ ] 改进的表格编辑器
- [ ] 工具提示显示
- [ ] 默认值对比

### 📋 待实现
- [ ] 参数导入/导出
- [ ] 参数模板
- [ ] 多 USV 对比
- [ ] 实时监控
- [ ] 修改历史
- [ ] 高级搜索
- [ ] 常用参数收藏

## 快速开始

### 1. 构建更新的代码

```bash
cd ~/usv_workspace
colcon build --packages-select gs_gui
source install/setup.bash
```

### 2. 测试元数据功能

```bash
# 在 Python 中测试
python3 -c "
from gs_gui.param_metadata import get_param_metadata
meta = get_param_metadata('GPS_TYPE')
print(f'描述: {meta.description}')
print(f'枚举值: {meta.values}')
"
```

### 3. 启动地面站

```bash
ros2 launch gs_bringup gs_launch.py
```

打开参数配置窗口，现在会看到：
- 参数描述（如果有）
- 参数单位
- 参数范围验证
- 修改警告

## 贡献指南

### 扩展参数元数据

编辑 `param_metadata.py` 的 `_load_built_in_metadata()` 方法：

```python
"NEW_PARAM": ParamMetadata(
    name="NEW_PARAM",
    display_name="Display Name",
    description="Short description",
    user_description="Detailed user-friendly description",
    unit="m",
    min_value=0.0,
    max_value=100.0,
    default_value=10.0,
    increment=0.1,
    values={
        0: "Option 1",
        1: "Option 2",
    },
    reboot_required=False
),
```

### 添加新的验证规则

编辑 `param_validator.py` 的 `get_warning_message()` 方法：

```python
elif param.name == 'YOUR_PARAM':
    messages.append("⚠️ Your custom warning")
```

## 参考资料

- **QGroundControl 参数管理**: https://docs.qgroundcontrol.com/master/en/SetupView/Parameters.html
- **ArduPilot 参数文档**: https://ardupilot.org/rover/docs/parameters.html
- **MAVLink 参数协议**: https://mavlink.io/en/services/parameter.html

---

**最后更新**: 2025-11-05  
**版本**: Phase 3.0  
**作者**: GitHub Copilot
