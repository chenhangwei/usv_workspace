# QGC 同级参数管理功能 - 快速开始

## 🎯 已实现功能（Phase 3.0）

### ✅ 参数元数据系统
- **30+ 内置参数定义**：GPS、ARMING、COMPASS、BATTERY、SYSID、LOG 等
- **完整元数据**：描述、单位、范围、默认值、步进值
- **枚举值支持**：GPS_TYPE、FRAME_TYPE 等
- **位掩码支持**：ARMING_CHECK、LOG_BITMASK 等
- **重启标记**：自动提示需要重启的参数

### ✅ 参数验证系统
- **类型检查**：整数/浮点数类型验证
- **范围验证**：最小值/最大值边界检查
- **步进验证**：参数增量合规性检查
- **值建议**：自动建议最近的有效值
- **4级警告系统**：
  - Level 0: 无警告
  - Level 1: 提示（普通修改）
  - Level 2: 警告（需重启/重要参数）
  - Level 3: 严重警告（危险操作）

### ✅ 智能值描述
- **枚举值解释**：自动显示枚举描述（如 GPS_TYPE=1 → "AUTO (自动检测)"）
- **位掩码解释**：自动解析位掩码组合（如 ARMING_CHECK=7 → "All + Barometer + Compass"）

---

## 🚀 快速测试

### 1. 运行演示脚本
```bash
cd /home/chenhangwei/usv_workspace
python3 src/gs_gui/scripts/demo_param_qgc_features.py
```

**预期输出**：
```
✓ 元数据已加载
参数: GPS_TYPE
  显示名称: GPS Type
  描述: GPS接收器类型
  单位: (无)
  范围: 0 ~ 19
  枚举值:
    0: None (无GPS)
    1: AUTO (自动检测)
    ...
```

### 2. 启动地面站
```bash
source install/setup.bash
ros2 launch gs_bringup gs_launch.py
```

### 3. 打开参数配置窗口
1. 在地面站 GUI 中点击 "参数配置" 按钮
2. 连接到在线 USV
3. 查看参数列表（现在已集成元数据）

---

## 💻 代码使用示例

### 获取参数元数据
```python
from gs_gui.param_metadata import get_param_metadata

# 获取 GPS 类型参数的元数据
meta = get_param_metadata("GPS_TYPE")
print(f"描述: {meta.description}")
print(f"单位: {meta.unit}")
print(f"范围: {meta.min_value} ~ {meta.max_value}")
print(f"默认值: {meta.default_value}")

# 枚举值
for value, desc in meta.values.items():
    print(f"{value}: {desc}")
```

### 验证参数值
```python
from gs_gui.param_manager import ParamInfo, ParamType
from gs_gui.param_validator import ParamValidator

# 创建参数对象
param = ParamInfo(
    name="GPS_TYPE",
    value=1.0,
    original_value=0.0,
    param_type=ParamType.INTEGER,
    min_value=0,
    max_value=19
)

# 验证新值
valid, error_msg = ParamValidator.validate(param, 25.0)
if not valid:
    print(f"验证失败: {error_msg}")
    # 获取建议值
    suggested = ParamValidator.suggest_valid_value(param, 25.0)
    print(f"建议值: {suggested}")
```

### 检查警告级别
```python
from gs_gui.param_validator import ParamValidator

# 检查修改 ARMING_CHECK 的警告级别
param = ParamInfo(name="ARMING_CHECK", ...)
level = ParamValidator.get_warning_level(param, 0.0)  # 禁用所有检查

if level >= 2:
    msg = ParamValidator.get_warning_message(param, 0.0)
    print(f"警告: {msg}")
```

### 获取值描述
```python
# GPS_TYPE 枚举值
param = ParamInfo(name="GPS_TYPE", ...)
desc = ParamValidator.get_value_description(param, 1.0)
print(desc)  # 输出: "AUTO (自动检测)"

# ARMING_CHECK 位掩码
param = ParamInfo(name="ARMING_CHECK", ...)
desc = ParamValidator.get_value_description(param, 7.0)
print(desc)  # 输出: "All (所有检查), Barometer (气压计), Compass (指南针)"
```

---

## 📋 当前状态总结

### ✅ 已完成（Phase 3.0）
| 功能 | 状态 | 文件 |
|------|------|------|
| 参数元数据定义 | ✅ | `param_metadata.py` (420 行) |
| 参数验证系统 | ✅ | `param_validator.py` (180 行) |
| 元数据集成 | ✅ | `param_manager.py` 已更新 |
| 演示脚本 | ✅ | `demo_param_qgc_features.py` |
| 升级指南 | ✅ | `PARAM_QGC_UPGRADE_GUIDE.md` |
| 构建验证 | ✅ | 编译成功无错误 |

### 🚧 待实现（Phase 3.1-3.5）
详见 `PARAM_QGC_UPGRADE_GUIDE.md` 中的实现路线图：
- **Phase 3.1**: UI 集成（自定义编辑器、工具提示、单位显示）
- **Phase 3.2**: 参数导入/导出（.param、JSON、INI 格式）
- **Phase 3.3**: 参数对比（默认值、USV 间对比）
- **Phase 3.4**: 高级搜索（描述搜索、正则表达式、过滤器）
- **Phase 3.5**: 实时监控（参数变化事件、修改日志）

---

## 🔧 故障排查

### 元数据未加载？
```python
from gs_gui.param_metadata import load_all_metadata
load_all_metadata()
print(f"已加载 {len(_param_metadata_db)} 个参数元数据")
```

### 验证失败？
检查参数类型、范围、步进值是否正确设置：
```python
param = ParamInfo(...)
print(f"类型: {param.param_type}")
print(f"范围: {param.min_value} ~ {param.max_value}")
print(f"步进: {param.increment}")
```

### 值描述为空？
确认参数是否有枚举值或位掩码定义：
```python
meta = get_param_metadata("PARAM_NAME")
print(f"枚举值: {meta.values if meta else '无'}")
print(f"位掩码: {meta.bitmask if meta else '无'}")
```

---

## 📚 相关文档

- **完整升级指南**: `PARAM_QGC_UPGRADE_GUIDE.md`
- **参数管理器设计**: `PARAM_MANAGER_DESIGN.md`
- **使用指南**: `PARAM_USAGE_GUIDE.md`
- **快速参考**: `PARAM_QUICK_REF.md`

---

## 🎉 下一步行动

1. **测试现有功能**：
   ```bash
   python3 src/gs_gui/scripts/demo_param_qgc_features.py
   ```

2. **启动地面站**：
   ```bash
   ros2 launch gs_bringup gs_launch.py
   ```

3. **体验元数据集成**：
   - 打开参数配置窗口
   - 查看参数描述（已集成到 tooltip）
   - 尝试修改参数（验证系统已激活）

4. **开始 Phase 3.1**：
   - 阅读 `PARAM_QGC_UPGRADE_GUIDE.md` 的 "Phase 3.1: UI 增强" 章节
   - 实现自定义参数编辑器
   - 添加工具提示和单位显示

**开发周期预估**：
- Phase 3.1 UI 集成：1-2 天
- Phase 3.2 导入/导出：1 天
- Phase 3.3 参数对比：0.5 天
- Phase 3.4 高级搜索：0.5 天
- Phase 3.5 实时监控：1 天

**总计**：4-5 天可达到完整 QGC 同级体验

---

**最后更新**: 2025-01-XX | **版本**: Phase 3.0 完成
