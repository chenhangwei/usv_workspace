# 任务坐标系偏移量设置功能说明

## 功能概述

该功能允许用户通过GUI界面设置任务坐标系原点（Area Center）在全局地图坐标系中的位置偏移量，无需修改任务文件和USV本地坐标系。

## 坐标系统架构

系统使用三层坐标变换：

```
XML任务文件坐标 (Area相对坐标)
           ↓  [+area_center]
    全局地图坐标 (Map全局坐标，以A0基站为原点)
           ↓  (无需转换，全局坐标=USV本地坐标)
  USV本地坐标 (也以A0基站为原点，通过set_home设置)
```

### 1. Area坐标系
- **定义**: XML任务文件中使用的相对坐标系
- **特点**: 以任务区域中心为原点(0,0,0)
- **用途**: 便于编写可重用的任务文件

### 2. Map全局坐标系
- **定义**: 全局地图的绝对坐标系
- **特点**: 固定的世界坐标系
- **用途**: 统一各USV的参考坐标系

### 3. USV本地坐标系
- **定义**: 每艘USV的本地坐标系
- **特点**: **也以定位基站A0为原点**（通过set_home设置）
- **用途**: 飞控实际执行的坐标系，与全局坐标系完全相同

## 使用方法

### 通过GUI设置偏移量

1. **打开设置对话框**
   - 启动地面站GUI
   - 点击菜单栏 → `坐标系设置` → `设置任务坐标系偏移量`

2. **输入偏移量**
   - **X偏移量**: Area Center在全局Map坐标系中的X坐标（单位：米）
   - **Y偏移量**: Area Center在全局Map坐标系中的Y坐标（单位：米）
   - **Z偏移量**: Area Center在全局Map坐标系中的Z坐标（单位：米）

3. **确认设置**
   - 点击"确定"按钮保存设置
   - 点击"取消"按钮放弃修改
   - 点击"重置为0"按钮将所有偏移量设为0

### 通过配置文件设置（可选）

编辑 `gs_bringup/config/gs_params.yaml`：

```yaml
main_gui_app:
  ros__parameters:
    # 任务坐标系原点在全局地图中的位置
    area_center_x: 100.0   # 例如：场地中心在全局坐标系的X位置
    area_center_y: 50.0    # 例如：场地中心在全局坐标系的Y位置
    area_center_z: 0.0     # 例如：场地中心在全局坐标系的Z位置
```

## 使用场景示例

### 场景1：湖面任务
- **任务文件**: 定义相对坐标，如三角形巡航 (0,0) → (10,0) → (5,8.66)
- **Area Center设置**: 
  - X: 120.5 (湖中心GPS坐标)
  - Y: 45.3
  - Z: 0.0
- **结果**: USV会在全局坐标系中正确的位置执行三角形巡航

### 场景2：多个场地切换
- **优势**: 同一套任务文件可用于不同场地
- **操作**: 只需修改Area Center偏移量，任务文件无需改动
- **示例**:
  - 场地A: area_center = (100, 50, 0)
  - 场地B: area_center = (200, 80, 0)

## 坐标转换实现

### 代码位置
- **转换函数**: `gs_gui/cluster_controller.py`
  - `_area_to_global()`: Area → Global
  - `_global_to_usv_local()`: Global → USV Local

### 转换公式

**Area → Global:**
```python
global_x = area_center_x + area_x
global_y = area_center_y + area_y
global_z = area_center_z + area_z
```

**Global → USV Local:**
```python
# 全局坐标系 = USV本地坐标系（都以A0为原点）
# 无需转换，直接返回
local_x = global_x
local_y = global_y
local_z = global_z
```

## 注意事项

1. **坐标单位**: 所有坐标单位为米(m)

2. **生效时机**: 
   - GUI设置立即生效
   - 影响后续发送的所有导航目标
   - 已发送的目标点不受影响

3. **坐标系一致性**:
   - 确保所有USV通过set_home设置相同的Home点（A0基站坐标）
   - Area Center、任务文件和全局地图使用相同的坐标系定义
   - 建议使用GPS坐标或本地ENU坐标系

5. **调试建议**:
   - 可以通过日志查看坐标转换过程
   - 建议先用小范围测试验证偏移量设置

## 故障排查

### 问题1: USV导航到错误位置
- **原因**: Area Center设置不正确或Home点未正确设置
- **解决**: 
  1. 检查全局地图坐标系定义
  2. 验证Area Center坐标值
  3. 确认所有USV的Home点都设置为A0基站坐标

### 问题2: 偏移量设置不生效
- **原因**: 信号未正确连接
- **解决**: 
  1. 查看GUI日志是否有"已更新任务坐标系偏移量"提示
  2. 检查ROS节点日志
  3. 重启地面站GUI

### 问题3: 不同USV导航结果不一致
- **原因**: 各USV的Home点设置不一致
- **解决**: 
  1. 确认所有USV的usv_params.yaml中Home点坐标完全相同
  2. 检查auto_set_home_node日志，确认Home点设置成功

## API参考

### ROSSignal
```python
# 更新Area Center偏移量
update_area_center = pyqtSignal(dict)  # {'x': float, 'y': float, 'z': float}
```

### GroundStationNode
```python
def update_area_center_callback(self, offset_dict):
    """更新任务坐标系偏移量"""
    # offset_dict: {'x': float, 'y': float, 'z': float}
```

### AreaOffsetDialog
```python
# 创建对话框
dialog = AreaOffsetDialog(parent, current_offset={'x': 0.0, 'y': 0.0, 'z': 0.0})

# 显示对话框
if dialog.exec_() == QDialog.Accepted:
    offset = dialog.get_offset()  # 返回 {'x': float, 'y': float, 'z': float}
```

## 相关文件

- **对话框实现**: `gs_gui/gs_gui/area_offset_dialog.py`
- **主窗口集成**: `gs_gui/gs_gui/main_gui_app.py`
- **ROS信号定义**: `gs_gui/gs_gui/ros_signal.py`
- **坐标转换逻辑**: `gs_gui/gs_gui/cluster_controller.py`
- **节点回调**: `gs_gui/gs_gui/ground_station_node.py`
- **配置文件**: `gs_bringup/config/gs_params.yaml`
- **测试文件**: `gs_gui/test/test_area_offset_dialog.py`

## 更新日志

- **2025-10-23**: 初始实现，添加GUI对话框和ROS节点支持
