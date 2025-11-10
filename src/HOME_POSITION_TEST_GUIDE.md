# Home Position 功能测试指南

## 当前系统状态

从启动日志可以看到：
- ✅ MAVROS 已连接到飞控（ArduRover V4.7.0-dev）
- ✅ EKF Origin 自动设置成功：(22.5180977, 113.9007239, -4.80m)
- ✅ 飞控已有 Home Position：(22.5180976, 113.9007232, -8.33m)
- ✅ GPS 定位正常（3D Fix）
- ✅ 控制节点已就绪

## 测试步骤

### 1. 查看当前 Home Position

打开新终端：
```bash
source ~/usv_workspace/install/setup.bash
ros2 topic echo /usv_01/mavros/home_position/home --once
```

**预期输出**：
```yaml
geo:
  latitude: 22.5180976
  longitude: 113.9007232
  altitude: -8.33
```

### 2. 启动地面站 GUI

在另一个终端：
```bash
source ~/usv_workspace/install/setup.bash
ros2 launch gs_bringup gs_launch.py
```

等待 GUI 启动（约 5-10 秒）。

### 3. 测试菜单功能

在 GUI 中：
1. 查看顶部菜单栏是否有 **"工具(T)"** 菜单
2. 点击 "工具" → 应该看到 **"🏠 设置 Home Position"** 菜单项
3. 快捷键测试：按 `Ctrl+H` 看是否打开对话框

### 4. 测试 "使用当前位置" 模式

1. 打开对话框（工具 → 设置 Home Position）
2. 在 "选择 USV" 下拉框中选择 `usv_01`
3. 选择 **"使用 USV 当前位置"** 单选按钮
4. 点击 **"确定"** 按钮
5. 查看 GUI 底部日志窗口，应该显示：
   ```
   📍 已向 usv_01 发送设置 Home Position 命令（使用当前位置）
   [OK] usv_01 Home Position 已设置为当前位置
   ```

6. 验证命令是否生效：
   ```bash
   # 在终端查看 Home Position 是否更新
   ros2 topic echo /usv_01/mavros/home_position/home --once
   ```

### 5. 测试 "指定坐标" 模式（A0 基站）

1. 再次打开对话框（Ctrl+H）
2. 选择 **"指定坐标"** 单选按钮
3. 点击 **"使用 A0 基站坐标"** 按钮
   - 应该自动填入：
     - 纬度：`22.5180977`
     - 经度：`113.9007239`
     - 高度：`-4.8`
4. 点击 **"确定"**
5. 查看日志窗口，应该显示：
   ```
   📍 已向 usv_01 发送设置 Home Position 命令
       坐标: 22.5180977, 113.9007239, -4.80m
   [OK] usv_01 Home Position 已设置为指定坐标
       坐标: 22.5180977, 113.9007239, -4.80m
   ```

6. 验证：
   ```bash
   ros2 topic echo /usv_01/mavros/home_position/home --once
   ```
   应该看到高度变为 `-4.8m`（与 EKF Origin 一致）

### 6. 测试 "指定坐标" 模式（自定义）

1. 打开对话框
2. 选择 "指定坐标"
3. 手动输入一个测试坐标（建议在附近）：
   - 纬度：`22.5181`（向北约 10 米）
   - 经度：`113.9008`（向东约 10 米）
   - 高度：`0.0`
4. 点击确定
5. 验证 Home Position 是否更新到新坐标

### 7. 测试数据验证功能

测试错误处理：

**测试 1: 无效纬度**
1. 打开对话框，选择 "指定坐标"
2. 输入纬度 `100`（超出 ±90° 范围）
3. 点击确定
4. 应该弹出警告："纬度必须在 -90° 到 90° 之间"

**测试 2: 无效经度**
1. 输入经度 `200`（超出 ±180° 范围）
2. 应该弹出警告："经度必须在 -180° 到 180° 之间"

**测试 3: 非数字输入**
1. 输入纬度 `abc`
2. 应该弹出警告："坐标输入格式错误，请输入有效的数字"

### 8. 测试 RTL 返航功能（可选，实机测试）

⚠️ **注意**：此步骤需要在安全的水域环境中进行！

1. 设置 Home Position 到岸边某个安全位置
2. 将 USV 手动驾驶到水域中央（距离 Home 约 10-20 米）
3. 通过地面站切换到 RTL 模式（或通过遥控器）
4. 观察 USV 是否自动返航到设置的 Home Position

**RTL 模式切换命令**（终端测试）：
```bash
ros2 service call /usv_01/mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'RTL'}"
```

## 监控命令

### 实时监控 Home Position 变化
```bash
ros2 topic echo /usv_01/mavros/home_position/home
```

### 监控 MAVLink 命令状态
```bash
ros2 topic echo /usv_01/mavros/command/status
```

### 监控地面站节点日志
```bash
ros2 topic echo /rosout | grep -E "groundstationnode|home"
```

### 查看所有可用的 MAVROS 服务
```bash
ros2 service list | grep usv_01
```

## 预期结果

### 成功标志
- ✅ 对话框能正常打开和关闭
- ✅ USV 列表显示 `usv_01`
- ✅ 坐标输入和验证正常工作
- ✅ 命令发送后有明确的成功/失败反馈
- ✅ Home Position topic 更新为新设置的坐标
- ✅ RTL 模式下 USV 返航到 Home Position

### 常见问题

**问题 1: 对话框显示 "无在线 USV"**
- **原因**: 地面站尚未检测到 usv_01
- **解决**: 等待 5-10 秒让命名空间检测完成，或重启地面站

**问题 2: 点击确定后无反馈**
- **原因**: MAVROS 服务不可用
- **检查**: `ros2 service list | grep /usv_01/cmd/command`
- **解决**: 确保 MAVROS 节点正常运行

**问题 3: Home Position 设置成功但 RTL 不返航**
- **原因**: 飞控参数配置问题
- **检查**: 
  - 飞控是否支持 RTL 模式
  - GPS 定位是否正常（需要 3D Fix）
  - Home Position 是否在安全距离内

**问题 4: 高度设置后不生效**
- **原因**: ArduPilot 可能使用气压计修正高度
- **说明**: 这是正常现象，高度会根据实际气压自动调整

## 测试检查清单

- [ ] GUI 菜单显示 "设置 Home Position"
- [ ] 快捷键 Ctrl+H 能打开对话框
- [ ] USV 选择器显示在线 USV
- [ ] "使用当前位置" 模式正常工作
- [ ] "指定坐标" 模式正常工作
- [ ] "使用 A0 基站坐标" 快捷按钮有效
- [ ] 坐标范围验证正常（±90°/±180°）
- [ ] 非法输入被正确拦截
- [ ] 日志窗口显示操作反馈
- [ ] Home Position topic 更新成功
- [ ] RTL 返航功能正常（实机测试）

## 下一步

测试完成后，如果发现任何问题，请记录：
1. 具体操作步骤
2. 预期结果 vs 实际结果
3. 相关日志输出
4. 错误消息（如果有）

然后我们可以针对性地修复和优化。

---

**准备好了吗？** 让我们开始测试 Home Position 设置功能！ 🚀
