# Home Position 设置功能 - 实现检查清单

## ✅ 已完成

### 1. 信号定义
- [x] 在 `ros_signal.py` 中添加 `set_home_position` 信号
- [x] 定义参数：`(str, bool, dict)` → (usv_namespace, use_current, coords)

### 2. ROS 节点实现
- [x] 在 `ground_station_node.py` 中添加 `set_home_position_callback()` 方法
- [x] 实现 MAVLink 命令发送逻辑（MAV_CMD_DO_SET_HOME = 179）
- [x] 支持两种模式：使用当前位置 / 指定坐标
- [x] 添加 `_handle_set_home_response()` 回调处理响应
- [x] 通过 MAVROS `/cmd/command` 服务发送命令
- [x] 异步调用，避免阻塞 GUI

### 3. 设置对话框
- [x] 创建 `set_home_dialog.py` 文件
- [x] 实现 `SetHomeDialog` 类
- [x] USV 选择下拉框（显示在线 USV）
- [x] 坐标来源选择（单选按钮）
  - [x] 使用 USV 当前位置
  - [x] 指定坐标
- [x] 坐标输入框（纬度/经度/高度）
- [x] 快捷按钮："使用 A0 基站坐标"
- [x] 数据验证
  - [x] 检查 USV 列表是否为空
  - [x] 验证坐标格式（数字）
  - [x] 验证坐标范围（纬度 ±90°，经度 ±180°）

### 4. GUI 菜单集成
- [x] 在 `main_gui_app.py` 的 "工具" 菜单添加 "设置 Home Position" 菜单项
- [x] 设置快捷键 `Ctrl+H`
- [x] 设置提示文本和图标（🏠）
- [x] 实现 `open_set_home_dialog()` 方法
- [x] 检查在线 USV（为空时显示警告）
- [x] 显示对话框并处理结果
- [x] 发送信号到 ROS 节点
- [x] 在 GUI 日志窗口显示操作反馈

### 5. 信号连接
- [x] 在 `main()` 函数中连接 `set_home_position` 信号到节点回调

### 6. 构建和测试
- [x] 构建 `gs_gui` 包（无错误）
- [x] 代码语法检查通过

### 7. 文档
- [x] 创建 `HOME_POSITION_SETTING_GUIDE.md` 完整实现文档
- [x] 创建 `HOME_POSITION_QUICK_REF.md` 快速参考
- [x] 创建 `HOME_POSITION_CHECKLIST.md` 检查清单
- [x] 更新 `MARKDOWN_FILES_MANAGEMENT.md` 索引

## 🔄 待测试

### 功能测试
- [ ] 启动地面站 GUI
- [ ] 验证 "工具" 菜单中出现 "设置 Home Position" 菜单项
- [ ] 验证快捷键 `Ctrl+H` 可以打开对话框
- [ ] 测试无 USV 在线时的警告消息
- [ ] 测试 USV 在线时对话框正常显示
- [ ] 测试 "使用当前位置" 模式
  - [ ] 验证坐标输入框被禁用
  - [ ] 发送命令并查看日志反馈
- [ ] 测试 "指定坐标" 模式
  - [ ] 验证坐标输入框被启用
  - [ ] 手动输入坐标
  - [ ] 测试 "使用 A0 基站坐标" 快捷按钮
  - [ ] 验证坐标范围检查（超出 ±90°/±180° 时报错）
  - [ ] 发送命令并查看日志反馈

### MAVROS 集成测试
- [ ] 启动 USV 节点和 MAVROS
- [ ] 通过 GUI 设置 Home Position（使用当前位置）
- [ ] 通过 `ros2 topic echo /usv_01/mavros/home_position/home` 验证
- [ ] 通过 GUI 设置 Home Position（指定坐标）
- [ ] 再次验证 home_position topic

### RTL 功能测试
- [ ] 设置 Home Position 到某个位置
- [ ] 将 USV 移动到其他位置
- [ ] 切换到 RTL 模式
- [ ] 观察 USV 是否返航到 Home Position

## 📝 后续优化（可选）

### 功能增强
- [ ] 在对话框中显示 USV 当前的 Home Position 坐标
- [ ] 支持批量设置多艘 USV 的 Home Position
- [ ] 添加地图交互（点击地图设置 Home Position）
- [ ] 显示 Home Position 在地图上的位置标记

### 用户体验
- [ ] 保存常用 Home Position 坐标到历史记录
- [ ] 添加快速选择历史坐标的功能
- [ ] 添加 "测试 RTL" 按钮（自动切换模式验证）
- [ ] 添加 Home Position 状态显示（已设置/未设置）

### 错误处理
- [ ] 更详细的错误消息（区分不同失败原因）
- [ ] 添加重试机制（命令发送失败时）
- [ ] 添加超时处理（长时间无响应）

## 🔍 验证命令

### 查看 Home Position
```bash
# 查看当前 Home Position
ros2 topic echo /usv_01/mavros/home_position/home

# 查看 EKF Origin（对比）
ros2 topic echo /usv_01/mavros/global_position/gp_origin
```

### 手动设置 Home Position（测试对比）
```bash
# 通过 MAVROS 服务手动设置（不推荐，仅用于测试）
ros2 service call /usv_01/cmd/command mavros_msgs/srv/CommandLong \
"{
  broadcast: false,
  command: 179,
  confirmation: 0,
  param1: 0.0,
  param5: 22.5180977,
  param6: 113.9007239,
  param7: -4.8
}"
```

### 监控命令发送
```bash
# 监听所有 MAVLink 命令
ros2 topic echo /usv_01/mavros/command/status

# 监听日志输出
ros2 topic echo /rosout | grep -i "home"
```

## 🐛 已知问题

无（截至当前）

## 📅 测试计划

### Phase 1: 基础功能测试（本地）
- [ ] GUI 菜单和对话框显示
- [ ] 对话框交互和数据验证
- [ ] 信号发送和日志输出

### Phase 2: MAVROS 集成测试（仿真）
- [ ] SITL 仿真环境
- [ ] 命令发送和响应
- [ ] Home Position topic 验证

### Phase 3: 实机测试
- [ ] 在真实 USV 上设置 Home Position
- [ ] RTL 返航功能验证
- [ ] 多 USV 并行测试

## 📌 注意事项

1. **EKF Origin 已自动设置**：启动时通过 `auto_set_home_node` 自动设置为 A0 基站
2. **Home Position 独立**：与 EKF Origin 分离，可以随时修改
3. **GPS 定位要求**：使用 "当前位置" 模式时需要 3D Fix
4. **飞控版本**：需要 ArduPilot 4.7.0+ 或 PX4 1.12+

## ✅ 最终检查

- [x] 代码编译无错误
- [x] 代码符合项目规范（命名、注释、格式）
- [x] 文档完整（实现指南、快速参考、检查清单）
- [x] 信号连接正确
- [x] 错误处理完善
- [ ] 功能测试通过（待实机验证）

---

**状态**: 实现完成，等待功能测试 ✅

**下一步**: 
1. 启动地面站进行基础功能测试
2. 连接 USV 进行 MAVROS 集成测试
3. 实机测试 RTL 返航功能
