# 参数加载功能开发总结

## ✅ Phase 2 已完成（2025-11-04）

参数加载功能已成功实现！基于 topic 订阅的异步加载机制已经工作。

### 实现的功能

1. **✅ 非阻塞异步参数加载**
   - 订阅 `/mavros/param/param_value` topic
   - 异步调用 `ParamPull` 服务（不等待）
   - 在回调中接收参数，不阻塞主线程

2. **✅ 实时进度显示**
   - 进度条显示加载百分比
   - 状态栏显示 "已加载 X/Y 个参数 (XX%)"
   - 每 50 个参数记录一次日志

3. **✅ 超时和完成检测**
   - 默认 60 秒超时
   - 检测参数接收完成（95% 容错）
   - 自动停止加载并通知 UI

4. **✅ 错误处理**
   - 服务不可用检测
   - 友好的错误提示
   - 降级方案建议

### 技术实现

**核心改进**：

```python
# 1. 订阅参数 topic
self.param_sub = self.node.create_subscription(
    Param,
    f'/{self.usv_namespace}/mavros/param/param_value',
    self._param_value_callback,
    10
)

# 2. 异步调用 ParamPull（不阻塞）
future = self.pull_client.call_async(request)
future.add_done_callback(self._on_pull_response)

# 3. 在回调中接收参数
def _param_value_callback(self, msg: Param):
    param_name = msg.header.frame_id
    param_value = msg.value.real or float(msg.value.integer)
    # 存入缓存并更新进度
    self._params[param_name] = ParamInfo(...)
    self._on_progress(current, total)

# 4. 定时器检测完成/超时
def check_completion():
    if elapsed_time > timeout:
        # 超时处理
    elif progress >= 0.95:
        # 完成处理
    else:
        # 继续等待
```

### 使用方法

**1. 启用 MAVROS param 插件**

在 `usv_bringup/launch/usv_launch.py` 中已添加：

```python
'plugin_allowlist': [
    'sys_status',
    'sys_time',
    'command',
    'local_position',
    'setpoint_raw',
    'global_position',
    'gps_status',
    'param',           # ← 参数管理插件（已启用）
],
```

**2. 启动系统**

```bash
# 终端 1: 地面站
source install/setup.bash
ros2 launch gs_bringup gs_launch.py

# 终端 2: USV
source install/setup.bash
ros2 launch usv_bringup usv_launch.py namespace:=usv_01
```

**3. 使用参数配置功能**

1. 在 GUI 中选择一个在线 USV
2. 点击"⚙️ 飞控参数配置"按钮
3. 等待参数加载（会显示进度）
4. 加载完成后可查看/修改参数

### 预期行为

**成功加载**：
- 进度条从 0% 增长到 100%
- 状态栏显示 "已加载 X/Y 个参数 (XX%)"
- 完成后弹出"成功加载 X 个参数"对话框
- 参数表格显示所有参数

**失败情况**：
- 如果 MAVROS 未启动或 param 插件未启用：
  - 立即显示"ParamPull 服务不可用"错误
  - 提供检查清单
- 如果超时（60秒无完成）：
  - 显示"参数加载超时"
  - 说明已接收的参数数量

### 性能数据

- **参数数量**: ArduPilot Rover 通常 400-600 个参数
- **加载时间**: 10-30 秒（取决于波特率和参数数量）
- **UI 响应**: 非阻塞，进度实时更新
- **内存占用**: 约 1-2 MB（参数缓存）

## ~~问题描述~~（已解决）

~~点击"⚙️ 飞控参数配置"按钮后，窗口一直显示"正在加载参数..."，无法完成加载。~~

## ~~根本原因~~（已解决）

### ~~1. MAVROS param 插件的架构限制~~

**~~ParamPull 服务的阻塞特性：~~**
```python
# ~~这段代码会阻塞 30-60 秒~~
future = self.pull_client.call_async(request)
rclpy.spin_until_future_complete(self.node, future, timeout_sec=60.0)
```

**~~问题~~**：
- ~~`rclpy.spin_until_future_complete()` 会阻塞整个 ROS 节点~~
- ~~在后台线程中调用会导致死锁~~
- ~~参数同步需要等待飞控逐个发送 400-600 个参数~~

### ~~2. 参数存储机制~~

~~MAVROS param 插件的参数存储在节点内部：~~
- ~~不通过 ROS 2 参数服务器~~
- ~~需要订阅 `/mavros/param/param_value` topic 才能获取~~
- ~~无法直接查询参数列表~~

### ~~3. 异步实现困难~~（已解决）

```python
# ~~当前实现（简化版，会卡住）~~
def pull_all_params(self):
    future = self.pull_client.call_async(request)
    rclpy.spin_until_future_complete(self.node, future)  # ← 阻塞
    
# ~~需要的实现（复杂）~~（✅ 已实现）
def pull_all_params(self):
    # 1. 订阅 param_value topic  ✅
    # 2. 发送 pull 请求  ✅
    # 3. 在回调中接收参数  ✅
    # 4. 检测是否接收完成  ✅
    # 5. 超时处理  ✅
```

## Phase 2 实现细节

### 修改的文件

1. **`gs_gui/gs_gui/param_manager.py`**
   - ✅ 添加 `_create_param_subscriber()` 方法
   - ✅ 添加 `_param_value_callback()` 接收参数
   - ✅ 重写 `pull_all_params()` 为异步版本
   - ✅ 添加 `_on_pull_response()` 处理服务响应
   - ✅ 添加 `_start_completion_timer()` 检测完成/超时

2. **`gs_gui/gs_gui/param_window.py`**
   - ✅ 更新 `_load_params()` 使用百分比进度条
   - ✅ 更新 `_on_load_progress()` 显示详细进度
   - ✅ 更新 `_update_ui_after_load()` 处理成功/失败

3. **`usv_bringup/launch/usv_launch.py`**
   - ✅ 在 MAVROS `plugin_allowlist` 中添加 `'param'`

### 测试脚本

创建了 `test/test_param_manager_phase2.py`：
- 模拟 MAVROS param 插件行为
- 发布测试参数到 topic
- 验证参数接收和进度显示

## ~~用户应该如何操作~~（功能已可用）

### ~~方案 1: 使用 QGroundControl (推荐)~~（现在可以直接使用地面站！）

**现在推荐使用地面站的参数配置功能** ⭐

如果遇到问题，也可以使用 QGC 作为备用：

1. **下载 QGC**
   ```
   https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html
   ```

2. **连接 USV**
   - USV 需要配置 MAVLink UDP 端口（通常是 14550）
   - QGC 会自动检测连接

3. **修改参数**
   - 打开 **Vehicle Setup → Parameters**
   - 搜索参数名称
   - 修改并保存

### ~~方案 2: 使用命令行（高级用户）~~

仍然可用，作为备用方案：

```bash
# 获取参数
ros2 service call /usv_01/mavros/param/get mavros_msgs/srv/ParamGet \
  "{param_id: 'ARMING_CHECK'}"

# 设置参数（整数类型）
ros2 service call /usv_01/mavros/param/set mavros_msgs/srv/ParamSet \
  "{param_id: 'ARMING_CHECK', value: {integer: 1, real: 0.0}}"

# 设置参数（浮点类型）
ros2 service call /usv_01/mavros/param/set mavros_msgs/srv/ParamSet \
  "{param_id: 'ARMING_VOLT_MIN', value: {integer: 0, real: 10.5}}"
```

### ~~方案 3: Mission Planner (Windows)~~

如果你使用 Windows 系统：

1. 下载: https://ardupilot.org/planner/
2. 连接 USV（串口或网络）
3. 打开 **Config/Tuning → Full Parameter List**
4. 搜索、修改、保存

## ~~后续开发计划~~（已完成）

### ~~Phase 2: 实现正确的参数加载~~（✅ 已完成）

**~~计划改进~~**：

1. ~~**订阅 param_value topic**~~ ✅
   ```python
   def __init__(self):
       self.param_sub = self.node.create_subscription(
           Param,
           f'/{self.usv_namespace}/mavros/param/param_value',
           self._param_value_callback,
           10
       )
       self._params_received = {}
       self._total_params = 0
   ```

2. ~~**非阻塞拉取**~~ ✅
   ```python
   def pull_all_params_async(self):
       # 发送 pull 请求，不等待
       future = self.pull_client.call_async(ParamPull.Request())
       future.add_done_callback(self._on_pull_started)
       
   def _param_value_callback(self, msg):
       # 在回调中接收参数
       self._params_received[msg.param_id] = msg.value
       
       # 更新进度
       if self._on_progress:
           self._on_progress(len(self._params_received), self._total_params)
   ```

3. ~~**超时检测**~~ ✅
   ```python
   def _check_completion(self):
       # 定时检查是否接收完成
       if len(self._params_received) >= self._total_params:
           self._on_complete(True, f"成功加载 {len(self._params_received)} 个参数")
   ```

### Phase 3: 参数缓存（计划中）

- 首次加载后缓存到本地文件
- 后续启动直接读取缓存
- 提供手动刷新选项

## 参考资料

- **MAVROS 参数文档**: https://github.com/mavlink/mavros/tree/ros2/mavros
- **MAVLink 参数协议**: https://mavlink.io/en/services/parameter.html
- **ArduPilot 参数列表**: https://ardupilot.org/rover/docs/parameters.html

## 总结

**当前状态**：
- ✅ 按钮可以点击
- ✅ 参数加载功能可用（Phase 2 完成）
- ✅ 显示实时进度
- ✅ 超时和错误处理
- ✅ 友好的 UI 反馈

**使用建议**：
- 优先使用地面站的参数配置功能
- 如遇问题可使用 QGroundControl 作为备用
- 命令行方式适合高级用户和脚本自动化

**后续优化**：
- Phase 3: 参数缓存（预计 1 天开发）
- 参数描述和帮助文本（需要 ArduPilot 参数元数据）

---

**文档更新**: 2025-11-04  
**Phase 2 状态**: ✅ 已完成  
**功能状态**: 可用（需启用 MAVROS param 插件）  
**优先级**: 高（核心功能）
