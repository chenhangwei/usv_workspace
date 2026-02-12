# V13 实测数据分析与优化总结

> **版本**: V14 (基于 V13 实测数据的优化迭代)  
> **日期**: 2026-02-12  
> **测试路径**: `混乱回字-8mx24m-白天-异步.xml` (12步, 2艘USV)  
> **参与USV**: usv_02, usv_03

---

## 一、V13 实测问题概述

### 1.1 测试场景

使用 `混乱回字-8mx24m-白天-异步.xml` 路径文件，2艘USV（usv_02、usv_03）执行异步集群任务。路径包含11个 async 步骤和1个 terminal 步骤（第12步），USV 沿回字形路径交叉导航。

### 1.2 发现的核心问题

| 优先级 | 问题 | 影响 | 根因文件 |
|--------|------|------|----------|
| **P0** | 最后一步(terminal)两艘USV控制输出为零 | USV完全失控 | `velocity_controller_node.py` |
| **P1** | 导航过程中频繁出现5.1s HOLD停顿 | USV刹车停顿，影响表演效果 | `velocity_controller_node.py` |
| **P1** | 异步路径文件实际执行为同步行为 | USV不必要地等待同伴 | `cluster_task_manager.py` |
| **P2** | 到达航点后急刹车 | 动作不流畅 | `velocity_controller_node.py` |

### 1.3 数据分析关键发现

**usv_03 数据分析:**
- 检测到 6 段 HOLD 停顿，每段约 5.1 秒
- 最后一步 (goal_id=30012) 全程 `cmd_omega=0`（控制为零）
- 在 goal_id=30012 处出现极端航向速率 -603.8°/s（急转）

**usv_02 数据分析:**
- 同样检测到 HOLD 停顿模式
- 最后一步滞留在距目标 20m 处 225 秒
- 位置模式因 setpoint 竞争导致失效

**双USV交叉验证结论:**
- 同步 bug 并非 HOLD 的直接原因（步骤过渡仅 ~0.1s）
- 5.1s HOLD 发生在步骤中间，源自飞控自动切换或外部触发
- P0 级别问题：两艘USV在 terminal 步骤均丧失有效控制

---

## 二、已实施的优化（4项）

### 2.1 [P0] Terminal 模式 Setpoint 竞争修复

**问题根因:**

当 `nav_mode=terminal`（离群定点停留）的目标发送到 velocity_controller 时，原代码执行 `return` 直接跳过，但**没有清理控制状态**。这导致：

1. velocity_controller 的控制循环继续运行，发布零速度指令到 `setpoint_raw/local`
2. usv_control_node 同时发布 GPS 位置指令到 `setpoint_raw/global`
3. ArduPilot GUIDED 模式下两个 setpoint 源竞争 → 控制失效

**修复方案 (`velocity_controller_node.py`):**

收到 terminal 目标时，velocity_controller **主动让出控制权**：

```python
if nav_mode == NAV_MODE_TERMINAL:
    # 清理控制状态，防止控制循环继续发布零速指令
    self.tracker.clear_waypoints()
    self._control_active = False
    self._current_goal_id = None
    # 取消残留的软减速和延迟 HOLD
    self._soft_decel_active = False
    self._delayed_stop_pending = False
    self._delayed_hold_pending = False
    # 旋转状态清理
    if self._rotation_active:
        self._rotation_active = False
        self._rotation_initialized = False
    # 设置导航状态为 IDLE
    self._set_navigation_state(NavigationState.IDLE, "TERMINAL模式让出控制权")
    return
```

**修复效果:** 消除 setpoint 竞争，terminal 模式下 usv_control_node 独占控制权，GPS 位置模式正常工作。

---

### 2.2 [P1] 异步路径 sync 默认值修复

**问题根因:**

`cluster_task_manager.py` 解析 XML 路径文件时，`sync` 属性的默认值硬编码为 `True`：

```python
# 旧代码：默认 sync=true，即使 nav_mode="async"
sync_attr = step.get("sync", "true").lower()
sync_val = (sync_attr == "true")
```

这导致 `混乱回字-8mx24m-白天-异步.xml` 中的 async 步骤实际以同步模式执行，GS 端等待所有 USV 完成当前步骤才推进下一步。

**修复方案 (`cluster_task_manager.py`):**

`sync` 默认值改为从 `nav_mode` 自动推导：

```python
explicit_sync = step.get("sync")
if explicit_sync is not None:
    # 显式指定 sync 属性 → 使用该值
    sync_val = (explicit_sync.lower() == "true")
else:
    # 自动推导: async → sync=false, 其他模式 → sync=true
    sync_val = (step_nav_mode != 0)  # 0 = async
```

**推导规则:**

| nav_mode | 未指定 sync 时的默认值 | 说明 |
|----------|----------------------|------|
| `async` (0) | `sync=false` | USV到达即推进，不等待 |
| `sync` (1) | `sync=true` | GS等待所有USV完成 |
| `rotate` (2) | `sync=true` | 协调旋转需要同步 |
| `terminal` (3) | `sync=true` | 终点定位需要同步 |

---

### 2.3 [P2] 渐进减速替代急刹车

**问题根因:**

导航完成（到达航点）后，原代码使用延迟急停策略：

```python
# 旧代码：延迟 1.5s 后直接发送零速指令 → 急刹车
self._delayed_stop_pending = True
self._delayed_stop_deadline = time.time() + self._delayed_stop_delay
```

这导致 USV 在同步等待或步骤过渡时突然刹车，影响视觉效果。

**修复方案 (`velocity_controller_node.py`):**

新增 `_start_soft_deceleration()` 和 `_handle_soft_deceleration()`，实现 3.0 秒线性减速：

```python
def _start_soft_deceleration(self):
    """从当前速度线性减速到零"""
    self._soft_decel_active = True
    self._soft_decel_start_time = time.time()
    self._soft_decel_start_vx = self._last_velocity_cmd.linear_x
    self._soft_decel_start_omega = self._last_velocity_cmd.angular_z

def _handle_soft_deceleration(self):
    """线性递减速度直到归零"""
    elapsed = time.time() - self._soft_decel_start_time
    if elapsed >= self._soft_decel_duration:
        self._soft_decel_active = False
        self._publish_velocity_command(VelocityCommand.stop())
        return
    # 线性减速: ratio 从 1.0 → 0.0
    ratio = 1.0 - (elapsed / self._soft_decel_duration)
    vx = self._soft_decel_start_vx * ratio
    omega = self._soft_decel_start_omega * ratio
    self._publish_velocity_command(VelocityCommand(vx, 0.0, omega))
```

**关键设计:**
- 收到新目标时自动取消减速：`self._soft_decel_active = False`
- 延迟 HOLD 时间调整为 `减速时间 + 1.0s`，确保 HOLD 在减速完成后
- 作为控制循环的优先级处理，在 `_check_preconditions()` 之前执行

---

### 2.4 [P1] GUIDED 模式最高优先级 / 零停顿策略

**问题根因:**

V13 测试中观测到 5.1 秒的 HOLD 停顿，根源是模式保护机制过于保守：

1. 飞控自动切换 HOLD → velocity_controller 检测到非 GUIDED
2. 进入 PAUSED 状态 + 发送 stop 指令 → **USV 立即刹车**
3. 等待 5.0s 保护期（等 cancel_navigation 消息）→ **长时间停顿**
4. 保护期过后恢复 GUIDED → 再经过 2.0s 冷却 → **继续等待**
5. 总计可达 **~7 秒停顿**

**修复方案 (`velocity_controller_node.py`) — 全面重构模式保护:**

#### (a) 核心策略变更

| 维度 | 旧策略 | 新策略 |
|------|--------|--------|
| 检测到 HOLD | 停止 → 等5s → 恢复 | **立即恢复 GUIDED，不停止** |
| 恢复冷却 | 2.0s | **0.2s** (仅防消息风暴) |
| 保护期 | 5.0s | **0.5s** |
| 切换保护 | 2.0s | **1.0s** |
| MANUAL 模式 | 取消导航 | **立即恢复 GUIDED** |
| 手动暂停 | 单次HOLD即暂停 | **连续3次HOLD(5s内)才暂停** |

#### (b) `_check_preconditions()` 新逻辑

```
检测到非 GUIDED 模式
    │
    ├─ GUIDED 切换刚发出？ → 等待生效（1.0s保护期）
    │
    ├─ 连续HOLD强制暂停中？ → 不恢复（尊重操作员意图）
    │
    ├─ ACTIVE 状态？ → ⚡ 立即恢复 GUIDED，不停止，不进 PAUSED
    │
    ├─ PAUSED 非强制？ → 恢复 GUIDED + 恢复 ACTIVE
    │
    └─ IDLE/COMPLETED/CANCELLED → 不恢复
```

#### (c) 连续 HOLD 强制暂停机制

```python
# cancel_navigation 回调中:
if (now - self._last_hold_request_time) <= 5.0:
    self._consecutive_hold_count += 1
else:
    self._consecutive_hold_count = 1  # 超出窗口，重新计数

if self._consecutive_hold_count >= 3:
    # 达到阈值，强制暂停
    self._manual_hold_requested = True
    self._set_navigation_state(NavigationState.PAUSED, "连续HOLD强制暂停")
else:
    # 未达阈值，GUIDED 在下一个控制循环自动恢复
    pass
```

**设计考量:**
- 飞控自动 HOLD（水流、GPS 漂移等）→ 不暂停，立即恢复
- 操作员偶尔误触 HOLD → 不暂停，立即恢复
- 操作员明确要暂停 → 5秒内连续点3次 HOLD → 强制暂停
- STOP 命令 → 不受连续计数限制，立即生效

**预期效果:** HOLD 导致的停顿从 ~7s 降低到 ~50ms（1-2个控制循环 × 20Hz）。

---

## 三、修改文件清单

| 文件 | 修改类型 | 关联问题 |
|------|----------|----------|
| `usv_control/usv_control/velocity_controller_node.py` | 重大重构 | P0 Terminal修复, P1 零停顿, P2 渐进减速 |
| `gs_gui/gs_gui/cluster_task_manager.py` | 逻辑修复 | P1 sync默认值 |
| `usv_control/usv_control/log_collector.py` | 格式修复 | 日志缩进修正 |
| `Path File/混乱回字-8mx24m-白天-异步.xml` | 新增 | 测试路径文件 |
| `Path File/混乱Z字形-8mx20m-白天-异步.xml` | 新增 | 测试路径文件 |

### velocity_controller_node.py 变更详情

| 函数/区域 | 变更内容 |
|-----------|----------|
| 初始化变量 (L261-310) | 冷却 2.0→0.2s, 保护期 5.0→0.5s, 新增连续HOLD变量, 新增渐进减速变量 |
| `_on_goal_reached()` | 延迟HOLD时间改为 减速时间+1.0s |
| `_cancel_navigation_callback()` | 重写为连续HOLD计数机制 |
| `_stop_navigation_callback()` | 增加连续计数重置, 注释STOP不受限 |
| `_process_nav_goal()` (TERMINAL) | 新增完整控制权让出逻辑 |
| `_process_nav_goal()` (新目标) | 取消渐进减速 |
| `_control_loop()` | 增加渐进减速处理入口 |
| `_check_preconditions()` | 全面重写模式检测逻辑 |
| `_end_navigation()` (COMPLETED) | 启动渐进减速替代延迟急停 |
| `_check_delayed_stop()` | 改为渐进减速的备用兜底 |
| `_start_soft_deceleration()` | 新增 |
| `_handle_soft_deceleration()` | 新增 |
| `_restore_guided_mode()` | 0.2s冷却, 设置切换保护期 |
| `_should_protect_navigation()` | 保护ACTIVE + 非强制暂停PAUSED |

---

## 四、架构影响分析

### 4.1 控制流变化

```
【旧控制流 - V13】
HOLD检测 → PAUSED + 停止 → 5s保护期 → 恢复GUIDED(2s冷却) → 继续
                    ↓
              最大7s停顿

【新控制流 - V14】
HOLD检测 → 立即恢复GUIDED → 继续 (无停顿)
             ↓
        ~50ms (1-2个控制循环)
```

### 4.2 与其他节点的交互

| 节点 | 交互变化 |
|------|----------|
| `usv_control_node` | Terminal模式下不再竞争setpoint |
| `navigate_to_point_node` | 无变化，仍负责航点队列和到达判断 |
| `command_processor` (GS) | 无代码修改，但HOLD按钮行为变化（需连续3次） |
| `cluster_controller` (GS) | sync推导修复使async步骤真正异步 |

### 4.3 向后兼容性

- XML 路径文件：**完全兼容**。显式指定 `sync="true"` 的文件行为不变
- GS 操作：HOLD 按钮行为变化（单次无效，需连续3次），**需告知操作员**
- STOP 命令：行为不变，立即生效
- 新任务下发：自动清除强制暂停状态，行为不变

---

## 五、V15 验证建议

### 5.1 必须验证的场景

1. **Terminal 模式定位**: usv_02 和 usv_03 在第12步能正确到达目标点并保持位置
2. **零停顿导航**: 全程无 HOLD 导致的刹车停顿（目标: 0次HOLD停顿 > 1s）
3. **连续HOLD暂停**: 操作员连续3次快速点击HOLD能成功暂停任务
4. **渐进减速**: 每步到达后观察 USV 是否平滑减速（vs 急刹车）
5. **异步行为**: async 步骤中，先到达的 USV 不等待另一艘

### 5.2 数据监控指标

| 指标 | V13 基线 | V14 预期 |
|------|----------|----------|
| HOLD 停顿次数 | 6次 | 0次 |
| HOLD 停顿时长 | ~5.1s | < 0.1s |
| Terminal 步到达距离 | 20m (usv_02 失效) | < 1.0m |
| 步间过渡时间 | ~0.1s | ~0.1s (不变) |
| 航点到达后刹车时间 | 瞬间急停 | 3.0s 线性减速 |

### 5.3 回滚方案

如果连续HOLD机制导致操作不便，可通过 YAML 配置文件调整（无需改代码）：

```yaml
# usv_params.yaml
velocity_controller_node:
  ros__parameters:
    consecutive_hold_threshold: 2   # 默认 3，改为 2 更灵敏，改为 1 等效旧行为
    soft_decel_duration: 2.0        # 默认 3.0，缩短减速时间
```

如果零停顿策略导致特殊情况下无法手动接管，可设置 `_mode_protection_enabled = False` 禁用自动恢复。

---

## 六、V14 测试准备工作

### 6.1 日志增强

| 改进项 | 文件 | 说明 |
|--------|------|------|
| `nav_mode` 列 | `log_collector.py` | CSV 日志新增导航模式字段 (0=async, 1=sync, 2=rotate, 3=terminal) |
| feedback `nav_mode` | `velocity_controller_node.py` | NavigationFeedback 消息补全 nav_mode 字段 |
| 版本检测 | `download_usv_logs.sh` | 修复 `get_remote_meta()` 仅识别 v5-v7，改为通用正则匹配 v5-v99+ |
| 版本检测 | `analyze_nav_log.py` | 修复头信息解析仅识别到 v8，改为通用正则提取 `(vXX)` |

### 6.2 参数可配置化

V14 将关键模式保护参数暴露为 ROS 参数，支持通过 YAML 配置现场微调：

| ROS 参数名 | 默认值 | 作用 |
|-----------|--------|------|
| `consecutive_hold_threshold` | 3 | 连续 HOLD 触发强制暂停的次数 |
| `soft_decel_duration` | 3.0 | 渐进减速持续时间 (秒) |

### 6.3 下载脚本修复

`download_usv_logs.sh --all` 原版本检测逻辑只硬编码了 v5/v6/v7 三个版本：

```bash
# 旧: V14 日志会被归类到 V5 目录
if echo "$header" | grep -qi "v7"; then version="V7"
elif echo "$header" | grep -qi "v6"; then version="V6"
elif echo "$header" | grep -qi "v5"; then version="V5"
fi

# 新: 通用正则提取版本号，自动归类到正确的 VXX 目录
detected_version=$(echo "$header" | grep -oiE '\(v([0-9]+)\)' | ...)
version="V${detected_version}"  # 自动得到 V14
```

### 6.4 部署检查清单

| 检查项 | 状态 | 说明 |
|--------|------|------|
| Launch 包含 velocity_controller | ✅ | `usv_launch.py` 硬件/SITL 双路径均包含 |
| Launch 包含 log_collector | ✅ | 默认启用 (`enable_log_collector=true`) |
| log_collector entry_point | ✅ | `setup.py` 已注册 |
| NavigationFeedback.msg | ✅ | `nav_mode` 字段已存在 |
| YAML 旧值冲突 | ✅ 无冲突 | 模式保护参数是新增的 ROS 参数 |
| usv_03 专属 MPC 参数 | ⚠️ 确认 | `usv_params.yaml` 中有 `/usv_03/velocity_controller_node` 覆盖 |
| 默认命名空间 | ℹ️ `usv_03` | 部署到 usv_02 需指定 `namespace:=usv_02` |

### 6.5 测试前操作流程

```bash
# 1. 编译（在 GS 电脑）
cd ~/usv_workspace && colcon build --packages-select usv_control common_interfaces

# 2. 部署到 USV
scp -r ~/usv_workspace/src chenhangwei@192.168.68.52:~/usv_workspace  # usv_02
scp -r ~/usv_workspace/src chenhangwei@192.168.68.53:~/usv_workspace  # usv_03

# 3. 在每台 USV 上编译
ssh chenhangwei@192.168.68.52 "cd ~/usv_workspace && colcon build --packages-select usv_control common_interfaces"
ssh chenhangwei@192.168.68.53 "cd ~/usv_workspace && colcon build --packages-select usv_control common_interfaces"

# 4. 测试后下载日志（日志自动归类到 ~/usv_logs/V14/usv_XX/）
./download_usv_logs.sh --all

# 5. 分析日志
python3 ~/usv_workspace/src/usv_control/scripts/analyze_nav_log.py ~/usv_logs/V14/usv_02
python3 ~/usv_workspace/src/usv_control/scripts/analyze_nav_log.py ~/usv_logs/V14/usv_03
```

---

## 七、附录

### A. 分析使用的日志文件

- `~/usv_logs/V13/usv_02/nav_log_*.csv` — usv_02 导航日志
- `~/usv_logs/V13/usv_03/nav_log_*.csv` — usv_03 导航日志

### B. 分析工具

```bash
python3 ~/usv_workspace/src/usv_control/scripts/analyze_nav_log.py ~/usv_logs/V13/usv_02
python3 ~/usv_workspace/src/usv_control/scripts/analyze_nav_log.py ~/usv_logs/V13/usv_03
```

### C. 测试路径文件

- `Path File/混乱回字-8mx24m-白天-异步.xml` — 12步, steps 1-11 async, step 12 terminal
- `Path File/混乱Z字形-8mx20m-白天-异步.xml` — 11步, 全 async
