# 集群任务步骤切换Bug修复报告

## 问题描述

集群任务在步骤1完成0/3个USV时就错误地进入了步骤2，违反了"完成度不到80%不会进入下一步"的设计要求。

### 错误日志

```
集群任务进度: 步骤 1/2, 完成 0/3 个USV (0.0%), 耗时 46.0s, 状态 运行中
集群任务进度: 步骤 1/2, 完成 3/3 个USV (100.0%), 耗时 51.0s, 状态 运行中  ← 突然从0跳到3
集群任务进度: 步骤 2/2, 完成 0/3 个USV (0.0%), 耗时 0.0s, 状态 运行中  ← 错误进入步骤2
```

## 根本原因

### Bug 1: 空确认状态触发步骤切换
**位置**: `gs_gui/gs_gui/cluster_controller.py` 第257-260行

**错误代码**:
```python
all_acked = all(state.acked for state in self._ack_states.values()) if self._ack_states else False
if all_acked:
    self._proceed_to_next_step()
```

**问题分析**:

**Python的 `all()` 函数对空序列返回 `True`！**

```python
all([])  # 返回 True
```

当 `self._ack_states` 为空字典 `{}` 时：
1. `all(state.acked for state in {}.values())` 等价于 `all([])`
2. 结果为 `True`
3. 触发 `_proceed_to_next_step()`，即使没有任何USV确认

---

### Bug 2: 超时失败被错误标记为"已确认"（★主要原因★）
**位置**: `gs_gui/gs_gui/cluster_controller.py` 第409行

**错误代码**:
```python
else:
    # 达到最大重试次数，标记为已确认并记录日志，不再重试
    state.acked = True  # ← 这是错误的！
    state.ack_time = now
```

**问题分析**:

这是导致"0/3突然跳到3/3"的**真正原因**！

1. 3个USV在步骤1都超时，未能到达目标点（距离仍然是5-6米）
2. 重试次数用尽后（`retry >= max_retries`），代码错误地将 `acked` 设置为 `True`
3. 下一次检查时，`all_acked = True`（因为所有USV都被错误标记为"已确认"）
4. 触发进入下一步，进度从 0/3 跳到 3/3

**逻辑错误**:
- **超时失败** ≠ **成功确认**
- **达到重试次数** ≠ **完成任务**
- 应该保持 `acked = False`，让基于确认率的逻辑来决定是否进入下一步

### 触发场景

可能导致问题的情况：
1. ✅ USV响应慢或网络延迟大，导致超时
2. ✅ USV实际未到达目标点但重试次数用尽
3. ✅ `_ack_states` 为空（Bug 1）
4. ✅ 步骤切换时状态未正确同步

## 修复方案

### 修复1: 空确认状态检查（第260-279行）

```python
# ⚠️ 注意：必须确保_ack_states不为空且所有状态都已确认
# 原bug：当_ack_states为空时，all()返回True导致错误进入下一步
all_acked = bool(self._ack_states) and all(state.acked for state in self._ack_states.values())

# 调试日志：输出all_acked判断结果
if all_acked:
    self.node.get_logger().info(
        f"步骤 {self.node.run_step} 所有USV已确认 "
        f"({len([s for s in self._ack_states.values() if s.acked])}/{len(self._ack_states)})，"
        f"准备进入下一步"
    )

if all_acked:
    self._proceed_to_next_step()
```

**关键改进**:
1. 添加非空检查：`bool(self._ack_states)` 确保字典不为空
2. 短路求值：只有当字典非空时才执行 `all()` 判断
3. 添加调试日志：记录步骤切换时的详细状态

---

### 修复2: 超时失败不标记为已确认（第408-414行）★核心修复★

```python
# 修改前（错误）
else:
    state.acked = True  # ← 错误！超时失败被标记为成功
    state.ack_time = now
    self.node.get_logger().error(f"{usv_id} 超时且已达最大重试次数，跳过并继续下一步")

# 修改后（正确）
else:
    # 达到最大重试次数，但不应标记为"已确认"
    # ⚠️ 修复：超时失败不等于成功确认，不应设置 acked=True
    # 只记录失败状态，让 _check_and_proceed_on_ack_rate 根据确认率判断是否进入下一步
    state.acked = False  # 明确标记为未确认
    self.node.get_logger().error(f"{usv_id} 超时且已达最大重试次数，标记为失败（不进入下一步）")
```

**关键改进**:
1. **不设置 `acked=True`**：超时失败不等于成功确认
2. **明确设置 `acked=False`**：清楚标记失败状态
3. **依赖确认率逻辑**：让 `_check_and_proceed_on_ack_rate` 函数根据实际确认率（需≥80%）来决定是否进入下一步

---

### 修复逻辑对比

#### 修复1: 空状态检查

| 条件 | 旧代码 | 新代码 | 说明 |
|------|--------|--------|------|
| `_ack_states = {}` | `all([]) = True` ✗ | `bool({}) = False` ✓ | 空字典不应进入下一步 |
| `_ack_states = {'usv_01': AckState(acked=False)}` | `all([False]) = False` ✓ | `True and all([False]) = False` ✓ | 有未确认USV |
| `_ack_states = {'usv_01': AckState(acked=True)}` | `all([True]) = True` ✓ | `True and all([True]) = True` ✓ | 所有USV已确认 |

#### 修复2: 超时处理

| 场景 | 旧代码 | 新代码 | 说明 |
|------|--------|--------|------|
| 3个USV都超时 | `acked=True` for all ✗<br>→ 0/3跳到3/3 | `acked=False` for all ✓<br>→ 保持0/3 | 超时失败不标记为成功 |
| 2个成功，1个超时 | `acked=True` for all ✗<br>→ 进入下一步 | 2个True，1个False ✓<br>→ 确认率66.7%<80%<br>→ 不进入下一步 | 依赖确认率判断 |
| 3个成功 | `acked=True` for all ✓<br>→ 进入下一步 | `acked=True` for all ✓<br>→ 确认率100%≥80%<br>→ 进入下一步 | 正常流程 |

## 其他改进

### 添加调试日志（第253-257行）

```python
# 调试日志：输出当前步骤的USV列表和确认状态
self.node.get_logger().debug(
    f"步骤 {self.node.run_step}: USV列表={[u.get('usv_id') for u in cluster_usv_list if isinstance(u, dict)]}, "
    f"确认状态={[(k, v.acked, v.retry) for k, v in self._ack_states.items()]}"
)
```

这将帮助诊断未来可能出现的类似问题。

## 测试建议

### 测试用例1：空确认状态
```python
controller._ack_states = {}
all_acked = bool(controller._ack_states) and all(...)
assert all_acked == False  # 应该不进入下一步
```

### 测试用例2：部分确认
```python
controller._ack_states = {
    'usv_01': AckState(acked=True),
    'usv_02': AckState(acked=False),
    'usv_03': AckState(acked=False)
}
all_acked = bool(controller._ack_states) and all(s.acked for s in controller._ack_states.values())
assert all_acked == False  # 只有1/3确认，不应进入下一步
```

### 测试用例3：全部确认
```python
controller._ack_states = {
    'usv_01': AckState(acked=True),
    'usv_02': AckState(acked=True),
    'usv_03': AckState(acked=True)
}
all_acked = bool(controller._ack_states) and all(s.acked for s in controller._ack_states.values())
assert all_acked == True  # 3/3确认，应该进入下一步
```

## 相关配置

确保以下参数正确配置：

```python
# ground_station_node.py
MIN_ACK_RATE_FOR_PROCEED = 0.8  # 最小确认率阈值（80%）
DEFAULT_MAX_RETRIES = 1          # 默认最大重试次数
```

## 预期效果

修复后的行为：
1. ✅ 只有当 `_ack_states` 非空且所有USV都已确认时才进入下一步
2. ✅ 空确认状态不会触发步骤切换
3. ✅ 部分确认不会触发步骤切换
4. ✅ 符合"完成度≥80%才进入下一步"的设计要求

## 修改文件

- `gs_gui/gs_gui/cluster_controller.py`（第257-271行）

## 修复日期

2025-11-19

## 相关问题与修复

### 问题2：任务完成后继续发送目标点

**位置**：`cluster_controller.py` 第207-222行

**问题描述**：
任务完成后，`_reset_cluster_task` 将 `run_step` 重置为 0，但定时回调 `publish_cluster_targets_callback` 仍在运行，导致发送 Step 0 的目标点。

**修复方案**：
```python
# 修改前
if self._state == ClusterTaskState.PAUSED:
    return

# 修改后
if self._state in (ClusterTaskState.PAUSED, ClusterTaskState.COMPLETED, ClusterTaskState.IDLE):
    return
```

### 其他潜在问题

1. **第570-591行**：基于 `ack_rate` 和 `all_processed` 的步骤切换逻辑
2. **第243行**：USV列表为空时直接 `run_step += 1` 而不调用 `_proceed_to_next_step()`

建议在测试时关注这些相关逻辑的正确性。
