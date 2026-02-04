# MPC v5: 一阶惯性转向模型升级

**日期**: 2026-02-03  
**作者**: chenhangwei  
**版本**: v5  
**状态**: 已实现，待测试

---

## 1. 背景与问题

### 1.1 问题现象

在 2026年1月30日 至 2月3日 的多次实地测试中，USV 持续出现 **S形路径振荡** (俗称"画龙")：

| 版本 | 日期 | 参数调整 | 结果 |
|------|------|----------|------|
| v1 | 1/30 | w_max=0.4, q_pos=10, r_w=5 | 大振幅低频振荡 |
| v2 | 2/02 | r_dw=20 (↑) | 振幅↓55%, 频率↑19% |
| v3 | 2/03 AM | q_pos=5, q_theta=8, w_max=0.3 | 饱和54.2%, 振荡持续 |
| v4 | 2/03 PM | q_pos=10, q_theta=12, w_max=0.5 | 仍有振荡 |

### 1.2 数据分析结论

通过对 `nav_log_20260203_141308_goal_7.csv` (USV_02) 和 `nav_log_20260203_141751_goal_24.csv` (USV_03) 的分析：

- **角速度饱和率**: 54.2% (USV_02), 51.7% (USV_03)
- **方向反转频率**: ~0.30 Hz (每3.3秒反向一次)
- **横向偏差 (CTE)**: ±1.2m
- **FFT分析**: 主振荡频率 0.3-0.5 Hz

### 1.3 根本原因分析

经过深入分析 MPC 架构，确定了 **两个根本原因**：

#### 原因1: 预测模型与实船不匹配 (主要原因)

| 对比项 | MPC预测模型 (Unicycle) | 实际船舶动力学 |
|--------|------------------------|----------------|
| 转向响应 | θ̇ = ω_cmd (即时响应) | 存在0.3-0.8秒惯性延迟 |
| 角速度 | 命令即等于实际 | 需要时间建立 |
| 预测精度 | 高估转向速度 | - |

**后果**: MPC 以为船能立即转向，实际船有延迟 → MPC 过度补偿 → 振荡

#### 原因2: 代价函数设计问题

| 问题 | 旧设计 | 新设计 |
|------|--------|--------|
| θ_target 计算 | 船→目标点方向 (随船移动) | 航线方向 (固定) |
| 横向控制 | 无 CTE 项 | 新增 CTE 权重 |

**后果**: 目标航向随船位置变化 → 追逐效应 → 振荡

---

## 2. 解决方案

### 2.1 一阶惯性转向模型

将 Unicycle 运动学模型升级为一阶惯性转向模型：

**旧模型 (Unicycle)**:
```
ẋ = v·cos(θ)
ẏ = v·sin(θ)
θ̇ = ω_cmd          ← 假设即时响应
```

**新模型 (一阶惯性)**:
```
ẋ = v·cos(θ)
ẏ = v·sin(θ)
θ̇ = ω_actual       ← 使用实际角速度
τ·ω̇_actual + ω_actual = ω_cmd   ← 一阶惯性方程
```

**离散化形式**:
```
ω_actual[k+1] = α·ω_actual[k] + (1-α)·ω_cmd[k]
其中 α = exp(-dt/τ)
```

### 2.2 状态扩展

| 对比 | 旧 (v1-v4) | 新 (v5) |
|------|------------|---------|
| 状态维度 | 3D: [x, y, θ] | 4D: [x, y, θ, ω_actual] |
| 控制维度 | 2D: [v, w] | 2D: [v, w_cmd] |
| 参数向量 P | 6维 | 9维 |

### 2.3 代价函数改进

**旧代价函数**:
```python
# 点对点距离
obj += Q_pos * ((x - target_x)² + (y - target_y)²)
# 航向误差 (目标方向随船位置变化!)
θ_target = atan2(target_y - y, target_x - x)
obj += Q_theta * (θ - θ_target)²
```

**新代价函数**:
```python
# Cross Track Error (横向偏差)
CTE = (x - start_x)·path_ny - (y - start_y)·path_nx
obj += Q_cte * CTE²

# 沿航线进度
along_track = (x - start_x)·path_nx + (y - start_y)·path_ny
remaining = path_len - along_track
obj += Q_pos * remaining²

# 航向误差 (使用固定航线方向!)
# 注意: 传入 MPC 的 θ 已经是 effective_yaw (速度向量方向)
# 见 velocity_path_tracker.py: _get_control_heading()
obj += Q_theta * (θ - path_theta)²
```

> **关于航向的说明**：
> 
> 代价函数中的 `θ` 并不是原始的船头朝向，而是 `effective_yaw`：
> - 当速度 > 0.2 m/s 时，使用 GPS/UWB 反馈的**速度向量方向 (Course)**
> - 当速度 < 0.2 m/s 时，回退到船头朝向 (Heading)
> 
> 这借鉴了 L1 导航算法的思想，能自动补偿侧向漂移和风流影响。
> 详见 `velocity_path_tracker.py` 中的 `_get_control_heading()` 方法。

---

## 3. 实现详情

### 3.1 修改的文件

| 文件 | 修改内容 |
|------|----------|
| `usv_control/usv_control/mpc_path_tracker.py` | 核心 MPC 控制器升级 |
| `usv_control/usv_control/velocity_path_tracker.py` | 添加新参数，传递航线起点 |
| `usv_control/usv_control/velocity_controller_node.py` | ROS 参数声明 |
| `usv_bringup/config/usv_params.yaml` | 新增 v5 参数 |

### 3.2 新增参数

```yaml
# v5 新增参数 (一阶惯性转向模型)
mpc_tau_omega: 0.4    # 转向时间常数 (秒) - 核心参数!
mpc_weight_cte: 15.0  # Cross Track Error权重
```

### 3.3 关键代码段

**动力学模型** (`mpc_path_tracker.py` L150-165):
```python
# 一阶惯性转向模型
alpha = ca.exp(-self.dt / self.tau_omega)
omega_next = alpha * omega_actual + (1 - alpha) * w_cmd

# 状态导数
rhs = ca.vertcat(
    v * ca.cos(theta),     # ẋ
    v * ca.sin(theta),     # ẏ
    omega_actual,          # θ̇ = 实际角速度 (关键!)
    (omega_next - omega_actual) / self.dt  # ω̇_actual
)
```

**CTE 计算** (`mpc_path_tracker.py` L195-205):
```python
# 船到航线起点的向量
dx_to_start = px - start_x
dy_to_start = py - start_y

# 横向误差 = 叉积 (垂直于航线的距离)
cross_track_error = dx_to_start * path_ny - dy_to_start * path_nx
obj += self.Q_cte * cross_track_error**2
```

---

## 4. 参数调优指南

### 4.1 tau_omega (转向时间常数)

**物理含义**: 船从发出角速度命令到实际角速度达到 63.2% 的时间

| tau_omega | 效果 |
|-----------|------|
| 0.3 秒 | MPC 认为船转向较快，输出较保守 |
| 0.4 秒 | 默认值，平衡点 |
| 0.5-0.6 秒 | MPC 认为船转向慢，会提前转向，输出更激进 |

**调参策略**:
- 仍有 S 形振荡 → 增大 tau_omega (0.5~0.6)
- 响应迟钝/转弯不及时 → 减小 tau_omega (0.3)

**系统辨识方法**:
```
1. 发送阶跃角速度命令 (如 0 → 0.3 rad/s)
2. 记录实际角速度响应曲线
3. tau = 达到目标值 63.2% 的时间
```

### 4.2 Q_cte (横向偏差权重)

| Q_cte | 效果 |
|-------|------|
| 10.0 | 允许较大横向偏差，轨迹更平滑 |
| 15.0 | 默认值 |
| 20-30 | 严格沿航线，横向偏差小 |

---

## 5. 预期效果

| 指标 | v4 (改进前) | v5 预期 |
|------|-------------|---------|
| 角速度饱和率 | 54.2% | < 30% |
| 方向反转频率 | 0.30 Hz | < 0.1 Hz |
| 横向偏差 (CTE) | ±1.2 m | < ±0.5 m |
| S形振荡 | 明显 | 基本消除 |

---

## 6. 测试计划

### 6.1 仿真测试
- [ ] 直线航行测试 (50m)
- [ ] 90° 转弯测试
- [ ] 8字形轨迹测试
- [ ] 不同 tau_omega 值对比

### 6.2 实地测试
- [ ] 白天静水条件
- [ ] 记录完整日志
- [ ] 对比 v4 vs v5 轨迹

### 6.3 数据收集
- 角速度饱和率
- CTE 分布
- 航向误差分布
- 求解时间

---

## 7. 回滚方案

如果 v5 效果不佳，可通过调整参数回退到接近 v4 的行为：

```yaml
# 回退到 v4 行为 (不推荐，仅供紧急情况)
mpc_tau_omega: 0.05   # 极小值，接近 Unicycle 模型
mpc_weight_cte: 0.1   # 极小值，禁用 CTE 控制
```

---

## 8. 参考资料

1. MPC 路径跟踪理论: [Model Predictive Control for Vehicle Dynamics](https://arxiv.org/...)
2. 船舶动力学建模: Kong, J. et al. "Kinematic and dynamic vehicle models for autonomous driving control design"
3. CasADi 文档: https://web.casadi.org/docs/

---

## 9. 变更日志

| 日期 | 版本 | 变更内容 |
|------|------|----------|
| 2026-02-03 | v5.0 | 初始实现：一阶惯性模型 + CTE 控制 |
