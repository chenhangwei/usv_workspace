# 速度规划算法设计说明 (Velocity Planning Design)

**作者**: chenhangwei
**日期**: 2026-01-26
**状态**: 待实现

## 1. 概述
当前我们的控制器主要关注“位置”和“航向”的准确性，而速度控制相对简单（通常是固定的巡航速度或简单的线性衰减）。

引入**速度规划 (Velocity Planning)** 的目的是为了解决以下问题：
1.  **过弯失稳**: 在急转弯时若保持高速，会导致侧滑、甚至翻船，且会导致巨大的超调（Overshoot）。
2.  **到达震荡**: 快到达目标点时如果突然刹车，容易造成船体俯仰震荡；如果没有减速逻辑，则会冲过头。
3.  **运动不平滑**: 简单的启停逻辑会导致电机频繁加减速，损害硬件且不节能。

## 2. 详细设计方案

我们将速度规划分为三个独立的因子，最终输出速度为三者的乘积（或最小值）。

$$ V_{final} = V_{max} \times k_{curvature} \times k_{distance} $$

### 2.1 基于曲率的减速 (Curvature-based Slowdown)

由于我们目前主要基于离散航点（Waypoint），无法直接获取连续曲线的数学曲率 $\kappa$。因此，我们使用**航向偏差 (Heading Error)** 作为曲率的近似代理。

**逻辑**:
当 USV 当前航向与此时刻应有的期望航向偏差越大，说明正在进行越急剧的转向，此时应降低速度。

**算法公式**:
使用简单的线性映射或余弦衰减：

$$ k_{curvature} = \max(V_{min\_ratio}, 1.0 - \frac{|\theta_{error}|}{\pi}) $$

*   $\theta_{error}$: 当前航向与 Pure Pursuit 目标点的夹角。
*   $V_{min\_ratio}$: 允许的最小转弯速度比例（例如 0.3，表示转弯时至少保留 30% 速度以维持舵效）。

### 2.2 基于剩余距离的速度规划 (Distance-based Planning)

为了实现平滑停船，我们需要在接近目标点时生成一个梯形或 S 型速度曲线。

**逻辑**:
定义一个“减速半径” (Slowdown Radius)。
*   **距离 > 减速半径**: 保持巡航速度。
*   **距离 < 减速半径**: 速度随距离线性或平方根下降。

**算法公式**:
$$ k_{distance} = \min(1.0, \frac{d_{remain}}{d_{slowdown}})^\alpha $$

*   $d_{remain}$: 距离当前目标点的欧几里得距离。
*   $d_{slowdown}$: 开始减速的距离阈值（例如 5.0 米）。
*   $\alpha$: 衰减指数。
    *   $\alpha=1$: 线性减速（简单，但最后阶段可能太慢）。
    *   $\alpha=0.5$: 平方根减速（刹车感更强，入位更准）。

### 2.3 优化的 Heading Factor (Smart Heading Factor)

这与 2.1 类似，但更侧重于**起步阶段**或**原地掉头阶段**。

**逻辑**:
目前的 `heading_factor` 可能过于简单，导致只要有一点偏航速度就掉很多。我们需要一个**死区 (Deadzone)** 和 **过渡区**。

*   **小误差 (e.g., < 10度)**: 全速前进，忽略微小航向抖动。
*   **大误差 (e.g., > 90度)**: 停车或极低速，原地旋转（如果由于双发差速支持）。

## 3. 实现指南

### 代码位置
修改 `src/usv_control/usv_control/velocity_path_tracker.py` 中的 `compute_velocity` 方法。

### 伪代码示例

```python
def compute_velocity(self, current_pose, target_pose, params):
    # 1. 计算基础数据
    dist = self.get_distance(current_pose, target_pose)
    heading_error = self.get_heading_error(current_pose, target_pose)
    
    # 2. 距离因子 (入位减速)
    slowdown_dist = 5.0  # 5米开始减速
    min_speed_at_target = 0.2
    
    if dist < slowdown_dist:
        # 线性减速，但保留最低速度防止由于阻力停在目标前
        k_dist = max(min_speed_at_target, dist / slowdown_dist)
    else:
        k_dist = 1.0
        
    # 3. 曲率因子 (过弯减速)
    # 假设 heading_error 为弧度
    # 这是一个比较柔和的衰减曲线，误差 90度时速度减半
    k_curve = max(0.4, 1.0 - abs(heading_error) / 3.14159)
    
    # 4. 综合计算
    target_speed = params.max_speed * k_dist * k_curve
    
    return target_speed
```

## 4. 预期效果

| 场景 | 现有表现 | 优化后表现 |
| :--- | :--- | :--- |
| **直角转弯** | 船只全速冲过去，转弯半径极大，严重偏离航线。 | 接近转弯点前自动减速，慢速小半径转过弯角，然后加速。 |
| **到达终点** | 全速冲向终点，然后突然急停，船头下沉震荡。 | 在终点前 5 米开始均匀减速，温柔停在目标点。 |
| **受风浪偏航** | 频繁加减速，甚至停下调整角度。 | 在小角度偏差内保持速度，只在一侧电机发力修正，不损失整体动量。 |
