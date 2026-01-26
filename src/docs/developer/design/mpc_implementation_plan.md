# MPC 实施规划与算力评估 (MPC Implementation Strategy)

**作者**: chenhangwei
**日期**: 2026-01-26
**目标硬件**: Raspberry Pi 5 (8GB/4GB)

## 1. 硬件算力评估 (Raspberry Pi 5)

对于自动控制任务，RPi 5 是一个巨大的飞跃。

*   **CPU**: Broadcom BCM2712 (四核 Cortex-A76 @ 2.4GHz)。
*   **性能对比**: 相比 RPi 4，单核性能提升约 2.5 倍，多核性能提升类似。
*   **负载预测**:
    *   **USV 动态特性**: 水面艇惯性大，控制频率通常只需 **10Hz - 20Hz**。
    *   **求解器消耗**: 一般的非线性 MPC (NMPC) 在 Python/CasADi 优化后的单次求解时间在 5ms - 20ms 之间。
    *   **结论**: RPi 5 可以在使用单核不到 30% 占用的情况下运行复杂的 NMPC 算法。这给感知（Lidar/Camera）和通信留出了充足的余量，是性能“绰绰有余”的选择。

## 2. 软件技术栈选型

鉴于您目前使用 Python 进行开发，且 RPi 5 算力充足，推荐以下“快速开发”路线：

| 组件 | 推荐选择 | 理由 |
| :--- | :--- | :--- |
| **编程语言** | **Python** | 保持与现有节点 (`velocity_controller_node`) 一致，便于调试和集成。 |
| **数学建模库** | **CasADi** | 目前最流行的 Python 优化库，支持自动微分，语法接近 MATLAB，上手极快。 |
| **求解器** | **Ipopt** | CasADi 内置，对于几十个变量的优化问题（USV 通常的规模），稳定且足够快。 |
| **备选方案** | **Acados** | 如果未来需要 >100Hz 的频率，可以切换到 Acados（基于 C 代码生成），但配置较复杂，目前阶段非必须。 |

## 3. MPC 建模步骤 (Roadmap)

### 第一阶段：运动学 MPC (Kinematic MPC)
*不考虑质量和力，只考虑几何关系和速度约束。*

*   **状态量 (State)**: $x = [x, y, \psi]$ (位置 x, y, 航向)
*   **控制量 (Control)**: $u = [v, \omega]$ (线速度, 角速度)
*   **约束 (Constraints)**:
    *   $v_{min} \le v \le v_{max}$ (防止倒车或过快)
    *   $|\omega| \le \omega_{max}$ (最大转弯速率限制)

**优势**: 此模型可以直接替换您现在的 Pure Pursuit/Stanley，能更好地处理“死角”和“预瞄”，且计算量极小。

### 第二阶段：动力学 MPC (Dynamic MPC)
*考虑水的阻力、船的质量。*

*   **状态量**: $x = [x, y, \psi, u, v, r]$ (增加了纵向速度 u, 横向速度 v, 角速度 r)
*   **控制量**: $u = [T_{left}, T_{right}]$ (左右推力) 或 $[T, \delta]$ (推力 + 舵角)

**核心模型 (Fossen 模型简化版)**:
$$
\dot{u} = \frac{T - X_u u - X_{uu}|u|u}{m}
$$
$$
\dot{r} = \frac{N_r r - N_{rr}|r|r + M_{rudder}}{I_z}
$$

**优势**: 能完美处理**惯性**。例如快到终点时，控制器知道“水有阻力但船有惯性”，它会提前断油，利用水的阻力让船滑行减速，而不是像 PID 那样容易产生激烈的反向刹车。

## 4. 行动计划 (Action Plan)

1.  **环境准备** (在 RPi 5 上):
    ```bash
    pip3 install casadi numpy
    ```
2.  **数据采集 (System Identification)**:
    *   记录船在不同油门下的加速度、最大速度、旋转半径。
    *   利用采集的数据（GPS 速度变化）估算 $X_u$ (阻力系数) 和 $m$ (质量) 参数。
3.  **开发**:
    *   创建一个新的节点 `mpc_controller_node.py`。
    *   使用 CasADi 编写目标函数：$$ J = \sum (x_{err}^2 + \lambda \cdot u^2) $$。

## 5. 常见风险
*   **风险**: 计算延迟导致控制发散。
*   **对策**: RPi 5 性能足够，如果遇到延迟，可利用 CasADi 的 C 代码生成功能，或者降低预测时域 (Prediction Horizon)。
