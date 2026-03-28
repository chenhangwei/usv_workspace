# Teacher-Student 蒸馏方案设计

## 1. 背景与动机

### 1.1 问题陈述

PPO 微调已被证明无法改善或维持 a10 的避碰能力：

| 实验 | 基线 | 方法 | 结果 |
|------|------|------|------|
| headon21 | h20@9600 | PPO (LR=1e-5, clip=0.1) | 冻结策略：零碰撞伴随负进度 |
| headon22 | a10@358400 | PPO (LR=5e-6, clip=0.05) | a10 策略在 ~25K 步内被摧毁 |

**根因**：PPO 的 on-policy exploration 即使在极保守参数下也会产生破坏性梯度更新，特别是在多智能体密集避碰场景中，策略对微小扰动极其敏感。

### 1.2 为什么选择蒸馏

| 方案 | 优势 | 风险 |
|------|------|------|
| PPO 微调 | 理论上可持续改进 | 已证实破坏 a10 策略 |
| BC 纯模仿 | 离线训练，不破坏教师 | 需要大量轨迹，分布偏移 |
| **蒸馏 (DAgger 变体)** | **离线收集+迭代修正，无 on-policy 风险** | **需要环境交互收轨迹** |

蒸馏方案将 a10 作为教师，offline 收集轨迹后训练学生网络，**完全避免 PPO 梯度更新对教师策略的破坏**。

---

## 2. 整体架构

```
┌─────────────┐     obs      ┌──────────────┐
│  MultiAgent │ ───────────► │  Teacher a10 │ ──► teacher_action
│  Env        │              │  (frozen)    │
│  (5 USV)    │              └──────────────┘
│             │     obs      ┌──────────────┐
│             │ ───────────► │  Student Net │ ──► student_action
│             │              │  (trainable) │
└─────────────┘              └──────────────┘
                                    │
                    L_distill = MSE(student_action, teacher_action)
                                  + α · L_feature (optional)
```

### 2.1 阶段划分

| 阶段 | 名称 | 输入 | 输出 |
|------|------|------|------|
| Phase 1 | 轨迹收集 | teacher + env | `{obs, action, reward, done}` 数据集 |
| Phase 2 | 离线蒸馏 (BC) | 数据集 | 学生模型 v0 |
| Phase 3 | DAgger 迭代 | 学生模型 + teacher + env | 学生模型 v1..vN |
| Phase 4 | 评测 & 毕业 | 学生模型 | 通过/不通过 |

---

## 3. Phase 1: 轨迹收集

### 3.1 收集脚本设计

```python
# usv_rl/usv_rl/collect_teacher_trajectories.py

def collect_trajectories(
    model_path: str,
    output_path: str,
    episodes: int = 200,
    steps_per_episode: int = 180,
    scenarios: tuple[str, ...] = (
        'five_usv_dense_head_on',
        'five_usv_dense_crossing',
        'five_usv_dense_overtaking',
    ),
):
    """
    用教师策略在 MultiAgentEnv 中收集 (obs, action) 对。

    输出格式 (.npz):
        observations: float32 [N, obs_dim]    -- 所有智能体所有步的观测
        actions:      float32 [N, action_dim] -- 教师的动作输出
        scenarios:    str     [N]             -- 每条数据对应的场景
        episode_ids:  int32   [N]             -- 每条数据对应的 episode
        agent_ids:    str     [N]             -- 每条数据对应的智能体
    """
```

### 3.2 收集参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 教师模型 | a10@358400 | 当前正式基线 |
| 场景 | 3 个 dense (5-USV) | head_on, crossing, overtaking |
| 每场景 episode 数 | 200 | 总 600 episodes |
| 步数/episode | 180 | 匹配训练 horizon |
| 预计样本量 | ~600 × 180 × 5 = 540,000 | 5 agent × 180 steps × 600 episodes |

### 3.3 数据增强（可选）

- **动作噪声注入**：在教师动作上加 $\sigma=0.02$ 高斯噪声后重新查询教师，模拟分布偏移
- **初始姿态抖动**：在场景初始位置添加 $\pm 0.3 \text{m}$ 随机偏移

---

## 4. Phase 2: 离线蒸馏 (Behavior Cloning)

### 4.1 学生网络架构

学生网络与教师使用 **相同架构**，确保运行时兼容性：

```
MLP: obs_dim(34) → 256 → Tanh → 256 → Tanh → action_dim(2)
```

> 架构一致让学生模型可以直接替换教师模型，无需修改 `policy_inference_node.py` 或 `MappoActorPolicyRuntime`。

### 4.2 训练配置

```python
# distillation_config
student_hidden_sizes = (256, 256)   # 与教师一致
learning_rate = 1e-3                # BC 可以用较大 LR
batch_size = 512
epochs = 100
optimizer = 'Adam'
loss = 'MSE'                        # L2 on action space
validation_split = 0.1
early_stopping_patience = 10
```

### 4.3 损失函数

$$
\mathcal{L}_{\text{BC}} = \frac{1}{N} \sum_{i=1}^{N} \| \pi_\theta(o_i) - a_i^{\text{teacher}} \|^2
$$

其中 $o_i$ 是观测，$a_i^{\text{teacher}}$ 是教师在该观测下的动作。

### 4.4 检查点保存格式

学生模型保存为 `.pt` 格式，包含与 MAPPO 检查点相同的元数据字段：

```python
checkpoint = {
    'actor_state_dict': student.state_dict(),
    'hidden_sizes': [256, 256],
    'local_observation_size': 34,
    'action_dim': 2,
    'action_mode': 'full',
    'action_bounds': {'linear_delta': 0.7, 'angular_delta': 0.6},
    'agent_namespaces': ['usv_01', ..., 'usv_05'],
    'max_neighbors': 4,
    'scenarios': [...],
    # 完整复制 a10 的 reward_config, env params 等
    'distillation_metadata': {
        'teacher_checkpoint': 'a10@358400',
        'phase': 'bc',      # 或 'dagger_round_N'
        'training_samples': 540000,
    },
}
```

---

## 5. Phase 3: DAgger 迭代

### 5.1 DAgger 流程

```
for round in range(N_DAGGER_ROUNDS):
    1. 用当前学生策略在 env 中 rollout
    2. 对每步观测，查询教师策略获取 teacher_action
    3. 将 (obs, teacher_action) 加入训练集
    4. 在扩充后的数据集上重新训练学生
    5. 评测学生 → 如果通过则毕业
```

### 5.2 DAgger 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| DAgger 轮次 | 最多 5 轮 | 通常 2-3 轮即足够 |
| 每轮新增 episodes | 100 (× 3 场景) | 追加到已有数据集 |
| 混合比例 β | 从 0.9 → 0.0 | 第 1 轮 90% 学生控制，逐轮降低教师介入 |
| 训练 epoch | 50/轮 | 在累积数据集上微调 |

### 5.3 关键实现细节

```python
def dagger_collect(student, teacher, env, episodes, beta=0.5):
    """
    beta: 学生动作占比。1.0 = 纯学生控制, 0.0 = 纯教师控制
    无论谁控制，标签始终是教师的动作。
    """
    dataset = []
    for ep in range(episodes):
        obs, info = env.reset(options={'scenario_kind': next_scenario()})
        for step in range(max_steps):
            teacher_actions = {aid: teacher.predict(obs[aid]) for aid in env.agent_ids}
            student_actions = {aid: student.predict(obs[aid]) for aid in env.agent_ids}

            # 实际执行动作 = 混合
            exec_actions = {}
            for aid in env.agent_ids:
                if np.random.random() < beta:
                    exec_actions[aid] = student_actions[aid]
                else:
                    exec_actions[aid] = teacher_actions[aid]

            # 标签始终是教师的动作（在学生观测下的）
            for aid in env.agent_ids:
                dataset.append((obs[aid].copy(), teacher_actions[aid].copy()))

            obs, _, terminated, truncated, info = env.step(exec_actions)
            if terminated['__all__'] or truncated['__all__']:
                break
    return dataset
```

---

## 6. Phase 4: 评测与毕业标准

### 6.1 毕业标准

学生模型必须在 **全部 6 个场景** 的 24-episode 评测中满足：

| 指标 | 阈值 | 说明 |
|------|------|------|
| 碰撞率 | ≤ 0.05 (5%) | 所有场景加权平均 |
| head_on 碰撞率 | = 0.0 | head-on 必须零碰撞 |
| 进度比 | ≥ +0.02 | 整体正向进度 |
| 最差场景碰撞率 | ≤ 0.20 | 任何单场景碰撞不超 20% |

### 6.2 评测命令

```bash
evaluate_mappo_policy \
  --policy mappo \
  --model <student_model.pt> \
  --episodes 24 \
  --steps-per-episode 180 \
  --scenario five_usv_dense_head_on \
  --scenario five_usv_dense_crossing \
  --scenario five_usv_dense_overtaking \
  --scenario two_usv_head_on \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --output-json <eval_output.json>
```

### 6.3 与 a10 基线对比

评测结果需与 a10 回归基线（`regression_summary.json`）对比。学生模型的目标不是在所有指标上超越 a10，而是：
1. 维持 a10 在 head-on 的零碰撞能力
2. **改善** overtaking 场景的碰撞率（a10 当前 37.5-50%）
3. 维持或改善整体进度

---

## 7. 实现计划

### 7.1 文件结构

```
usv_rl/usv_rl/
├── collect_teacher_trajectories.py   # Phase 1: 轨迹收集 CLI
├── train_distill_policy.py           # Phase 2+3: BC + DAgger 训练 CLI
└── evaluate_mappo_policy.py          # Phase 4: 已有评测工具
```

### 7.2 执行步骤

```bash
# Step 1: 收集教师轨迹 (~540K samples)
python3 -m usv_rl.collect_teacher_trajectories \
  --model /path/to/a10_step_358400.pt \
  --episodes 200 \
  --output /mnt/data/distillation/teacher_trajectories.npz

# Step 2: 离线 BC 训练
python3 -m usv_rl.train_distill_policy \
  --teacher /path/to/a10_step_358400.pt \
  --trajectories /mnt/data/distillation/teacher_trajectories.npz \
  --output /mnt/data/distillation/student_bc_v1.pt \
  --epochs 100 --lr 1e-3 --batch-size 512

# Step 3: 评测 BC 学生
evaluate_mappo_policy --policy mappo --model student_bc_v1.pt ...

# Step 4: DAgger 迭代 (如果 BC 不达标)
python3 -m usv_rl.train_distill_policy \
  --teacher /path/to/a10_step_358400.pt \
  --student /mnt/data/distillation/student_bc_v1.pt \
  --dagger-rounds 3 \
  --dagger-episodes 100 \
  --output /mnt/data/distillation/student_dagger_v1.pt

# Step 5: 最终评测
evaluate_mappo_policy --policy mappo --model student_dagger_v1.pt ...
```

---

## 8. 风险与缓解

| 风险 | 影响 | 缓解措施 |
|------|------|----------|
| BC 分布偏移 | 学生在未见状态下表现差 | DAgger 迭代修正 |
| 教师本身 overtaking 弱 | 学生继承弱点 | 可混合场景权重或手写 overtaking 规则教师 |
| 训练数据量不足 | 学生泛化差 | 每场景 200 episodes + 数据增强 |
| 架构兼容性 | 部署失败 | 保持与 MAPPO 完全相同的 checkpoint 格式 |
| 环境代码有变更 | 评测结果不可比 | 锁定代码版本后再收集轨迹 |

---

## 9. 当前 a10 基线参考指标

回归评测结果 (24 episodes, 180 steps, 最新代码):

| 场景 | 碰撞率 | 进度比 | 注解 |
|------|--------|--------|------|
| five_usv_dense_head_on | 0.000 | -0.069 | 零碰撞，但进度为负 |
| five_usv_dense_crossing | 0.000 | -0.020 | 零碰撞 |
| five_usv_dense_overtaking | 0.375 | -0.061 | **蒸馏重点改善目标** |
| two_usv_head_on | 0.000 | -0.147 | 零碰撞 |
| three_usv_crossing | 0.000 | +0.014 | 零碰撞 |
| three_usv_overtaking | 0.500 | +0.064 | **蒸馏重点改善目标** |

> 注意：由于 `multi_agent_env.py` 有未提交的代码变更（heading gate、spin penalty、head-on reward 等），当前评测结果与原始 a10 12-episode 评测不一致。蒸馏方案应基于最新代码版本进行。
