# MAPPO Migration Plan For Multi-USV Training

## 1. 结论

对于当前项目，长期最值得投入的 RL 路线是 MAPPO，而不是继续在单智能体 PPO residual 上做局部修补。

原因不是 MAPPO 更“先进”，而是当前瓶颈已经明确来自多艇耦合会遇，尤其是 dense head_on。现有训练栈本质上仍然是：

1. 单个 ego USV 作为唯一 agent
2. 其它船只只作为观测中的邻船输入
3. 策略输出 residual action 覆盖本艇 raw 导航命令

这条路线对 crossing 和 overtaking 已经接近上限，但在 dense 多船 head_on 里明显受限。

## 2. 当前代码结构

当前训练栈是单智能体：

1. 环境: [usv_rl/usv_rl/env.py](usv_rl/usv_rl/env.py)
2. 单艇观测定义: [usv_rl/usv_rl/types.py](usv_rl/usv_rl/types.py)
3. 单艇桥接: [usv_rl/usv_rl/ros_bridge.py](usv_rl/usv_rl/ros_bridge.py)
4. 单艇轻量动力学: [usv_rl/usv_rl/simple_sim.py](usv_rl/usv_rl/simple_sim.py)
5. 训练入口: [usv_rl/usv_rl/train_ppo_residual.py](usv_rl/usv_rl/train_ppo_residual.py)

它的核心假设是“只学一条船如何对多邻船做 residual avoidance”。

这和 MAPPO 的目标不同。MAPPO 需要：

1. 多个 agent 同时决策
2. actor 分散执行
3. critic 集中训练，看到联合状态

## 3. 目标架构

建议的目标形态是 CTDE：Centralized Training, Decentralized Execution。

### 3.1 Actor

1. 每条 USV 一个 actor 输入本地观测
2. 同构船只共享 actor 权重
3. actor 仍输出 residual action，而不是直接替代 MPC/AMPC

保留 residual 设计很重要，因为它和当前控制器架构兼容，风险最小。

### 3.2 Critic

1. critic 输入联合全局状态
2. 全局状态至少包含所有训练船的 pose、speed、goal error、raw/final cmd、邻船关系摘要
3. critic 只用于训练，不参与在线部署

### 3.3 Reward

建议用混合奖励，而不是纯全局或纯个体奖励：

1. 个体项: progress、collision、min separation、smoothness、heading
2. 团队项: fleet min separation、整体冲突率、任务完成率、阻塞惩罚
3. 最终奖励 = 个体奖励 + alpha * 团队奖励

其中 alpha 建议从 0.2 到 0.4 起步，不要一开始就全团队共享。

## 4. 最小改造路径

不要一上来重写整个训练框架。建议分 4 个阶段。

### 阶段 A: 环境多智能体化

新增一套并行于现有单智能体环境的多智能体环境，不要直接破坏当前 [usv_rl/usv_rl/env.py](usv_rl/usv_rl/env.py)。

建议新增文件：

1. `usv_rl/usv_rl/multi_agent_env.py`
2. `usv_rl/usv_rl/multi_agent_types.py`
3. `usv_rl/usv_rl/multi_agent_bridge.py`
4. `usv_rl/usv_rl/multi_usv_sim.py`

目标接口：

1. `reset()` 返回 `dict[agent_id, obs]`
2. `step(actions)` 接收 `dict[agent_id, action]`
3. 同时返回 `obs_dict, reward_dict, done_dict, info_dict`

### 阶段 B: 联合状态构造

新增 centralized critic 所需的全局状态编码器。

建议新增：

1. `build_local_observation(agent_id)`
2. `build_global_state()`

全局状态不要直接拼接无限原始对象，先固定长度。

建议编码：

1. 每艇自身特征 10 到 14 维
2. 每艇最近 K 个邻居特征
3. 额外 fleet-level 摘要特征，例如最小艇间距、平均 closing risk、goal completion ratio

### 阶段 C: MAPPO 训练器

不要继续复用 stable-baselines3 的单智能体 PPO 训练入口。SB3 标准 PPO 不适合直接做 MAPPO。

建议新增独立训练入口：

1. `usv_rl/usv_rl/train_mappo_residual.py`

训练器至少需要：

1. shared actor
2. centralized critic
3. multi-agent rollout buffer
4. GAE
5. PPO clipped objective
6. 可选 value normalization

如果你想降低首轮工程量，第一版只做：

1. 参数共享 actor
2. 单 centralized critic
3. 同步 on-policy rollout

不要第一版就上 RNN、attention、通信模块。

### 阶段 D: 在线部署保持单艇接口

在线部署时不需要同时发多条艇的 actor 到当前 SITL overlay 入口。

建议保持现有部署思路：

1. 每条艇各自运行一个 inference node
2. 都加载同一份共享 actor 权重
3. 不在线使用 critic

也就是说，训练是 MAPPO，部署仍然是当前 residual policy inference 模式。

## 5. 针对当前仓库的具体改造点

### 5.1 观测层

当前 [usv_rl/usv_rl/types.py](usv_rl/usv_rl/types.py) 里的 `UsvObservation` 是单艇结构。

MAPPO 版本建议新增：

1. `AgentObservation`
2. `FleetState`

其中：

1. `AgentObservation` 给 actor
2. `FleetState` 给 critic

### 5.2 ROS bridge

当前 [usv_rl/usv_rl/ros_bridge.py](usv_rl/usv_rl/ros_bridge.py) 只维护一个 namespace 下的 pose、velocity、feedback、raw_cmd、final_cmd。

MAPPO 版本需要变成：

1. 支持多个 namespace
2. 按 `agent_id -> topic set` 管理缓存
3. 同时发布多个 navigation goal
4. 同时发布多个 RL residual action

建议保留当前 `TrainingBridge`，新增 `MultiAgentTrainingBridge`。

### 5.3 轻量仿真

当前 [usv_rl/usv_rl/simple_sim.py](usv_rl/usv_rl/simple_sim.py) 只模拟一条本艇。

MAPPO 需要：

1. `MultiUsvSimNode` 同时维护 N 条训练艇状态
2. 每条艇有独立命令输入和状态输出
3. 邻船关系由真实其它训练艇自动形成，而不是只靠 synthetic neighbors

如果第一版工程量受限，也可以采用混合方案：

1. 训练 2 到 3 条可控艇
2. 其余背景艇仍由 scenario 脚本驱动

这会比一步到位 5 条全可控艇更稳。

### 5.4 场景系统

当前 [usv_rl/usv_rl/scenarios.py](usv_rl/usv_rl/scenarios.py) 更偏“ego + scripted neighbors”。

MAPPO 需要把场景拆成两部分：

1. 初始 fleet layout
2. 每个 agent 的目标与角色

建议新结构：

1. `FleetScenario`
2. `AgentSpawnConfig`
3. `AgentGoalConfig`
4. `BackgroundTrack`

## 6. 算法细节建议

### 6.1 动作空间

第一版仍然使用当前 residual full-action：

1. `linear_delta`
2. `angular_delta`

不要在 MAPPO 第一版引入直接控制绝对速度或额外高维动作，否则训练会更不稳。

### 6.2 参数共享

对同构 USV，建议共享 actor 参数。

原因：

1. 样本效率更高
2. 部署更简单
3. 多艇策略一致性更好

### 6.3 critic 输入

critic 输入建议包含：

1. 所有训练艇的本地状态
2. 所有训练艇的目标误差
3. 所有训练艇的 raw/final 控制量
4. 关键邻接关系摘要

不要把 critic 输入做得过度稀疏，否则 MAPPO 的优势出不来。

### 6.4 奖励共享策略

建议：

1. 碰撞惩罚做团队共享
2. 进度奖励保留个体项
3. 最小间距约束加入团队项
4. 抢路、互锁、拥塞加入 fleet 级惩罚

这比完全共享 reward 更适合你这个“既要协同又要每艇到点”的任务。

## 7. 为什么不是别的算法

### 7.1 不是继续单智能体 PPO

因为它已经在当前仓库里被验证过上限：

1. crossing 和 overtaking 能做
2. dense head_on 长期过不了 0.5m floor
3. teacher + BC 反而更强

### 7.2 不是 SAC / TD3

这类离策略连续控制算法理论上可用，但对于你现在这种：

1. ROS 闭环环境
2. 非平稳多艇交互
3. residual safety semantics

工程稳定性通常不如 PPO/MAPPO 体系。

### 7.3 不是 QMIX

QMIX 更适合离散动作或显式 value decomposition 场景，不适合你当前这种连续 residual control。

## 8. 推荐实施顺序

### 短期

1. 保持 BC v2 作为默认上线基线
2. 不再继续在单智能体 PPO residual 上做大规模超参搜索

### 中期

1. 新建多智能体环境和 bridge
2. 先做 2 到 3 艇 MAPPO smoke
3. 先验证是否能稳定优于单智能体 PPO，而不是立即替代 BC

### 长期

1. 扩到 5 艇全可控训练
2. 加入 richer fleet reward
3. 再评估是否需要 attention critic 或 RNN

## 9. 最小可交付版本

如果只做一个最小可交付的 MAPPO 版本，建议范围控制在：

1. 2 艇或 3 艇可控 USV
2. shared actor + centralized critic
3. full residual action
4. 只支持 `head_on`, `crossing_starboard`, `overtaking` 三类基础场景
5. 单独训练入口 `train_mappo_residual.py`

这个版本的目标不是立刻替代所有现有策略，而是回答一个关键问题：

“联合训练是否真的能把 dense 多艇 head_on 的安全边界拉过单智能体 residual PPO/BC 的当前上限？”

只有这个问题被证明成立，后续扩大工程投入才值得。