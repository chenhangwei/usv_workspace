# usv_rl

面向当前仓库的最小 RL 训练骨架。

当前目标不是端到端接管整条导航链，而是保持 MPC/AMPC 导航层不变，由 RL 策略承担局部避让；在 RL 在线路径中不再执行 ORCA 避让算法。

当前实现采用下面这条 MVP 路线：

1. 在训练进程内直接创建 `velocity_controller_node`。
2. 在同一训练进程内启动一个轻量动力学仿真节点，模拟本船状态与飞控状态。
3. 由训练环境合成邻船轨迹，并发布到 `apf/neighbors`。
3. 控制器额外发布进入 RL 避让前的原始导航命令 `velocity_controller/raw_cmd`。
4. 专家数据直接使用 scripted full-action 策略动作定义。
5. 先做行为克隆基线，再做 PPO 微调。

## 已新增的运行时接缝

- `rl_policy_enabled`: 是否启用 RL 旁路。
- `rl_policy/cmd_vel`: RL 动作输入话题，类型为 `geometry_msgs/msg/TwistStamped`。
- `velocity_controller/raw_cmd`: RL 策略接管前的原始导航命令，类型为 `geometry_msgs/msg/TwistStamped`。

## 目录说明

- `usv_rl/env.py`: 训练环境包装。
- `usv_rl/simple_sim.py`: 轻量本船动力学仿真节点。
- `usv_rl/controller_session.py`: 旧的子进程控制器启动器，当前 MVP 路径已不再默认使用。
- `usv_rl/scenarios.py`: 单艇 + 合成邻船会遇场景。
- `usv_rl/ros_bridge.py`: 观测采样、目标注入、RL 动作发布、参数切换。
- `usv_rl/collect_scripted_dataset.py`: 基于可解释制动规则的 scripted teacher 数据采集。
- `usv_rl/train_behavior_cloning.py`: 支持线性 ridge 和 MLP 的行为克隆基线。
- `ros2 run usv_rl train_ppo_policy`: PPO 微调入口，需要额外安装依赖。
- `usv_rl/policy_inference_node.py`: 在线加载策略并发布到 `rl_policy/cmd_vel`。

## 最小使用流程

### 1. 构建

```bash
cd ~/usv_workspace/src
colcon build --packages-select usv_control usv_rl
source install/setup.bash
```

### 2. 采集 scripted teacher 数据

如果当前主要瓶颈是 `head_on`，可以先采一份以“提前制动 + 让路转向”为主的 scripted teacher 数据，用于行为克隆或 PPO 预训练：

```bash
ros2 run usv_rl collect_scripted_dataset \
  --output ~/usv_rl_datasets/scripted_teacher_wide.npz \
  --episodes 18
```

这份 teacher 当前更适合做初始化，而不是直接作为最终策略。

如果你想在不破坏 `head_on` 强退让的前提下，尝试提升 `crossing_starboard` 和 `overtaking` 的在线通行效率，可以使用新的安全锚定 teacher：

```bash
ros2 run usv_rl collect_scripted_dataset \
  --output ~/usv_rl_datasets/scripted_teacher_safety_anchor_v3.npz \
  --episodes 18 \
  --teacher-profile safety_anchor_v3
```

这条 profile 会完整保留 v2 的 `head_on` 强制制动规则，只对 crossing/overtaking 的策略动作做更轻的制动和更明确的转向承诺，因此它适合作为“效率改进候选”，不适合作为放宽安全约束的激进通过策略。

### 3. 训练行为克隆基线

```bash
ros2 run usv_rl train_behavior_cloning \
  --dataset ~/usv_rl_datasets/scripted_teacher_wide.npz \
  --output ~/usv_rl_models/bc_linear_policy.npz
```

如果要训练一个更强的非线性基线，可以直接切到 MLP：

```bash
ros2 run usv_rl train_behavior_cloning \
  --dataset ~/usv_rl_datasets/scripted_teacher_wide.npz \
  --output ~/usv_rl_models/bc_mlp_policy.npz \
  --model-type mlp \
  --hidden-size 128 \
  --hidden-size 128 \
  --epochs 120
```

当前基线是线性 ridge policy，目的不是直接追求最强性能，而是先验证：

1. 观测定义是否稳定。
2. 专家策略动作是否有一致模式。
3. 环境与数据链是否闭环通顺。

在当前仓库内，`scripted teacher + MLP BC` 已经验证可把 `head_on` 和 `overtaking` 都从碰撞推到超时保守策略，因此它是目前最稳的“安全优先”初始化基线。

### 4. PPO 微调

这一步需要你额外安装 `gymnasium` 和 `stable-baselines3`。当前仓库里的 PPO 入口已经支持小 rollout 参数，便于快速冒烟验证。

```bash
ros2 run usv_rl train_ppo_policy \
  --output ~/usv_rl_models/ppo_policy_usv03 \
  --total-timesteps 20000
```

快速冒烟可直接用：

```bash
ros2 run usv_rl train_ppo_policy \
  --output ~/usv_rl_models/ppo_smoke \
  --total-timesteps 32 \
  --n-steps 16 \
  --batch-size 8
```

如果要做“教师预训练 + PPO 微调”，可以在已有 PPO checkpoint 基础上增加：

```bash
ros2 run usv_rl train_ppo_policy \
  --load-model ~/usv_rl_models/ppo_policy_usv03.zip \
  --pretrain-dataset ~/usv_rl_datasets/scripted_teacher_wide.npz \
  --pretrain-epochs 12 \
  --output ~/usv_rl_models/ppo_teacher_pretrain_ft \
  --total-timesteps 768 \
  --scenario head_on \
  --scenario head_on \
  --scenario head_on \
  --scenario crossing_starboard \
  --scenario overtaking
```

如果只想导出预训练后的 PPO 权重而不做在线更新，可以把 `--total-timesteps` 设为 `0`。

### 5. 统一评估

可以直接评估 `zero`、`bc`、`ppo` 三类策略，输出 JSON 摘要：

```bash
ros2 run usv_rl evaluate_policy \
  --policy bc \
  --model ~/usv_rl_models/bc_linear_policy.npz \
  --episodes 9
```

PPO 评估示例：

```bash
ros2 run usv_rl evaluate_policy \
  --policy ppo \
  --model ~/usv_rl_models/ppo_policy_usv03.zip \
  --episodes 9
```

### 6. 在线推理节点

## 当前推荐默认流程

如果你现在只需要一份冻结后的最小操作手册，优先看 `docs/developer/manuals/rl_recommended_workflow.md`。下面这一节保留完整背景和扩展说明。

如果你的目标是直接把当前通过门槛的 v2 候选接到现有 SITL 叠加链，默认入口现在收敛为：

```bash
ros2 launch usv_rl recommended_sitl.launch.py \
  namespace:=usv_03
```

这条 launch 内部会复用 `usv_sim/sitl_rl_launch.py`，并固定使用当前唯一通过门槛的模型：`./tmp_rl/bc_scripted_teacher_mlp_v2.npz`。它适合作为“当前推荐上线候选”的默认跑法，而不是实验入口。

兼容性说明：旧命令 `recommended_v2_sitl.launch.py` 仍然保留，但后续建议统一使用不带版本号的 `recommended_sitl.launch.py`。

这条默认路径现在已经收敛到代码里的单一来源配置，不需要手工在多个脚本里重复修改推荐模型路径。

如果你确实需要在同一入口下临时覆盖默认候选，也可以直接覆盖这些参数，而不用切回底层 launch：

```bash
ros2 launch usv_rl recommended_sitl.launch.py \
  namespace:=usv_03 \
  rl_policy_model:=./tmp_rl/your_candidate_model.npz \
  rl_policy_kind:=bc
```

默认情况下仍然建议不要覆盖，只有在做受控对比时才这么用。

如果你只想确认默认上线入口本身能否把策略节点拉起，而不想手工读启动日志，可以直接跑：

```bash
ros2 run usv_rl smoke_recommended_sitl \
  --namespace usv_03 \
  --output-json ~/usv_workspace/tmp_rl/recommended_sitl_smoke.json
```

这条命令会短时启动 `recommended_sitl.launch.py`，然后自动汇总：

1. `policy_process_started`
2. `controller_param_enabled`
3. `observation_ready`
4. `stale_data_warning`
5. `error_detected`
6. `passed`

此外，命令现在会默认把这次启动的原始 launch 输出一并保存到 `./tmp_rl/recommended_sitl_smoke.log`，这样后续如果要核对 `namespace`、`mavros_system_id`、`target_system_id` 之类的生效配置，不需要再手工重跑一次 launch 抓日志。

这里的判定标准是“默认推荐入口是否把策略节点和控制接缝成功拉起来”，所以如果外部 FCU/SITL 数据流不存在，`stale_data_warning=true` 和 `observation_ready=false` 仍然可能出现，但这不等价于入口失败。只要 `policy_process_started=true`、`controller_param_enabled=true` 且 `error_detected=false`，就说明默认上线入口本身是通的。

如果你想快速查看“当前推荐候选”的整体状态，而不想分别打开 model、benchmark、gate、smoke 四个文件，可以直接运行：

```bash
ros2 run usv_rl recommended_status
```

它会汇总：

1. 当前推荐模型路径和是否存在
2. 当前推荐 benchmark / gate / smoke JSON 是否存在
3. 当前推荐 benchmark 原始日志目录是否存在，以及三场景日志是否齐全
4. 当前推荐 smoke 原始日志是否存在
5. 当前推荐 benchmark 三场景日志是否都包含 scenario 启动、observation ready、首帧策略动作，且没有异常 traceback
6. 当前推荐 smoke 日志是否包含生效配置、策略进程启动、控制器参数打开，且没有异常 traceback
7. gate 是否通过
8. smoke 是否通过
9. 三个标准场景的当前趋势摘要
10. 当前冻结整体验证样本目录与汇总统计；当前基线固定样本为 `./tmp_rl/post_goal_guard_validate_batch_8_final/`，结果是 `8/8` 全通过、三场景均 `8/8`、启动重试 `0` 次

如果你只是想做仓库内部的闭环功能确认，仍然用：

```bash
ros2 launch usv_rl online_policy_validation.launch.py \
  model:=./tmp_rl/bc_scripted_teacher_mlp_v2.npz \
  namespace:=usv_03 \
  encounter_enabled:=true \
  encounter_scenario:=head_on
```

前者是推荐默认上线跑法，后者是最小在线验证跑法。

如果要把当前最稳的安全基线直接接回控制链，可以运行在线推理节点。它会订阅本船位姿、速度、导航反馈、原始导航命令和 `apf/neighbors`，然后发布策略动作到 `rl_policy/cmd_vel`。

如果你想用一条命令同时拉起现有 SITL 栈和在线策略节点，可以直接使用新的叠加 launch：

```bash
ros2 launch usv_sim sitl_rl_launch.py \
  namespace:=usv_03 \
  rl_policy_model:=~/usv_workspace/tmp_rl/bc_scripted_teacher_mlp_v2.npz
```

这个 launch 会先复用 `usv_sim/sitl_launch.py` 启动原有仿真链，再延迟拉起 `policy_inference_node`。如果只想观测节点而不自动打开控制器参数，可以加：

```bash
disable_controller_param:=true
```

需要注意，这条 `sitl_rl_launch.py` 只负责把 ROS 控制栈和在线策略节点叠加到现有 SITL 连接路径上，它本身不启动外部 FCU/SITL 数据源。如果本机没有已经在对应端口上提供飞控数据流，你会看到 `Pose/State` 过期告警，但这不代表策略节点接入失败。

如果你要在仓库内部直接做一条可闭环的在线功能验证，可以使用新的 simple 验证 launch：

```bash
ros2 launch usv_rl online_policy_validation.launch.py \
  model:=~/usv_workspace/tmp_rl/bc_scripted_teacher_mlp_v2.npz \
  namespace:=usv_03
```

这条 launch 会启动：

1. 轻量本船仿真节点 `simple_sim_node`
2. `navigate_to_point_node`
3. `velocity_controller_node`
4. `policy_inference_node`
5. 一次性测试导航目标发布器

如果要直接在这条 simple 验证链里加入合成会遇场景，可以打开 `encounter_enabled`。例如用 head_on：

```bash
ros2 launch usv_rl online_policy_validation.launch.py \
  model:=~/usv_workspace/tmp_rl/bc_scripted_teacher_mlp_v2.npz \
  namespace:=usv_03 \
  goal_x:=8.0 \
  goal_y:=0.0 \
  encounter_enabled:=true \
  encounter_scenario:=head_on
```

这样会额外启动 `publish_synthetic_neighbors`，在 `apf/neighbors` 上发布与训练场景一致的合成邻船轨迹。它的用途不是替代正式评估，而是快速确认在线策略在有会遇输入时确实会介入。

如果你想把三种在线会遇场景一次性跑完并输出 JSON 摘要，可以直接用：

```bash
ros2 run usv_rl benchmark_online_policy \
  --model ~/usv_workspace/tmp_rl/bc_scripted_teacher_mlp_v2.npz \
  --output-json ~/usv_workspace/tmp_rl/online_bc_v2_benchmark.json
```

这个工具会依次拉起 `online_policy_validation.launch.py` 的 `head_on`、`crossing_starboard`、`overtaking` 三个场景，并汇总：

1. 首帧策略动作
2. 距离起点与终点
3. 距离变化趋势（progress / retreat / stalled）
4. 观测流是否就绪、是否出现运行错误

默认情况下，`benchmark_online_policy` 仍然保持严格语义，不会自动重试启动抖动。如果你是在人工排查环境时想让它也做一次窄重试，可以额外传：

```bash
ros2 run usv_rl benchmark_online_policy \
  --model ~/usv_workspace/tmp_rl/bc_scripted_teacher_mlp_v2.npz \
  --startup-retries 1 \
  --output-json ~/usv_workspace/tmp_rl/online_bc_v2_benchmark.json
```

这和 `validate_online_candidate` 使用同一套判定条件，只会对“场景已启动、但没有 observation ready、没有首帧策略动作、趋势还是 unknown、且没有异常 traceback”的启动类抖动做重试。

当前推荐把这份基准摘要作为后续所有模型的统一入口，再通过对比工具执行验收门槛。当前推荐在线候选仍然是 `bc_scripted_teacher_mlp_v2.npz`，它对应的目标门槛是：

1. `head_on` 必须是 `retreat`
2. `crossing_starboard` 必须是 `progress`
3. `overtaking` 必须是 `progress`
4. 所有场景都必须满足 `observation_ready=true` 且 `error_detected=false`

对任意新模型，先生成它自己的 benchmark JSON，再和当前 v2 基线做对比：

```bash
ros2 run usv_rl compare_online_benchmark \
  --candidate ~/usv_workspace/tmp_rl/online_candidate_benchmark.json \
  --baseline ~/usv_workspace/tmp_rl/online_bc_v2_benchmark.json \
  --output-json ~/usv_workspace/tmp_rl/online_candidate_gate.json
```

如果候选模型通过门槛，这条命令会以退出码 `0` 结束；只要出现会遇趋势退化、观测未就绪或运行错误，就会以非零退出码结束。这样后续实验就不需要再手工盯着 JSON 判断“能不能替换 v2”。

如果 candidate benchmark 里已经包含按场景落盘的原始日志，这条 compare 输出现在也会把每个场景对应的 `raw_log_path` 一起带出来。失败时可以直接跳到具体场景的 launch 原始日志，不用再手工回查 benchmark JSON。

现在 compare / validate 的报告里还会直接附带每个场景的 `raw_log_health` 和 `raw_log_diagnostics`。也就是说，如果失败不是单纯的趋势退化，而是日志里缺了 `scenario_started`、`Observation stream ready`、`Publishing first pure RL action`，或者出现了异常 traceback，报告本身就会把这些缺失信号列出来。

如果你不想分两步手工执行 `benchmark_online_policy` 和 `compare_online_benchmark`，现在可以直接用一条命令完成候选回归验收：

```bash
ros2 run usv_rl validate_online_candidate \
  --model ~/usv_workspace/tmp_rl/<candidate_model>.npz
```

这条命令会自动：

1. 跑标准三场景在线 benchmark
2. 保存候选 benchmark JSON
3. 对照 `./tmp_rl/online_bc_v2_benchmark.json` 执行替换门槛
4. 输出 gate JSON，并用退出码表达通过/失败
5. 为每个标准场景保存一份原始 launch 日志

默认情况下，`validate_online_candidate` 还会对“明显像启动抖动”的场景自动重试 1 次。判定条件很窄：没有 `observation_ready`、没有首帧策略动作、趋势仍是 `unknown`、且日志里没有异常 traceback。这是为了避免把单次 ROS 启动时序抖动误判成模型退化；如果你想关闭它，可以显式传：

```bash
ros2 run usv_rl validate_online_candidate \
  --model ~/usv_workspace/tmp_rl/<candidate_model>.npz \
  --startup-retries 0
```

默认情况下，这条命令会自动对照当前推荐基线 `./tmp_rl/online_bc_v2_benchmark.json`，因此只有在你想替换基线时才需要显式传 `--baseline-benchmark`。

这些原始日志默认会保存到 `./tmp_rl/<model_stem>_validation_logs/`，例如 `head_on.log`、`crossing_starboard.log`、`overtaking.log`。如果你想自己指定目录，可以直接加：

```bash
ros2 run usv_rl validate_online_candidate \
  --model ~/usv_workspace/tmp_rl/<candidate_model>.npz \
  --log-dir ~/usv_workspace/tmp_rl/my_candidate_logs
```

如果你单独使用 `benchmark_online_policy`，也可以显式要求它按场景保存原始日志：

```bash
ros2 run usv_rl benchmark_online_policy \
  --model ~/usv_workspace/tmp_rl/bc_scripted_teacher_mlp_v2.npz \
  --output-json ~/usv_workspace/tmp_rl/online_bc_v2_benchmark.json \
  --output-log-dir ~/usv_workspace/tmp_rl/online_bc_v2_logs
```

当前推荐基线也建议固定维护这套 `online_bc_v2_logs/` 目录。这样后续 compare 输出里的 `baseline_raw_log_path` 就不会是空值，候选和基线都能直接落到对应场景的 launch 原始日志。

当前推荐的回归验收顺序就是：

1. 新模型训练完成
2. 运行 `validate_online_candidate`
3. 只有退出码为 `0` 的模型才允许进入替换讨论
4. 只要 `head_on` 不再是 `retreat`，即使其它场景更快，也视为失败

当前已验证这条链能进入 `Observation stream ready`，并打印首帧策略动作。对于 BC 模型，在线节点现在会自动根据模型里的观测维度匹配 `max_neighbors`，不再要求手工保持一致。

当前推荐直接先跑 MLP BC v2：

```bash
ros2 run usv_rl policy_inference_node \
  --namespace usv_03 \
  --model ~/usv_workspace/tmp_rl/bc_scripted_teacher_mlp_v2.npz \
  --policy bc
```

默认会自动尝试把 `velocity_controller_node` 的 `rl_policy_enabled` 打开。如果只想先单独起节点不改控制器参数，可以加：

```bash
--disable-controller-param
```

这个节点也支持 `.zip` PPO 模型：

```bash
ros2 run usv_rl policy_inference_node \
  --namespace usv_03 \
  --model ~/usv_rl_models/ppo_policy_usv03.zip \
  --policy ppo
```

## 当前限制

1. 当前不是 ArduPilot/Gazebo 级 SITL，而是训练专用的轻量动力学闭环。
2. 第一版只适合训练局部避让策略，不适合端到端直接接管导航。
3. PPO 入口已接好，但依赖未自动安装到仓库内，需要自行准备 `.venv`。
4. 在线推理已经可单独运行，也可通过 `usv_sim/sitl_rl_launch.py` 叠加到现有 SITL 流程里；但还没有做正式的 launch 参数体系收敛和实船启动集成。

## 下一步建议

1. 先采 10 到 30 个 episode，确认数据分布和专家策略动作量级。
2. 先训行为克隆基线，看 `val_mse` 是否稳定下降。
3. 再决定是继续线性/MLP 模仿学习，还是直接上 PPO 微调。