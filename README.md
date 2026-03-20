# USV Swarm Control System (USV Workspace)

这是一个基于 ROS2 的无人水面艇 (USV) 集群控制系统。该项目旨在实现多艇协同作业，架构上分为地面站 (Ground Station) 和无人艇 (USV) 两大核心部分，支持通过 MAVROS 与 Pixhawk 飞控进行通信。

## 📚 目录

- [项目概述](#项目概述)
- [核心架构](#核心架构)
- [安装与构建](#安装与构建)
- [快速开始](#快速开始)
- [系统模块](#系统模块)
- [开发指南](#开发指南)

## 🚀 项目概述

本系统设计用于管理和控制 USV 集群，具备以下关键特性：
- **ROS2 架构**: 利用 ROS2 的通信机制实现分布式的控制与监控。
- **集群支持**: 灵活的命名空间设计，支持多艘 USV 同时在线。
- **MAVROS 集成**: 无缝对接 Pixhawk 飞控，实现底层运动控制。
- **图形化界面**: 基于 PyQt5 的地面站 GUI，实时显示 USV 状态并发送导航指令。

## 🏗 核心架构

### 1. 命名空间体系
系统通过 ROS2 命名空间隔离不同实体：
- **地面站 (Ground Station)**: 运行在无命名空间下（全局），负责 GUI 展示和集群任务调度。
- **无人艇 (USV)**: 每艘艇运行在独立的命名空间下（如 `usv_01`, `usv_02`）。
    - *MAVROS ID 映射*: 系统自动根据命名空间推导 MAVLink target ID（例如 `usv_02` -> `tgt_system=2`）。

### 2. 关键数据流
- **状态上报**: USV -> 发布 `UsvStatus` 消息 -> 地面站订阅显示。
- **导航控制**: 地面站 -> 发送 `NavigateToPoint` Action 目标 -> USV 执行动作。
- **飞控桥接**: USV 节点 <-> MAVROS <-> 串口/MAVLink <-> 飞控硬件。

## 🛠 安装与构建

### 环境要求
- Ubuntu 20.04 / 22.04
- ROS2 (Foxy / Humble)
- MAVROS
- Python 3 + PyQt5

### 构建工作空间

```bash
# 克隆仓库到你的工作空间 src 目录
# git clone ...

# 安装依赖 (根据实际情况)
rosdep install --from-paths src --ignore-src -r -y

# 构建
colcon build

# 刷新环境
source install/setup.bash
```

## ⚡ PAI-DSW A10 训练

如果你在阿里云 PAI-DSW 上使用 NVIDIA A10 进行 MAPPO 训练，建议优先使用仓库内的脚本 [scripts/train_mappo_a10_pai.sh](scripts/train_mappo_a10_pai.sh)。

该脚本默认启用以下更适合 A10 的训练设置：
- 自动优先使用 CUDA
- CUDA 下自动启用 AMP 混合精度
- 开启高精度 matmul 模式与 TF32
- 将 PyTorch CPU 线程数限制为 1，减少与 ROS 采样线程争抢 8 核 CPU
- 默认使用 5 艇、`dense` 场景集、`256x256` MLP、`rollout_steps=256`、`minibatch_size=512`

训练过程中会输出每次策略更新的性能画像，例如：
- `rollout_time` / `rollout_sps`: 采样耗时与采样速度
- `update_time` / `update_sps`: 策略更新耗时与更新吞吐
- `update_share`: 单轮总时间里用于神经网络更新的占比
- `gpu_mem_alloc` / `gpu_mem_peak`: 当前和峰值显存占用

如果 `rollout_time` 明显大于 `update_time`，说明当前瓶颈主要在 ROS 仿真采样而不是 A10 计算，本阶段优先考虑并行采样而不是继续放大模型。

当前仓库已经在 A10 上做过短探针实测：5 艇 dense、`rollout_steps=128` 时，单轮采样约 `62s`，单轮 PPO 更新约 `6s`，更新阶段只占总时间约 `9%`，显存峰值约 `23MiB`。这说明当前训练绝大部分时间耗在 ROS 同步采样，A10 大部分时间处于空闲状态，因此正式训练默认值已从 `512` 下调到 `256`，优先缩短首个 update 和训练反馈回路。

示例：

```bash
cd /mnt/workspace/usv_workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash

./scripts/train_mappo_a10_pai.sh /mnt/data/checkpoints/usv_rl/mappo_dense_a10_run.pt
```

如果你需要覆盖默认参数，可以把额外参数追加在命令后面，例如：

```bash
./scripts/train_mappo_a10_pai.sh /mnt/data/checkpoints/usv_rl/mappo_dense_a10_run.pt \
    --total-timesteps 1000000 \
    --checkpoint-interval 51200 \
    --learning-rate 2e-4
```

如果你只想快速看当前机器的训练画像，而不是直接开长训，优先使用 [scripts/start_mappo_a10_profiled.sh](scripts/start_mappo_a10_profiled.sh)。该脚本默认执行一个短探针：`rollout_steps=128`、`total_timesteps=1280`、关闭 checkpoint、每次 update 打印 telemetry，目的是在几分钟内判断当前瓶颈到底在采样还是更新。

示例：

```bash
./scripts/start_mappo_a10_profiled.sh mappo_dense_a10_probe_v1
```

如果要把 profiling 脚本改成更长的观测窗口，可以直接追加参数，或者通过环境变量覆盖：

```bash
PROFILE_ROLLOUT_STEPS=256 PROFILE_TOTAL_TIMESTEPS=5120 \
./scripts/start_mappo_a10_profiled.sh mappo_dense_a10_probe_v2
```

如果你要尝试降低采样壁钟时间，训练器现在还提供了一个实验性的多进程并行采样开关：`--num-sampler-workers`。其实现方式是为每个 sampler worker 分配独立的 `ROS_DOMAIN_ID`，从而避免 5 艇环境在同一 DDS 域里串话。

示例：

```bash
./scripts/train_mappo_a10_pai.sh /mnt/data/checkpoints/usv_rl/mappo_dense_a10_parallel.pt \
    --num-sampler-workers 2 \
    --base-ros-domain-id 190 \
    --rollout-steps 256
```

如果你准备直接开并行长训，优先使用 [scripts/start_mappo_a10_parallel_train.sh](scripts/start_mappo_a10_parallel_train.sh)。它会统一生成输出路径和日志路径，并默认使用 `2` 个 sampler worker。

示例：

```bash
./scripts/start_mappo_a10_parallel_train.sh mappo_dense_a10_parallel_v1
```

如需覆盖并行配置，可以用环境变量指定：

```bash
NUM_SAMPLER_WORKERS=2 BASE_ROS_DOMAIN_ID=240 \
./scripts/start_mappo_a10_parallel_train.sh mappo_dense_a10_parallel_v2
```

如果你下一轮训练的目标已经从“继续采样更多 checkpoint”切到“主动提高安全偏置”，优先使用 [scripts/start_mappo_a10_parallel_safe_train.sh](scripts/start_mappo_a10_parallel_safe_train.sh)。这个脚本基于并行长训入口，默认会显式抬高碰撞/近失/冲突前视惩罚，并下调团队推进奖励，避免再次落到“离线 progress 看起来不错，但 collision 持续偏高”的区间。

示例：

```bash
NUM_SAMPLER_WORKERS=2 BASE_ROS_DOMAIN_ID=180 \
./scripts/start_mappo_a10_parallel_safe_train.sh mappo_dense_a10_parallel_safe_v1
```

如果问题已经进一步收敛到“间距够了，但 `head_on` 该退不退，继续训练还会拖坏 `crossing_starboard`”，优先使用 [scripts/start_mappo_a10_parallel_headon_fix_train.sh](scripts/start_mappo_a10_parallel_headon_fix_train.sh)。这条入口会明显降低 `head_on` 场景里“保持前进”的奖励，同时抬高右侧走廊、右转承诺和中心线惩罚，并单独给 `crossing_starboard` 更强的前进保护。

示例：

```bash
NUM_SAMPLER_WORKERS=2 BASE_ROS_DOMAIN_ID=180 \
./scripts/start_mappo_a10_parallel_headon_fix_train.sh mappo_dense_a10_parallel_headon_fix_v1
```

如果后续结果已经进一步收敛到“`overtaking` 基本稳定、`head_on` 偶尔还会波动，但 `crossing_starboard` 在线经常变成 `stalled/unknown`”，优先改用 [scripts/start_mappo_a10_parallel_crossing_fix_train.sh](scripts/start_mappo_a10_parallel_crossing_fix_train.sh)。这条入口会在保留 head_on 右转偏置的前提下，适度降低 head_on 过强的中心线/走廊压制，抬高 `crossing_starboard` 的右转承诺、前进奖励和停滞惩罚，目标是把 crossing 从“安全但不走”拉回“安全且稳定 progress”。

示例：

```bash
NUM_SAMPLER_WORKERS=2 BASE_ROS_DOMAIN_ID=180 \
./scripts/start_mappo_a10_parallel_crossing_fix_train.sh mappo_dense_a10_parallel_crossing_fix_v1
```

如果结果已经进一步收敛到“`overtaking` 已经稳定、`crossing_starboard` 大多数时候也能保持 `progress`，但 `head_on` 仍然只偶发满足 `retreat`”，优先改用 [scripts/start_mappo_a10_parallel_headon_stabilize_train.sh](scripts/start_mappo_a10_parallel_headon_stabilize_train.sh)。这条入口保持 crossing_fix 的 `crossing_starboard` / `overtaking` 推进参数不变，只把 `head_on` 相关引导切回更强的右侧走廊、中心线惩罚和右转承诺，用来单独补强 head_on retreat，而不再重新压低 crossing/overtaking 的推进性。

示例：

```bash
NUM_SAMPLER_WORKERS=2 BASE_ROS_DOMAIN_ID=180 \
./scripts/start_mappo_a10_parallel_headon_stabilize_train.sh mappo_dense_a10_parallel_headon_stabilize_v1
```

训练器现在还额外支持直接通过 CLI 覆盖这几个原先硬编码的安全核心项：`--collision-penalty`、`--conflict-distance`、`--anticipation-distance`。这意味着后续做 safety ablation 时，不需要再改 Python 源码，只要覆盖脚本参数即可。

如果你想把云端长训迁到本地继续，可以直接使用新的恢复入口 [scripts/resume_mappo_training.sh](scripts/resume_mappo_training.sh)。训练器现已支持 `--resume-from` 从已有 MAPPO checkpoint scaffold 恢复 actor / critic / log_std，并在 checkpoint 内存在优化器状态时一并恢复优化器和 GradScaler。对于旧 checkpoint（只保存了模型权重），也可以做“权重续训”，只是优化器状态会从头开始。

示例：

```bash
./scripts/resume_mappo_training.sh \
    /mnt/data/checkpoints/usv_rl/mappo_dense_a10_parallel_headon_stabilize_20260320_003331.pt \
    750000
```

如果你想在当前服务器训练跑完后，自动准备一个“迁到本地”的打包结果，可以使用：
- [scripts/watch_training_and_prepare_local_migration.sh](scripts/watch_training_and_prepare_local_migration.sh): 持续轮询某个训练 run，等最终模型保存后自动调用打包脚本。
- [scripts/prepare_local_migration_bundle.sh](scripts/prepare_local_migration_bundle.sh): 直接把当前 workspace 源码、该 run 的最终模型、日志和 checkpoint 目录打成迁移包，并附带本地恢复步骤说明。

示例：

```bash
./scripts/watch_training_and_prepare_local_migration.sh \
    mappo_dense_a10_parallel_headon_stabilize_20260320_003331
```

如果你希望长训产出的 checkpoint 自动离线评估，可以使用这两个脚本：
- [scripts/evaluate_mappo_checkpoint.sh](scripts/evaluate_mappo_checkpoint.sh): 对单个 checkpoint 执行 9x90 离线评估，并把 `.eval.json` / `.eval.log` 写回同目录。
- [scripts/watch_mappo_checkpoint_eval.sh](scripts/watch_mappo_checkpoint_eval.sh): 持续轮询 checkpoint 目录，发现新的 `*_step_*.pt` 就自动评估。

如果某个 checkpoint 的离线结果已经达到你愿意继续追踪的门槛，还可以直接把在线 benchmark 和 compare gate 结果也写回 checkpoint 目录：
- [scripts/validate_mappo_checkpoint_online.sh](scripts/validate_mappo_checkpoint_online.sh): 对单个 checkpoint 调用 `validate_online_candidate`，并把 `.online.benchmark.json`、`.online.gate.json`、`.online.validate.log` 以及场景原始日志目录 `.online_logs/` 固定保存到同目录。
- [scripts/repeat_validate_mappo_checkpoint_online.sh](scripts/repeat_validate_mappo_checkpoint_online.sh): 对同一个 checkpoint 连续执行多次在线 gate 复跑，把每次 run 的 benchmark / gate / 日志写入 `.online_repeats/`，并额外生成一份 `.online.repeat.summary.json` 聚合稳定性统计。
- [scripts/watch_mappo_checkpoint_online_repeat.sh](scripts/watch_mappo_checkpoint_online_repeat.sh): 持续轮询 checkpoint 目录，对已经完成离线评估且满足阈值的 checkpoint 自动触发重复在线复跑；不达标的 checkpoint 会写 `.online.repeat.skip.json`，避免重复尝试。

示例：

```bash
./scripts/watch_mappo_checkpoint_eval.sh \
    /mnt/data/checkpoints/usv_rl/mappo_dense_a10_parallel_v2_20260319_033621_checkpoints
```

```bash
./scripts/validate_mappo_checkpoint_online.sh \
    /mnt/data/checkpoints/usv_rl/mappo_dense_a10_parallel_v2_20260319_033621_checkpoints/mappo_dense_a10_parallel_v2_20260319_033621_step_0051200.pt \
    /tmp/recover_7680_online_benchmark.json
```

```bash
./scripts/repeat_validate_mappo_checkpoint_online.sh \
    /mnt/data/checkpoints/usv_rl/mappo_dense_a10_parallel_v2_20260319_033621_checkpoints/mappo_dense_a10_parallel_v2_20260319_033621_step_0051200.pt \
    /tmp/recover_7680_online_benchmark.json \
    --runs 3
```

```bash
MIN_PROGRESS=0.16 MAX_COLLISION_RATE=0.0 \
./scripts/watch_mappo_checkpoint_online_repeat.sh \
    /mnt/data/checkpoints/usv_rl/mappo_dense_a10_parallel_v2_20260319_033621_checkpoints \
    /tmp/recover_7680_online_benchmark.json \
    --runs 3
```

说明：
- 该能力当前属于实验骨架，默认仍然使用单 worker。
- `--base-ros-domain-id` 需要为每次并行训练选择不与其他 ROS2 任务冲突的起始域 ID，并保证最高使用到的域号不要超过 `232`，否则 Fast DDS 会报端口计算过高错误。
- 当前 2-worker smoke 已验证可以进入 MAPPO update 并保存 checkpoint，但退出阶段仍可能出现 ROS teardown 噪声；这不影响训练结果落盘。
- 在 A10 上做过对照探针后，`2-worker + rollout_steps=128` 的采样时间约为 `28s~33s`，对比单 worker 的约 `62s` 已接近翻倍提速；因此如果你当前更在意 wall-clock 训练效率，而不是最稳的运行形态，优先尝试 2-worker 是合理的。
- 在正式长训口径下，`2-worker + rollout_steps=256` 已实测将单轮 rollout 从单 worker 的约 `124s~131s` 降到约 `54s~59s`，采样吞吐从约 `10` 提升到约 `22~24` agent-steps/s，当前收益已经足够支撑把 2-worker 作为 A10 默认推荐方案。
- 当前 workspace 默认推荐基线产物缺失，因此 `validate_online_candidate` 不能直接依赖默认 baseline；在这条并行训练线上，建议显式传入当前安全基线的 benchmark JSON，例如 `/tmp/recover_7680_online_benchmark.json` 或你持久化保存过的等价文件。
- 如果某个 checkpoint 的单次在线 gate 和历史结果冲突，优先用 `repeat_validate_mappo_checkpoint_online.sh` 看多次复跑的 `pass_rate` 与各场景 `required_trend_match_rate`，不要只依赖单次通过或单次失败做替换决定。
- `watch_mappo_checkpoint_online_repeat.sh` 的默认离线触发门槛是 `collision_rate<=0`、`mean_team_goal_progress_ratio>=0.16`、`worst_pairwise_min_separation>=0`；这组阈值是为了优先把“离线安全且推进不太差”的 checkpoint 送去做在线复跑，后续可以通过环境变量覆盖。

## GitHub-first 协作与训练产物管理

当前更推荐把 GitHub 作为源码事实源，而不是继续把 bundle 或 `scp` 当成主工作流。

推荐分层如下：

- GitHub：只管理源码、脚本、参数、launch、文档、以及“关键模型登记”这类轻量元数据。
- `/mnt/data`：保存训练过程中的原始 checkpoint、日志、评估 JSON、在线复跑目录。
- GitHub Release 或对象存储：只发布少量经过筛选的里程碑产物，例如当前主候选 checkpoint、final 备份模型、关键 benchmark 摘要。

不建议提交到 Git 历史的内容：

- `build/`、`install/`、`log/`
- `tmp_rl/` 以及临时实验目录
- `.pt`、`.pth`、`.npz`、`.onnx` 等大模型文件
- 自动生成的 `.eval.json`、`.online.*`、重复在线复跑目录与大体积日志
- 迁移打包产物，例如 `.tgz`、`SHA256SUMS`

推荐日常闭环：

1. 本地修改源码，走分支、提交、push、PR。
2. 云端或 PAI-DSW 只做 `git pull` / `git switch` 后构建和训练，所有训练输出写到 `/mnt/data`。
3. 某个 checkpoint 被确认值得保留后，不把模型本体提交进 Git，而是更新仓库里的“关键模型登记”文档，再把模型发布到 GitHub Release 或外部存储。
4. 本地机器继续开发时，先从 GitHub 拉源码，再按登记文档去下载对应模型，而不是反复打整仓 bundle。

当前建议优先登记而不是直接当主模型迁移的产物：

- 当前主候选：`mappo_dense_a10_parallel_headon_stabilize_20260320_003331_step_0307200.pt`
- final 备份：`mappo_dense_a10_parallel_headon_stabilize_20260320_003331.pt`

如果你准备把某个 checkpoint 对外发布到 GitHub Release，而不是继续手工拷文件，可以先用仓库脚本整理一个小型 staging 目录：

```bash
./scripts/prepare_mappo_release_bundle.sh \
    /mnt/data/checkpoints/usv_rl/mappo_dense_a10_parallel_headon_stabilize_20260320_003331_checkpoints/mappo_dense_a10_parallel_headon_stabilize_20260320_003331_step_0307200.pt \
    --summary-json /mnt/data/checkpoints/usv_rl/mappo_dense_a10_parallel_headon_stabilize_20260320_003331_checkpoints/mappo_dense_a10_parallel_headon_stabilize_20260320_003331_step_0307200.online.repeat.summary.json \
    --role primary_candidate
```

这个脚本会生成：

- 待上传的模型文件
- `manifest.json`
- `RELEASE_NOTES.md`
- `SHA256SUMS`

对应登记文件见：

- [src/docs/developer/manuals/pai_dsw_git_remote_workflow.md](src/docs/developer/manuals/pai_dsw_git_remote_workflow.md)
- [src/docs/developer/manuals/mappo_model_registry.md](src/docs/developer/manuals/mappo_model_registry.md)

如果本地显卡和当前 PyTorch CUDA 轮子不兼容，不要让恢复脚本自动走 GPU。优先强制 CPU 续训，例如：

```bash
./scripts/resume_mappo_training_cpu_local.sh \
    /mnt/data/checkpoints/usv_rl/mappo_dense_a10_parallel_headon_stabilize_20260320_003331_checkpoints/mappo_dense_a10_parallel_headon_stabilize_20260320_003331_step_0307200.pt \
    600000
```
