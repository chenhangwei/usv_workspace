# PAI-DSW MAPPO Runbook

这份手册用于当前 usv_rl 分支在 PAI-DSW 上执行多艇 MAPPO 训练与评估。

适用前提：

1. 代码位于 /mnt/workspace/usv_workspace
2. 训练产物统一写入 /mnt/data
3. 当前阶段以离线训练和离线评估为主

当前限制：

1. `train_mappo_residual` 已可训练多艇 MAPPO checkpoint
2. `evaluate_mappo_residual` 已可离线评估 MAPPO checkpoint
3. 在线 `policy_inference_node` 现已支持直接加载 MAPPO `.pt` checkpoint
4. 但当前更推荐先完成离线训练和离线评估，再进入 SITL 在线接入阶段

## 1. 存储布局

建议在 NAS 挂载点下固定以下目录：

```bash
mkdir -p /mnt/data/datasets/usv_rl
mkdir -p /mnt/data/checkpoints/usv_rl
mkdir -p /mnt/data/evals/usv_rl
mkdir -p /mnt/data/logs/usv_rl
```

目录含义：

1. `/mnt/data/datasets/usv_rl`: teacher 数据集、行为克隆输入
2. `/mnt/data/checkpoints/usv_rl`: MAPPO / PPO / BC 模型与周期 checkpoint
3. `/mnt/data/evals/usv_rl`: 评估 JSON、checkpoint 排名、实验摘要
4. `/mnt/data/logs/usv_rl`: 长训练日志、人工记录

## 2. 构建环境

首次进入 DSW 会话后，推荐先构建依赖到训练链的包：

```bash
cd /mnt/workspace/usv_workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to usv_rl usv_sim --symlink-install
source install/setup.bash
```

如果你本轮只修改了 Python 训练脚本，也可以用同一条命令快速增量构建。

## 3. 一键预检

在开始训练前，建议先运行预检脚本：

```bash
cd /mnt/workspace/usv_workspace
python3 src/usv_rl/usv_rl/preflight_check.py \
  --output-json /mnt/data/evals/usv_rl/preflight.json
```

如果后续已经重新构建并 source 了工作空间，也可以使用：

```bash
cd /mnt/workspace/usv_workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run usv_rl preflight_check \
  --output-json /mnt/data/evals/usv_rl/preflight.json
```

预检会检查：

1. `ros2`、`colcon`、`python3`、`pip3` 是否可用
2. `torch`、`numpy`、`rclpy`、`gymnasium`、`stable_baselines3` 是否能被当前解释器导入
3. `/mnt/data` 下训练产物目录是否存在且可写
4. 当前环境是否满足最小 `mappo_train_ready`

## 4. 训练前最小冒烟

先确认多艇环境能在当前 DSW 环境里创建成功，再开始长训练：

```bash
cd /mnt/workspace/usv_workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run usv_rl evaluate_mappo_residual \
  --policy zero \
  --num-agents 5 \
  --episodes 1 \
  --steps-per-episode 20 \
  --scenario five_usv_dense_head_on \
  --output-json /mnt/data/evals/usv_rl/mappo_zero_smoke.json
```

这一步的目标不是性能，而是确认：

1. ROS 2 环境已加载
2. `usv_rl` 和 `usv_sim` 可执行入口可用
3. 多艇轻量仿真环境可以正常 reset / step

## 5. 推荐的首轮多艇 MAPPO 训练命令

下面这组命令适合作为 PAI-DSW 上的首轮 dense 场景训练模板。

```bash
cd /mnt/workspace/usv_workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export EXP_NAME=mappo_dense_v1

ros2 run usv_rl train_mappo_residual \
  --output /mnt/data/checkpoints/usv_rl/${EXP_NAME}.pt \
  --checkpoint-dir /mnt/data/checkpoints/usv_rl/${EXP_NAME}_checkpoints \
  --checkpoint-interval 2560 \
  --auto-evaluate-checkpoints \
  --checkpoint-ranking-json /mnt/data/evals/usv_rl/${EXP_NAME}_ranking.json \
  --checkpoint-eval-json-dir /mnt/data/evals/usv_rl/${EXP_NAME}_checkpoint_eval \
  --checkpoint-eval-episodes 15 \
  --checkpoint-eval-steps 90 \
  --num-agents 5 \
  --max-agents 5 \
  --max-neighbors 4 \
  --total-timesteps 51200 \
  --rollout-steps 128 \
  --update-epochs 4 \
  --minibatch-size 128 \
  --learning-rate 3e-4 \
  --scenario five_usv_dense_head_on \
  --scenario five_usv_dense_crossing \
  --scenario five_usv_dense_overtaking \
  --device cpu
```

说明：

1. `--output` 是最终导出的主 checkpoint
2. `--checkpoint-dir` 是训练过程中定期保存的 checkpoint 目录
3. `--auto-evaluate-checkpoints` 会在训练结束后自动批量评估周期 checkpoint
4. `--num-agents 5`、`--max-agents 5` 对齐当前 dense 五艇场景
5. 如果 DSW 镜像有可用 GPU 且 `torch.cuda.is_available()` 为真，可把 `--device cpu` 改成 `--device cuda`

## 6. 单次离线评估最终模型

训练完成后，优先先评估最终导出的主 checkpoint：

```bash
cd /mnt/workspace/usv_workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export EXP_NAME=mappo_dense_v1

ros2 run usv_rl evaluate_mappo_residual \
  --policy mappo \
  --model /mnt/data/checkpoints/usv_rl/${EXP_NAME}.pt \
  --episodes 15 \
  --steps-per-episode 120 \
  --scenario five_usv_dense_head_on \
  --scenario five_usv_dense_crossing \
  --scenario five_usv_dense_overtaking \
  --output-json /mnt/data/evals/usv_rl/${EXP_NAME}_final_eval.json \
  --device cpu
```

输出文件会落到：

1. `/mnt/data/evals/usv_rl/${EXP_NAME}_final_eval.json`

## 7. 重新评估整组周期 Checkpoint

如果你想在训练后单独重新做一次 checkpoint 排名，可以用下面这条命令。

注意：这个工具当前没有 `ros2 run` 入口，应直接用 Python 模块方式启动。

```bash
cd /mnt/workspace/usv_workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export EXP_NAME=mappo_dense_v1

/bin/python3 -m usv_rl.evaluate_mappo_checkpoints \
  --checkpoint-dir /mnt/data/checkpoints/usv_rl/${EXP_NAME}_checkpoints \
  --episodes 15 \
  --steps-per-episode 90 \
  --scenario five_usv_dense_head_on \
  --scenario five_usv_dense_crossing \
  --scenario five_usv_dense_overtaking \
  --summary-json /mnt/data/evals/usv_rl/${EXP_NAME}_checkpoint_ranking.json \
  --per-checkpoint-json-dir /mnt/data/evals/usv_rl/${EXP_NAME}_checkpoint_eval_manual \
  --device cpu
```

## 8. 推荐实验命名方式

为了方便后续做多轮 DSW 重训，推荐把一次实验的所有产物统一用同一个实验名前缀。

示例：

1. 模型：`/mnt/data/checkpoints/usv_rl/mappo_dense_v1.pt`
2. 周期 checkpoint：`/mnt/data/checkpoints/usv_rl/mappo_dense_v1_checkpoints/`
3. 最终评估：`/mnt/data/evals/usv_rl/mappo_dense_v1_final_eval.json`
4. checkpoint 排名：`/mnt/data/evals/usv_rl/mappo_dense_v1_checkpoint_ranking.json`

这样在做 v2、v3、reward ablation、scenario curriculum 实验时会非常清楚。

## 9. 当前阶段不建议做的事

1. 不要把 MAPPO checkpoint 写回仓库目录，例如 `./tmp_rl` 或源码树下的任意子目录
2. 不要一开始就把 MAPPO 直接用于替换当前推荐基线；应先完成离线训练和离线评估闭环
3. 不要同时改奖励、场景、控制器接缝和在线部署，建议单轮实验只改一到两类变量

## 10. 当前最推荐的 DSW 工作流

每轮实验按下面顺序执行：

1. 构建：`colcon build --packages-up-to usv_rl usv_sim --symlink-install`
2. 冒烟：`evaluate_mappo_residual --policy zero`
3. 训练：`train_mappo_residual`
4. 最终评估：`evaluate_mappo_residual --policy mappo`
5. 周期 checkpoint 排名：`python -m usv_rl.evaluate_mappo_checkpoints`
6. 比较不同实验前缀下的评估 JSON，再决定是否进入在线接入阶段

## 11. 现阶段的工程结论

对当前分支，最合理的推进顺序是：

1. 先在 PAI-DSW 上把多艇 MAPPO 训练稳定跑起来
2. 再用 `/mnt/data/evals` 下的 JSON 结果筛选候选模型
3. 最后再把候选 MAPPO actor 接进在线 `policy_inference_node`

这比直接边训练边改在线部署更稳。

## 12. MAPPO 在线验证命令

如果离线评估结果已经满意，可以把 `.pt` 模型直接接到现有在线验证链：

```bash
cd /mnt/workspace/usv_workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch usv_rl online_policy_validation.launch.py \
  model:=/mnt/data/checkpoints/usv_rl/mappo_dense_v1.pt \
  policy:=mappo \
  namespace:=usv_03 \
  encounter_enabled:=true \
  encounter_scenario:=head_on
```

如果你要把它叠加到现有 SITL 栈，则可以使用：

```bash
cd /mnt/workspace/usv_workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch usv_sim sitl_rl_launch.py \
  namespace:=usv_03 \
  rl_policy_model:=/mnt/data/checkpoints/usv_rl/mappo_dense_v1.pt \
  rl_policy_kind:=mappo
```