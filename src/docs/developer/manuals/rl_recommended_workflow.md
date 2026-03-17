# RL Recommended Workflow

## 1. 当前冻结基线

- 推荐模型: `./tmp_rl/bc_scripted_teacher_mlp_v2.npz`
- 推荐 benchmark: `./tmp_rl/online_bc_v2_benchmark.json`
- 推荐 benchmark logs: `./tmp_rl/online_bc_v2_logs/`
- 推荐 gate: `./tmp_rl/online_bc_v2_gate.json`
- 推荐 smoke: `./tmp_rl/recommended_sitl_smoke.json`
- 推荐 smoke log: `./tmp_rl/recommended_sitl_smoke.log`

当前唯一接受的替换门槛:

1. `head_on = retreat`
2. `crossing_starboard = progress`
3. `overtaking = progress`
4. 所有场景都必须满足 `observation_ready = true`
5. 所有场景都必须满足 `error_detected = false`

当前稳定性样本（2026-03-12）:

1. 压力验证产物: `./tmp_rl/post_goal_guard_validate_batch_8_final/`
2. 样本规模: 对当前推荐模型连续执行 8 轮 `validate_online_candidate`
3. 结果: `8/8` 全部通过，`0` 次 gate 失败
4. 分场景统计: `head_on=8/8`，`crossing_starboard=8/8`，`overtaking=8/8`
5. 启动重试: `0` 次
6. 当前解读: 目标激活门控修正后，三场景整体验证已稳定通过；默认一次窄重试仍保留为启动抖动保护，但这轮数据下没有触发依赖

## 2. 默认上线入口

直接启动当前推荐候选:

```bash
ros2 launch usv_rl recommended_sitl.launch.py \
  namespace:=usv_03
```

只在受控对比时覆盖候选模型:

```bash
ros2 launch usv_rl recommended_sitl.launch.py \
  namespace:=usv_03 \
  rl_policy_model:=./tmp_rl/your_candidate_model.npz \
  rl_policy_kind:=bc
```

## 3. 默认健康检查

检查默认上线入口是否拉起成功:

```bash
ros2 run usv_rl smoke_recommended_sitl \
  --namespace usv_03 \
  --output-json ./tmp_rl/recommended_sitl_smoke.json
```

这会同时刷新:

1. `./tmp_rl/recommended_sitl_smoke.json`
2. `./tmp_rl/recommended_sitl_smoke.log`

快速查看当前冻结基线整体状态:

```bash
ros2 run usv_rl recommended_status
```

重点看这些字段:

1. `gate_passed`
2. `smoke_passed`
3. `recommended_benchmark_scenario_log_health_complete`
4. `recommended_smoke_log_health.content_valid`

## 4. 基线维护

如果需要重刷当前推荐 baseline benchmark 和三场景原始日志，使用:

```bash
ros2 run usv_rl benchmark_online_policy \
  --model ./tmp_rl/bc_scripted_teacher_mlp_v2.npz \
  --output-json ./tmp_rl/online_bc_v2_benchmark.json \
  --output-log-dir ./tmp_rl/online_bc_v2_logs
```

默认情况下，`benchmark_online_policy` 保持严格语义，不自动重试启动抖动。

只在人工排查环境抖动时，才临时开启一次窄重试:

```bash
ros2 run usv_rl benchmark_online_policy \
  --model ./tmp_rl/bc_scripted_teacher_mlp_v2.npz \
  --startup-retries 1 \
  --output-json ./tmp_rl/online_bc_v2_benchmark.json \
  --output-log-dir ./tmp_rl/online_bc_v2_logs
```

## 5. 新候选回归验收

推荐的一条命令入口:

```bash
ros2 run usv_rl validate_online_candidate \
  --model ./tmp_rl/<candidate_model>.npz
```

这会自动生成:

1. `./tmp_rl/<model_stem>_benchmark.json`
2. `./tmp_rl/<model_stem>_gate.json`
3. `./tmp_rl/<model_stem>_validation_logs/`

如果某个场景触发启动类重试，validation logs 下还会额外保留:

1. `./tmp_rl/<model_stem>_validation_logs/<scenario>.attempt_1.log`
2. `./tmp_rl/<model_stem>_validation_logs/<scenario>.attempt_2.log`

同时 `./tmp_rl/<model_stem>_validation_logs/<scenario>.log` 仍然保持为最终一次 attempt 的兼容入口。

`validate_online_candidate` 默认会对启动类抖动自动重试 1 次。

关闭这个保护时再显式传:

```bash
ros2 run usv_rl validate_online_candidate \
  --model ./tmp_rl/<candidate_model>.npz \
  --startup-retries 0
```

## 6. 失败判定与定位

候选只有在退出码为 `0` 时，才允许进入替换讨论。

优先看 gate / compare 输出里的这几类字段:

1. `checks[].reason`
2. `checks[].raw_log_diagnostics`
3. `checks[].raw_log_health`
4. `comparison_to_baseline[].candidate_raw_log_health`
5. `results[].attempt_raw_log_paths`

常见失败含义:

1. `trend_mismatch`: 行为退化，不接受替换。
2. `observation_not_ready`: 启动链未就绪，先看原始日志而不是先怀疑模型。
3. `runtime_error_detected`: 运行异常，先修环境或启动链。
4. `raw_log_missing_observation_ready`: 推理节点没有进入有效观测状态。
5. `raw_log_missing_first_residual_action`: 策略节点已起，但没真正发出第一帧残差动作。

## 7. 当前推荐顺序

日常操作顺序固定为:

1. `recommended_status`
2. `smoke_recommended_sitl`
3. 如需刷新基线，再跑 `benchmark_online_policy`
4. 新模型一律走 `validate_online_candidate`
5. 只有 gate 通过才讨论替换