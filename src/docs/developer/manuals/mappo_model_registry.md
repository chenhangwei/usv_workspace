# MAPPO Model Registry

这份文件只登记关键模型元数据，不把模型二进制提交进 Git。

使用规则：

1. 只有值得长期保留、复现实验、迁移到本地或发布到 GitHub Release 的模型，才写进这里。
2. 每次更新主候选时，同时记录对应训练线、步数、用途和简短结论。
3. 模型本体继续放在 `/mnt/data` 或外部产物存储，不进入仓库历史。

## 当前登记

| 角色 | 产物 | 路径 | 状态 | 说明 |
| --- | --- | --- | --- | --- |
| 当前主候选 | `mappo_dense_a10_parallel_headon_stabilize_20260320_003331_step_0307200.pt` | `/mnt/data/checkpoints/usv_rl/mappo_dense_a10_parallel_headon_stabilize_20260320_003331_checkpoints/mappo_dense_a10_parallel_headon_stabilize_20260320_003331_step_0307200.pt` | 推荐保留 | 当前已知最稳的在线候选，3 次在线复跑 `pass_rate=1.0`，适合作为本地续训起点。 |
| final 备份 | `mappo_dense_a10_parallel_headon_stabilize_20260320_003331.pt` | `/mnt/data/checkpoints/usv_rl/mappo_dense_a10_parallel_headon_stabilize_20260320_003331.pt` | 备份保留 | 该 run 已完整训练到 `500000` timesteps，但在线稳定性不如 `307200-step`，更适合作为归档备份。 |
| 旧安全基线 | `mappo_dense_colregs_v2_10240_recover_step_0007680.pt` | `/mnt/data/checkpoints/usv_rl/mappo_dense_colregs_v2_10240_recover_checkpoints/mappo_dense_colregs_v2_10240_recover_step_0007680.pt` | 参考保留 | 早期稳定安全基线，适合作为后续在线 benchmark 的比较参考。 |

## 当前发布建议

如果要把模型版本正式纳入 GitHub-first 工作流，第一批只建议发布下面这些产物：

1. 当前主候选 `307200-step` checkpoint
2. final `500000-step` 备份模型
3. 一份和当前基线比较有关的 benchmark / summary JSON
4. 本文件本身

不建议发布整套 `_checkpoints` 目录，也不建议把所有 `.eval.json`、`.online_repeats/` 原始目录都搬到 GitHub Release。

## 发布前整理

如果要把某个登记模型整理成可上传资产，优先使用：

[/scripts/prepare_mappo_release_bundle.sh](scripts/prepare_mappo_release_bundle.sh)

它会把选定 checkpoint 连同摘要文件、`manifest.json`、`RELEASE_NOTES.md` 和 `SHA256SUMS` 放到 `/mnt/data/releases/usv_rl` 下的独立 staging 目录，方便后续上传到 GitHub Release。

## 推荐发布命名

如果使用 GitHub Release，建议采用稳定的资产命名，而不是直接暴露临时实验目录结构：

- `mappo-headon-stabilize-primary-step-307200.pt`
- `mappo-headon-stabilize-final-step-500000.pt`
- `mappo-headon-stabilize-summary-20260320.json`

这样本地机器只需要：

1. `git pull` 最新源码
2. 查看本登记文件确认“当前主候选”
3. 从 Release 或外部存储下载对应资产

## 更新模板

新增或替换主候选时，至少补齐以下信息：

- 训练线名称
- checkpoint 文件名
- 完成步数
- 推荐角色：主候选 / 备份 / 基线 / 废弃
- 离线结论一句话
- 在线结论一句话
- 是否已对外发布