# USV 多船自动驾驶 RL 交接说明（给后续 Agent）

> 更新时间：2026-07-10（UTC+8）  
> 节点：CHENHANGWEI WSL2（16 CPU / 15GB RAM，**CPU 训练**）  
> 目标部署：Raspberry Pi 5（~3.3MB MLP+attention actor，CPU 推理）

把本文当作继续研究/训练的入口。先读「当前状态」和「用户优先级」，再动代码或开新 run。

---

## 1. 当前训练状态（fresh625）

| 项 | 值 |
|----|-----|
| 状态 | **训练完成**（2026-07-09 16:46，400k/400k，86 updates） |
| 最终模型 | `/mnt/data/checkpoints/usv_rl/fresh625_route_first_straight_3usv.pt` |
| Offline gate | **完成（2026-07-10）**，见 §7a；输出 `src/logs/eval_fresh625_gate/` |
| Gate 工具 | `src/scripts/eval_fresh625_gate.sh` + `summarize_fresh625_gate.py`（/tmp 版已丢，此为持久重建） |
| 日志 | `/mnt/data/checkpoints/usv_rl/fresh625_train.log` |
| 训练脚本 | `/home/chenhangwei/usv_workspace/src/scripts/train_fresh625_route_first_straight_3usv.sh` |
| 下一步 | **等用户手动 SITL 星形航线**（主判据），然后 `analyze_nav_log.py` 对比 fresh624 |

### 查进度

```bash
pgrep -af "python3 -m usv_rl.train_mappo_policy" | grep -v grep
grep '\[MAPPO\]\[update=' /mnt/data/checkpoints/usv_rl/fresh625_train.log | tail -1
ls -lt /mnt/data/checkpoints/usv_rl/fresh625_checkpoints/ | head -5
ls -la /mnt/data/checkpoints/usv_rl/fresh625_route_first_straight_3usv.pt
```

### 若再次中断：如何 resume

**不要**再跑 `train_fresh625_*.sh`（那是 `--load-weights-from` 热启动，会从头计步）。  
用 `--resume-from` 最新 step ckpt，超参与 train 脚本一致，日志 append：

```bash
RESUME_FROM="$(ls -1t /mnt/data/checkpoints/usv_rl/fresh625_checkpoints/*.pt | head -1)"
# 参考上次 resume：同 train_fresh625 全部 CLI，仅把
#   --load-weights-from ...  换成  --resume-from "$RESUME_FROM"
# 并 >> /mnt/data/checkpoints/usv_rl/fresh625_train.log
```

参考模式：`src/scripts/resume_fresh613_balanced_curriculum_3usv.sh`、`resume_fresh516_goalhold.sh`。

---

## 2. 项目目标与用户优先级（必须遵守）

多 USV（3 船）MAPPO 自动驾驶，可部署到树莓派。

**用户明确优先级（2026-07-08，fresh624 SITL 后）：**

1. **贴原始航线走直线**（低 CTE、少绕弯）——最高优先  
2. **起点少绕圈**、航段少大弧线  
3. 能有效到点（6 航点星形任务）  
4. 密起点能散开、不缠死  
5. **碰撞不是大问题**——不要为避碰牺牲直线贴线；允许接触风险

Arbiter（验收顺序）：

- **主判据 = SITL 星形航线**：每段 detour ratio（希望多数 <1.3×；fresh624 有 10/18 >1.4×）、每段净转角、loop 事件（希望 <5；fresh624=12）、CTE p90、三船 6/6 到点  
- **辅判据 = offline gate**：cluster 9/9、crossing 成功保持；**碰撞数只报告，不作为硬失败**（除非缠死式堆叠）

用户会**手动跑 SITL**；agent 负责训练、offline gate、用 `analyze_nav_log.py` 分析 `~/usv_logs/`。

---

## 3. 模型谱系（勿删祖先 ckpt）

```
fresh616 → fresh620/621 → fresh622 → fresh624 → fresh625(当前)
```

| Run | 模型路径 | 要点 |
|-----|----------|------|
| fresh616 | `.../fresh616_turn_speed_coupling_3usv.pt` | 安全基线 |
| fresh622 | `.../fresh622_balanced_no_entangle_3usv.pt` | 平衡课表+反缠绕；cluster 好；**head_on 3/3 coll** |
| fresh624 | `.../fresh624_dense_transit_route_hold_3usv.pt` | 预测贴线门控、密起点、pentagram×2；SITL 大改善；**overtaking 3/3 coll**（head_on 走廊套件副作用） |
| fresh625 | 训练中 → `.../fresh625_route_first_straight_3usv.pt` | **ROUTE-FIRST 重加权**：贴线/直线↑，避碰↓；去掉 head_on 走廊套件；6-waypoint episodes |

**Risk-4（warm-start 漂移）**：链已到第 4 代（616→622→624→625）。若 fresh625 gate/SITL 回退，下一轮应 **consolidation**：从 **fresh622** 热启，把 624+625 配方一次重建，不要再盲链第 5 代。

---

## 4. fresh625 设计摘要（相对 fresh624）

**零新增 reward 项**（Risk-1：复杂度只降不升）。只改权重 + 去掉 6 个 head_on 走廊项。

上调（贴线/直线）：

- path-deviation `0.45→0.90`，conflict-scale `1.0→0.4`
- clear-ahead CTE/heading/omega `0.30→0.45`
- straight-line-omega `1.5→2.2`
- turn-speed-coupling `0.6→0.8`，floor `0.5→0.4`
- heading-convergence `0.20→0.30`，pure-cruise `0.5→0.7`

下调（避碰）：

- collision `-30→-15`，near-miss `6→3`
- team-safety-brake `0.25→0.10`（end `0.05→0.02`）
- conflict-turn-relief `0.80→0.60`
- head_on corridor suite **全部 0**（保留基础 COLREGS head-on-turn-reward 3.0）

结构：

- `max-waypoints-per-episode` **4→6**（对齐 6 段星形任务）
- 继承 624：预测 clear-ahead 门控、per-agent entanglement、非对称 dense cluster spawn
- 400k steps，anchor 1.0→0.35 锚定 fresh624

---

## 5. 关键代码与路径

| 用途 | 路径 |
|------|------|
| 训练入口 | `src/usv_rl/usv_rl/train_mappo_policy.py` |
| 环境/奖励 | `src/usv_rl/usv_rl/multi_agent_env.py` |
| 场景 | `src/usv_rl/usv_rl/multi_agent_scenarios.py` |
| 配置默认值 | `src/usv_rl/usv_rl/config.py` |
| 训练脚本目录 | `src/scripts/train_fresh*.sh` |
| SITL 日志分析 | `src/tools/analyze_nav_log.py` |
| SITL 日志 | `~/usv_logs/` |
| 全部 ckpt 根 | `/mnt/data/checkpoints/usv_rl/` |
| Workspace | `/home/chenhangwei/usv_workspace` |
| 环境 | `source /home/chenhangwei/usv_workspace/install/setup.bash`；`PYTHONPATH=.../src/usv_rl` |

历史对话（含设计决策）：  
`/home/chenhangwei/.cursor/projects/home-chenhangwei-usv-workspace/agent-transcripts/2775b9b5-d34c-4c67-96f1-a710d1160de8/`

---

## 6. fresh625 完成后建议流程

1. **确认最终模型存在**  
   `ls -la /mnt/data/checkpoints/usv_rl/fresh625_route_first_straight_3usv.pt`

2. **Offline gate（辅判据）**  
   参考历史：`/tmp/diag_622/`、`/tmp/diag_624/`（`run_eval.sh` + `summarize.py`）。  
   场景：cluster_escape / crossing / overtaking / head_on。  
   **碰撞不硬卡**；关注 reach、缠死、是否灾难性堆叠。

3. **用户手动 SITL 星形航线**（主判据）  
   部署 fresh625 模型后跑 3 船 6 航点；日志进 `~/usv_logs/`。

4. **分析**  
   ```bash
   python3 /home/chenhangwei/usv_workspace/src/tools/analyze_nav_log.py ~/usv_logs/<run_dir>
   ```  
   对比 fresh624：first-leg net turn、detour ratio、loop 事件、CTE p90、6/6 到点。

5. **若仍不够直/仍绕圈 → fresh626 候选（单变量）**  
   - **首选**：`action-speed-scale-min` 从 0.30 下调（允许更慢让行/更小转弯半径）——此前标为 Risk-3，**单独一轮**，不要叠在 reward 重加权上。  
   - 或：sim-tau / obs-delay domain randomization（Risk-2 中期项）。  
   - 若整体回退：consolidation from fresh622，勿继续盲链。

---

## 7. 已知结果速查（fresh624 SITL，2026-07-08）

相对 fresh622 的改善：

- first-leg net turn：647–1339°（曾 4976–6100°）
- first-leg time：45–82s（曾 66–283s）
- loop events：12（曾 17，强度更弱）
- CTE p90（2–5m 邻船带）：0.35–0.69m（曾 0.85–0.99）
- 三船均 6/6 航点

仍存问题（驱动 fresh625）：

- 仍有大弧线（10/18 段 detour >1.4×）
- 起点仍有绕圈
- 用户原话：「起点还是会绕圈 / 航线还是绕弯弯 / 要走出直线 / 贴着原始航线走 / 碰撞不是大问题」

Offline gate 摘要：

- fresh622：cluster 9/9；crossing/overtaking 0 coll；**head_on 3/3 coll** → FAIL（旧硬门）
- fresh624：cluster 9/9；crossing 3/3；**overtaking 3/3 coll**；head_on 3/3 → FAIL（旧硬门）

### 7a. fresh625 offline gate 结果（2026-07-10，4 scen × 3 seeds × 3 模型 = 36 eps）

- **cluster_escape：9/9 reach、0 coll、CTE 0.11（624=0.15、622=0.42）、prog_eff 1.000、hdg_flip 0** —— 三代最好，ROUTE-FIRST 目标在 offline 侧达成
- CTE 全场景下降：0.11 / 0.11 / 0.26 / 0.13（624：0.15 / 0.38 / 0.40 / 0.20）
- 代价（符合设计取舍）：encounter coll 7/9→8/9；crossing succ 2/3→1/3（s5242/5243 于 82–111 步擦碰，minsep 0.74 vs 阈值 0.75）；head_on 3/3 擦碰（622/624 同样 3/3）
- **无缠死堆叠（pileup=0）** → 按新 arbiter（碰撞 report-only）判 **PASS**，等 SITL 主判据
- 若 SITL 通过但想收回 crossing：按 §6 fresh626 单变量（action-speed-scale-min），或 consolidation from fresh622

---

## 8. 工作纪律（后续 Agent）

1. **一次只改一类变量**（reward 重加权 / 动作边界 / 场景结构 不要同轮堆叠）。  
2. **不新增 reward 项**，除非有强证据且用户同意（Risk-1）。  
3. **祖先模型永不删**：至少保留 fresh616 / 622 / 624。  
4. 训练用 `nohup` + 日志 append；断电后用 `--resume-from`，不要误用 `--load-weights-from`。  
5. 训练占用 `ROS_DOMAIN_ID=294`；SITL/其他实验换不同 domain。  
6. 用户跑 SITL；agent 做训练、gate、日志分析与下一轮脚本设计。  
7. 写新 `train_freshNNN_*.sh` 时：文件头写清动机、相对上一版 diff、arbiter、风险登记。

---

## 9. 一句话现状

**fresh625 训练完成、offline gate PASS（cluster 9/9、CTE 全场景三代最低、无堆叠；crossing/head_on 有 minsep≈0.74 的擦碰，属设计内取舍）；模型待用户手动 SITL 星形航线裁决，之后用 `analyze_nav_log.py` 对比 fresh624 的 detour/loop/CTE p90。**
