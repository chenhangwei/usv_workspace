# fresh318-fresh319 random hard-seed replay

Date: 2026-05-12

Baseline checkpoint:

`/mnt/data/checkpoints/usv_rl/fresh313_single_overtake_early_return_from_fresh310.pt`

## fresh318

Checkpoint:

`/mnt/data/checkpoints/usv_rl/fresh318_random_hard_replay_from_fresh313.pt`

Training script:

```bash
bash scripts/train_fresh318.sh
```

Evaluation script:

```bash
bash scripts/eval_fresh318_long_smoke.sh
```

Full long smoke summary from `/tmp/fresh318_eval_long/summary.tsv`:

| Scenario | Result |
| --- | --- |
| `single_usv_overtaking` seeds 3071-3074 | 4/4 success, 0 collision, avg progress 0.945064 |
| `three_usv_clear_route` seeds 3001/3002 | 2/2 success, 0 collision |
| `two_usv_head_on` seed 3101 | success, 0 collision |
| `two_usv_crossing` seed 3101 | success, 0 collision |
| `three_usv_random_encounter` seeds 1458/1460/1461 | 0/3 success, 2 collisions, 1 timeout, avg progress 0.609959 |

Interpretation: fresh318 preserves the fresh313 scripted-lead overtaking fix and improves random hard-seed average progress/safety versus fresh313, but it is not a final random-hard solution.

Intermediate checkpoint scan from `/tmp/fresh318_checkpoint_random/summary.tsv`:

| Model | 1458 | 1460 | 1461 | Note |
| --- | --- | --- | --- | --- |
| `step1260` | timeout, sep 1.299920 | collision, sep 0.732856 | timeout, sep 1.293134 | best fresh318 intermediate safety shape |
| `step2520` | collision | timeout | collision | worse |
| `step4096` | timeout | collision | collision | mixed |
| `final` | collision | collision | collision in focused rerun | not stable enough |

## fresh319

Checkpoint:

`/mnt/data/checkpoints/usv_rl/fresh319_random_omega_replay_from_fresh318_step1260.pt`

Training script:

```bash
bash scripts/train_fresh319.sh
```

Focused random eval from `/tmp/fresh319_random_focus/summary.tsv`:

| Seed | Result |
| --- | --- |
| 1458 | timeout, sep 2.253854, progress 0.002153 |
| 1460 | collision, sep 0.745599, progress 0.111402 |
| 1461 | collision, sep 0.741928, progress 0.758416 |

Interpretation: fresh319 overcorrected and is not recommended.

## Historical candidate comparison

Compared fresh228, fresh233, fresh237_a070, fresh239, fresh240, and fresh318_step1260 under the same long profile in `/tmp/random_frontier_compare_1458_1460_1461/summary.tsv`. No candidate solved 1458/1460/1461 together; collisions/timeouts move between seeds.

Next direction: avoid more local trace-fit, checkpoint blends, or short omega-only replays. The remaining random-hard issue likely needs a structured pairwise role mechanism or a broader rollout objective that jointly covers 1458/1460/1461/1463 without relying on one global branch update.