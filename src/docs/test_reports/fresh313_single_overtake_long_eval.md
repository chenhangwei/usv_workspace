# fresh313 single overtaking long-timeout evaluation

Date: 2026-05-12

Recommended checkpoint:

`/mnt/data/checkpoints/usv_rl/fresh313_single_overtake_early_return_from_fresh310.pt`

Evaluation command:

```bash
bash scripts/eval_fresh313_long_smoke.sh
```

Long profile defaults:

- `STEPS=1200`
- `EPISODE_TIMEOUT=240.0`
- `NO_PROGRESS_TIMEOUT=220.0`
- `BASE_DOMAIN=210`

Summary from `/tmp/fresh313_eval_long/summary.tsv`:

| Scenario | Seeds | Result |
| --- | --- | --- |
| `single_usv_overtaking` | 3071, 3072, 3073, 3074 | 4/4 success, 0 collision, avg progress 0.945256 |
| `three_usv_clear_route` | 3001, 3002 | 2/2 success, 0 collision |
| `two_usv_head_on` | 3101 | success, 0 collision |
| `two_usv_crossing` | 3101 | success, 0 collision |
| `three_usv_random_encounter` | 1458, 1460, 1461 | still collision/fail; not the target of this stage |

Key finding: the previous short smoke profile (`NO_PROGRESS_TIMEOUT=108.0`) was too aggressive for scripted-lead `single_usv_overtaking`. Fresh313 finishes safely, but needs roughly 639-789 steps on the checked seeds.

Trace-fit follow-ups fresh315-fresh317 did not improve over fresh313, so fresh313 remains the recommended artifact for this phase.