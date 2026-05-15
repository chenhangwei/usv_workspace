# fresh320 Pairwise Role Guard Results

Date: 2026-05-12

## Goal

Continue from the fresh313/fresh318 random-hard frontier and test a structured pairwise role guard for `three_usv_random_encounter` hard seeds. The first acceptance target was zero collisions on seeds 1458, 1460, 1461, and 1463, even if the policy still timed out.

## Code Changes

- Added `random_pairwise_role_guard` trainer auxiliary loss in `usv_rl/usv_rl/train_mappo_policy.py`.
- The guard is default-off and configured by `--random-pairwise-role-guard-*` flags.
- Initial implementation used nearest-distance danger only; after fresh320b showed `rand_pair_active=0.000`, the active gate was changed to use CPA/local deconflict threat plus nearest-distance danger.
- Added `scripts/eval_random_hard_focus.sh` for focused long-timeout hard-seed evaluation.
- Added training/eval wrappers for fresh320a through fresh320f.
- Added `usv_rl/usv_rl/diagnose_mappo_action_delta.py` to compare two MAPPO checkpoints on identical traced raw observations.
- Added `reset_seed` and `scenario_geometry` to MAPPO evaluation JSON episode metrics so hard-seed reports can identify the exact spawn/goal geometry.
- Added low-disturbance event tracing to MAPPO evaluation: `--trace-event-separation`, `--trace-event-window`, optional `--trace-event-raw-observation`, and collision-only `--trace-collision-raw-observation`. The hard-seed focus/repeat scripts pass these through via `TRACE_EVENT_SEPARATION`, `TRACE_EVENT_WINDOW`, `TRACE_EVENT_RAW_OBSERVATION`, and `TRACE_COLLISION_RAW_OBSERVATION`.
- Added fresh322/fresh323/fresh324 trace-fit wrappers for reproducible offline follow-up from fresh321 event windows.

## Candidates

### fresh320a

Model/checkpoint: `/mnt/data/checkpoints/usv_rl/fresh320a_checkpoints/fresh320a_pairwise_guard_from_fresh313_step_0001260.pt`

Source: fresh313 baseline.

Focused random result:

| seed | collision | success | timeout | progress | min sep |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1458 | 1 | 0 | 0 | 0.381 | 0.721 |
| 1460 | 1 | 0 | 0 | 0.666 | 0.727 |
| 1461 | 1 | 0 | 0 | 0.632 | 0.744 |
| 1463 | 0 | 0 | 1 | -0.904 | 2.990 |

Conclusion: not recommended. Directly training from fresh313 with the first guard version regressed safety.

### fresh320b

Model: `/mnt/data/checkpoints/usv_rl/fresh320b_pairwise_guard_from_fresh318_step1260.pt`

Source: fresh318 step1260 safety-oriented checkpoint.

Training note: the first guard version did not activate in rollout (`rand_pair_active=0.000`), so fresh320b's gain is mostly inherited from fresh318 plus legacy random deconflict/role-balance constraints.

Focused random result:

| seed | collision | success | timeout | progress | min sep |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1458 | 0 | 0 | 1 | 0.164 | 0.995 |
| 1460 | 0 | 0 | 1 | 0.513 | 0.964 |
| 1461 | 0 | 0 | 1 | 0.822 | 0.856 |
| 1463 | 0 | 0 | 1 | -0.904 | 3.159 |

Regression long-smoke rows completed before stopping the duplicate random section:

| scenario | seed | collision | success | timeout | progress | min sep |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| single_usv_overtaking | 3071 | 0 | 1 | 0 | 0.943 | 2.333 |
| single_usv_overtaking | 3072 | 0 | 1 | 0 | 0.944 | 2.332 |
| single_usv_overtaking | 3073 | 0 | 1 | 0 | 0.943 | 2.336 |
| single_usv_overtaking | 3074 | 0 | 1 | 0 | 0.946 | 2.353 |
| three_usv_clear_route | 3001 | 0 | 1 | 0 | 0.950 | 4.152 |
| three_usv_clear_route | 3002 | 0 | 1 | 0 | 0.944 | 4.169 |
| two_usv_head_on | 3101 | 0 | 1 | 0 | 0.942 | 4.564 |
| two_usv_crossing | 3101 | 0 | 1 | 0 | 0.953 | 1.243 |

Conclusion: best current fresh320 candidate. It preserves the solved single-overtaking behavior and normal regressions while reaching the first random-hard target of 0 collisions on 1458/1460/1461/1463. It is not a final policy because all hard random seeds time out.

### fresh320c

Model: `/mnt/data/checkpoints/usv_rl/fresh320c_pairwise_guard_progress_from_fresh320b.pt`

Source: fresh320b. This run used the corrected CPA/local active gate; training showed the new guard was active (`rand_pair_active=0.456` then `0.474`).

Focused random result:

| seed | collision | success | timeout | progress | min sep |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1458 | 0 | 0 | 1 | -0.412 | 1.197 |
| 1460 | 1 | 0 | 0 | 0.640 | 0.736 |
| 1461 | 0 | 0 | 1 | 0.302 | 1.186 |
| 1463 | 0 | 0 | 1 | -0.899 | 3.110 |

Conclusion: not recommended. The corrected guard activates, but the progress-recovery settings over-relaxed safety and reintroduced a 1460 collision.

### fresh320d

Model: `/mnt/data/checkpoints/usv_rl/fresh320d_conservative_release_from_fresh320b.pt`

Source: fresh320b. This run kept a lighter pairwise guard and tried a conservative release/progress setting. Training still showed the pairwise guard active (`rand_pair_active=0.257`) while `rand_finish_release=0.000`.

Focused random result:

| seed | collision | success | timeout | progress | min sep |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1458 | 0 | 0 | 1 | -0.717 | 2.237 |
| 1460 | 1 | 0 | 0 | 0.664 | 0.722 |
| 1461 | 0 | 0 | 1 | 0.665 | 1.407 |
| 1463 | 0 | 0 | 1 | -0.870 | 3.136 |

Conclusion: not recommended. Even a light active pairwise guard broke seed 1460 safety.

### fresh320e

Model: `/mnt/data/checkpoints/usv_rl/fresh320e_release_no_pairguard_from_fresh320b.pt`

Source: fresh320b. This run disabled the pairwise guard and attempted a safer release/offroute/cte recovery tune. Training had `rand_pair_w=0.000`, `rand_finish_release=0.000`, and no auxiliary pretrain was intended to dominate the update.

Focused random result:

| seed | collision | success | timeout | progress | min sep |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1458 | 0 | 0 | 1 | 0.363 | 1.257 |
| 1460 | 0 | 0 | 1 | 0.550 | 0.932 |
| 1461 | 1 | 0 | 0 | 0.746 | 0.679 |

Seed 1463 was not completed because seed 1461 had already failed the hard gate.

Conclusion: not recommended. Disabling the pairwise guard improved 1458/1460 progress, but the 1260-step tune still broke seed 1461.

### fresh320f

Model: `/mnt/data/checkpoints/usv_rl/fresh320f_micro_no_release_from_fresh320b.pt`

Source: fresh320b. This was a final micro-tune attempt: total timesteps 420, learning rate 1.6e-8 to 8.0e-9, no pairwise guard, no safe-finish, no deconflict/role pretrain, and a stronger policy anchor. Training completed with `rand_pair_w=0.000`, `rand_finish_w=0.000`, `rand_deconf_pre=0.0000`, `rand_role_pre=0.0000`, and `anchor=0.0000`.

Focused random early-stop result:

| seed | collision | success | timeout | progress | min sep |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1461 | 0 | 0 | 1 | 0.563 | 1.257 |
| 1458 | 0 | 0 | 1 | -0.352 | 1.030 |
| 1460 | 1 | 0 | 0 | 0.179 | 0.748 |

Seed 1463 was not run because seed 1460 failed the hard gate.

Conclusion: not recommended. Even a very small PPO update from fresh320b can reintroduce a hard-seed collision, so the issue is not limited to the corrected pairwise guard or release objective.

## Reproducibility Addendum

After the fresh320f failure, seed 1460 was rechecked to separate PPO drift from evaluation/code drift. Under the current working tree, fresh320b no longer reproduces the earlier seed-1460 pass:

| check | code state | seed | collision | success | timeout | progress | min sep | steps |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| fresh320b final | current working tree | 1460 | 1 | 0 | 0 | 0.181 | 0.716 | 73 |
| fresh320b final trace | current working tree | 1460 | 1 | 0 | 0 | 0.263 | 0.746 | 131 |
| fresh320f trace | current working tree | 1460 | 1 | 0 | 0 | 0.656 | 0.745 | 447 |
| fresh320b final | temporary HEAD worktree | 1460 | 0 | 1 | 0 | 1.000 | 7.540 | 451 |

The current-code collision also reproduced with the fresh320b step checkpoint and the fresh318 step1260 source checkpoint. Action-delta diagnostics comparing fresh320b and fresh320f on identical current traces showed only tiny action-mean differences (`max_norm` about `3e-6` to `5e-6`), so the fresh320f failure is not primarily a meaningful action drift from the current fresh320b policy.

This means the earlier fresh320b table is code-version dependent. The old zero-collision claim should not be used as a current-working-tree gate result until the evaluation semantics are pinned and re-run. The primary mechanism is random-scenario RNG plumbing: in the original `HEAD` reset path, `MultiAgentScenarioFactory.create(...)` received `rng=None` whenever spawn/heading/goal std was zero, so `reset(seed=1460)` did not actually fix the random encounter geometry. The current working tree always passes `self._rng`, so seed 1460 now maps to a deterministic geometry. Collision/min-separation accounting also changed, but the seed plumbing alone is enough to invalidate direct comparison with the original table.

Policy-anchor diagnosis: the anchor implementation captures a frozen deep copy of the loaded actor and computes normalized action-mean MSE. The logged `anchor` value is the raw unweighted loss printed to four decimals, so `anchor=0.0000` can be expected when the action-mean drift is extremely small even with large anchor weights.

## Current Deterministic Gate Recheck

After adding `reset_seed` and `scenario_geometry`, the current working tree was re-run as a new deterministic-geometry gate. This is not comparable to the original fresh320b table because seed 1460 now maps to a fixed random-encounter geometry.

| model | collisions | timeouts | mean progress | worst min sep |
| --- | ---: | ---: | ---: | ---: |
| fresh313 | 3 | 1 | 0.175 | 0.688 |
| fresh318 step1260 | 2 | 2 | 0.025 | 0.692 |
| fresh320b | 1 | 3 | 0.170 | 0.679 |
| fresh320f | 1 | 3 | 0.010 | 0.741 |

The fresh320b seed rows from that run were:

| seed | collision | timeout | progress | min sep |
| ---: | ---: | ---: | ---: | ---: |
| 1458 | 0 | 1 | 0.175 | 0.851 |
| 1460 | 0 | 1 | 0.672 | 0.787 |
| 1461 | 1 | 0 | 0.746 | 0.679 |
| 1463 | 0 | 1 | -0.911 | 3.181 |

Repeated fresh320b runs with the same deterministic geometry were not stable:

| check | seed | collision | timeout | progress | min sep | steps |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| repeat A | 1460 | 1 | 0 | - | 0.736 | 78 |
| repeat A | 1461 | 0 | 1 | - | 1.346 | 879 |
| repeat B | 1460 | 1 | 0 | - | 0.700 | 33 |
| repeat B | 1461 | 1 | 0 | - | 0.680 | 725 |
| trace safe | 1460 | 0 | 1 | 0.623 | 0.962 | 865 |
| trace collision | 1460 | 1 | 0 | 0.191 | 0.741 | 64 |

A focused 5x repeated gate on the fragile seeds confirmed this was not a one-off:

| seed | runs | collisions | successes | timeouts | collision-free | worst min sep | mean progress | mean steps |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1460 | 5 | 3 | 0 | 2 | 0 | 0.706 | 0.439 | 457.2 |
| 1461 | 5 | 1 | 0 | 4 | 0 | 0.677 | 0.629 | 850.2 |

The safe and collision seed1460 trace runs had identical `scenario_geometry`: `usv_01` starts at `(0.0, 0.0)` and goes to `(12.0, 0.0)`, `usv_02` starts at `(-1.1440, 0.7756)` and goes to `(9.5440, -0.7756)`, and `usv_03` starts at `(6.4363, -4.9152)` and goes to `(1.9637, 4.9152)`. The usv_01/usv_02 initial separation is only about 1.38m, so this seed begins near the local-danger region.

Trace interpretation:

- Safe trace: minimum separation was 0.962m at step 51. At the closest point, `usv_01` had priority `+1.0` and stood on, while `usv_02` had priority `-1.0` and yielded. Both deconflict diagnostics were active.
- Collision trace: minimum separation was 0.741m at step 64. The dynamic ETA/priority ordering had diverged: `usv_01` was the yielding agent with priority `0.0`, `usv_02` was stand-on with priority `+1.0`, and both local/cpa threat diagnostics were active. The deconflict target for `usv_01` was approximately `(linear=0.000, omega=-0.440)`, but the executed action was `(linear=0.116, omega=+0.077)`, so the actor did not follow the yield target strongly enough at the critical moment.
- The pairwise separation sequence in the collision trace was `1.147 -> 1.033 -> 0.969 -> 0.927 -> 0.877 -> 0.795 -> 0.741` over steps 58 through 64.

Conclusion for the current gate: deterministic geometry is now pinned, but runtime outcomes are still nondeterministic enough that a single-run pass is not meaningful. Current hard-seed acceptance must be a repeated-run gate, and seed1460/1461 should be treated as fragile even when one run times out safely.

Tooling note: added `scripts/eval_random_hard_repeat_gate.sh` as a thin repeated-run wrapper around `scripts/eval_random_hard_focus.sh`. It writes per-run `summary.tsv` and per-seed `aggregate.tsv`; use it for current hard-gate acceptance instead of reading any single run in isolation.

## fresh321 Trace-Fit Follow-Up

Because the fresh320b failure trace showed weak yield-target compliance at close range, fresh321 was created as an offline trace-fit rather than another online PPO run.

Model: `/mnt/data/checkpoints/usv_rl/fresh321_yield_guard_tracefit_from_fresh320b.pt`

Source: fresh320b final checkpoint.

Tooling:

- Added `scripts/fit_fresh321_yield_guard_trace.sh` as a reproducible wrapper for collecting seed1460/1461 raw traces and fitting close-range yield/deconflict targets.
- The first automated run was stopped after the fourth trace collection took too long; the final checkpoint was fit directly from three fresh321-collected raw traces plus the earlier seed1460 collision raw trace.
- Fit summary: 2,652 target samples, 13,374 anchor samples, `deconf=1600`, `guard=1052`, final trace-fit loss about `0.0610`.

Focused fragile-seed 5x result:

| seed | runs | collisions | successes | timeouts | collision-free | worst min sep | mean progress | mean steps |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1460 | 5 | 0 | 0 | 5 | 1 | 0.785 | 0.649 | 854.6 |
| 1461 | 5 | 0 | 0 | 5 | 1 | 1.442 | 0.671 | 842.2 |

This is a clear improvement over fresh320b on the same focused repeated gate (`1460: 3/5 collisions`, `1461: 1/5 collision`). However, a later four-seed no-trace sanity run still produced one seed1461 collision:

| seed | collision | timeout | progress | min sep | steps |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 1458 | 0 | 1 | 0.070 | 1.104 | 921 |
| 1460 | 0 | 1 | 0.614 | 0.974 | 852 |
| 1461 | 1 | 0 | 0.752 | 0.698 | 670 |
| 1463 | 0 | 1 | -1.194 | 1.232 | 949 |

Three subsequent traced seed1461 attempts did not reproduce the collision, which suggests trace instrumentation can alter the timing enough to hide a failure. A separate seed1463 3x no-trace check had 0 collisions and 3 timeouts, with worst min separation `2.878` and mean progress `-0.699`.

Conclusion: fresh321 is a useful safety-improvement candidate and the correct direction for close-range yield compliance, but it is not yet an accepted hard-gate policy because current no-trace evaluation still found a seed1461 collision. Treat it as the new starting point for the next repair, not as final.

## Event-Trace Follow-Up: fresh322/fresh323/fresh324

To avoid the timing disturbance from full `TRACE_STRIDE=1` traces, event tracing was added. With `TRACE_STRIDE=0`, the evaluator records only a short diagnostic window after `pairwise_min_separation <= TRACE_EVENT_SEPARATION`. Raw observations are omitted by default and are included only when `TRACE_EVENT_RAW_OBSERVATION=1`.

Validation notes:

- A direct forced event-trace smoke with threshold `20` recorded 20 `event_trigger` samples without raw observations.
- A forced raw event-trace smoke recorded 12 event samples with per-agent raw observation length 60.
- Short low-threshold smokes with threshold `1.6` produced zero samples because the early episode minimum separation stayed above the threshold.

Fresh321 seed1461 low-disturbance event raw windows were collected at domains 104 and 105 with `TRACE_EVENT_SEPARATION=2.5`, `TRACE_EVENT_WINDOW=24`, and `TRACE_EVENT_RAW_OBSERVATION=1`:

| run | collision | timeout | progress | steps | trace samples | raw agent samples | min trace sep | min trace step |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| d104 | 0 | 1 | 0.796 | 994 | 206 | 618 | 1.677 | 341 |
| d105 | 0 | 1 | 0.724 | 931 | 203 | 609 | 1.619 | 338 |

At the closest d104/d105 samples, `usv_02` was the yielding agent with priority `-1.0`, distance-to-goal about 10m, and linear speed about `0.20-0.22` while the deconflict target linear speed was about `0.012`. These windows were near misses rather than true fresh321 collisions.

Three follow-up checkpoints were tried and rejected:

| model | source | trace inputs | key fit settings | samples | guard samples | focus 1460 collisions | focus 1461 collisions | conclusion |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: | --- |
| fresh322 | fresh321 | d104+d105 | anchor 28, guard 3.2, max linear 0.055, fixed negative omega guard | 746 | 169 | 0/3 | 1/3 | Rejected: seed1461 collision remained. |
| fresh323 | fresh321 | d104+d105+fresh322 d115 collision trace | anchor 32, guard 3.0, max linear 0.075, no fixed omega sign | 885 | 250 | 2/3 | 0/3 | Rejected: repaired seed1461 but broke seed1460. |
| fresh324 | fresh321 | d104+d105 only | anchor 40, guard 2.2, max linear 0.100, no fixed omega sign | 746 | 169 | 2/3 | 0/3 | Rejected: still broke seed1460. |
| fresh325 | fresh321 | d207 near-miss only | anchor 70, guard 1.25, max linear 0.120, no fixed omega sign | 163 | 47 | 1/3 | 3/3 | Rejected: micro-fit made seed1461 worse. |
| fresh326 | fresh321 | d207 true collision raw only | anchor 120, guard 1.6, max linear 0.070, deconf 0.6 | 2 | 1 | 1/3 | 1/3 | Rejected: single collision sample did not repair gate. |

fresh322 focus gate details:

| seed | runs | collisions | timeouts | collision-free | worst min sep | mean progress |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1460 | 3 | 0 | 3 | 1 | 0.791 | 0.618 |
| 1461 | 3 | 1 | 2 | 0 | 0.742 | 0.626 |

fresh323 focus gate details:

| seed | runs | collisions | timeouts | collision-free | worst min sep | mean progress |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1460 | 3 | 2 | 1 | 0 | 0.723 | 0.624 |
| 1461 | 3 | 0 | 3 | 1 | 1.345 | 0.630 |

fresh324 focus gate details:

| seed | runs | collisions | timeouts | collision-free | worst min sep | mean progress |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1460 | 3 | 2 | 1 | 0 | 0.736 | 0.620 |
| 1461 | 3 | 0 | 3 | 1 | 1.655 | 0.642 |

fresh325 was a final micro-fit check using only the same-domain d207 fresh321 event near-miss raw trace. It did not use the d104/d105 windows. A no-trace d207 run immediately before this captured a true fresh321 seed1461 collision at step 696 with min separation 0.735, but the low-disturbance raw capture attempts did not preserve the collision. The closest raw event capture reached 1.176m at step 636 with the same geometry; `usv_02` was yielding and closest to `usv_03`, with actual linear speed `0.289` versus deconf/guard target linear about `0.012`. Fitting even this small slice made the focused gate worse:

| seed | runs | collisions | timeouts | collision-free | worst min sep | mean progress |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1460 | 3 | 1 | 2 | 0 | 0.742 | 0.627 |
| 1461 | 3 | 3 | 0 | 0 | 0.714 | 0.736 |

After fresh325, collision-only raw capture was added to avoid event-window perturbation. With `TRACE_COLLISION_RAW_OBSERVATION=1`, the evaluator records exactly one raw diagnostic sample only after the collision condition is already true. This successfully captured a fresh321 seed1461/domain207 true collision on the first attempt:

| capture | collision | timeout | progress | step | min sep | trace samples | raw agent samples |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| fresh321 d207 collision raw | 1 | 0 | 0.708 | 749 | 0.708 | 1 | 3 |

At that collision point, `usv_01` was the yielding vessel nearest to `usv_03`: actual action `(linear=0.224, omega=0.226)`, deconf target approximately `(linear=0.000, omega=0.440)`, with raw observation length 60. A very small fresh326 fit used only this true collision sample, producing 2 target samples and 8,775 anchor samples. It still failed the focused repeated gate:

| seed | runs | collisions | timeouts | collision-free | worst min sep | mean progress |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1460 | 3 | 1 | 2 | 0 | 0.731 | 0.624 |
| 1461 | 3 | 1 | 2 | 0 | 0.745 | 0.673 |

The fresh322 seed1461 collision was reproduced with event raw at domain 115. At collision step 715, the closest pair was `usv_01/usv_03` at 0.697m. `usv_01` was yielding with deconflict target approximately `(linear=0.000, omega=0.440)` but executed `(linear=0.263, omega=0.259)`; `usv_03` was stand-on and closest to `usv_01`. Adding this fresh322-induced trace to fresh323 repaired seed1461 in the small gate but broke seed1460, so this was not a valid repair path.

Conclusion: fresh322/fresh323/fresh324/fresh325/fresh326 are negative results. Event and collision-only tracing are useful and validated, and collision-only capture can now preserve a true fresh321 seed1461 failure raw sample. However, fitting seed1461 near-miss windows or a single true collision sample over-specialized the actor and damaged either seed1460 or seed1461. The current best random-hard branch remains fresh321; do not accept any of fresh322/fresh323/fresh324/fresh325/fresh326.

## Current Recommendation

Keep fresh313 as the overall stable checkpoint for the previously solved single-overtaking objective. For the random-hard branch, fresh321 is safer than fresh320b on the focused seed1460/1461 repeated gate, but it is still not final because no-trace seed1461 collisions remain reproducible. fresh322/fresh323/fresh324/fresh325/fresh326 are rejected negative follow-ups because each failed the focused repeated no-trace gate. Do not compare progress from any candidate that fails the repeated collision gate; progress remains secondary until the repeated no-trace collision count is zero.

## Next Step

Do not continue fresh320c/d/e/f. Any further work should first diagnose why fresh320b is so fragile under PPO updates before adding more progress pressure:

- Re-evaluate fresh320b as the frozen reference whenever trainer, environment, scenario, or eval settings change.
- Pin the code revision and random-scenario seed semantics used for hard-seed gates before accepting/rejecting a checkpoint.
- Treat the current deterministic seed mapping as a new gate; do not mix it with older reports where `std=0` made random geometry unseeded.
- Use repeated runs for deterministic hard seeds because ROS/sim timing can still alter the dynamic ETA priority ordering and final outcome.
- Prefer offline/logit-level or action-delta diagnostics before another online PPO fine-tune.
- Collision-only raw capture now provides true failure samples with minimal overhead. The first single-sample actor micro-fit still failed, so the next repair should not be another one-off trace-fit. Prefer collecting a small set of true collision raw samples across repeated d207/seed1461 and seed1460 failures, then fitting only if the target direction is consistent across samples.
- Next repair should start from fresh321 and target the residual no-trace seed1461 collision without adding progress pressure. Do not continue near-miss-only or single-sample trace-fit variants. Acceptance must be no-trace repeated evaluation, and any seed1461 repair must immediately re-run seed1460 to guard against regression.
- Treat 0 collisions as a hard gate; only compare progress among zero-collision candidates.

Validation commands used:

- `/bin/python3 -m py_compile usv_rl/usv_rl/train_mappo_policy.py`
- `bash -n scripts/eval_random_hard_focus.sh scripts/eval_random_hard_repeat_gate.sh scripts/fit_fresh322_event_guard_trace.sh scripts/fit_fresh323_event_guard_trace.sh scripts/fit_fresh324_conservative_event_guard.sh`
- `/bin/python3 -m py_compile usv_rl/usv_rl/evaluate_mappo_policy.py usv_rl/usv_rl/fit_mappo_trace_targets.py usv_rl/usv_rl/diagnose_mappo_action_delta.py`
- `bash -n scripts/eval_random_hard_focus.sh scripts/train_fresh320a.sh scripts/train_fresh320b.sh scripts/train_fresh320c.sh scripts/train_fresh320d.sh scripts/train_fresh320e.sh scripts/train_fresh320f.sh scripts/eval_fresh320a_long_smoke.sh scripts/eval_fresh320b_long_smoke.sh scripts/eval_fresh320c_long_smoke.sh scripts/eval_fresh320d_long_smoke.sh scripts/eval_fresh320e_long_smoke.sh scripts/eval_fresh320f_long_smoke.sh`
- `MODEL=/mnt/data/checkpoints/usv_rl/fresh320b_pairwise_guard_from_fresh318_step1260.pt OUT=/tmp/fresh320b_random_focus LABEL=fresh320b bash scripts/eval_random_hard_focus.sh`
- `MODEL=/mnt/data/checkpoints/usv_rl/fresh320c_pairwise_guard_progress_from_fresh320b.pt OUT=/tmp/fresh320c_random_focus LABEL=fresh320c bash scripts/eval_random_hard_focus.sh`
- `MODEL=/mnt/data/checkpoints/usv_rl/fresh320d_conservative_release_from_fresh320b.pt OUT=/tmp/fresh320d_random_focus LABEL=fresh320d bash scripts/eval_random_hard_focus.sh`
- `MODEL=/mnt/data/checkpoints/usv_rl/fresh320e_release_no_pairguard_from_fresh320b.pt OUT=/tmp/fresh320e_random_focus LABEL=fresh320e bash scripts/eval_random_hard_focus.sh` stopped after seed 1461 failed.
- `MODEL=/mnt/data/checkpoints/usv_rl/fresh320f_micro_no_release_from_fresh320b.pt OUT=/tmp/fresh320f_random_focus LABEL=fresh320f bash scripts/eval_random_hard_focus.sh` run as early-stop per seed; stopped after seed 1460 failed.
- `/bin/python3 -m py_compile usv_rl/usv_rl/diagnose_mappo_action_delta.py`
- Current working-tree recheck: `SEEDS=1460 MODEL=/mnt/data/checkpoints/usv_rl/fresh320b_pairwise_guard_from_fresh318_step1260.pt OUT=/tmp/fresh320b_seed1460_notrace_recheck LABEL=fresh320b BASE_DOMAIN=... bash scripts/eval_random_hard_focus.sh`
- HEAD-code A/B recheck: temporary git worktree at `HEAD`, `PYTHONPATH=<tmp>/src/usv_rl`, same fresh320b model, seed 1460, direct `usv_rl.evaluate_mappo_policy` invocation; result was collision 0, success 1, progress 1.000, min separation 7.540, steps 451.
- RNG audit: `HEAD` used `rng=self._rng if (std > 0) else None`; current working tree uses `rng=self._rng` unconditionally, so deterministic hard-seed geometry is new relative to the original fresh320 table.
- Geometry-field smoke: a 3-step current-code fresh320b seed1460 eval wrote `reset_seed=1460` and `scenario_geometry` for `three_usv_random_encounter`; current deterministic geometry included `usv_02` spawn `(-1.1440, 0.7756, 6.1391)` to goal `(9.5440, -0.7756)` and `usv_03` spawn `(6.4363, -4.9152, 8.2810)` to goal `(1.9637, 4.9152)`.