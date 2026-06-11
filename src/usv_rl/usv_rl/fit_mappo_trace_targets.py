import argparse
import json
from pathlib import Path

import numpy as np
import torch

from .observation_normalizer import ObservationNormalizer
from .multi_agent_types import ENCOUNTER_TYPE_COUNT, LOCAL_EGO_FEATURE_COUNT, NEIGHBOR_FEATURE_COUNT


TARGET_SPECS = {
    'deconf': ('random_deconflict_weighted_active', 'deconf_target'),
    'offroute': ('random_offroute_finish_weighted_active', 'offroute_target'),
    'cte': ('random_cte_recovery_weighted_active', 'cte_recovery_target'),
    'finish': ('random_safe_finish_weighted_active', 'safe_finish_target'),
}
TARGET_KINDS = tuple(TARGET_SPECS) + ('clear', 'guard', 'stall_start', 'goal_return', 'late_clear', 'recovery_turn', 'recovery', 'edge_coast', 'goal_hold')
TARGET_KINDS = TARGET_KINDS + ('imitate', 'scripted_overtake')
TARGET_KIND_TO_ID = {kind: index for index, kind in enumerate(TARGET_KINDS)}


def parse_args():
    parser = argparse.ArgumentParser(description='Fit MAPPO actor on traced active target slices.')
    parser.add_argument('--model', required=True, help='Input MAPPO checkpoint.')
    parser.add_argument('--trace-json', action='append', required=True, help='Trace JSON containing raw_observation and mask_diagnostics. Repeatable.')
    parser.add_argument('--anchor-trace-json', action='append', default=[], help='Extra raw-observation trace JSON used only for source-policy anchoring. Repeatable.')
    parser.add_argument('--preserve-trace-json', action='append', default=[], help='Trace JSON used as supervised action-preservation samples. Repeatable.')
    parser.add_argument('--output', required=True, help='Output checkpoint path.')
    parser.add_argument('--epochs', type=int, default=80, help='Actor-only fitting epochs over traced slices.')
    parser.add_argument('--batch-size', type=int, default=128, help='Trace fitting minibatch size.')
    parser.add_argument('--learning-rate', type=float, default=2.0e-5, help='Adam learning rate for trace fitting.')
    parser.add_argument('--max-grad-norm', type=float, default=0.5, help='Gradient clipping norm.')
    parser.add_argument('--anchor-weight', type=float, default=1.5, help='Weight for preserving the source policy on all traced observations.')
    parser.add_argument('--anchor-batch-size', type=int, default=256, help='Anchor minibatch size sampled from all traced observations.')
    parser.add_argument('--min-error-norm', type=float, default=0.08, help='Only fit traced targets whose current normalized L2 error is at least this value.')
    parser.add_argument('--target-priority', default='deconf,offroute,cte,finish', help='Comma-separated target priority order.')
    parser.add_argument('--offroute-weight', type=float, default=2.2, help='Sample weight for offroute targets.')
    parser.add_argument('--cte-weight', type=float, default=1.8, help='Sample weight for CTE recovery targets.')
    parser.add_argument('--deconf-weight', type=float, default=1.4, help='Sample weight for deconflict targets.')
    parser.add_argument('--finish-weight', type=float, default=1.0, help='Sample weight for safe-finish targets.')
    parser.add_argument('--deconf-min-linear', type=float, default=0.0, help='Absolute lower bound for fitted deconflict target linear speed. 0 preserves traced targets.')
    parser.add_argument('--deconf-max-linear-drop', type=float, default=-1.0, help='Limit deconflict target speed reduction from the source policy. Negative disables.')
    parser.add_argument('--deconf-linear-blend', type=float, default=1.0, help='Blend deconflict linear target toward source policy action. 1 uses traced target; 0 uses source action.')
    parser.add_argument('--deconf-omega-blend', type=float, default=1.0, help='Blend deconflict omega target toward source policy action. 1 uses traced target; 0 uses source action.')
    parser.add_argument('--target-min-linear', type=float, default=0.0, help='Absolute lower bound for every fitted target linear speed. 0 preserves traced targets.')
    parser.add_argument('--target-max-linear-drop', type=float, default=-1.0, help='Limit every target speed reduction from the source policy. Negative disables.')
    parser.add_argument('--target-linear-blend', type=float, default=1.0, help='Blend every linear target toward source policy action. 1 uses traced target; 0 uses source action.')
    parser.add_argument('--target-omega-blend', type=float, default=1.0, help='Blend every omega target toward source policy action. 1 uses traced target; 0 uses source action.')
    parser.add_argument('--target-shape-kinds', default='deconf,offroute,cte,finish', help='Comma-separated target kinds affected by target-* shaping controls.')
    parser.add_argument('--low-speed-source-threshold', type=float, default=-1.0, help='If nonnegative, multiply target sample weights where source-policy linear speed is below this threshold.')
    parser.add_argument('--low-speed-weight', type=float, default=1.0, help='Weight multiplier for low-source-speed target samples.')
    parser.add_argument('--low-speed-weight-kinds', default='deconf,offroute,cte,finish', help='Comma-separated target kinds affected by low-source-speed reweighting.')
    parser.add_argument('--clear-speedup-weight', type=float, default=0.0, help='Sample weight for synthetic clear-ahead speed-up targets. 0 disables.')
    parser.add_argument('--clear-speedup-source-threshold', type=float, default=0.13, help='Only add clear speed-up samples when traced source linear speed is below this value.')
    parser.add_argument('--clear-speedup-target-linear', type=float, default=0.26, help='Target linear speed for synthetic clear-ahead speed-up samples.')
    parser.add_argument('--clear-speedup-min-distance', type=float, default=2.0, help='Only add clear speed-up samples farther than this distance from goal.')
    parser.add_argument('--clear-speedup-max-cte', type=float, default=0.8, help='Only add clear speed-up samples with absolute CTE below this value.')
    parser.add_argument('--clear-speedup-min-separation', type=float, default=3.0, help='Only add clear speed-up samples when team min separation is at least this value.')
    parser.add_argument('--clear-speedup-max-threat', type=float, default=0.08, help='Only add clear speed-up samples when traced threat score is at most this value.')
    parser.add_argument('--clear-speedup-min-step', type=int, default=0, help='Only add clear speed-up samples at or after this traced step index.')
    parser.add_argument('--late-clear-weight', type=float, default=0.0, help='Sample weight for late low-speed clear-ahead recovery targets. 0 disables.')
    parser.add_argument('--late-clear-source-threshold', type=float, default=0.17, help='Only add late clear samples when traced source linear speed is below this value.')
    parser.add_argument('--late-clear-target-linear', type=float, default=0.28, help='Target linear speed for late clear recovery samples.')
    parser.add_argument('--late-clear-min-distance', type=float, default=2.5, help='Only add late clear samples farther than this distance from goal.')
    parser.add_argument('--late-clear-max-cte', type=float, default=2.8, help='Only add late clear samples with absolute CTE below this value.')
    parser.add_argument('--late-clear-min-separation', type=float, default=3.0, help='Only add late clear samples when team min separation is at least this value.')
    parser.add_argument('--late-clear-max-threat', type=float, default=0.08, help='Only add late clear samples when traced threat score is at most this value.')
    parser.add_argument('--late-clear-min-step', type=int, default=140, help='Only add late clear samples at or after this traced step index.')
    parser.add_argument('--late-clear-cte-omega-blend', type=float, default=0.6, help='Blend late clear omega toward the traced CTE recovery omega target.')
    parser.add_argument('--late-clear-cte-slow-threshold', type=float, default=-1.0, help='If positive, cap late clear linear target when absolute CTE is above this value.')
    parser.add_argument('--late-clear-cte-recovery-linear', type=float, default=0.16, help='Late clear linear target used when CTE exceeds late-clear-cte-slow-threshold.')
    parser.add_argument('--late-clear-allow-deconf', action='store_true', help='Allow late clear samples even while deconflict target is active.')
    parser.add_argument('--late-clear-agent', action='append', dest='late_clear_agents', default=None, help='Only add late-clear samples for this agent id. Repeatable; unset allows all agents.')
    parser.add_argument('--stall-start-weight', type=float, default=0.0, help='Sample weight for clear low-progress stall-start targets. 0 disables.')
    parser.add_argument('--stall-start-agent', action='append', dest='stall_start_agents', default=None, help='Only add stall-start samples for this agent id. Repeatable; unset allows all agents.')
    parser.add_argument('--stall-start-source-threshold', type=float, default=0.04, help='Only add stall-start samples when traced source linear speed is below this value.')
    parser.add_argument('--stall-start-target-linear', type=float, default=0.14, help='Target linear speed for stall-start samples.')
    parser.add_argument('--stall-start-min-distance', type=float, default=5.0, help='Only add stall-start samples farther than this distance from goal.')
    parser.add_argument('--stall-start-max-route-progress', type=float, default=0.03, help='Only add stall-start samples whose route progress is at most this value.')
    parser.add_argument('--stall-start-max-cte', type=float, default=1.0, help='Only add stall-start samples with absolute CTE below this value.')
    parser.add_argument('--stall-start-min-separation', type=float, default=2.75, help='Only add stall-start samples when team min separation is at least this value.')
    parser.add_argument('--stall-start-max-threat', type=float, default=0.08, help='Only add stall-start samples when traced threat score is at most this value.')
    parser.add_argument('--stall-start-min-step', type=int, default=40, help='Only add stall-start samples at or after this traced step index.')
    parser.add_argument('--stall-start-max-step', type=int, default=-1, help='If nonnegative, only add stall-start samples at or before this traced step index.')
    parser.add_argument('--stall-start-cte-omega-blend', type=float, default=0.55, help='Blend stall-start omega toward the traced CTE recovery omega target.')
    parser.add_argument('--stall-start-heading-slow-threshold', type=float, default=2.20, help='If positive, cap stall-start linear target when absolute heading error is above this value.')
    parser.add_argument('--stall-start-heading-recovery-linear', type=float, default=0.07, help='Stall-start linear target used when heading error exceeds stall-start-heading-slow-threshold.')
    parser.add_argument('--stall-start-allow-deconf', action='store_true', help='Allow stall-start samples even while deconflict target is active.')
    parser.add_argument('--goal-return-weight', type=float, default=0.0, help='Sample weight for high-progress far-from-goal return targets. 0 disables.')
    parser.add_argument('--goal-return-agent', action='append', dest='goal_return_agents', default=None, help='Only add goal-return samples for this agent id. Repeatable; unset allows all agents.')
    parser.add_argument('--goal-return-source-threshold', type=float, default=0.12, help='Only add goal-return samples when traced source linear speed is above this value.')
    parser.add_argument('--goal-return-target-linear', type=float, default=0.11, help='Target linear speed for goal-return samples.')
    parser.add_argument('--goal-return-min-distance', type=float, default=2.5, help='Only add goal-return samples farther than this distance from goal.')
    parser.add_argument('--goal-return-min-route-progress', type=float, default=0.92, help='Only add goal-return samples whose route progress is at least this value.')
    parser.add_argument('--goal-return-max-cte', type=float, default=3.2, help='Only add goal-return samples with absolute CTE below this value.')
    parser.add_argument('--goal-return-min-separation', type=float, default=2.75, help='Only add goal-return samples when team min separation is at least this value.')
    parser.add_argument('--goal-return-max-threat', type=float, default=0.10, help='Only add goal-return samples when traced threat score is at most this value.')
    parser.add_argument('--goal-return-min-step', type=int, default=320, help='Only add goal-return samples at or after this traced step index.')
    parser.add_argument('--goal-return-safe-omega-blend', type=float, default=0.75, help='Blend goal-return omega toward the traced safe-finish omega target.')
    parser.add_argument('--goal-return-allow-deconf', action='store_true', help='Allow goal-return samples even while deconflict target is active.')
    parser.add_argument('--recovery-speedup-weight', type=float, default=0.0, help='Sample weight for low-threat recovery speed-up targets. 0 disables.')
    parser.add_argument('--recovery-speedup-source-threshold', type=float, default=0.34, help='Only add recovery speed-up samples when traced source linear speed is below this value.')
    parser.add_argument('--recovery-speedup-target-linear', type=float, default=0.30, help='Target linear speed for recovery speed-up samples.')
    parser.add_argument('--recovery-speedup-min-distance', type=float, default=2.5, help='Only add recovery samples farther than this distance from goal.')
    parser.add_argument('--recovery-speedup-max-cte', type=float, default=3.2, help='Only add recovery samples with absolute CTE below this value.')
    parser.add_argument('--recovery-speedup-min-separation', type=float, default=2.4, help='Only add recovery samples when team min separation is at least this value.')
    parser.add_argument('--recovery-speedup-max-threat', type=float, default=0.16, help='Only add recovery samples when traced threat score is at most this value.')
    parser.add_argument('--recovery-speedup-min-step', type=int, default=120, help='Only add recovery samples at or after this traced step index.')
    parser.add_argument('--recovery-speedup-min-route-progress', type=float, default=0.0, help='Only add recovery samples whose route progress is at least this value.')
    parser.add_argument('--recovery-speedup-max-route-progress', type=float, default=1.1, help='Only add recovery samples whose route progress is at most this value.')
    parser.add_argument('--recovery-speedup-max-heading-error', type=float, default=3.2, help='Only add recovery speed-up samples whose absolute heading error is at most this value.')
    parser.add_argument('--recovery-speedup-cte-omega-blend', type=float, default=0.75, help='Blend recovery omega toward the traced CTE recovery omega target.')
    parser.add_argument('--recovery-speedup-agent', action='append', dest='recovery_speedup_agents', default=None, help='Only add recovery speed-up samples for this agent id. Repeatable; unset allows all agents.')
    parser.add_argument('--edge-coast-weight', type=float, default=0.0, help='Sample weight for low-threat edge coasting targets that keep progress after reaching high CTE. 0 disables.')
    parser.add_argument('--edge-coast-agent', action='append', dest='edge_coast_agents', default=None, help='Only add edge-coast samples for this agent id. Repeatable; unset allows all agents.')
    parser.add_argument('--edge-coast-source-threshold', type=float, default=0.12, help='Only add edge-coast samples when traced source linear speed is below this value.')
    parser.add_argument('--edge-coast-target-linear', type=float, default=0.08, help='Target linear speed for edge-coast samples.')
    parser.add_argument('--edge-coast-min-distance', type=float, default=2.5, help='Only add edge-coast samples farther than this distance from goal.')
    parser.add_argument('--edge-coast-min-abs-cte', type=float, default=2.6, help='Only add edge-coast samples with absolute CTE at least this value.')
    parser.add_argument('--edge-coast-max-abs-cte', type=float, default=3.1, help='Only add edge-coast samples with absolute CTE at most this value.')
    parser.add_argument('--edge-coast-min-separation', type=float, default=2.4, help='Only add edge-coast samples when team min separation is at least this value.')
    parser.add_argument('--edge-coast-max-threat', type=float, default=0.16, help='Only add edge-coast samples when traced threat score is at most this value.')
    parser.add_argument('--edge-coast-min-step', type=int, default=120, help='Only add edge-coast samples at or after this traced step index.')
    parser.add_argument('--edge-coast-min-route-progress', type=float, default=0.0, help='Only add edge-coast samples whose route progress is at least this value.')
    parser.add_argument('--edge-coast-max-route-progress', type=float, default=1.1, help='Only add edge-coast samples whose route progress is at most this value.')
    parser.add_argument('--edge-coast-max-heading-error', type=float, default=3.2, help='Only add edge-coast samples whose absolute heading error is at most this value.')
    parser.add_argument('--edge-coast-omega-zero-blend', type=float, default=0.75, help='Blend edge-coast omega toward zero; 0 preserves source omega, 1 targets zero.')
    parser.add_argument('--edge-coast-max-omega-abs', type=float, default=0.04, help='Clamp absolute edge-coast omega to this value. Negative disables.')
    parser.add_argument('--recovery-turn-weight', type=float, default=0.0, help='Sample weight for low-threat route-heading recovery turn targets. 0 disables.')
    parser.add_argument('--recovery-turn-agent', action='append', dest='recovery_turn_agents', default=None, help='Only add recovery-turn samples for this agent id. Repeatable; unset allows all agents.')
    parser.add_argument('--recovery-turn-target-linear', type=float, default=0.05, help='Target linear speed for route-heading recovery turn samples.')
    parser.add_argument('--recovery-turn-min-distance', type=float, default=2.5, help='Only add recovery turn samples farther than this distance from goal.')
    parser.add_argument('--recovery-turn-min-abs-cte', type=float, default=1.2, help='Only add recovery turn samples with absolute CTE at least this value.')
    parser.add_argument('--recovery-turn-min-heading-error', type=float, default=1.2, help='Only add recovery turn samples with absolute heading error at least this value.')
    parser.add_argument('--recovery-turn-max-heading-error', type=float, default=3.2, help='Only add recovery turn samples with absolute heading error at most this value.')
    parser.add_argument('--recovery-turn-heading-sign', choices=('any', 'negative', 'positive'), default='any', help='Optional signed heading-error gate for recovery turn samples.')
    parser.add_argument('--recovery-turn-min-separation', type=float, default=2.4, help='Only add recovery turn samples when team min separation is at least this value.')
    parser.add_argument('--recovery-turn-max-threat', type=float, default=0.16, help='Only add recovery turn samples when traced threat score is at most this value.')
    parser.add_argument('--recovery-turn-min-step', type=int, default=120, help='Only add recovery turn samples at or after this traced step index.')
    parser.add_argument('--recovery-turn-max-step', type=int, default=-1, help='If nonnegative, only add recovery turn samples at or before this traced step index.')
    parser.add_argument('--recovery-turn-min-route-progress', type=float, default=0.0, help='Only add recovery turn samples whose route progress is at least this value.')
    parser.add_argument('--recovery-turn-max-route-progress', type=float, default=0.98, help='Only add recovery turn samples whose route progress is at most this value.')
    parser.add_argument('--recovery-turn-omega-blend', type=float, default=1.0, help='Blend recovery turn omega toward the traced CTE/offroute recovery omega target.')
    parser.add_argument('--recovery-turn-min-omega-abs', type=float, default=0.20, help='Minimum absolute omega magnitude for recovery turn samples.')
    parser.add_argument('--recovery-turn-heading-gain', type=float, default=0.0, help='If positive, use heading_error * gain as the recovery turn omega teacher before clipping, scaled by --recovery-turn-heading-omega-sign.')
    parser.add_argument('--recovery-turn-heading-omega-sign', type=float, default=-1.0, help='Sign multiplier for recovery-turn heading omega. -1 preserves legacy behavior; +1 matches yaw += omega * dt.')
    parser.add_argument('--recovery-turn-max-omega', type=float, default=0.45, help='Absolute omega cap for recovery-turn heading controller samples.')
    parser.add_argument('--recovery-turn-target-omega-cap', type=float, default=-1.0, help='If nonnegative, clip the recovery turn teacher omega before blending.')
    parser.add_argument('--risk-guard-weight', type=float, default=0.0, help='Sample weight for synthetic risk guard targets that preserve/cap cautious source actions. 0 disables.')
    parser.add_argument('--risk-guard-min-threat', type=float, default=0.08, help='Add risk guard samples when threat score is at least this value.')
    parser.add_argument('--risk-guard-max-separation', type=float, default=2.8, help='Add risk guard samples when team min separation is at most this value.')
    parser.add_argument('--risk-guard-max-linear', type=float, default=0.12, help='Maximum linear target for risk guard samples.')
    parser.add_argument('--risk-guard-yield-max-linear', type=float, default=-1.0, help='Role-specific maximum linear target for yield risk guard samples. Negative uses risk-guard-max-linear.')
    parser.add_argument('--risk-guard-standon-max-linear', type=float, default=-1.0, help='Role-specific maximum linear target for stand-on risk guard samples. Negative uses risk-guard-max-linear.')
    parser.add_argument('--risk-guard-omega-blend', type=float, default=1.0, help='Blend risk guard omega toward the deconflict target when active; 0 preserves traced source omega.')
    parser.add_argument('--risk-guard-yield-min-omega-abs', type=float, default=0.0, help='Minimum absolute omega target for yield risk guard samples. 0 disables.')
    parser.add_argument('--risk-guard-standon-min-omega-abs', type=float, default=0.0, help='Minimum absolute omega target for stand-on risk guard samples. 0 disables.')
    parser.add_argument('--risk-guard-min-starboard-omega', type=float, default=0.0, help='If positive, enforce at least this starboard-turn magnitude (negative omega) for yield risk guard samples.')
    parser.add_argument('--risk-guard-min-starboard-threat', type=float, default=0.0, help='Only enforce risk-guard-min-starboard-omega when threat score is at least this value.')
    parser.add_argument('--risk-guard-min-distance', type=float, default=2.0, help='Only add risk guard samples farther than this distance from goal.')
    parser.add_argument('--risk-guard-agent', action='append', dest='risk_guard_agents', default=None, help='Only add risk guard samples for this agent id. Repeatable; unset allows all agents.')
    parser.add_argument('--risk-guard-min-route-progress', type=float, default=0.0, help='Only add risk guard samples whose route progress is at least this value.')
    parser.add_argument('--risk-guard-max-route-progress', type=float, default=1.1, help='Only add risk guard samples whose route progress is at most this value.')
    parser.add_argument('--risk-guard-min-abs-cte', type=float, default=0.0, help='Only add risk guard samples whose absolute CTE is at least this value.')
    parser.add_argument('--risk-guard-max-step', type=int, default=-1, help='If nonnegative, only add risk guard samples at or before this traced step index.')
    parser.add_argument('--risk-guard-yield-only', action='store_true', help='Only add risk guard samples for traced deconf yield agents.')
    parser.add_argument('--risk-guard-standon-only', action='store_true', help='Only add risk guard samples for traced stand-on agents.')
    parser.add_argument('--risk-guard-require-threat', action='store_true', help='Only add risk guard samples when threat score is at least risk-guard-min-threat.')
    parser.add_argument('--goal-hold-weight', type=float, default=0.0, help='Sample weight for near-goal blocked hold targets. 0 disables.')
    parser.add_argument('--goal-hold-source-threshold', type=float, default=0.08, help='Only add goal-hold samples when traced source linear speed is above this value.')
    parser.add_argument('--goal-hold-max-distance', type=float, default=1.8, help='Only add goal-hold samples within this distance from goal.')
    parser.add_argument('--goal-hold-max-cte', type=float, default=1.2, help='Only add goal-hold samples with absolute CTE at most this value.')
    parser.add_argument('--goal-hold-max-separation', type=float, default=2.4, help='Add goal-hold samples when team min separation is at most this value.')
    parser.add_argument('--goal-hold-min-threat', type=float, default=0.15, help='Add goal-hold samples when traced threat score is at least this value.')
    parser.add_argument('--goal-hold-target-linear', type=float, default=0.04, help='Target linear speed for near-goal blocked hold samples.')
    parser.add_argument('--goal-hold-omega-blend', type=float, default=1.0, help='Blend goal-hold omega toward the traced safe-finish omega target.')
    parser.add_argument('--imitate-action-weight', type=float, default=0.0, help='Sample weight for imitating traced actual final_linear_x/final_angular_z actions. 0 disables.')
    parser.add_argument('--imitate-action-scenario', action='append', dest='imitate_action_scenarios', default=None, help='Scenario name to imitate from trace. Repeatable; unset allows all scenarios.')
    parser.add_argument('--imitate-action-min-step', type=int, default=0, help='Only imitate samples at or after this trace step.')
    parser.add_argument('--imitate-action-max-step', type=int, default=-1, help='If nonnegative, only imitate samples at or before this trace step.')
    parser.add_argument('--imitate-action-min-distance', type=float, default=0.8, help='Only imitate samples farther than this distance from goal.')
    parser.add_argument('--imitate-action-max-distance', type=float, default=13.0, help='Only imitate samples within this distance from goal.')
    parser.add_argument('--imitate-action-max-abs-cte', type=float, default=3.1, help='Only imitate samples whose absolute CTE is at most this value.')
    parser.add_argument('--imitate-action-min-team-separation', type=float, default=0.0, help='Only imitate samples whose team min separation is at least this value. 0 disables.')
    parser.add_argument('--preserve-action-weight', type=float, default=0.0, help='Sample weight for preserve-trace-json source-action imitation samples. 0 disables.')
    parser.add_argument('--preserve-min-route-progress', type=float, default=0.0, help='Only preserve samples whose route progress is at least this value.')
    parser.add_argument('--preserve-max-route-progress', type=float, default=1.1, help='Only preserve samples whose route progress is at most this value.')
    parser.add_argument('--preserve-min-team-separation', type=float, default=0.0, help='Only preserve samples whose team min separation is at least this value. 0 disables.')
    parser.add_argument('--preserve-min-episode-progress', type=float, default=-1.0, help='Only preserve samples from episodes whose team progress is at least this value.')
    parser.add_argument('--preserve-exclude-collisions', action='store_true', help='Skip preserve samples from collision episodes.')
    parser.add_argument('--scripted-overtake-weight', type=float, default=0.0, help='Sample weight for deterministic overtaking teacher targets. 0 disables.')
    parser.add_argument('--scripted-overtake-scenario', action='append', dest='scripted_overtake_scenarios', default=None, help='Scenario name for scripted overtaking targets. Defaults to two_usv_overtaking.')
    parser.add_argument('--scripted-overtake-min-distance', type=float, default=0.8, help='Only add scripted overtaking samples farther than this distance from goal.')
    parser.add_argument('--scripted-overtake-max-distance', type=float, default=13.0, help='Only add scripted overtaking samples within this distance from goal.')
    parser.add_argument('--scripted-overtake-front-min-speed', type=float, default=0.24, help='Front/stand-on vessel target minimum speed.')
    parser.add_argument('--scripted-overtake-front-max-omega', type=float, default=0.08, help='Front/stand-on vessel yaw target cap around raw route yaw.')
    parser.add_argument('--scripted-overtake-rear-approach-speed', type=float, default=0.12, help='Rear vessel target speed before corridor is opened.')
    parser.add_argument('--scripted-overtake-rear-close-speed', type=float, default=0.04, help='Rear vessel speed cap when close and corridor is not open.')
    parser.add_argument('--scripted-overtake-rear-pass-speed', type=float, default=0.22, help='Rear vessel target speed once the passing corridor is open.')
    parser.add_argument('--scripted-overtake-close-separation', type=float, default=2.4, help='Close-distance threshold for rear-vessel speed cap.')
    parser.add_argument('--scripted-overtake-release-separation', type=float, default=6.0, help='Do not add scripted overtaking samples beyond this nearest-neighbor distance.')
    parser.add_argument('--scripted-overtake-target-starboard-offset', type=float, default=1.25, help='Desired rear-vessel starboard offset relative to the front vessel.')
    parser.add_argument('--scripted-overtake-rear-omega', type=float, default=0.30, help='Starboard yaw target magnitude for rear vessel; negative omega is starboard.')
    parser.add_argument('--scripted-overtake-omega-weight-scale', type=float, default=1.0, help='Additional sample-weight scale for scripted overtaking yaw targets.')
    parser.add_argument('--scripted-overtake-single-return-progress', type=float, default=0.55, help='Route-progress gate where single_usv_overtaking scripted targets switch to return-to-route.')
    parser.add_argument('--scripted-overtake-single-return-rel-x', type=float, default=0.05, help='Nearest-lead rel_x threshold where single_usv_overtaking scripted targets switch to return-to-route.')
    parser.add_argument('--scripted-overtake-single-return-min-separation', type=float, default=2.0, help='Minimum nearest-neighbor separation for single_usv_overtaking return targets.')
    parser.add_argument('--scripted-overtake-single-return-cte-start', type=float, default=0.35, help='Absolute CTE where single_usv_overtaking return targets activate.')
    parser.add_argument('--scripted-overtake-single-return-cte-full', type=float, default=2.4, help='Absolute CTE where single_usv_overtaking return yaw reaches full magnitude.')
    parser.add_argument('--scripted-overtake-single-return-speed', type=float, default=0.34, help='Forward speed target while returning to route in single_usv_overtaking.')
    parser.add_argument('--scripted-overtake-single-return-omega', type=float, default=0.50, help='Maximum yaw magnitude while returning to route in single_usv_overtaking.')
    parser.add_argument('--scripted-overtake-single-finish-distance', type=float, default=5.0, help='Distance-to-goal gate where single_usv_overtaking scripted targets prefer straight finish.')
    parser.add_argument('--scripted-overtake-single-finish-max-omega', type=float, default=0.08, help='Yaw cap for single_usv_overtaking final finish when CTE is controlled.')
    parser.add_argument('--device', default='cpu', help='Torch device.')
    parser.add_argument('--train-all-actor', action='store_true', help='Train all actor parameters instead of only scenario-specific branches.')
    return parser.parse_args()


def _load_actor(torch, nn, checkpoint: dict, device):
    if checkpoint.get('neighbor_attention', False):
        from usv_rl.neighbor_attention import build_attention_actor_from_checkpoint
        actor = build_attention_actor_from_checkpoint(checkpoint, nn, torch, device)
    else:
        hidden_sizes = tuple(int(value) for value in checkpoint.get('hidden_sizes', [128, 128]))
        action_dim = int(checkpoint['action_dim'])
        obs_dim = int(checkpoint['local_observation_size'])
        layers = []
        current_dim = obs_dim
        for hidden_size in hidden_sizes:
            layers.append(nn.Linear(current_dim, hidden_size))
            layers.append(nn.Tanh())
            current_dim = hidden_size
        layers.append(nn.Linear(current_dim, action_dim))
        actor = nn.Sequential(*layers).to(device)
        actor.load_state_dict(checkpoint['actor_state_dict'])
    return actor.train()


def _set_trainable(actor, train_all: bool) -> list:
    if train_all:
        for parameter in actor.parameters():
            parameter.requires_grad_(True)
        return [parameter for parameter in actor.parameters() if parameter.requires_grad]

    trainable_prefixes = (
        'encounter_residual.',
        'scenario_residual_heads.',
        'scenario_action_heads.',
        'scenario_actor_trunks.',
    )
    trainable = []
    for name, parameter in actor.named_parameters():
        should_train = name.startswith(trainable_prefixes)
        parameter.requires_grad_(should_train)
        if should_train:
            trainable.append(parameter)
    if trainable:
        return trainable
    for parameter in actor.parameters():
        parameter.requires_grad_(True)
    return [parameter for parameter in actor.parameters() if parameter.requires_grad]


def _target_order(text: str) -> list[str]:
    order = []
    for item in str(text).split(','):
        key = item.strip().lower()
        if key in TARGET_SPECS and key not in order:
            order.append(key)
    return order or ['deconf', 'offroute', 'cte', 'finish']


def _kind_names(text: str) -> list[str]:
    names = []
    for item in str(text).split(','):
        key = item.strip().lower()
        if key in TARGET_KIND_TO_ID and key not in names:
            names.append(key)
    return names


def _scenario_index(checkpoint: dict, scenario_name: str) -> int:
    scenarios = tuple(str(name) for name in checkpoint.get('scenarios', ()))
    try:
        return scenarios.index(str(scenario_name))
    except ValueError:
        return 0


def _nearest_neighbor_from_raw(raw_observation: np.ndarray) -> dict | None:
    ego_dim = LOCAL_EGO_FEATURE_COUNT
    encounter_dim = ENCOUNTER_TYPE_COUNT
    neighbor_dim = NEIGHBOR_FEATURE_COUNT
    available = int(raw_observation.shape[0]) - ego_dim - encounter_dim
    slots = max(0, available // neighbor_dim)
    if slots <= 0:
        return None
    best = None
    for slot in range(slots):
        start = ego_dim + slot * neighbor_dim
        block = raw_observation[start:start + neighbor_dim]
        if block.shape[0] < neighbor_dim:
            continue
        distance = float(block[4])
        if distance <= 1e-6:
            continue
        candidate = {
            'rel_x': float(block[0]),
            'rel_y': float(block[1]),
            'rel_vx': float(block[2]),
            'rel_vy': float(block[3]),
            'distance': distance,
            'bearing': float(block[5]),
        }
        if best is None or distance < best['distance']:
            best = candidate
    return best


def _checkpoint_float(checkpoint: dict, key: str, default: float) -> float:
    try:
        return float(checkpoint.get(key, default))
    except (TypeError, ValueError):
        return float(default)


def _signed_cte_from_raw(checkpoint: dict, raw_observation: np.ndarray) -> float:
    clipped_cte = float(raw_observation[11]) if raw_observation.shape[0] > 11 else 0.0
    source = str(checkpoint.get('random_cte_source', 'clipped')).strip().lower()
    raw_cte_index = LOCAL_EGO_FEATURE_COUNT - 2
    if source == 'raw' and raw_observation.shape[0] > raw_cte_index:
        return float(raw_observation[raw_cte_index])
    return clipped_cte


def _cte_recovery_target_from_raw(checkpoint: dict, raw_observation: np.ndarray) -> dict:
    signed_cte = _signed_cte_from_raw(checkpoint, raw_observation)
    abs_cte = abs(signed_cte)
    min_abs_cte = max(0.0, _checkpoint_float(checkpoint, 'random_cte_recovery_min_abs_cte', 1.10))
    full_abs_cte = max(min_abs_cte + 0.05, _checkpoint_float(checkpoint, 'random_cte_recovery_full_abs_cte', 3.00))
    cte_urgency = float(np.clip((abs_cte - min_abs_cte) / max(full_abs_cte - min_abs_cte, 1e-3), 0.0, 1.0))
    target_speed = max(0.0, _checkpoint_float(checkpoint, 'random_cte_recovery_target_speed', 0.18))
    min_speed = max(0.0, _checkpoint_float(checkpoint, 'random_cte_recovery_min_speed', 0.07))
    max_omega = max(0.0, _checkpoint_float(checkpoint, 'random_cte_recovery_max_omega', 0.30))
    omega_reference = max(0.05, _checkpoint_float(checkpoint, 'random_cte_recovery_omega_reference', 0.55))
    speed_cte_slowdown = max(0.0, _checkpoint_float(checkpoint, 'random_cte_recovery_speed_cte_slowdown', 1.0))
    target_linear = target_speed - (target_speed - min_speed) * float(np.clip(cte_urgency * speed_cte_slowdown, 0.0, 1.0))

    heading_error = float(np.arctan2(float(raw_observation[5]), float(raw_observation[6]))) if raw_observation.shape[0] > 6 else 0.0
    recovery_heading_error = heading_error
    heading_sign = 1.0 if _checkpoint_float(checkpoint, 'goal_heading_omega_sign', -1.0) >= 0.0 else -1.0
    target_omega = heading_sign * float(np.clip(heading_error / omega_reference, -1.0, 1.0)) * max_omega
    omega_mode = str(checkpoint.get('random_cte_recovery_omega_mode', 'goal-heading')).strip().lower().replace('_', '-')
    if omega_mode == 'signed-cte':
        cte_sign = 0.0 if abs(signed_cte) <= 1e-6 else float(np.sign(signed_cte))
        target_omega = -cte_sign * max_omega * (0.45 + 0.55 * cte_urgency)
    elif omega_mode == 'signed-cte-inverted':
        cte_sign = 0.0 if abs(signed_cte) <= 1e-6 else float(np.sign(signed_cte))
        target_omega = cte_sign * max_omega * (0.45 + 0.55 * cte_urgency)
    elif omega_mode == 'goal-cte-lookahead':
        cte_lookahead = max(0.25, _checkpoint_float(checkpoint, 'random_cte_recovery_cte_lookahead', 4.0))
        cte_heading_scale = max(0.0, _checkpoint_float(checkpoint, 'random_cte_recovery_cte_heading_scale', 1.0))
        cte_heading_bias = float(np.arctan2(signed_cte, cte_lookahead)) * cte_heading_scale
        recovery_heading_error = float(np.arctan2(np.sin(heading_error + cte_heading_bias), np.cos(heading_error + cte_heading_bias)))
        target_omega = heading_sign * float(np.clip(recovery_heading_error / omega_reference, -1.0, 1.0)) * max_omega

    speed_heading_gate = min(1.0, max(0.0, _checkpoint_float(checkpoint, 'random_cte_recovery_speed_heading_gate', 0.0)))
    if speed_heading_gate > 0.0:
        recovery_alignment = max(0.0, float(np.cos(min(abs(recovery_heading_error), np.pi / 2.0))))
        target_linear *= (1.0 - speed_heading_gate) + speed_heading_gate * recovery_alignment
        target_linear = max(target_linear, min_speed)

    low = np.asarray(checkpoint.get('action_low', [0.0, -0.5]), dtype=np.float32)
    high = np.asarray(checkpoint.get('action_high', [float(checkpoint.get('cruise_speed', 0.34)), float(checkpoint.get('max_angular_velocity', 0.5))]), dtype=np.float32)
    target = np.maximum(low[:2], np.minimum(high[:2], np.asarray([target_linear, target_omega], dtype=np.float32)))
    return {
        'target_linear': float(target[0]),
        'target_omega': float(target[1]),
        'cte_urgency': float(cte_urgency),
        'signed_cte': float(signed_cte),
        'omega_mode': omega_mode,
    }


def _merge_corrected_cte_target(checkpoint: dict, diagnostics: dict, raw_observation: np.ndarray) -> dict:
    corrected = _cte_recovery_target_from_raw(checkpoint, raw_observation)
    diagnostics = dict(diagnostics)
    cte_target = dict(diagnostics.get('cte_recovery_target', {}))
    cte_target.update(corrected)
    diagnostics['cte_recovery_target'] = cte_target
    return diagnostics


def _scripted_overtake_target(args, scenario_name: str, agent_id: str, raw_observation: np.ndarray) -> tuple[np.ndarray, float] | None:
    neighbor = _nearest_neighbor_from_raw(raw_observation)
    if neighbor is None:
        return None
    distance = float(neighbor['distance'])
    if distance > float(args.scripted_overtake_release_separation):
        return None

    raw_linear = float(raw_observation[7]) if raw_observation.shape[0] > 7 else 0.0
    raw_omega = float(raw_observation[8]) if raw_observation.shape[0] > 8 else 0.0
    distance_to_goal = float(raw_observation[4]) if raw_observation.shape[0] > 4 else 0.0
    heading_error = float(np.arctan2(float(raw_observation[5]), float(raw_observation[6]))) if raw_observation.shape[0] > 6 else 0.0
    cross_track_error = float(raw_observation[11]) if raw_observation.shape[0] > 11 else 0.0
    route_progress = float(raw_observation[12]) if raw_observation.shape[0] > 12 else 0.0
    rel_x = float(neighbor['rel_x'])
    rel_y = float(neighbor['rel_y'])

    suffix = str(agent_id).rsplit('_', 1)[-1]
    try:
        agent_number = int(suffix)
    except (TypeError, ValueError):
        agent_number = 0
    single_overtake = str(scenario_name) == 'single_usv_overtaking'
    front_role = (agent_number == 1 or rel_x < -0.3) and not single_overtake
    rear_role = (agent_number == 2 or rel_x > 0.3) or (single_overtake and agent_number == 1)

    if front_role and not rear_role:
        target_linear = max(float(args.scripted_overtake_front_min_speed), min(0.34, raw_linear if raw_linear > 0.0 else 0.30))
        target_omega = float(np.clip(raw_omega, -float(args.scripted_overtake_front_max_omega), float(args.scripted_overtake_front_max_omega)))
        return np.asarray([target_linear, target_omega], dtype=np.float32), float(args.scripted_overtake_weight)

    if not rear_role:
        return None

    if single_overtake:
        min_return_sep = max(0.1, float(args.scripted_overtake_single_return_min_separation))
        abs_cte = abs(cross_track_error)
        return_gate = (
            distance >= min_return_sep
            and (
                route_progress >= float(args.scripted_overtake_single_return_progress)
                or rel_x <= float(args.scripted_overtake_single_return_rel_x)
                or distance_to_goal <= float(args.scripted_overtake_single_finish_distance)
                or abs_cte >= float(args.scripted_overtake_single_return_cte_start)
            )
        )
        if return_gate:
            cte_start = max(0.0, float(args.scripted_overtake_single_return_cte_start))
            cte_full = max(cte_start + 1e-3, float(args.scripted_overtake_single_return_cte_full))
            max_omega = max(0.0, float(args.scripted_overtake_single_return_omega))
            cte_strength = float(np.clip((abs_cte - cte_start) / (cte_full - cte_start), 0.0, 1.0))
            cte_omega = -np.sign(cross_track_error) * max_omega * cte_strength
            target_omega = cte_omega
            if distance_to_goal <= float(args.scripted_overtake_single_finish_distance) and abs_cte <= cte_start:
                target_omega = float(np.clip(raw_omega, -float(args.scripted_overtake_single_finish_max_omega), float(args.scripted_overtake_single_finish_max_omega)))
            return np.asarray([
                float(args.scripted_overtake_single_return_speed),
                target_omega,
            ], dtype=np.float32), float(args.scripted_overtake_weight) * max(0.1, float(args.scripted_overtake_omega_weight_scale))

    # For the rear vessel, starboard passing means moving to the right of its
    # own route.  In the current overtaking geometry (eastbound route), the
    # front vessel should appear at positive relative Y once the rear vessel has
    # opened enough starboard offset.  This uses rel_y, not -rel_y.
    target_offset = max(0.1, float(args.scripted_overtake_target_starboard_offset))
    corridor = float(np.clip(max(0.0, rel_y) / target_offset, 0.0, 1.0))
    target_linear = float(args.scripted_overtake_rear_approach_speed) + (
        float(args.scripted_overtake_rear_pass_speed) - float(args.scripted_overtake_rear_approach_speed)
    ) * corridor
    if distance < float(args.scripted_overtake_close_separation) and corridor < 0.85:
        target_linear = min(target_linear, float(args.scripted_overtake_rear_close_speed))
    target_omega = -float(args.scripted_overtake_rear_omega) * (1.0 - corridor)
    weight = float(args.scripted_overtake_weight) * max(0.1, float(args.scripted_overtake_omega_weight_scale))
    return np.asarray([target_linear, target_omega], dtype=np.float32), weight


def _kind_mask(kind_ids, kinds_text: str):
    kinds = set(_kind_names(kinds_text))
    if not kinds:
        return torch.zeros_like(kind_ids, dtype=torch.bool)
    mask = torch.zeros_like(kind_ids, dtype=torch.bool)
    for kind in kinds:
        mask = mask | (kind_ids == int(TARGET_KIND_TO_ID[kind]))
    return mask


def _collect_samples(args, checkpoint: dict):
    order = _target_order(args.target_priority)
    target_weights = {
        'deconf': float(args.deconf_weight),
        'offroute': float(args.offroute_weight),
        'cte': float(args.cte_weight),
        'finish': float(args.finish_weight),
        'clear': float(args.clear_speedup_weight),
        'guard': float(args.risk_guard_weight),
        'stall_start': float(args.stall_start_weight),
        'goal_return': float(args.goal_return_weight),
        'late_clear': float(args.late_clear_weight),
        'recovery_turn': float(args.recovery_turn_weight),
        'recovery': float(args.recovery_speedup_weight),
        'edge_coast': float(args.edge_coast_weight),
        'goal_hold': float(args.goal_hold_weight),
        'imitate': float(args.imitate_action_weight),
        'scripted_overtake': float(args.scripted_overtake_weight),
    }
    observations = []
    targets = []
    scenario_ids = []
    weights = []
    kind_ids = []
    anchor_observations = []
    anchor_scenario_ids = []
    counts = {key: 0 for key in TARGET_KINDS}

    for trace_path in args.trace_json:
        payload = json.loads(Path(trace_path).read_text(encoding='utf-8'))
        for episode in payload.get('episode_metrics', []):
            scenario_name = str(episode.get('scenario', ''))
            scenario_id = _scenario_index(checkpoint, scenario_name)
            for sample in episode.get('trace_samples', []):
                step = int(sample.get('step', 0))
                for agent_id, agent in sample.get('agents', {}).items():
                    raw_observation = agent.get('raw_observation')
                    if raw_observation is None:
                        continue
                    raw_observation_np = np.asarray(raw_observation, dtype=np.float32)
                    anchor_observations.append(np.asarray(raw_observation, dtype=np.float32))
                    anchor_scenario_ids.append(int(scenario_id))
                    diagnostics = agent.get('mask_diagnostics', {})
                    diagnostics = _merge_corrected_cte_target(checkpoint, diagnostics, raw_observation_np)
                    source_linear = float(agent.get('final_linear_x', 0.0))
                    source_omega = float(agent.get('final_angular_z', 0.0))
                    team_min_separation = float(diagnostics.get('team_min_separation', 0.0))
                    threat_score = float(diagnostics.get('threat_score', 1.0))
                    distance_to_goal = float(agent.get('distance_to_goal', 0.0))
                    route_progress = float(agent.get('route_progress', 0.0))
                    cross_track_error = abs(float(agent.get('cross_track_error', 0.0)))
                    heading_error_signed = float(agent.get('heading_error', 0.0))
                    heading_error = abs(heading_error_signed)
                    scripted_weight = float(args.scripted_overtake_weight)
                    scripted_scenarios = set(str(value) for value in (args.scripted_overtake_scenarios or ['two_usv_overtaking']))
                    if (
                        scripted_weight > 0.0
                        and scenario_name in scripted_scenarios
                        and distance_to_goal >= float(args.scripted_overtake_min_distance)
                        and distance_to_goal <= float(args.scripted_overtake_max_distance)
                    ):
                        scripted = _scripted_overtake_target(args, scenario_name, str(agent_id), raw_observation_np)
                        if scripted is not None:
                            scripted_target, scripted_sample_weight = scripted
                            observations.append(raw_observation_np)
                            targets.append(scripted_target)
                            scenario_ids.append(int(scenario_id))
                            weights.append(scripted_sample_weight)
                            kind_ids.append(int(TARGET_KIND_TO_ID['scripted_overtake']))
                            counts['scripted_overtake'] += 1
                            continue
                    imitate_weight = float(args.imitate_action_weight)
                    imitate_scenarios = set(str(value) for value in (args.imitate_action_scenarios or []))
                    imitate_max_step = int(args.imitate_action_max_step)
                    if (
                        imitate_weight > 0.0
                        and (not imitate_scenarios or scenario_name in imitate_scenarios)
                        and step >= int(args.imitate_action_min_step)
                        and (imitate_max_step < 0 or step <= imitate_max_step)
                        and distance_to_goal >= float(args.imitate_action_min_distance)
                        and distance_to_goal <= float(args.imitate_action_max_distance)
                        and cross_track_error <= float(args.imitate_action_max_abs_cte)
                        and team_min_separation >= float(args.imitate_action_min_team_separation)
                    ):
                        observations.append(raw_observation_np)
                        targets.append(np.asarray([
                            source_linear,
                            source_omega,
                        ], dtype=np.float32))
                        scenario_ids.append(int(scenario_id))
                        weights.append(imitate_weight)
                        kind_ids.append(int(TARGET_KIND_TO_ID['imitate']))
                        counts['imitate'] += 1
                        continue
                    goal_hold_weight = float(args.goal_hold_weight)
                    if (
                        goal_hold_weight > 0.0
                        and bool(diagnostics.get('finish_phase', False))
                        and source_linear > float(args.goal_hold_source_threshold)
                        and distance_to_goal <= float(args.goal_hold_max_distance)
                        and cross_track_error <= float(args.goal_hold_max_cte)
                        and (
                            team_min_separation <= float(args.goal_hold_max_separation)
                            or threat_score >= float(args.goal_hold_min_threat)
                            or not bool(diagnostics.get('finish_neighbor_clear', True))
                            or not bool(diagnostics.get('finish_team_clear', True))
                        )
                    ):
                        safe_target = diagnostics.get('safe_finish_target', {})
                        safe_omega = float(safe_target.get('target_omega', source_omega))
                        omega_blend = float(np.clip(args.goal_hold_omega_blend, 0.0, 1.0))
                        hold_omega = (1.0 - omega_blend) * source_omega + omega_blend * safe_omega
                        observations.append(np.asarray(raw_observation, dtype=np.float32))
                        targets.append(np.asarray([
                            float(args.goal_hold_target_linear),
                            hold_omega,
                        ], dtype=np.float32))
                        scenario_ids.append(int(scenario_id))
                        weights.append(goal_hold_weight)
                        kind_ids.append(int(TARGET_KIND_TO_ID['goal_hold']))
                        counts['goal_hold'] += 1
                        continue
                    guard_weight = float(args.risk_guard_weight)
                    guard_max_step = int(args.risk_guard_max_step)
                    guard_agents = set(str(value) for value in (args.risk_guard_agents or []))
                    guard_risk_active = (
                        team_min_separation <= float(args.risk_guard_max_separation)
                        or threat_score >= float(args.risk_guard_min_threat)
                        or bool(diagnostics.get('random_deconflict_weighted_active', False))
                    )
                    if bool(args.risk_guard_require_threat):
                        guard_risk_active = threat_score >= float(args.risk_guard_min_threat)
                    if (
                        guard_weight > 0.0
                        and distance_to_goal > float(args.risk_guard_min_distance)
                        and (not guard_agents or str(agent_id) in guard_agents)
                        and route_progress >= float(args.risk_guard_min_route_progress)
                        and route_progress <= float(args.risk_guard_max_route_progress)
                        and cross_track_error >= float(args.risk_guard_min_abs_cte)
                        and (guard_max_step < 0 or step <= guard_max_step)
                        and (not bool(args.risk_guard_yield_only) or bool(diagnostics.get('deconf_is_yield', False)))
                        and (not bool(args.risk_guard_standon_only) or not bool(diagnostics.get('deconf_is_yield', False)))
                        and guard_risk_active
                    ):
                        is_yield = bool(diagnostics.get('deconf_is_yield', False))
                        deconf_target = diagnostics.get('deconf_target', {})
                        guard_omega = source_omega
                        if diagnostics.get('random_deconflict_weighted_active', False):
                            deconf_omega = float(deconf_target.get('target_omega', guard_omega))
                            omega_blend = float(np.clip(args.risk_guard_omega_blend, 0.0, 1.0))
                            guard_omega = (1.0 - omega_blend) * source_omega + omega_blend * deconf_omega
                        min_omega_abs = float(args.risk_guard_yield_min_omega_abs if is_yield else args.risk_guard_standon_min_omega_abs)
                        if min_omega_abs > 0.0 and abs(guard_omega) < min_omega_abs:
                            reference_omega = float(deconf_target.get('target_omega', guard_omega))
                            if abs(reference_omega) <= 1e-6:
                                reference_omega = source_omega
                            omega_sign = -1.0 if reference_omega < 0.0 else 1.0
                            guard_omega = omega_sign * min_omega_abs
                        min_starboard_omega = max(0.0, float(args.risk_guard_min_starboard_omega))
                        min_starboard_threat = max(0.0, float(args.risk_guard_min_starboard_threat))
                        if (
                            min_starboard_omega > 0.0
                            and threat_score >= min_starboard_threat
                            and is_yield
                        ):
                            guard_omega = min(guard_omega, -min_starboard_omega)
                        max_linear = float(args.risk_guard_max_linear)
                        role_max_linear = float(args.risk_guard_yield_max_linear if is_yield else args.risk_guard_standon_max_linear)
                        if role_max_linear >= 0.0:
                            max_linear = role_max_linear
                        observations.append(np.asarray(raw_observation, dtype=np.float32))
                        targets.append(np.asarray([
                            min(source_linear, max_linear),
                            guard_omega,
                        ], dtype=np.float32))
                        scenario_ids.append(int(scenario_id))
                        weights.append(guard_weight)
                        kind_ids.append(int(TARGET_KIND_TO_ID['guard']))
                        counts['guard'] += 1
                        continue
                    recovery_turn_weight = float(args.recovery_turn_weight)
                    recovery_turn_agents = set(str(value) for value in (args.recovery_turn_agents or []))
                    recovery_turn_heading_sign = str(args.recovery_turn_heading_sign).lower()
                    recovery_turn_heading_sign_ok = (
                        recovery_turn_heading_sign == 'any'
                        or (recovery_turn_heading_sign == 'negative' and heading_error_signed < 0.0)
                        or (recovery_turn_heading_sign == 'positive' and heading_error_signed > 0.0)
                    )
                    recovery_turn_max_step = int(args.recovery_turn_max_step)
                    if (
                        recovery_turn_weight > 0.0
                        and (not recovery_turn_agents or str(agent_id) in recovery_turn_agents)
                        and distance_to_goal > float(args.recovery_turn_min_distance)
                        and step >= int(args.recovery_turn_min_step)
                        and (recovery_turn_max_step < 0 or step <= recovery_turn_max_step)
                        and route_progress >= float(args.recovery_turn_min_route_progress)
                        and route_progress <= float(args.recovery_turn_max_route_progress)
                        and cross_track_error >= float(args.recovery_turn_min_abs_cte)
                        and heading_error >= float(args.recovery_turn_min_heading_error)
                        and heading_error <= float(args.recovery_turn_max_heading_error)
                        and recovery_turn_heading_sign_ok
                        and team_min_separation >= float(args.recovery_turn_min_separation)
                        and threat_score <= float(args.recovery_turn_max_threat)
                    ):
                        heading_gain = max(0.0, float(args.recovery_turn_heading_gain))
                        max_omega = max(0.0, float(args.recovery_turn_max_omega))
                        if heading_gain > 0.0 and max_omega > 0.0:
                            omega_sign = 1.0 if float(args.recovery_turn_heading_omega_sign) >= 0.0 else -1.0
                            target_omega = float(np.clip(omega_sign * heading_error_signed * heading_gain, -max_omega, max_omega))
                        else:
                            cte_target = diagnostics.get('cte_recovery_target', {})
                            offroute_target = diagnostics.get('offroute_target', {})
                            target_omega = float(cte_target.get('target_omega', offroute_target.get('target_omega', source_omega)))
                        target_omega_cap = float(args.recovery_turn_target_omega_cap)
                        if target_omega_cap >= 0.0:
                            target_omega = float(np.clip(target_omega, -target_omega_cap, target_omega_cap))
                        omega_blend = float(np.clip(args.recovery_turn_omega_blend, 0.0, 1.0))
                        turn_omega = (1.0 - omega_blend) * source_omega + omega_blend * target_omega
                        min_omega = max(0.0, float(args.recovery_turn_min_omega_abs))
                        if min_omega > 0.0 and abs(turn_omega) < min_omega:
                            sign_source = target_omega if abs(target_omega) > 1.0e-6 else turn_omega
                            turn_omega = float(np.sign(sign_source) or 1.0) * min_omega
                        observations.append(np.asarray(raw_observation, dtype=np.float32))
                        targets.append(np.asarray([
                            float(args.recovery_turn_target_linear),
                            turn_omega,
                        ], dtype=np.float32))
                        scenario_ids.append(int(scenario_id))
                        weights.append(recovery_turn_weight)
                        kind_ids.append(int(TARGET_KIND_TO_ID['recovery_turn']))
                        counts['recovery_turn'] += 1
                        continue
                    stall_start_weight = float(args.stall_start_weight)
                    stall_start_agents = set(str(value) for value in (args.stall_start_agents or []))
                    stall_start_max_step = int(args.stall_start_max_step)
                    if (
                        stall_start_weight > 0.0
                        and (not stall_start_agents or str(agent_id) in stall_start_agents)
                        and source_linear < float(args.stall_start_source_threshold)
                        and distance_to_goal > float(args.stall_start_min_distance)
                        and route_progress <= float(args.stall_start_max_route_progress)
                        and step >= int(args.stall_start_min_step)
                        and (stall_start_max_step < 0 or step <= stall_start_max_step)
                        and cross_track_error <= float(args.stall_start_max_cte)
                        and team_min_separation >= float(args.stall_start_min_separation)
                        and threat_score <= float(args.stall_start_max_threat)
                        and (bool(args.stall_start_allow_deconf) or not bool(diagnostics.get('random_deconflict_weighted_active', False)))
                        and bool(diagnostics.get('finish_team_clear', False))
                        and bool(diagnostics.get('finish_neighbor_clear', False))
                    ):
                        cte_target = diagnostics.get('cte_recovery_target', {})
                        cte_target_omega = float(cte_target.get('target_omega', source_omega))
                        omega_blend = float(np.clip(args.stall_start_cte_omega_blend, 0.0, 1.0))
                        stall_omega = (1.0 - omega_blend) * source_omega + omega_blend * cte_target_omega
                        stall_linear = float(args.stall_start_target_linear)
                        heading_slow_threshold = float(args.stall_start_heading_slow_threshold)
                        if heading_slow_threshold > 0.0 and heading_error > heading_slow_threshold:
                            stall_linear = min(stall_linear, float(args.stall_start_heading_recovery_linear))
                        observations.append(np.asarray(raw_observation, dtype=np.float32))
                        targets.append(np.asarray([
                            stall_linear,
                            stall_omega,
                        ], dtype=np.float32))
                        scenario_ids.append(int(scenario_id))
                        weights.append(stall_start_weight)
                        kind_ids.append(int(TARGET_KIND_TO_ID['stall_start']))
                        counts['stall_start'] += 1
                        continue
                    goal_return_weight = float(args.goal_return_weight)
                    goal_return_agents = set(str(value) for value in (args.goal_return_agents or []))
                    if (
                        goal_return_weight > 0.0
                        and (not goal_return_agents or str(agent_id) in goal_return_agents)
                        and source_linear > float(args.goal_return_source_threshold)
                        and distance_to_goal > float(args.goal_return_min_distance)
                        and route_progress >= float(args.goal_return_min_route_progress)
                        and step >= int(args.goal_return_min_step)
                        and cross_track_error <= float(args.goal_return_max_cte)
                        and team_min_separation >= float(args.goal_return_min_separation)
                        and threat_score <= float(args.goal_return_max_threat)
                        and (bool(args.goal_return_allow_deconf) or not bool(diagnostics.get('random_deconflict_weighted_active', False)))
                        and bool(diagnostics.get('finish_team_clear', False))
                        and bool(diagnostics.get('finish_neighbor_clear', False))
                    ):
                        safe_target = diagnostics.get('safe_finish_target', {})
                        safe_target_omega = float(safe_target.get('target_omega', source_omega))
                        omega_blend = float(np.clip(args.goal_return_safe_omega_blend, 0.0, 1.0))
                        return_omega = (1.0 - omega_blend) * source_omega + omega_blend * safe_target_omega
                        observations.append(np.asarray(raw_observation, dtype=np.float32))
                        targets.append(np.asarray([
                            float(args.goal_return_target_linear),
                            return_omega,
                        ], dtype=np.float32))
                        scenario_ids.append(int(scenario_id))
                        weights.append(goal_return_weight)
                        kind_ids.append(int(TARGET_KIND_TO_ID['goal_return']))
                        counts['goal_return'] += 1
                        continue
                    late_clear_weight = float(args.late_clear_weight)
                    late_clear_agents = set(str(value) for value in (args.late_clear_agents or []))
                    if (
                        late_clear_weight > 0.0
                        and (not late_clear_agents or str(agent_id) in late_clear_agents)
                        and source_linear < float(args.late_clear_source_threshold)
                        and distance_to_goal > float(args.late_clear_min_distance)
                        and step >= int(args.late_clear_min_step)
                        and cross_track_error <= float(args.late_clear_max_cte)
                        and team_min_separation >= float(args.late_clear_min_separation)
                        and threat_score <= float(args.late_clear_max_threat)
                        and (bool(args.late_clear_allow_deconf) or not bool(diagnostics.get('random_deconflict_weighted_active', False)))
                        and bool(diagnostics.get('finish_team_clear', False))
                        and bool(diagnostics.get('finish_neighbor_clear', False))
                    ):
                        late_clear_omega = source_omega
                        cte_target = diagnostics.get('cte_recovery_target', {})
                        cte_target_omega = float(cte_target.get('target_omega', source_omega))
                        omega_blend = float(np.clip(args.late_clear_cte_omega_blend, 0.0, 1.0))
                        late_clear_omega = (1.0 - omega_blend) * late_clear_omega + omega_blend * cte_target_omega
                        late_clear_linear = float(args.late_clear_target_linear)
                        cte_slow_threshold = float(args.late_clear_cte_slow_threshold)
                        if cte_slow_threshold > 0.0 and cross_track_error > cte_slow_threshold:
                            late_clear_linear = min(late_clear_linear, float(args.late_clear_cte_recovery_linear))
                        observations.append(np.asarray(raw_observation, dtype=np.float32))
                        targets.append(np.asarray([
                            late_clear_linear,
                            late_clear_omega,
                        ], dtype=np.float32))
                        scenario_ids.append(int(scenario_id))
                        weights.append(late_clear_weight)
                        kind_ids.append(int(TARGET_KIND_TO_ID['late_clear']))
                        counts['late_clear'] += 1
                        continue
                    recovery_weight = float(args.recovery_speedup_weight)
                    recovery_agents = set(str(value) for value in (args.recovery_speedup_agents or []))
                    if (
                        recovery_weight > 0.0
                        and (not recovery_agents or str(agent_id) in recovery_agents)
                        and source_linear < float(args.recovery_speedup_source_threshold)
                        and distance_to_goal > float(args.recovery_speedup_min_distance)
                        and step >= int(args.recovery_speedup_min_step)
                        and route_progress >= float(args.recovery_speedup_min_route_progress)
                        and route_progress <= float(args.recovery_speedup_max_route_progress)
                        and heading_error <= float(args.recovery_speedup_max_heading_error)
                        and cross_track_error <= float(args.recovery_speedup_max_cte)
                        and team_min_separation >= float(args.recovery_speedup_min_separation)
                        and threat_score <= float(args.recovery_speedup_max_threat)
                    ):
                        recovery_omega = source_omega
                        cte_target = diagnostics.get('cte_recovery_target', {})
                        cte_target_omega = float(cte_target.get('target_omega', source_omega))
                        omega_blend = float(np.clip(args.recovery_speedup_cte_omega_blend, 0.0, 1.0))
                        recovery_omega = (1.0 - omega_blend) * recovery_omega + omega_blend * cte_target_omega
                        observations.append(np.asarray(raw_observation, dtype=np.float32))
                        targets.append(np.asarray([
                            float(args.recovery_speedup_target_linear),
                            recovery_omega,
                        ], dtype=np.float32))
                        scenario_ids.append(int(scenario_id))
                        weights.append(recovery_weight)
                        kind_ids.append(int(TARGET_KIND_TO_ID['recovery']))
                        counts['recovery'] += 1
                        continue
                    edge_coast_weight = float(args.edge_coast_weight)
                    edge_coast_agents = set(str(value) for value in (args.edge_coast_agents or []))
                    if (
                        edge_coast_weight > 0.0
                        and (not edge_coast_agents or str(agent_id) in edge_coast_agents)
                        and source_linear < float(args.edge_coast_source_threshold)
                        and distance_to_goal > float(args.edge_coast_min_distance)
                        and step >= int(args.edge_coast_min_step)
                        and route_progress >= float(args.edge_coast_min_route_progress)
                        and route_progress <= float(args.edge_coast_max_route_progress)
                        and heading_error <= float(args.edge_coast_max_heading_error)
                        and cross_track_error >= float(args.edge_coast_min_abs_cte)
                        and cross_track_error <= float(args.edge_coast_max_abs_cte)
                        and team_min_separation >= float(args.edge_coast_min_separation)
                        and threat_score <= float(args.edge_coast_max_threat)
                    ):
                        zero_blend = float(np.clip(args.edge_coast_omega_zero_blend, 0.0, 1.0))
                        coast_omega = (1.0 - zero_blend) * source_omega
                        max_omega_abs = float(args.edge_coast_max_omega_abs)
                        if max_omega_abs >= 0.0:
                            coast_omega = float(np.clip(coast_omega, -max_omega_abs, max_omega_abs))
                        observations.append(np.asarray(raw_observation, dtype=np.float32))
                        targets.append(np.asarray([
                            float(args.edge_coast_target_linear),
                            coast_omega,
                        ], dtype=np.float32))
                        scenario_ids.append(int(scenario_id))
                        weights.append(edge_coast_weight)
                        kind_ids.append(int(TARGET_KIND_TO_ID['edge_coast']))
                        counts['edge_coast'] += 1
                        continue
                    clear_weight = float(args.clear_speedup_weight)
                    if (
                        clear_weight > 0.0
                        and source_linear < float(args.clear_speedup_source_threshold)
                        and distance_to_goal > float(args.clear_speedup_min_distance)
                        and step >= int(args.clear_speedup_min_step)
                        and cross_track_error <= float(args.clear_speedup_max_cte)
                        and team_min_separation >= float(args.clear_speedup_min_separation)
                        and threat_score <= float(args.clear_speedup_max_threat)
                        and bool(diagnostics.get('finish_team_clear', False))
                        and bool(diagnostics.get('finish_neighbor_clear', False))
                        and bool(diagnostics.get('finish_cte_clear', False))
                    ):
                        observations.append(np.asarray(raw_observation, dtype=np.float32))
                        targets.append(np.asarray([
                            float(args.clear_speedup_target_linear),
                            source_omega,
                        ], dtype=np.float32))
                        scenario_ids.append(int(scenario_id))
                        weights.append(clear_weight)
                        kind_ids.append(int(TARGET_KIND_TO_ID['clear']))
                        counts['clear'] += 1
                        continue
                    selected_kind = None
                    selected_target = None
                    for kind in order:
                        active_key, target_key = TARGET_SPECS[kind]
                        target = diagnostics.get(target_key, {})
                        if not diagnostics.get(active_key, False):
                            continue
                        if float(target.get('l2_error_norm', 0.0)) < float(args.min_error_norm):
                            continue
                        selected_kind = kind
                        selected_target = target
                        break
                    if selected_kind is None or selected_target is None:
                        continue
                    observations.append(np.asarray(raw_observation, dtype=np.float32))
                    targets.append(np.asarray([
                        float(selected_target.get('target_linear', 0.0)),
                        float(selected_target.get('target_omega', 0.0)),
                    ], dtype=np.float32))
                    scenario_ids.append(int(scenario_id))
                    weights.append(float(target_weights.get(selected_kind, 1.0)))
                    kind_ids.append(int(TARGET_KIND_TO_ID[selected_kind]))
                    counts[selected_kind] += 1

    preserve_weight = float(args.preserve_action_weight)
    if preserve_weight > 0.0:
        for trace_path in args.preserve_trace_json or []:
            payload = json.loads(Path(trace_path).read_text(encoding='utf-8'))
            for episode in payload.get('episode_metrics', []):
                if bool(args.preserve_exclude_collisions) and bool(episode.get('collision', False)):
                    continue
                if float(episode.get('team_goal_progress_ratio', 0.0)) < float(args.preserve_min_episode_progress):
                    continue
                scenario_id = _scenario_index(checkpoint, episode.get('scenario', ''))
                for sample in episode.get('trace_samples', []):
                    for agent in sample.get('agents', {}).values():
                        raw_observation = agent.get('raw_observation')
                        if raw_observation is None:
                            continue
                        diagnostics = agent.get('mask_diagnostics', {})
                        route_progress = float(agent.get('route_progress', 0.0))
                        team_min_separation = float(diagnostics.get('team_min_separation', 0.0))
                        if route_progress < float(args.preserve_min_route_progress):
                            continue
                        if route_progress > float(args.preserve_max_route_progress):
                            continue
                        if team_min_separation < float(args.preserve_min_team_separation):
                            continue
                        raw_observation_np = np.asarray(raw_observation, dtype=np.float32)
                        anchor_observations.append(raw_observation_np)
                        anchor_scenario_ids.append(int(scenario_id))
                        observations.append(raw_observation_np)
                        targets.append(np.asarray([
                            float(agent.get('final_linear_x', 0.0)),
                            float(agent.get('final_angular_z', 0.0)),
                        ], dtype=np.float32))
                        scenario_ids.append(int(scenario_id))
                        weights.append(preserve_weight)
                        kind_ids.append(int(TARGET_KIND_TO_ID['imitate']))
                        counts['imitate'] += 1

    for trace_path in args.anchor_trace_json or []:
        payload = json.loads(Path(trace_path).read_text(encoding='utf-8'))
        for episode in payload.get('episode_metrics', []):
            scenario_id = _scenario_index(checkpoint, episode.get('scenario', ''))
            for sample in episode.get('trace_samples', []):
                for agent in sample.get('agents', {}).values():
                    raw_observation = agent.get('raw_observation')
                    if raw_observation is None:
                        continue
                    anchor_observations.append(np.asarray(raw_observation, dtype=np.float32))
                    anchor_scenario_ids.append(int(scenario_id))

    if not observations:
        raise RuntimeError('No active trace target samples found. Re-run evaluation with --trace-raw-observation or lower --min-error-norm.')
    return (
        np.stack(observations, axis=0),
        np.stack(targets, axis=0),
        np.asarray(scenario_ids, dtype=np.int64),
        np.asarray(weights, dtype=np.float32),
        np.asarray(kind_ids, dtype=np.int64),
        np.stack(anchor_observations, axis=0),
        np.asarray(anchor_scenario_ids, dtype=np.int64),
        counts,
    )


def _actor_action(actor, obs, scenario_ids, checkpoint: dict, low, high):
    if hasattr(actor, 'scenario_actor_trunks'):
        raw_action = actor(obs, scenario_ids=scenario_ids)
    else:
        raw_action = actor(obs)
    if checkpoint.get('squash_actions', False):
        raw_action = raw_action.clamp(-3.0, 3.0)
        return torch.tanh(raw_action) * ((high - low) / 2.0) + ((high + low) / 2.0)
    return raw_action


def _shape_targets(args, targets, kind_ids, source_actions, low, high):
    shaped = targets.clone()
    target_shape_mask = _kind_mask(kind_ids, args.target_shape_kinds)
    target_linear_blend = min(1.0, max(0.0, float(args.target_linear_blend)))
    target_omega_blend = min(1.0, max(0.0, float(args.target_omega_blend)))
    if bool(target_shape_mask.any().detach().cpu()):
        if target_linear_blend < 1.0:
            shaped[target_shape_mask, 0] = source_actions[target_shape_mask, 0] + target_linear_blend * (
                shaped[target_shape_mask, 0] - source_actions[target_shape_mask, 0]
            )
        target_max_drop = float(args.target_max_linear_drop)
        if target_max_drop >= 0.0:
            shaped[target_shape_mask, 0] = torch.maximum(
                shaped[target_shape_mask, 0],
                source_actions[target_shape_mask, 0] - target_max_drop,
            )
        target_min_linear = max(0.0, float(args.target_min_linear))
        if target_min_linear > 0.0:
            shaped[target_shape_mask, 0] = torch.maximum(
                shaped[target_shape_mask, 0],
                torch.full_like(shaped[target_shape_mask, 0], target_min_linear),
            )
        if target_omega_blend < 1.0:
            shaped[target_shape_mask, 1] = source_actions[target_shape_mask, 1] + target_omega_blend * (
                shaped[target_shape_mask, 1] - source_actions[target_shape_mask, 1]
            )

    deconf_mask = kind_ids == int(TARGET_KIND_TO_ID['deconf'])
    if bool(deconf_mask.any().detach().cpu()):
        linear_blend = min(1.0, max(0.0, float(args.deconf_linear_blend)))
        omega_blend = min(1.0, max(0.0, float(args.deconf_omega_blend)))
        if linear_blend < 1.0:
            shaped[deconf_mask, 0] = source_actions[deconf_mask, 0] + linear_blend * (
                shaped[deconf_mask, 0] - source_actions[deconf_mask, 0]
            )
        max_drop = float(args.deconf_max_linear_drop)
        if max_drop >= 0.0:
            shaped[deconf_mask, 0] = torch.maximum(
                shaped[deconf_mask, 0],
                source_actions[deconf_mask, 0] - max_drop,
            )
        min_linear = max(0.0, float(args.deconf_min_linear))
        if min_linear > 0.0:
            shaped[deconf_mask, 0] = torch.maximum(
                shaped[deconf_mask, 0],
                torch.full_like(shaped[deconf_mask, 0], min_linear),
            )
        if omega_blend < 1.0:
            shaped[deconf_mask, 1] = source_actions[deconf_mask, 1] + omega_blend * (
                shaped[deconf_mask, 1] - source_actions[deconf_mask, 1]
            )
    return torch.maximum(torch.minimum(shaped, high), low)


def main():
    args = parse_args()
    import torch
    from torch import nn

    device = torch.device(args.device)
    checkpoint = torch.load(args.model, map_location=device, weights_only=False)
    actor = _load_actor(torch, nn, checkpoint, device)
    trainable = _set_trainable(actor, bool(args.train_all_actor))
    if not trainable:
        raise RuntimeError('No trainable actor parameters found.')

    raw_obs_np, targets_np, scenario_ids_np, weights_np, kind_ids_np, anchor_raw_obs_np, anchor_scenario_ids_np, counts = _collect_samples(args, checkpoint)
    obs_dim = int(checkpoint['local_observation_size'])
    if raw_obs_np.shape[1] != obs_dim:
        raise RuntimeError(f'Trace observation dim {raw_obs_np.shape[1]} does not match checkpoint obs dim {obs_dim}.')
    obs_np = raw_obs_np
    anchor_obs_np = anchor_raw_obs_np
    if checkpoint.get('normalize_observations') and 'obs_normalizer' in checkpoint:
        normalizer = ObservationNormalizer(obs_dim)
        normalizer.load_state_dict(checkpoint['obs_normalizer'])
        obs_np = normalizer.normalize(obs_np)
        anchor_obs_np = normalizer.normalize(anchor_obs_np)

    obs = torch.as_tensor(obs_np, dtype=torch.float32, device=device)
    targets = torch.as_tensor(targets_np, dtype=torch.float32, device=device)
    scenario_ids = torch.as_tensor(scenario_ids_np, dtype=torch.long, device=device)
    weights = torch.as_tensor(weights_np, dtype=torch.float32, device=device)
    kind_ids = torch.as_tensor(kind_ids_np, dtype=torch.long, device=device)
    anchor_obs = torch.as_tensor(anchor_obs_np, dtype=torch.float32, device=device)
    anchor_scenario_ids = torch.as_tensor(anchor_scenario_ids_np, dtype=torch.long, device=device)

    low = torch.as_tensor(checkpoint.get('action_low', [0.0, -0.5]), dtype=torch.float32, device=device)
    high = torch.as_tensor(checkpoint.get('action_high', [float(checkpoint.get('cruise_speed', 0.34)), float(checkpoint.get('max_angular_velocity', 0.5))]), dtype=torch.float32, device=device)
    range_scale = torch.clamp(high - low, min=1e-3)
    with torch.no_grad():
        source_target_actions = _actor_action(actor, obs, scenario_ids, checkpoint, low, high).detach()
        source_anchor_actions = _actor_action(actor, anchor_obs, anchor_scenario_ids, checkpoint, low, high).detach()
        targets = _shape_targets(args, targets, kind_ids, source_target_actions, low, high)
        low_speed_threshold = float(args.low_speed_source_threshold)
        low_speed_weight = max(1.0, float(args.low_speed_weight))
        if low_speed_threshold >= 0.0 and low_speed_weight > 1.0:
            low_speed_kind_mask = _kind_mask(kind_ids, args.low_speed_weight_kinds)
            weights = weights * torch.where(
                low_speed_kind_mask & (source_target_actions[:, 0] < low_speed_threshold),
                torch.full_like(weights, low_speed_weight),
                torch.ones_like(weights),
            )
    optimizer = torch.optim.Adam(trainable, lr=float(args.learning_rate))
    batch_size = min(max(1, int(args.batch_size)), int(obs.shape[0]))
    anchor_batch_size = min(max(1, int(args.anchor_batch_size)), int(anchor_obs.shape[0]))
    last_loss = 0.0
    for _ in range(max(1, int(args.epochs))):
        permutation = torch.randperm(obs.shape[0], device=device)
        for start in range(0, obs.shape[0], batch_size):
            batch_indices = permutation[start:start + batch_size]
            action = _actor_action(actor, obs[batch_indices], scenario_ids[batch_indices], checkpoint, low, high)
            per_sample = (((action - targets[batch_indices]) / range_scale) ** 2).mean(dim=-1)
            target_loss = (per_sample * weights[batch_indices]).sum() / torch.clamp(weights[batch_indices].sum(), min=1.0)
            anchor_indices = torch.randint(0, anchor_obs.shape[0], (anchor_batch_size,), device=device)
            anchor_action = _actor_action(actor, anchor_obs[anchor_indices], anchor_scenario_ids[anchor_indices], checkpoint, low, high)
            anchor_loss = (((anchor_action - source_anchor_actions[anchor_indices]) / range_scale) ** 2).mean()
            loss = target_loss + float(args.anchor_weight) * anchor_loss
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(trainable, float(args.max_grad_norm))
            optimizer.step()
            last_loss = float(loss.detach().cpu().item())

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    checkpoint = dict(checkpoint)
    checkpoint['actor_state_dict'] = actor.state_dict()
    checkpoint['trace_target_fit'] = {
        'source_model': str(args.model),
        'trace_json': [str(path) for path in args.trace_json],
        'anchor_trace_json': [str(path) for path in args.anchor_trace_json or []],
        'samples': int(obs.shape[0]),
        'anchor_samples': int(anchor_obs.shape[0]),
        'counts': counts,
        'epochs': int(args.epochs),
        'learning_rate': float(args.learning_rate),
        'anchor_weight': float(args.anchor_weight),
        'deconf_min_linear': float(args.deconf_min_linear),
        'deconf_max_linear_drop': float(args.deconf_max_linear_drop),
        'deconf_linear_blend': float(args.deconf_linear_blend),
        'deconf_omega_blend': float(args.deconf_omega_blend),
        'target_min_linear': float(args.target_min_linear),
        'target_max_linear_drop': float(args.target_max_linear_drop),
        'target_linear_blend': float(args.target_linear_blend),
        'target_omega_blend': float(args.target_omega_blend),
        'target_shape_kinds': str(args.target_shape_kinds),
        'low_speed_source_threshold': float(args.low_speed_source_threshold),
        'low_speed_weight': float(args.low_speed_weight),
        'low_speed_weight_kinds': str(args.low_speed_weight_kinds),
        'clear_speedup_weight': float(args.clear_speedup_weight),
        'clear_speedup_source_threshold': float(args.clear_speedup_source_threshold),
        'clear_speedup_target_linear': float(args.clear_speedup_target_linear),
        'clear_speedup_min_distance': float(args.clear_speedup_min_distance),
        'clear_speedup_max_cte': float(args.clear_speedup_max_cte),
        'clear_speedup_min_separation': float(args.clear_speedup_min_separation),
        'clear_speedup_max_threat': float(args.clear_speedup_max_threat),
        'clear_speedup_min_step': int(args.clear_speedup_min_step),
        'late_clear_weight': float(args.late_clear_weight),
        'late_clear_source_threshold': float(args.late_clear_source_threshold),
        'late_clear_target_linear': float(args.late_clear_target_linear),
        'late_clear_min_distance': float(args.late_clear_min_distance),
        'late_clear_max_cte': float(args.late_clear_max_cte),
        'late_clear_min_separation': float(args.late_clear_min_separation),
        'late_clear_max_threat': float(args.late_clear_max_threat),
        'late_clear_min_step': int(args.late_clear_min_step),
        'late_clear_cte_omega_blend': float(args.late_clear_cte_omega_blend),
        'late_clear_cte_slow_threshold': float(args.late_clear_cte_slow_threshold),
        'late_clear_cte_recovery_linear': float(args.late_clear_cte_recovery_linear),
        'late_clear_allow_deconf': bool(args.late_clear_allow_deconf),
        'late_clear_agents': [str(value) for value in (args.late_clear_agents or [])],
        'stall_start_weight': float(args.stall_start_weight),
        'stall_start_agents': [str(value) for value in (args.stall_start_agents or [])],
        'stall_start_source_threshold': float(args.stall_start_source_threshold),
        'stall_start_target_linear': float(args.stall_start_target_linear),
        'stall_start_min_distance': float(args.stall_start_min_distance),
        'stall_start_max_route_progress': float(args.stall_start_max_route_progress),
        'stall_start_max_cte': float(args.stall_start_max_cte),
        'stall_start_min_separation': float(args.stall_start_min_separation),
        'stall_start_max_threat': float(args.stall_start_max_threat),
        'stall_start_min_step': int(args.stall_start_min_step),
        'stall_start_max_step': int(args.stall_start_max_step),
        'stall_start_cte_omega_blend': float(args.stall_start_cte_omega_blend),
        'stall_start_heading_slow_threshold': float(args.stall_start_heading_slow_threshold),
        'stall_start_heading_recovery_linear': float(args.stall_start_heading_recovery_linear),
        'stall_start_allow_deconf': bool(args.stall_start_allow_deconf),
        'goal_return_weight': float(args.goal_return_weight),
        'goal_return_agents': [str(value) for value in (args.goal_return_agents or [])],
        'goal_return_source_threshold': float(args.goal_return_source_threshold),
        'goal_return_target_linear': float(args.goal_return_target_linear),
        'goal_return_min_distance': float(args.goal_return_min_distance),
        'goal_return_min_route_progress': float(args.goal_return_min_route_progress),
        'goal_return_max_cte': float(args.goal_return_max_cte),
        'goal_return_min_separation': float(args.goal_return_min_separation),
        'goal_return_max_threat': float(args.goal_return_max_threat),
        'goal_return_min_step': int(args.goal_return_min_step),
        'goal_return_safe_omega_blend': float(args.goal_return_safe_omega_blend),
        'goal_return_allow_deconf': bool(args.goal_return_allow_deconf),
        'recovery_speedup_weight': float(args.recovery_speedup_weight),
        'recovery_speedup_source_threshold': float(args.recovery_speedup_source_threshold),
        'recovery_speedup_target_linear': float(args.recovery_speedup_target_linear),
        'recovery_speedup_min_distance': float(args.recovery_speedup_min_distance),
        'recovery_speedup_max_cte': float(args.recovery_speedup_max_cte),
        'recovery_speedup_min_separation': float(args.recovery_speedup_min_separation),
        'recovery_speedup_max_threat': float(args.recovery_speedup_max_threat),
        'recovery_speedup_min_step': int(args.recovery_speedup_min_step),
        'recovery_speedup_min_route_progress': float(args.recovery_speedup_min_route_progress),
        'recovery_speedup_max_route_progress': float(args.recovery_speedup_max_route_progress),
        'recovery_speedup_max_heading_error': float(args.recovery_speedup_max_heading_error),
        'recovery_speedup_cte_omega_blend': float(args.recovery_speedup_cte_omega_blend),
        'recovery_speedup_agents': [str(value) for value in (args.recovery_speedup_agents or [])],
        'recovery_turn_weight': float(args.recovery_turn_weight),
        'recovery_turn_target_linear': float(args.recovery_turn_target_linear),
        'recovery_turn_min_distance': float(args.recovery_turn_min_distance),
        'recovery_turn_min_abs_cte': float(args.recovery_turn_min_abs_cte),
        'recovery_turn_min_heading_error': float(args.recovery_turn_min_heading_error),
        'recovery_turn_min_separation': float(args.recovery_turn_min_separation),
        'recovery_turn_max_threat': float(args.recovery_turn_max_threat),
        'recovery_turn_min_step': int(args.recovery_turn_min_step),
        'recovery_turn_min_route_progress': float(args.recovery_turn_min_route_progress),
        'recovery_turn_max_route_progress': float(args.recovery_turn_max_route_progress),
        'recovery_turn_omega_blend': float(args.recovery_turn_omega_blend),
        'recovery_turn_min_omega_abs': float(args.recovery_turn_min_omega_abs),
        'recovery_turn_heading_gain': float(args.recovery_turn_heading_gain),
        'recovery_turn_max_omega': float(args.recovery_turn_max_omega),
        'risk_guard_weight': float(args.risk_guard_weight),
        'risk_guard_min_threat': float(args.risk_guard_min_threat),
        'risk_guard_max_separation': float(args.risk_guard_max_separation),
        'risk_guard_max_linear': float(args.risk_guard_max_linear),
        'risk_guard_yield_max_linear': float(args.risk_guard_yield_max_linear),
        'risk_guard_standon_max_linear': float(args.risk_guard_standon_max_linear),
        'risk_guard_omega_blend': float(args.risk_guard_omega_blend),
        'risk_guard_yield_min_omega_abs': float(args.risk_guard_yield_min_omega_abs),
        'risk_guard_standon_min_omega_abs': float(args.risk_guard_standon_min_omega_abs),
        'risk_guard_min_starboard_omega': float(args.risk_guard_min_starboard_omega),
        'risk_guard_min_starboard_threat': float(args.risk_guard_min_starboard_threat),
        'risk_guard_min_distance': float(args.risk_guard_min_distance),
        'risk_guard_agents': [str(value) for value in (args.risk_guard_agents or [])],
        'risk_guard_min_route_progress': float(args.risk_guard_min_route_progress),
        'risk_guard_max_route_progress': float(args.risk_guard_max_route_progress),
        'risk_guard_min_abs_cte': float(args.risk_guard_min_abs_cte),
        'risk_guard_max_step': int(args.risk_guard_max_step),
        'risk_guard_yield_only': bool(args.risk_guard_yield_only),
        'risk_guard_standon_only': bool(args.risk_guard_standon_only),
        'risk_guard_require_threat': bool(args.risk_guard_require_threat),
        'goal_hold_weight': float(args.goal_hold_weight),
        'goal_hold_source_threshold': float(args.goal_hold_source_threshold),
        'goal_hold_max_distance': float(args.goal_hold_max_distance),
        'goal_hold_max_cte': float(args.goal_hold_max_cte),
        'goal_hold_max_separation': float(args.goal_hold_max_separation),
        'goal_hold_min_threat': float(args.goal_hold_min_threat),
        'goal_hold_target_linear': float(args.goal_hold_target_linear),
        'goal_hold_omega_blend': float(args.goal_hold_omega_blend),
        'preserve_trace_json': [str(path) for path in args.preserve_trace_json or []],
        'preserve_action_weight': float(args.preserve_action_weight),
        'preserve_min_route_progress': float(args.preserve_min_route_progress),
        'preserve_max_route_progress': float(args.preserve_max_route_progress),
        'preserve_min_team_separation': float(args.preserve_min_team_separation),
        'preserve_min_episode_progress': float(args.preserve_min_episode_progress),
        'preserve_exclude_collisions': bool(args.preserve_exclude_collisions),
        'last_loss': last_loss,
        'train_all_actor': bool(args.train_all_actor),
    }
    torch.save(checkpoint, output_path)
    print(
        'trace-fit saved',
        str(output_path),
        'samples=', int(obs.shape[0]),
        'anchor_samples=', int(anchor_obs.shape[0]),
        'counts=', counts,
        'last_loss=', f'{last_loss:.6f}',
        flush=True,
    )


if __name__ == '__main__':
    main()