"""Attention-based neighbor aggregation for MAPPO USV policy.

Replaces fixed max-neighbor zero-padded concatenation with learned
scaled dot-product attention.  The ego agent's state serves as the
query, neighbor feature vectors are keys/values.  The network learns
to attend to the most safety-relevant neighbor (closest, highest-TCPA,
head-on, etc.) from per-neighbor relative-geometry and pairwise timing
features.

Architecture drop-in: AttentionActor / AttentionCritic accept the SAME
flat observation vector as the baseline MLP — they parse it internally.
"""

from __future__ import annotations

import math

import torch
import torch.nn as nn

# Fixed observation layout constants (must match multi_agent_types / types).
_EGO_DIM = 17               # base ego + route/crossing timing coordination features
_NEIGHBOR_FEATURE_DIM = 10   # rel_x, rel_y, rel_vx, rel_vy, distance, bearing, tcpa, dcpa, route_eta_delta, route_priority_delta
_LEGACY_NEIGHBOR_FEATURE_DIMS = (6,)
_OLD_EGO_DIM = 11            # previous ego dimension (before sin/cos heading_error split)


def _infer_ego_dim_from_obs_dim(obs_dim: int, encounter_dim: int) -> int:
    """Infer a saved local-observation ego width from total obs dimension."""
    for candidate in (_EGO_DIM, 12, _OLD_EGO_DIM, 10):
        for neighbor_dim in (_NEIGHBOR_FEATURE_DIM, *_LEGACY_NEIGHBOR_FEATURE_DIMS):
            remaining = int(obs_dim) - int(candidate) - int(encounter_dim)
            if remaining >= 0 and remaining % int(neighbor_dim) == 0:
                return int(candidate)
    return _OLD_EGO_DIM


def _infer_neighbor_dim_from_obs_dim(
    obs_dim: int,
    ego_dim: int,
    encounter_dim: int,
    max_neighbors: int | None = None,
) -> int:
    """Infer saved per-neighbor width, preferring explicit max_neighbors metadata."""
    remaining = int(obs_dim) - int(ego_dim) - int(encounter_dim)
    if remaining < 0:
        return _NEIGHBOR_FEATURE_DIM
    if max_neighbors is not None and int(max_neighbors) > 0 and remaining % int(max_neighbors) == 0:
        return max(1, remaining // int(max_neighbors))
    for neighbor_dim in (_NEIGHBOR_FEATURE_DIM, *_LEGACY_NEIGHBOR_FEATURE_DIMS):
        if remaining % int(neighbor_dim) == 0:
            return int(neighbor_dim)
    return _NEIGHBOR_FEATURE_DIM


def _migrate_policy_input_weight(
    old_weight: torch.Tensor,
    new_weight: torch.Tensor,
    torch_module,
    old_ego_dim: int,
    new_ego_dim: int,
) -> torch.Tensor:
    """Copy [ego, rest...] input weights when ego features are inserted.

    The new route/timing ego columns are zero-initialised while the attended
    neighbour and encounter columns are shifted to their new positions.
    """
    migrated = torch_module.zeros_like(new_weight)
    ego_cols = min(int(old_ego_dim), old_weight.shape[1], new_weight.shape[1])
    if ego_cols > 0:
        migrated[:, :ego_cols] = old_weight[:, :ego_cols]
    old_rest_start = int(old_ego_dim)
    new_rest_start = int(new_ego_dim)
    rest_cols = min(
        max(0, old_weight.shape[1] - old_rest_start),
        max(0, new_weight.shape[1] - new_rest_start),
    )
    if rest_cols > 0:
        migrated[:, new_rest_start:new_rest_start + rest_cols] = old_weight[:, old_rest_start:old_rest_start + rest_cols]
    return migrated


def _clone_sequential_modules(modules: list[nn.Module]) -> nn.Sequential:
    cloned: list[nn.Module] = []
    for module in modules:
        if isinstance(module, nn.Linear):
            copied = nn.Linear(module.in_features, module.out_features, bias=module.bias is not None)
            copied.load_state_dict(module.state_dict())
        elif isinstance(module, nn.Tanh):
            copied = nn.Tanh()
        else:
            raise TypeError(f'Unsupported module type for scenario head cloning: {type(module).__name__}')
        cloned.append(copied)
    return nn.Sequential(*cloned)


# ───────────────────────────────────────────────────────────
# Core attention encoder
# ───────────────────────────────────────────────────────────

class NeighborAttentionEncoder(nn.Module):
    """Multi-head scaled dot-product attention: ego→neighbors."""

    def __init__(self, ego_dim: int = _EGO_DIM,
                 neighbor_feature_dim: int = _NEIGHBOR_FEATURE_DIM,
                 embed_dim: int = 32, num_heads: int = 1):
        super().__init__()
        if embed_dim % num_heads != 0:
            raise ValueError(
                f'embed_dim ({embed_dim}) must be divisible by num_heads ({num_heads})'
            )
        self.ego_dim = ego_dim
        self.neighbor_feature_dim = neighbor_feature_dim
        self.embed_dim = embed_dim
        self.num_heads = num_heads
        self.head_dim = embed_dim // num_heads

        self.query_proj = nn.Linear(ego_dim, embed_dim)
        self.key_proj = nn.Linear(neighbor_feature_dim, embed_dim)
        self.value_proj = nn.Linear(neighbor_feature_dim, embed_dim)
        self.output_proj = nn.Linear(embed_dim, embed_dim)

        self._scale = math.sqrt(self.head_dim)

    def forward(
        self,
        ego_state: torch.Tensor,
        neighbor_features: torch.Tensor,
        neighbor_mask: torch.Tensor,
    ) -> torch.Tensor:
        """
        Args:
            ego_state:         [B, ego_dim]
            neighbor_features: [B, N, neighbor_feature_dim]
            neighbor_mask:     [B, N]  True = valid neighbor
        Returns:
            [B, embed_dim]
        """
        B, N, _ = neighbor_features.shape
        H = self.num_heads
        D = self.head_dim

        # Q from ego, K/V from neighbors
        Q = self.query_proj(ego_state).view(B, 1, H, D).transpose(1, 2)       # [B,H,1,D]
        K = self.key_proj(neighbor_features).view(B, N, H, D).transpose(1, 2)  # [B,H,N,D]
        V = self.value_proj(neighbor_features).view(B, N, H, D).transpose(1, 2)

        attn_scores = torch.matmul(Q, K.transpose(-2, -1)) / self._scale      # [B,H,1,N]

        # Mask padded slots (distance==0)
        inv_mask = ~neighbor_mask                                               # [B,N]
        attn_scores = attn_scores.masked_fill(
            inv_mask.unsqueeze(1).unsqueeze(2), -1e9,                          # [B,1,1,N]
        )

        attn_weights = torch.softmax(attn_scores, dim=-1)                      # [B,H,1,N]

        # If ALL neighbors are masked, zero the weights (softmax gives uniform
        # weights on -inf which is numerically ≈ 1/N — we want zeros instead).
        no_neighbors = ~neighbor_mask.any(dim=-1)                              # [B]
        if no_neighbors.any():
            attn_weights = attn_weights * (
                (~no_neighbors).float().view(B, 1, 1, 1)
            )

        attended = torch.matmul(attn_weights, V)                              # [B,H,1,D]
        attended = attended.transpose(1, 2).contiguous().view(B, self.embed_dim)
        return self.output_proj(attended)


# ───────────────────────────────────────────────────────────
# Observation parser (shared by actor & critic)
# ───────────────────────────────────────────────────────────

def _parse_local_obs(obs: torch.Tensor, max_neighbors: int, encounter_dim: int):
    """Split a flat obs vector into (ego, neighbors, mask, encounter)."""
    ego = obs[:, :_EGO_DIM]
    nb_end = _EGO_DIM + max_neighbors * _NEIGHBOR_FEATURE_DIM
    neighbors = obs[:, _EGO_DIM:nb_end].view(-1, max_neighbors, _NEIGHBOR_FEATURE_DIM)
    encounter = obs[:, nb_end:nb_end + encounter_dim]
    # Neighbor is valid when its distance field (index 4) is positive.
    mask = neighbors[:, :, 4] > 1e-6
    return ego, neighbors, mask, encounter


# ───────────────────────────────────────────────────────────
# AttentionActor  (drop-in replacement for flat nn.Sequential)
# ───────────────────────────────────────────────────────────

class AttentionActor(nn.Module):
    """Actor that uses attention to aggregate neighbor information.

    Input / output shapes are *identical* to the flat MLP actor so
    that the training loop, inference node, and evaluation script can
    call ``actor(obs_tensor)`` without any change.
    """

    def __init__(
        self,
        max_neighbors: int,
        encounter_dim: int,
        hidden_sizes: tuple[int, ...],
        action_dim: int,
        embed_dim: int = 32,
        num_heads: int = 1,
        encounter_residual: bool = False,
        scenario_names: tuple[str, ...] = (),
        scenario_residual: bool = False,
        scenario_head: bool = False,
        scenario_trunk: bool = False,
    ):
        super().__init__()
        self.max_neighbors = max_neighbors
        self.encounter_dim = encounter_dim
        self.obs_dim = _EGO_DIM + max_neighbors * _NEIGHBOR_FEATURE_DIM + encounter_dim
        self.encounter_residual_enabled = bool(encounter_residual)
        self.scenario_names = tuple(str(name) for name in scenario_names)
        self.scenario_name_to_index = {
            scenario_name: index for index, scenario_name in enumerate(self.scenario_names)
        }
        self.scenario_residual_enabled = bool(scenario_residual)
        self.scenario_head_enabled = bool(scenario_head)
        self.scenario_trunk_enabled = bool(scenario_trunk)
        if self.scenario_residual_enabled and not self.scenario_names:
            raise ValueError('scenario_residual requires a non-empty scenario_names tuple.')
        if self.scenario_head_enabled and not self.scenario_names:
            raise ValueError('scenario_head requires a non-empty scenario_names tuple.')
        if self.scenario_trunk_enabled and not self.scenario_names:
            raise ValueError('scenario_trunk requires a non-empty scenario_names tuple.')
        self._active_scenario_index = -1

        self.neighbor_attention = NeighborAttentionEncoder(
            embed_dim=embed_dim, num_heads=num_heads,
        )

        mlp_input_dim = _EGO_DIM + embed_dim + encounter_dim
        layers: list[nn.Module] = []
        cur = mlp_input_dim
        for h in hidden_sizes:
            layers.append(nn.Linear(cur, h))
            layers.append(nn.Tanh())
            cur = h
        layers.append(nn.Linear(cur, action_dim))
        self.mlp = nn.Sequential(*layers)
        self._scenario_head_tail_start = max(0, len(self.mlp) - 3)

        self.encounter_residual = None
        if self.encounter_residual_enabled:
            residual_hidden_dim = cur
            self.encounter_residual = nn.Sequential(
                nn.Linear(cur + encounter_dim, residual_hidden_dim),
                nn.Tanh(),
                nn.Linear(residual_hidden_dim, action_dim),
            )
            nn.init.zeros_(self.encounter_residual[-1].weight)
            nn.init.zeros_(self.encounter_residual[-1].bias)

        self.scenario_residual_heads = None
        if self.scenario_residual_enabled:
            residual_hidden_dim = cur
            self.scenario_residual_heads = nn.ModuleList()
            for _ in self.scenario_names:
                head = nn.Sequential(
                    nn.Linear(cur, residual_hidden_dim),
                    nn.Tanh(),
                    nn.Linear(residual_hidden_dim, action_dim),
                )
                nn.init.zeros_(head[-1].weight)
                nn.init.zeros_(head[-1].bias)
                self.scenario_residual_heads.append(head)

        self.scenario_action_heads = None
        if self.scenario_head_enabled:
            self.scenario_action_heads = nn.ModuleList()
            tail_modules = list(self.mlp[self._scenario_head_tail_start:])
            for _ in self.scenario_names:
                self.scenario_action_heads.append(_clone_sequential_modules(tail_modules))

        self.scenario_actor_trunks = None
        if self.scenario_trunk_enabled:
            self.scenario_actor_trunks = nn.ModuleList()
            trunk_modules = list(self.mlp)
            for _ in self.scenario_names:
                self.scenario_actor_trunks.append(_clone_sequential_modules(trunk_modules))

    def set_active_scenario_name(self, scenario_name: str | None) -> None:
        if scenario_name is None:
            self._active_scenario_index = -1
            return
        self._active_scenario_index = int(self.scenario_name_to_index.get(str(scenario_name), -1))

    def _resolve_scenario_ids(self, obs: torch.Tensor, scenario_ids: torch.Tensor | None) -> torch.Tensor | None:
        if not (self.scenario_residual_enabled or self.scenario_head_enabled or self.scenario_trunk_enabled):
            return None
        if scenario_ids is not None:
            return scenario_ids.to(device=obs.device, dtype=torch.long).reshape(-1)
        if self._active_scenario_index < 0:
            return None
        return torch.full(
            (obs.shape[0],),
            int(self._active_scenario_index),
            dtype=torch.long,
            device=obs.device,
        )

    def forward(self, obs: torch.Tensor, scenario_ids: torch.Tensor | None = None) -> torch.Tensor:
        ego, neighbors, mask, encounter = _parse_local_obs(
            obs, self.max_neighbors, self.encounter_dim,
        )
        attended = self.neighbor_attention(ego, neighbors, mask)
        policy_input = torch.cat([ego, attended, encounter], dim=-1)
        hidden = self.mlp[:-1](policy_input)
        action = self.mlp[-1](hidden)
        scenario_head_input = None
        if self.encounter_residual is not None:
            action = action + self.encounter_residual(torch.cat([hidden, encounter], dim=-1))
        resolved_scenario_ids = self._resolve_scenario_ids(obs, scenario_ids)
        if self.scenario_residual_heads is not None and resolved_scenario_ids is not None:
            scenario_residual = torch.zeros_like(action)
            for scenario_index, head in enumerate(self.scenario_residual_heads):
                mask = resolved_scenario_ids == scenario_index
                if mask.any():
                    scenario_residual[mask] = head(hidden[mask])
            action = action + scenario_residual
        if self.scenario_action_heads is not None and resolved_scenario_ids is not None:
            if self._scenario_head_tail_start > 0:
                scenario_head_input = self.mlp[:self._scenario_head_tail_start](policy_input)
            else:
                scenario_head_input = policy_input
            scenario_action = torch.zeros_like(action)
            assigned_mask = torch.zeros_like(resolved_scenario_ids, dtype=torch.bool)
            for scenario_index, head in enumerate(self.scenario_action_heads):
                mask = resolved_scenario_ids == scenario_index
                if mask.any():
                    scenario_action[mask] = head(scenario_head_input[mask])
                    assigned_mask = assigned_mask | mask
            if assigned_mask.any():
                action = action.clone()
                action[assigned_mask] = scenario_action[assigned_mask]
        if self.scenario_actor_trunks is not None and resolved_scenario_ids is not None:
            scenario_action = torch.zeros_like(action)
            assigned_mask = torch.zeros_like(resolved_scenario_ids, dtype=torch.bool)
            for scenario_index, trunk in enumerate(self.scenario_actor_trunks):
                mask = resolved_scenario_ids == scenario_index
                if mask.any():
                    scenario_action[mask] = trunk(policy_input[mask])
                    assigned_mask = assigned_mask | mask
            if assigned_mask.any():
                action = action.clone()
                action[assigned_mask] = scenario_action[assigned_mask]
        return action


# ───────────────────────────────────────────────────────────
# AttentionCritic  (drop-in replacement for flat nn.Sequential)
# ───────────────────────────────────────────────────────────

class AttentionCritic(nn.Module):
    """Critic with attention on the local-observation neighbor block.

    Input: ``[local_obs, global_state]`` (concatenated, same as flat MLP).
    Attention is applied only to the ego agent's neighbor block;
    the global fleet state passes through unchanged.
    """

    def __init__(
        self,
        max_neighbors: int,
        encounter_dim: int,
        global_state_dim: int,
        hidden_sizes: tuple[int, ...],
        embed_dim: int = 32,
        num_heads: int = 1,
    ):
        super().__init__()
        self.max_neighbors = max_neighbors
        self.encounter_dim = encounter_dim
        self.local_obs_dim = _EGO_DIM + max_neighbors * _NEIGHBOR_FEATURE_DIM + encounter_dim
        self.global_state_dim = global_state_dim

        self.neighbor_attention = NeighborAttentionEncoder(
            embed_dim=embed_dim, num_heads=num_heads,
        )

        mlp_input_dim = _EGO_DIM + embed_dim + encounter_dim + global_state_dim
        layers: list[nn.Module] = []
        cur = mlp_input_dim
        for h in hidden_sizes:
            layers.append(nn.Linear(cur, h))
            layers.append(nn.Tanh())
            cur = h
        layers.append(nn.Linear(cur, 1))
        self.mlp = nn.Sequential(*layers)

    def forward(self, critic_input: torch.Tensor) -> torch.Tensor:
        local_obs = critic_input[:, :self.local_obs_dim]
        global_state = critic_input[:, self.local_obs_dim:]

        ego, neighbors, mask, encounter = _parse_local_obs(
            local_obs, self.max_neighbors, self.encounter_dim,
        )
        attended = self.neighbor_attention(ego, neighbors, mask)
        return self.mlp(torch.cat([ego, attended, encounter, global_state], dim=-1))


# ───────────────────────────────────────────────────────────
# Weight migration:  flat MLP → attention
# ───────────────────────────────────────────────────────────

def migrate_flat_actor_to_attention(
    old_state_dict: dict,
    new_actor: AttentionActor,
    torch_module,
) -> None:
    """Transfer compatible weights from a flat MLP checkpoint into an
    :class:`AttentionActor`, newly initialised with Xavier attention layers.

    * Hidden / output MLP layers are copied verbatim (shapes match).
    * First MLP layer: ego and encounter columns are copied;
      attention-output columns are left at zero (attention layers
      learn the new neighbor representation from scratch).
    """
    new_sd = new_actor.state_dict()

    old_obs_dim = int(old_state_dict['0.weight'].shape[1])
    enc_dim = new_actor.encounter_dim
    old_ego_dim = _infer_ego_dim_from_obs_dim(old_obs_dim, enc_dim)
    old_nb_flat = old_obs_dim - old_ego_dim - enc_dim
    embed_dim = new_actor.neighbor_attention.embed_dim

    # --- first MLP layer (input projection, shape changes) ---
    new_first_w = torch_module.zeros_like(new_sd['mlp.0.weight'])
    old_first_w = old_state_dict['0.weight']

    # ego columns
    new_first_w[:, :old_ego_dim] = old_first_w[:, :old_ego_dim]
    # encounter columns
    enc_old_start = old_ego_dim + old_nb_flat
    enc_new_start = _EGO_DIM + embed_dim
    new_first_w[:, enc_new_start:enc_new_start + enc_dim] = (
        old_first_w[:, enc_old_start:enc_old_start + enc_dim]
    )
    new_sd['mlp.0.weight'] = new_first_w
    new_sd['mlp.0.bias'] = old_state_dict['0.bias'].clone()

    # --- hidden + output layers (shapes identical) ---
    for key, value in old_state_dict.items():
        idx_str = key.split('.')[0]
        if idx_str.isdigit() and int(idx_str) >= 2:
            new_key = f'mlp.{key}'
            if new_key in new_sd and new_sd[new_key].shape == value.shape:
                new_sd[new_key] = value.clone()

    new_actor.load_state_dict(new_sd)


def migrate_flat_critic_to_attention(
    old_state_dict: dict,
    new_critic: AttentionCritic,
    torch_module,
) -> None:
    """Transfer compatible weights from a flat MLP critic checkpoint."""
    new_sd = new_critic.state_dict()

    enc_dim = new_critic.encounter_dim
    max_agents = max(1, (int(new_critic.global_state_dim) - 5) // max(1, int(new_critic.local_obs_dim)))
    old_input_dim = int(old_state_dict['0.weight'].shape[1])
    old_local_obs_dim = (old_input_dim - 5) // max(1, 1 + max_agents)
    old_ego_dim = _infer_ego_dim_from_obs_dim(old_local_obs_dim, enc_dim)
    old_nb_flat = old_local_obs_dim - old_ego_dim - enc_dim
    embed_dim = new_critic.neighbor_attention.embed_dim
    gs_dim = new_critic.global_state_dim

    old_first_w = old_state_dict['0.weight']
    new_first_w = torch_module.zeros_like(new_sd['mlp.0.weight'])

    # ego columns
    new_first_w[:, :old_ego_dim] = old_first_w[:, :old_ego_dim]
    # encounter columns
    enc_old_start = old_ego_dim + old_nb_flat
    enc_new_start = _EGO_DIM + embed_dim
    new_first_w[:, enc_new_start:enc_new_start + enc_dim] = (
        old_first_w[:, enc_old_start:enc_old_start + enc_dim]
    )
    # global state columns
    gs_old_start = old_local_obs_dim
    gs_new_start = _EGO_DIM + embed_dim + enc_dim
    old_pos = gs_old_start
    new_pos = gs_new_start
    for _ in range(max_agents):
        new_first_w[:, new_pos:new_pos + old_ego_dim] = old_first_w[:, old_pos:old_pos + old_ego_dim]
        old_pos += old_ego_dim
        new_pos += _EGO_DIM
        remaining_local = old_local_obs_dim - old_ego_dim
        new_first_w[:, new_pos:new_pos + remaining_local] = old_first_w[:, old_pos:old_pos + remaining_local]
        old_pos += remaining_local
        new_pos += remaining_local
    remaining = old_first_w.shape[1] - old_pos
    if remaining > 0:
        new_first_w[:, new_pos:new_pos + remaining] = old_first_w[:, old_pos:old_pos + remaining]

    new_sd['mlp.0.weight'] = new_first_w
    new_sd['mlp.0.bias'] = old_state_dict['0.bias'].clone()

    for key, value in old_state_dict.items():
        idx_str = key.split('.')[0]
        if idx_str.isdigit() and int(idx_str) >= 2:
            new_key = f'mlp.{key}'
            if new_key in new_sd and new_sd[new_key].shape == value.shape:
                new_sd[new_key] = value.clone()

    new_critic.load_state_dict(new_sd)


# ───────────────────────────────────────────────────────────
# Weight migration:  attention → attention (ego_dim change)
# ───────────────────────────────────────────────────────────

def _migrate_attention_actor_ego_dim(
    old_state_dict: dict,
    new_actor: AttentionActor,
    torch_module,
    old_ego_dim: int,
    new_ego_dim: int,
) -> None:
    """Migrate an AttentionActor checkpoint when ego_dim changes.

    Handles:
    - query_proj: Linear(old_ego, embed) → Linear(new_ego, embed)
    - mlp.0: ego columns expand from old_ego to new_ego
    Other layers are copied verbatim.
    """
    new_sd = new_actor.state_dict()
    delta = new_ego_dim - old_ego_dim

    # --- query_proj: weight [embed, old_ego] → [embed, new_ego] ---
    old_q_w = old_state_dict['neighbor_attention.query_proj.weight']  # [embed, old_ego]
    new_q_w = torch_module.zeros_like(new_sd['neighbor_attention.query_proj.weight'])
    new_q_w[:, :old_ego_dim] = old_q_w
    new_sd['neighbor_attention.query_proj.weight'] = new_q_w
    new_sd['neighbor_attention.query_proj.bias'] = old_state_dict['neighbor_attention.query_proj.bias'].clone()

    # --- mlp.0: input is [ego, attended, encounter] ---
    # ego grows by delta, rest stays the same.
    old_first_w = old_state_dict['mlp.0.weight']  # [hidden, old_ego + embed + enc]
    new_sd['mlp.0.weight'] = _migrate_policy_input_weight(
        old_first_w,
        new_sd['mlp.0.weight'],
        torch_module,
        old_ego_dim,
        new_ego_dim,
    )
    new_sd['mlp.0.bias'] = old_state_dict['mlp.0.bias'].clone()

    for key, value in old_state_dict.items():
        if key.startswith('scenario_actor_trunks.') and key.endswith('.0.weight') and key in new_sd:
            new_sd[key] = _migrate_policy_input_weight(
                value,
                new_sd[key],
                torch_module,
                old_ego_dim,
                new_ego_dim,
            )

    # --- Copy all other layers verbatim ---
    for key, value in old_state_dict.items():
        if key in ('neighbor_attention.query_proj.weight', 'neighbor_attention.query_proj.bias',
                    'mlp.0.weight', 'mlp.0.bias'):
            continue
        if key.startswith('scenario_actor_trunks.') and key.endswith('.0.weight'):
            continue
        if key in new_sd and new_sd[key].shape == value.shape:
            new_sd[key] = value.clone()

    new_actor.load_state_dict(new_sd)


def _migrate_attention_critic_ego_dim(
    old_state_dict: dict,
    new_critic: AttentionCritic,
    torch_module,
    old_ego_dim: int,
    new_ego_dim: int,
    max_agents: int,
) -> None:
    """Migrate an AttentionCritic checkpoint when ego_dim changes.

    The critic MLP input is: [ego, attended, encounter, global_state]
    where global_state = [agent1_obs, agent2_obs, ..., fleet_stats(5)].
    Each agent_obs block grows by (new_ego_dim - old_ego_dim).
    """
    new_sd = new_critic.state_dict()

    # --- query_proj: same as actor ---
    old_q_w = old_state_dict['neighbor_attention.query_proj.weight']
    new_q_w = torch_module.zeros_like(new_sd['neighbor_attention.query_proj.weight'])
    new_q_w[:, :old_ego_dim] = old_q_w
    new_sd['neighbor_attention.query_proj.weight'] = new_q_w
    new_sd['neighbor_attention.query_proj.bias'] = old_state_dict['neighbor_attention.query_proj.bias'].clone()

    # --- mlp.0: input is [ego, attended, encounter, global_state] ---
    # global_state = [obs_1, obs_2, ..., obs_max_agents, fleet_metrics(5)]
    # Each obs_i has old_obs_dim → new_obs_dim
    old_first_w = old_state_dict['mlp.0.weight']
    new_first_w = torch_module.zeros_like(new_sd['mlp.0.weight'])

    embed_dim = new_critic.neighbor_attention.embed_dim
    enc_dim = new_critic.encounter_dim
    old_local_obs_dim = old_ego_dim + new_critic.max_neighbors * _NEIGHBOR_FEATURE_DIM + enc_dim
    new_local_obs_dim = new_critic.local_obs_dim

    # Structure: [ego(old), attended(embed), encounter(enc), agent1_obs(old_local), ..., agentN_obs(old_local), fleet(5)]
    # After migration: [ego(new), attended(embed), encounter(enc), agent1_obs(new_local), ..., agentN_obs(new_local), fleet(5)]

    old_pos = 0
    new_pos = 0

    # 1+2. Self actor-critic input [ego, attended, encounter].
    ae_cols = embed_dim + enc_dim
    self_old_cols = old_ego_dim + ae_cols
    self_new_cols = new_ego_dim + ae_cols
    new_first_w[:, new_pos:new_pos + self_new_cols] = _migrate_policy_input_weight(
        old_first_w[:, old_pos:old_pos + self_old_cols],
        new_first_w[:, new_pos:new_pos + self_new_cols],
        torch_module,
        old_ego_dim,
        new_ego_dim,
    )
    old_pos += self_old_cols
    new_pos += self_new_cols

    # 3. Global state: max_agents obs blocks, each grows from old_local to new_local
    for _ in range(max_agents):
        new_first_w[:, new_pos:new_pos + old_ego_dim] = old_first_w[:, old_pos:old_pos + old_ego_dim]
        old_pos += old_ego_dim
        new_pos += new_ego_dim
        local_rest = old_local_obs_dim - old_ego_dim
        new_first_w[:, new_pos:new_pos + local_rest] = old_first_w[:, old_pos:old_pos + local_rest]
        old_pos += local_rest
        new_pos += local_rest

    # 4. Fleet metrics (5 features, unchanged)
    remaining = old_first_w.shape[1] - old_pos
    if remaining > 0:
        new_first_w[:, new_pos:new_pos + remaining] = old_first_w[:, old_pos:old_pos + remaining]

    new_sd['mlp.0.weight'] = new_first_w
    new_sd['mlp.0.bias'] = old_state_dict['mlp.0.bias'].clone()

    # --- Copy all other layers verbatim ---
    for key, value in old_state_dict.items():
        if key in ('neighbor_attention.query_proj.weight', 'neighbor_attention.query_proj.bias',
                    'mlp.0.weight', 'mlp.0.bias'):
            continue
        if key in new_sd and new_sd[key].shape == value.shape:
            new_sd[key] = value.clone()

    new_critic.load_state_dict(new_sd)


def _migrate_attention_projection(
    old_state_dict: dict,
    new_sd: dict,
    name: str,
    *,
    torch_module,
    zero_new_columns: bool = False,
) -> None:
    weight_key = f'neighbor_attention.{name}_proj.weight'
    bias_key = f'neighbor_attention.{name}_proj.bias'
    if weight_key not in old_state_dict or weight_key not in new_sd:
        return
    old_w = old_state_dict[weight_key]
    if zero_new_columns:
        new_w = torch_module.zeros_like(new_sd[weight_key])
    else:
        new_w = new_sd[weight_key].clone()
    rows = min(old_w.shape[0], new_w.shape[0])
    cols = min(old_w.shape[1], new_w.shape[1])
    new_w[:rows, :cols] = old_w[:rows, :cols]
    new_sd[weight_key] = new_w
    if bias_key in old_state_dict and bias_key in new_sd and old_state_dict[bias_key].shape == new_sd[bias_key].shape:
        new_sd[bias_key] = old_state_dict[bias_key].clone()


def _migrate_attention_actor_layout(
    old_state_dict: dict,
    new_actor: AttentionActor,
    torch_module,
    old_ego_dim: int,
    new_ego_dim: int,
    old_neighbor_dim: int,
    new_neighbor_dim: int,
) -> None:
    """Migrate an AttentionActor across ego and/or neighbor feature layout changes."""
    new_sd = new_actor.state_dict()

    _migrate_attention_projection(
        old_state_dict, new_sd, 'query', torch_module=torch_module, zero_new_columns=(old_ego_dim != new_ego_dim),
    )
    _migrate_attention_projection(old_state_dict, new_sd, 'key', torch_module=torch_module)
    _migrate_attention_projection(old_state_dict, new_sd, 'value', torch_module=torch_module)

    if 'mlp.0.weight' in old_state_dict and 'mlp.0.weight' in new_sd:
        if old_state_dict['mlp.0.weight'].shape == new_sd['mlp.0.weight'].shape:
            new_sd['mlp.0.weight'] = old_state_dict['mlp.0.weight'].clone()
        else:
            new_sd['mlp.0.weight'] = _migrate_policy_input_weight(
                old_state_dict['mlp.0.weight'],
                new_sd['mlp.0.weight'],
                torch_module,
                old_ego_dim,
                new_ego_dim,
            )
        new_sd['mlp.0.bias'] = old_state_dict['mlp.0.bias'].clone()

    for key, value in old_state_dict.items():
        if key.startswith('scenario_actor_trunks.') and key.endswith('.0.weight') and key in new_sd:
            if value.shape == new_sd[key].shape:
                new_sd[key] = value.clone()
            else:
                new_sd[key] = _migrate_policy_input_weight(
                    value,
                    new_sd[key],
                    torch_module,
                    old_ego_dim,
                    new_ego_dim,
                )

    skipped = {
        'neighbor_attention.query_proj.weight', 'neighbor_attention.query_proj.bias',
        'neighbor_attention.key_proj.weight', 'neighbor_attention.key_proj.bias',
        'neighbor_attention.value_proj.weight', 'neighbor_attention.value_proj.bias',
        'mlp.0.weight', 'mlp.0.bias',
    }
    for key, value in old_state_dict.items():
        if key in skipped:
            continue
        if key.startswith('scenario_actor_trunks.') and key.endswith('.0.weight'):
            continue
        if key in new_sd and new_sd[key].shape == value.shape:
            new_sd[key] = value.clone()

    new_actor.load_state_dict(new_sd)
    if old_neighbor_dim != new_neighbor_dim:
        print(
            f'Attention actor neighbor_feature_dim changed: {old_neighbor_dim} -> {new_neighbor_dim}. '
            'Copied legacy columns and left new pairwise columns initialized for training.',
            flush=True,
        )


def _copy_local_obs_block_weight(
    old_first_w,
    new_first_w,
    *,
    old_pos: int,
    new_pos: int,
    old_ego_dim: int,
    new_ego_dim: int,
    old_neighbor_dim: int,
    new_neighbor_dim: int,
    max_neighbors: int,
    encounter_dim: int,
) -> tuple[int, int]:
    ego_cols = min(old_ego_dim, new_ego_dim)
    new_first_w[:, new_pos:new_pos + ego_cols] = old_first_w[:, old_pos:old_pos + ego_cols]
    old_pos += old_ego_dim
    new_pos += new_ego_dim
    for _ in range(max_neighbors):
        nb_cols = min(old_neighbor_dim, new_neighbor_dim)
        new_first_w[:, new_pos:new_pos + nb_cols] = old_first_w[:, old_pos:old_pos + nb_cols]
        old_pos += old_neighbor_dim
        new_pos += new_neighbor_dim
    new_first_w[:, new_pos:new_pos + encounter_dim] = old_first_w[:, old_pos:old_pos + encounter_dim]
    old_pos += encounter_dim
    new_pos += encounter_dim
    return old_pos, new_pos


def _migrate_attention_critic_layout(
    old_state_dict: dict,
    new_critic: AttentionCritic,
    torch_module,
    old_ego_dim: int,
    new_ego_dim: int,
    old_neighbor_dim: int,
    new_neighbor_dim: int,
    max_agents: int,
) -> None:
    """Migrate an AttentionCritic across ego and/or neighbor feature layout changes."""
    new_sd = new_critic.state_dict()
    _migrate_attention_projection(
        old_state_dict, new_sd, 'query', torch_module=torch_module, zero_new_columns=(old_ego_dim != new_ego_dim),
    )
    _migrate_attention_projection(old_state_dict, new_sd, 'key', torch_module=torch_module)
    _migrate_attention_projection(old_state_dict, new_sd, 'value', torch_module=torch_module)

    old_first_w = old_state_dict['mlp.0.weight']
    new_first_w = torch_module.zeros_like(new_sd['mlp.0.weight'])
    embed_dim = new_critic.neighbor_attention.embed_dim
    enc_dim = new_critic.encounter_dim

    old_pos = 0
    new_pos = 0
    ae_cols = embed_dim + enc_dim
    self_old_cols = old_ego_dim + ae_cols
    self_new_cols = new_ego_dim + ae_cols
    new_first_w[:, new_pos:new_pos + self_new_cols] = _migrate_policy_input_weight(
        old_first_w[:, old_pos:old_pos + self_old_cols],
        new_first_w[:, new_pos:new_pos + self_new_cols],
        torch_module,
        old_ego_dim,
        new_ego_dim,
    )
    old_pos += self_old_cols
    new_pos += self_new_cols

    for _ in range(max_agents):
        old_pos, new_pos = _copy_local_obs_block_weight(
            old_first_w,
            new_first_w,
            old_pos=old_pos,
            new_pos=new_pos,
            old_ego_dim=old_ego_dim,
            new_ego_dim=new_ego_dim,
            old_neighbor_dim=old_neighbor_dim,
            new_neighbor_dim=new_neighbor_dim,
            max_neighbors=new_critic.max_neighbors,
            encounter_dim=enc_dim,
        )

    remaining = old_first_w.shape[1] - old_pos
    if remaining > 0:
        new_first_w[:, new_pos:new_pos + remaining] = old_first_w[:, old_pos:old_pos + remaining]

    new_sd['mlp.0.weight'] = new_first_w
    new_sd['mlp.0.bias'] = old_state_dict['mlp.0.bias'].clone()

    skipped = {
        'neighbor_attention.query_proj.weight', 'neighbor_attention.query_proj.bias',
        'neighbor_attention.key_proj.weight', 'neighbor_attention.key_proj.bias',
        'neighbor_attention.value_proj.weight', 'neighbor_attention.value_proj.bias',
        'mlp.0.weight', 'mlp.0.bias',
    }
    for key, value in old_state_dict.items():
        if key in skipped:
            continue
        if key in new_sd and new_sd[key].shape == value.shape:
            new_sd[key] = value.clone()

    new_critic.load_state_dict(new_sd)


def build_attention_actor_from_checkpoint(
    checkpoint: dict, nn_module, torch_module, device,
) -> AttentionActor:
    """Reconstruct an AttentionActor from a saved MAPPO checkpoint."""
    from usv_rl.multi_agent_types import ENCOUNTER_TYPE_COUNT
    hidden_sizes = tuple(int(v) for v in checkpoint.get('hidden_sizes', [256, 256]))
    action_dim = int(checkpoint['action_dim'])
    obs_dim = int(checkpoint['local_observation_size'])
    embed_dim = int(checkpoint.get('attention_embed_dim', 32))
    num_heads = int(checkpoint.get('attention_num_heads', 1))
    encounter_residual = bool(
        checkpoint.get(
            'attention_encounter_residual',
            any(str(key).startswith('encounter_residual.') for key in checkpoint['actor_state_dict'].keys()),
        )
    )
    scenario_residual = bool(
        checkpoint.get(
            'attention_scenario_residual',
            any(str(key).startswith('scenario_residual_heads.') for key in checkpoint['actor_state_dict'].keys()),
        )
    )
    scenario_head = bool(
        checkpoint.get(
            'attention_scenario_head',
            any(str(key).startswith('scenario_action_heads.') for key in checkpoint['actor_state_dict'].keys()),
        )
    )
    scenario_trunk = bool(
        checkpoint.get(
            'attention_scenario_trunk',
            any(str(key).startswith('scenario_actor_trunks.') for key in checkpoint['actor_state_dict'].keys()),
        )
    )
    encounter_dim = ENCOUNTER_TYPE_COUNT
    scenario_names = tuple(str(name) for name in checkpoint.get('scenarios', ()))

    actor_sd = checkpoint['actor_state_dict']
    # Backwards compatibility: infer old attention ego_dim from query_proj when available.
    if 'neighbor_attention.query_proj.weight' in actor_sd:
        saved_ego_dim = int(actor_sd['neighbor_attention.query_proj.weight'].shape[1])
    else:
        saved_ego_dim = int(checkpoint.get('ego_dim', _infer_ego_dim_from_obs_dim(obs_dim, encounter_dim)))
    saved_max_neighbors = checkpoint.get('max_neighbors')
    max_neighbors = int(saved_max_neighbors) if saved_max_neighbors is not None else 0
    saved_neighbor_dim = _infer_neighbor_dim_from_obs_dim(
        obs_dim,
        saved_ego_dim,
        encounter_dim,
        max_neighbors if max_neighbors > 0 else None,
    )
    if max_neighbors <= 0:
        max_neighbors = max(1, (obs_dim - saved_ego_dim - encounter_dim) // max(1, saved_neighbor_dim))

    actor = AttentionActor(
        max_neighbors=max_neighbors,
        encounter_dim=encounter_dim,
        hidden_sizes=hidden_sizes,
        action_dim=action_dim,
        embed_dim=embed_dim,
        num_heads=num_heads,
        encounter_residual=encounter_residual,
        scenario_names=scenario_names,
        scenario_residual=scenario_residual,
        scenario_head=scenario_head,
        scenario_trunk=scenario_trunk,
    ).to(device)

    # If checkpoint layout differs from the current observation layout, migrate weights.
    if saved_ego_dim != _EGO_DIM or saved_neighbor_dim != _NEIGHBOR_FEATURE_DIM:
        _migrate_attention_actor_layout(
            actor_sd,
            actor,
            torch_module,
            saved_ego_dim,
            _EGO_DIM,
            saved_neighbor_dim,
            _NEIGHBOR_FEATURE_DIM,
        )
    else:
        actor.load_state_dict(actor_sd)
    actor.eval()
    return actor
