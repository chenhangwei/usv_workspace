"""Attention-based neighbor aggregation for MAPPO USV policy.

Replaces fixed max-neighbor zero-padded concatenation with learned
scaled dot-product attention.  The ego agent's state serves as the
query, neighbor feature vectors are keys/values.  The network learns
to attend to the most safety-relevant neighbor (closest, highest-TCPA,
head-on, etc.) purely from the 6-dim feature vector
(rel_x, rel_y, rel_vx, rel_vy, distance, bearing).

Architecture drop-in: AttentionActor / AttentionCritic accept the SAME
flat observation vector as the baseline MLP — they parse it internally.
"""

from __future__ import annotations

import math

import torch
import torch.nn as nn

# Fixed observation layout constants (must match multi_agent_types / types).
_EGO_DIM = 11               # pose_x/y, yaw, speed, dist_goal, heading_err, raw/final vels, cross_track_error
_NEIGHBOR_FEATURE_DIM = 6    # rel_x, rel_y, rel_vx, rel_vy, distance, bearing
_OLD_EGO_DIM = 10            # previous ego dimension (before cross_track_error was added)


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
    ):
        super().__init__()
        self.max_neighbors = max_neighbors
        self.encounter_dim = encounter_dim
        self.obs_dim = _EGO_DIM + max_neighbors * _NEIGHBOR_FEATURE_DIM + encounter_dim

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

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        ego, neighbors, mask, encounter = _parse_local_obs(
            obs, self.max_neighbors, self.encounter_dim,
        )
        attended = self.neighbor_attention(ego, neighbors, mask)
        return self.mlp(torch.cat([ego, attended, encounter], dim=-1))


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
    old_nb_flat = old_obs_dim - _EGO_DIM - enc_dim
    embed_dim = new_actor.neighbor_attention.embed_dim

    # --- first MLP layer (input projection, shape changes) ---
    new_first_w = torch_module.zeros_like(new_sd['mlp.0.weight'])
    old_first_w = old_state_dict['0.weight']

    # ego columns
    new_first_w[:, :_EGO_DIM] = old_first_w[:, :_EGO_DIM]
    # encounter columns
    enc_old_start = _EGO_DIM + old_nb_flat
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

    local_obs_dim = new_critic.local_obs_dim
    enc_dim = new_critic.encounter_dim
    old_nb_flat = local_obs_dim - _EGO_DIM - enc_dim
    embed_dim = new_critic.neighbor_attention.embed_dim
    gs_dim = new_critic.global_state_dim

    old_first_w = old_state_dict['0.weight']
    new_first_w = torch_module.zeros_like(new_sd['mlp.0.weight'])

    # ego columns
    new_first_w[:, :_EGO_DIM] = old_first_w[:, :_EGO_DIM]
    # encounter columns
    enc_old_start = _EGO_DIM + old_nb_flat
    enc_new_start = _EGO_DIM + embed_dim
    new_first_w[:, enc_new_start:enc_new_start + enc_dim] = (
        old_first_w[:, enc_old_start:enc_old_start + enc_dim]
    )
    # global state columns
    gs_old_start = local_obs_dim
    gs_new_start = _EGO_DIM + embed_dim + enc_dim
    new_first_w[:, gs_new_start:gs_new_start + gs_dim] = (
        old_first_w[:, gs_old_start:gs_old_start + gs_dim]
    )

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
    new_first_w = torch_module.zeros_like(new_sd['mlp.0.weight'])  # [hidden, new_ego + embed + enc]
    embed_dim = new_actor.neighbor_attention.embed_dim
    enc_dim = new_actor.encounter_dim

    # Copy ego columns
    new_first_w[:, :old_ego_dim] = old_first_w[:, :old_ego_dim]
    # Copy attended + encounter columns (shifted by delta)
    old_rest_start = old_ego_dim
    new_rest_start = new_ego_dim
    rest_cols = embed_dim + enc_dim
    new_first_w[:, new_rest_start:new_rest_start + rest_cols] = old_first_w[:, old_rest_start:old_rest_start + rest_cols]
    new_sd['mlp.0.weight'] = new_first_w
    new_sd['mlp.0.bias'] = old_state_dict['mlp.0.bias'].clone()

    # --- Copy all other layers verbatim ---
    for key, value in old_state_dict.items():
        if key in ('neighbor_attention.query_proj.weight', 'neighbor_attention.query_proj.bias',
                    'mlp.0.weight', 'mlp.0.bias'):
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
    delta = new_ego_dim - old_ego_dim

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
    old_local_obs_dim = new_critic.local_obs_dim - delta  # old local_obs_dim
    new_local_obs_dim = new_critic.local_obs_dim

    # Structure: [ego(old), attended(embed), encounter(enc), agent1_obs(old_local), ..., agentN_obs(old_local), fleet(5)]
    # After migration: [ego(new), attended(embed), encounter(enc), agent1_obs(new_local), ..., agentN_obs(new_local), fleet(5)]

    old_pos = 0
    new_pos = 0

    # 1. ego columns
    new_first_w[:, new_pos:new_pos + old_ego_dim] = old_first_w[:, old_pos:old_pos + old_ego_dim]
    old_pos += old_ego_dim
    new_pos += new_ego_dim

    # 2. attended + encounter columns (unchanged size)
    ae_cols = embed_dim + enc_dim
    new_first_w[:, new_pos:new_pos + ae_cols] = old_first_w[:, old_pos:old_pos + ae_cols]
    old_pos += ae_cols
    new_pos += ae_cols

    # 3. Global state: max_agents obs blocks, each grows from old_local to new_local
    for _ in range(max_agents):
        new_first_w[:, new_pos:new_pos + old_local_obs_dim] = old_first_w[:, old_pos:old_pos + old_local_obs_dim]
        old_pos += old_local_obs_dim
        new_pos += new_local_obs_dim

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
    encounter_dim = ENCOUNTER_TYPE_COUNT

    # Backwards compatibility: old checkpoints used ego_dim=10.
    saved_ego_dim = int(checkpoint.get('ego_dim', _OLD_EGO_DIM))
    max_neighbors = (obs_dim - saved_ego_dim - encounter_dim) // _NEIGHBOR_FEATURE_DIM

    actor = AttentionActor(
        max_neighbors=max_neighbors,
        encounter_dim=encounter_dim,
        hidden_sizes=hidden_sizes,
        action_dim=action_dim,
        embed_dim=embed_dim,
        num_heads=num_heads,
    ).to(device)

    actor_sd = checkpoint['actor_state_dict']
    # If checkpoint ego_dim differs from current _EGO_DIM, migrate weights.
    if saved_ego_dim != _EGO_DIM:
        _migrate_attention_actor_ego_dim(actor_sd, actor, torch_module, saved_ego_dim, _EGO_DIM)
    else:
        actor.load_state_dict(actor_sd)
    actor.eval()
    return actor
