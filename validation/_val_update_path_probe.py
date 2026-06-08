"""Faithful re-test of the PPO_SNN update-path policy evaluation.

Earlier naive probe fed a single-env membrane with a multi-row state batch,
forcing the reshape-as-sequence branch. The REAL update feeds the stored
per-step membranes, whose batch dimension equals the minibatch size, so
`num_envs == batch_size` and the per-sample parallel path runs instead.

This probe replicates the real path:
  1. roll out T steps single-env, carrying membrane, recording per-step
     (state, initial-membrane, collection-time mean action);
  2. rebuild the batched RNN input exactly as `_prepare_mem` would receive it
     (batch dim = T), call `compute` once, and compare the per-row output to
     the collection-time mean actions.

If rows match, the update evaluates the policy correctly per sample (valid).
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
for p in (REPO / "Trajectory Generator" / "Prosthesis_SNN",):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import prosthesis_snn  # noqa: F401
import torch
import numpy as np
from gymnasium.spaces import Box

from prosthesis_snn.config import SNNConfig
from prosthesis_snn.training.actor_critic import ProsthesisSNNActorCritic


def make_model(input_size=6, hidden=8, knots=3, layers=1):
    cfg = SNNConfig.for_env_action(
        input_size=input_size, hidden_size=hidden, num_layers=layers,
        policy_knots=knots, feature_names=tuple(f"f{i}" for i in range(input_size)),
    )
    obs = Box(-np.inf, np.inf, (input_size,), dtype=np.float32)
    act = Box(-1.0, 1.0, (knots * 2,), dtype=np.float32)
    torch.manual_seed(1)
    m = ProsthesisSNNActorCritic(obs, act, device="cpu", cfg=cfg, num_envs=1,
                                 sequence_length=1, log_std_init=-2.0)
    # make it input-sensitive but not saturating
    with torch.no_grad():
        for prm in m.parameters():
            prm.mul_(1.5)
    m.eval()
    return m


def main():
    torch.manual_seed(0)
    m = make_model()
    T = 4
    states = torch.randn(T, m.num_observations)

    # ---- collection rollout: carry membrane, record per-step initial mem ----
    mem = m._init_mem(1)  # list of (1, dim) per slot
    init_mems = []        # per-step list of slot tensors (1, dim)
    mean_collect = []
    for t in range(T):
        init_mems.append([s.clone() for s in mem])
        m._clear_cache()
        mean_t, _, out = m.compute(
            {"states": states[t:t + 1], "rnn": [s.unsqueeze(0) for s in mem]},
            role="policy",
        )
        mean_collect.append(mean_t.detach().clone())
        # carry membrane to next step (stored_mem slots are (1, n, dim))
        mem = [sm.squeeze(0).detach() for sm in out["rnn"]]
    mean_collect = torch.cat(mean_collect, dim=0)

    # ---- update path: batch dim = T, each row uses its own stored init mem ----
    # This is what _prepare_mem receives from memory.sample_all (contiguous chunk).
    batched_rnn = []
    n_slots = len(init_mems[0])
    for slot in range(n_slots):
        stacked = torch.cat([init_mems[t][slot] for t in range(T)], dim=0)  # (T, dim)
        batched_rnn.append(stacked.unsqueeze(0))  # (1, T, dim) -> _prepare_mem squeezes
    m._clear_cache()
    mean_update, _, _ = m.compute({"states": states, "rnn": batched_rnn}, role="policy")
    mean_update = mean_update.detach()

    diff = (mean_update - mean_collect).abs().max().item()
    print("== P2 (corrected): update-path per-sample policy evaluation ==")
    print(f"  collection mean actions shape {tuple(mean_collect.shape)}")
    print(f"  update     mean actions shape {tuple(mean_update.shape)}")
    print(f"  max|diff| collection-time vs update-time = {diff:.3e}")
    print(f"  per-sample evaluation CORRECT (diff ~ 0): {diff < 1e-5}")

    # ---- contrast: the reshape branch (what happens if rnn batch dim != T) ----
    m._clear_cache()
    single_mem = [s.unsqueeze(0) for s in init_mems[0]]  # (1,1,dim): num_envs=1 != T
    mean_reshape, _, _ = m.compute({"states": states, "rnn": single_mem}, role="policy")
    diff2 = (mean_reshape.detach() - mean_collect).abs().max().item()
    print(f"  (reshape branch, single-env mem) max|diff| = {diff2:.3e}  "
          f"-> only triggered when rnn batch dim != minibatch size")


if __name__ == "__main__":
    main()
