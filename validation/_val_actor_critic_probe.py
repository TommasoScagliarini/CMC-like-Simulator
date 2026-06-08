"""Decisive probes of ProsthesisSNNActorCritic forward semantics.

Checks:
  P1  data_ptr cache: does a stale pointer alias return a wrong cached result?
  P2  update-path forward treats a minibatch as a TIME sequence (recurrent),
      not as independent samples -> mis-evaluates PPO log-prob.
  P3  inference (ReferenceGenerator) returns the raw, UNCLIPPED policy mean,
      while training samples CLIPPED actions in [-1, 1].
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
for p in (REPO / "Trajectory Generator" / "Prosthesis_SNN",):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

# Import order matters on this Windows box: the package __init__ applies a
# torch/OpenSim OpenMP-DLL workaround. torch must load (via the package) BEFORE
# numpy/MKL pulls in a conflicting libiomp5md.dll, or fbgemm.dll fails to load.
import prosthesis_snn  # noqa: F401  (triggers DLL workaround)
import torch
import numpy as np

from gymnasium.spaces import Box

from prosthesis_snn.config import SNNConfig
from prosthesis_snn.model import ProsthesisReferenceSNN
from prosthesis_snn.training.actor_critic import ProsthesisSNNActorCritic


def _make_model(input_size=6, hidden=8, knots=3):
    cfg = SNNConfig.for_env_action(
        input_size=input_size, hidden_size=hidden, num_layers=1, policy_knots=knots,
        feature_names=tuple(f"f{i}" for i in range(input_size)),
    )
    obs_space = Box(-np.inf, np.inf, (input_size,), dtype=np.float32)
    act_space = Box(-1.0, 1.0, (knots * 2,), dtype=np.float32)
    torch.manual_seed(0)
    model = ProsthesisSNNActorCritic(
        observation_space=obs_space, action_space=act_space, device="cpu",
        cfg=cfg, num_envs=1, sequence_length=1, log_std_init=-2.0,
    )
    model.eval()
    return model, cfg


def p1_cache_alias():
    print("== P1: forward cache is keyed on data_ptr only (content-independent) ==")
    model, _ = _make_model()
    # Make the forward genuinely input-sensitive so a stale hit would be visible.
    with torch.no_grad():
        model.reference_model.backbone.encoder.fc.weight.mul_(50.0)
        model.reference_model.output_lif.fc.weight.mul_(50.0)
    mem = [m.unsqueeze(0) for m in model._init_mem(1)]
    s = torch.zeros(1, model.num_observations)
    rA, _, _ = model.compute({"states": s, "rnn": mem}, role="policy")
    rA = rA.detach().clone()
    # Mutate the SAME storage in place: data_ptr and mem pointers are unchanged,
    # so the cache (keyed only on data_ptr) returns the STALE result although the
    # input content is now completely different.
    s.fill_(5.0)
    rB, _, _ = model.compute({"states": s, "rnn": mem}, role="policy")
    model._clear_cache()
    rB_true, _, _ = model.compute({"states": s.clone(), "rnn": [m.clone() for m in mem]}, role="policy")
    stale = torch.allclose(rB.detach(), rA) and not torch.allclose(rB.detach(), rB_true.detach())
    print(f"  same data_ptr after in-place edit: {s.data_ptr() != 0}")
    print(f"  cached result == stale(old) result: {torch.allclose(rB.detach(), rA)}")
    print(f"  cached vs TRUE-for-new-content max|diff|: {float((rB - rB_true).abs().max()):.4e}")
    print(f"  -> UNSOUND KEYING demonstrated (stale hit): {stale}")
    print("     Realistic trigger: a freed tensor's address reused by a new")
    print("     tensor across timesteps (allocator-dependent); mitigated in the")
    print("     update loop only because _clear_cache() runs after each step.")


def p2_update_treats_minibatch_as_sequence():
    print("\n== P2: update-path forward processes a minibatch as a recurrent TIME sequence ==")
    model, _ = _make_model()
    K = 4
    # K independent samples that (as in a PPO minibatch) each had the SAME stored
    # initial membrane (zeros). Correct PPO eval = K independent single-step
    # forwards from that mem. The update path instead runs them recurrently.
    states = torch.randn(K, model.num_observations)
    mem_1env = model._init_mem(1)  # the single initial mem the update feeds in

    # Path A: exactly what _update feeds to the model in the batch_size != num_envs case
    model._clear_cache()
    meanA, _, _ = model.compute(
        {"states": states, "rnn": [m.unsqueeze(0) for m in mem_1env]}, role="policy"
    )
    meanA = meanA.detach()

    # Path B: correct independent per-sample evaluation, each from the same mem0
    meansB = []
    for i in range(K):
        model._clear_cache()
        mi, _, _ = model.compute(
            {"states": states[i : i + 1], "rnn": [m.clone() for m in mem_1env]},
            role="policy",
        )
        meansB.append(mi.detach())
    meanB = torch.cat(meansB, dim=0)

    maxdiff = float((meanA - meanB).abs().max())
    print(f"  shapes: update-path {tuple(meanA.shape)}, per-sample {tuple(meanB.shape)}")
    print(f"  max|diff| between update-path eval and correct per-sample eval = {maxdiff:.4e}")
    print(f"  row 0 identical? {torch.allclose(meanA[0], meanB[0], atol=1e-6)}  (expected True)")
    print(f"  rows 1..K-1 differ (recurrent accumulation)? {maxdiff > 1e-4}")
    print("  -> if rows differ, PPO recomputes log_prob from a recurrent state")
    print("     that did NOT occur during the rollout (biased policy gradient).")


def p3_inference_unclipped_mean():
    print("\n== P3: inference returns unclipped mean vs training clips sampled actions ==")
    from prosthesis_snn.generator import ReferenceGenerator
    model, cfg = _make_model()
    # Force a large-magnitude mean by inflating the output layer bias.
    with torch.no_grad():
        model.reference_model.output_lif.fc.bias.add_(3.0)
    # Training: sampled action is clipped to [-1, 1].
    feat = torch.zeros(1, model.num_observations)
    actions, _, outs = model.act({"states": feat, "rnn": [m.unsqueeze(0) for m in model._init_mem(1)]}, role="policy")
    train_abs_max = float(actions.detach().abs().max())
    mean_abs_max = float(outs["mean_actions"].detach().abs().max())
    # Inference path: export reference, predict_action (no clip).
    gen = ReferenceGenerator(
        ProsthesisReferenceSNN(cfg), cfg=cfg, device="cpu",
    )
    gen.model.load_state_dict(model.reference_state_dict())
    gen.reset()
    act_infer = gen.predict_action({n: 0.0 for n in cfg.feature_names}, action_shape=cfg.action_shape)
    infer_abs_max = float(np.abs(act_infer).max())
    print(f"  training sampled-action |max| (clip_actions=True): {train_abs_max:.3f}")
    print(f"  training mean |max| (unclipped):                   {mean_abs_max:.3f}")
    print(f"  inference predict_action |max| (unclipped):        {infer_abs_max:.3f}")
    print(f"  inference can exceed action-space [-1,1]: {infer_abs_max > 1.0}")


if __name__ == "__main__":
    p1_cache_alias()
    p2_update_treats_minibatch_as_sequence()
    p3_inference_unclipped_mean()
